#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::style)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![warn(clippy::use_self)]

use bsp::entry;
use bsp::hal::{self, rosc::RingOscillator, Timer};
use defmt::{error, info, warn};
use defmt_rtt as _;
use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
use heapless::Vec;
use panic_probe as _;
use rp_pico as bsp;
use smoltcp::iface::PollResult;
use smoltcp::socket::dhcpv4::RetryConfig;
use smoltcp::time::Duration;
use smoltcp::wire::DhcpOption;
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::{dhcpv4, tcp},
    time::Instant,
    wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr},
};
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use web_server::WebServer;

use usbd_ethernet::DeviceState;
use usbd_ethernet::Ethernet;

mod web_server;

const HOST_MAC_ADDR: [u8; 6] = [0x88, 0x88, 0x88, 0x88, 0x88, 0x88];
const DEVICE_MAC_ADDR: [u8; 6] = [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
const HOST_NAME: &[u8] = b"pico";

#[entry]
fn main() -> ! {
    static mut HTTP_SOCKET_RX_DATA: [u8; 1024] = [0; 1024];
    static mut HTTP_SOCKET_TX_DATA: [u8; 1024] = [0; 1024];
    static mut ETHERNET_IN_BUFFER: [u8; 2048] = [0; 2048];
    static mut ETHERNET_OUT_BUFFER: [u8; 2048] = [0; 2048];
    static mut WEB_SERVER_BUFFER: Vec<u8, 512> = Vec::new();

    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let ring_oscillator = RingOscillator::new(pac.ROSC);
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led_pin = pins.led.into_push_pull_output();

    let usb_bus = hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    let usb_alloc = UsbBusAllocator::new(usb_bus);
    let mut ethernet = Ethernet::new(
        &usb_alloc,
        HOST_MAC_ADDR,
        64,
        ETHERNET_IN_BUFFER,
        ETHERNET_OUT_BUFFER,
    );
    let mut usb_dev = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x0004))
        .strings(&[StringDescriptors::default()
            .manufacturer("dlkj")
            .product("usbd-ethernet")
            .serial_number("TEST")])
        .unwrap()
        .device_class(usbd_ethernet::USB_CLASS_CDC)
        .build();

    let mut interface_config = Config::new(EthernetAddress(DEVICE_MAC_ADDR).into());
    let ring_oscillator = ring_oscillator.initialize();
    interface_config.random_seed = get_random_seed(&ring_oscillator);

    let mut interface = Interface::new(
        interface_config,
        &mut ethernet,
        Instant::from_micros(i64::try_from(timer.get_counter().ticks()).unwrap()),
    );
    interface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0).into())
            .unwrap();
    });

    // Create sockets
    let mut dhcp_socket = dhcpv4::Socket::new();

    let mut retry_config = RetryConfig::default();
    retry_config.discover_timeout = Duration::from_secs(5);

    dhcp_socket.set_retry_config(retry_config);
    // register hostname with dhcp
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12, // Host Name
        data: HOST_NAME,
    }]);

    let http_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut HTTP_SOCKET_RX_DATA[..]),
        tcp::SocketBuffer::new(&mut HTTP_SOCKET_TX_DATA[..]),
    );

    let mut sockets: [_; 3] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);
    let http_handle = sockets.add(http_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    let mut web_server = WebServer::new(http_handle, led_pin, WEB_SERVER_BUFFER);

    loop {
        usb_poll(&mut usb_dev, &mut ethernet);

        if ethernet.state() == DeviceState::Connected {
            // panic safety - will take 292_277 years to overflow at one tick per microsecond
            let timestamp =
                Instant::from_micros(i64::try_from(timer.get_counter().ticks()).unwrap());

            if let PollResult::SocketStateChanged =
                interface.poll(timestamp, &mut ethernet, &mut sockets)
            {
                dhcp_poll(
                    &mut interface,
                    sockets.get_mut::<dhcpv4::Socket>(dhcp_handle),
                );
                web_server.poll(|handle| sockets.get_mut(handle));
            }
        }
    }
}

fn usb_poll<B: UsbBus>(usb_dev: &mut UsbDevice<B>, cdc_ethernet: &mut Ethernet<B>) {
    if usb_dev.poll(&mut [cdc_ethernet]) && cdc_ethernet.state() == DeviceState::Disconnected {
        if cdc_ethernet.connection_speed().is_none() {
            // 1000 Kps upload and download
            match cdc_ethernet.set_connection_speed(1_000_000, 1_000_000) {
                Ok(()) | Err(UsbError::WouldBlock) => {}
                Err(e) => error!("Failed to set connection speed: {}", e),
            }
        } else if cdc_ethernet.state() == DeviceState::Disconnected {
            match cdc_ethernet.connect() {
                Ok(()) | Err(UsbError::WouldBlock) => {}
                Err(e) => error!("Failed to connect: {}", e),
            }
        }
    }
}

fn get_random_seed(ring_oscillator: &RingOscillator<hal::rosc::Enabled>) -> u64 {
    let mut seed = 0u64;
    for i in 0..64 {
        seed += u64::from(ring_oscillator.get_random_bit()) << i;
    }
    seed
}

fn dhcp_poll(iface: &mut Interface, socket: &mut dhcpv4::Socket) {
    let event = socket.poll();
    match event {
        None => {}
        Some(dhcpv4::Event::Configured(config)) => {
            info!("dhcp: DHCP configured");

            info!("     IP address:      {}", config.address);
            set_ipv4_addr(iface, config.address);

            if let Some(router) = config.router {
                info!("     Default gateway: {}", router);
                iface.routes_mut().add_default_ipv4_route(router).unwrap();
            } else {
                info!("     Default gateway: None");
                iface.routes_mut().remove_default_ipv4_route();
            }

            for (i, s) in config.dns_servers.iter().enumerate() {
                info!("     DNS server {}:    {}", i, s);
            }
        }
        Some(dhcpv4::Event::Deconfigured) => {
            info!("dhcp: DHCP deconfigured");
            set_ipv4_addr(iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
            iface.routes_mut().remove_default_ipv4_route();
        }
    }
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}
