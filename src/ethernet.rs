#![allow(dead_code)]

use super::buffer::RWBuffer;
use super::bytes::{Buf, BufMut};
use heapless::String;
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::Result;

/*
 * References:
 *   - Universal Serial Bus Communications Class Subclass Specifications for Network
 *     Control Model Devices - Revision 1.0 (Errata 1) - November 24, 2010
 *   - Universal Serial Bus Class Definitions for Communications Devices  - Revision
 *     1.2 (Errata 1) - November 3, 2010
 *   - Universal Serial Bus Communications Class Subclass Specification for Ethernet
 *     Control Model Devices - Revision 1.2 - February 9, 2007
 *
 * Acknowledgement to:
 *   - TinyUSB:     https://github.com/hathach/tinyusb
 *   - embassy-usb: https://github.com/embassy-rs/embassy/tree/master/embassy-usb
 *   - usbd-serial: https://github.com/rust-embedded-community/usbd-serial
 */

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_CDC: u8 = 0x02;

const NTB_MAX_SIZE: u32 = 2048;
const NTB_MAX_SIZE_USIZE: usize = NTB_MAX_SIZE as usize;
const MAX_SEGMENT_SIZE: u16 = 1514;

const SIG_NTH: &[u8; 4] = b"NCMH";
const SIG_NDP_NO_FCS: &[u8; 4] = b"NCM0";
const SIG_NDP_WITH_FCS: &[u8; 4] = b"NCM1";

const REQ_TYPE_DEVICE_TO_HOST: u8 = 0xA1;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DeviceState {
    Disabled,
    Disconnected,
    Connected,
}
#[derive(PartialEq, Eq)]
enum HostNotificationState {
    Complete,
    InProgress(HostNotification),
}

#[derive(PartialEq, Eq)]
enum HostNotification {
    Connect,
    Disconnect,
    Speed(Speed),
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Speed {
    pub upload_bit_rate: u32,
    pub download_bit_rate: u32,
}

pub struct Ethernet<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    mac_address: String<12>,
    mac_address_idx: StringIndex,
    state: DeviceState,
    request_state: HostNotificationState,
    in_buf: InBuf<'a, B>,
    out_buf: OutBuf<'a, B>,
    connection_speed: Option<Speed>,
}

struct InBuf<'a, B: UsbBus> {
    write_ep: EndpointIn<'a, B>,
    buffer: RWBuffer<'a, NTB_MAX_SIZE_USIZE>,
    next_seq: u16,
}

struct OutBuf<'a, B: UsbBus> {
    read_ep: EndpointOut<'a, B>,
    buffer: RWBuffer<'a, NTB_MAX_SIZE_USIZE>,
    datagram_len: Option<usize>,
}

impl<'a, B: UsbBus> Ethernet<'a, B> {
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        mac_address: [u8; 6],
        max_packet_size: u16,
        in_buffer: &'a mut [u8; NTB_MAX_SIZE_USIZE],
        out_buffer: &'a mut [u8; NTB_MAX_SIZE_USIZE],
    ) -> Self {
        let mac_address_idx = alloc.string();

        Self {
            comm_if: alloc.interface(),
            comm_ep: alloc.interrupt(16, 255),
            data_if: alloc.interface(),
            mac_address: mac_bytes_to_string(mac_address),
            mac_address_idx,
            state: DeviceState::Disabled,
            request_state: HostNotificationState::Complete,
            in_buf: InBuf {
                write_ep: alloc.bulk(max_packet_size),
                buffer: RWBuffer::new(in_buffer),
                next_seq: 0,
            },
            out_buf: OutBuf {
                read_ep: alloc.bulk(max_packet_size),
                buffer: RWBuffer::new(out_buffer),
                datagram_len: None,
            },
            connection_speed: None,
        }
    }

    pub fn connect(&mut self) -> Result<()> {
        self.network_connection_notification(true)
    }

    pub fn disconnect(&mut self) -> Result<()> {
        self.network_connection_notification(false)
    }

    fn network_connection_notification(&mut self, connect: bool) -> Result<()> {
        const NOTE_TYPE_NETWORK_CONNECTION: u8 = 0x00;

        if self.state == DeviceState::Disabled {
            #[cfg(feature = "defmt")]
            defmt::warn!("ethernet: device can't change connection state while disabled",);
            return Err(UsbError::WouldBlock);
        }

        if let HostNotificationState::InProgress(_) = self.request_state {
            return Err(UsbError::WouldBlock);
        }

        let mut data = [0x00; 8];
        let mut writer = &mut data[..];
        writer.put_u8(REQ_TYPE_DEVICE_TO_HOST); // bmRequestType
        writer.put_u8(NOTE_TYPE_NETWORK_CONNECTION); // bNotificationType
        writer.put_u16_le(connect.into()); // wValue
        writer.put_u16_le(u8::from(self.data_if).into()); // wIndex = interface
        writer.put_u16_le(0x00); // wLength
        assert!(!writer.has_remaining_mut());
        let result = self.comm_ep.write(&data);

        if result.is_ok() {
            #[cfg(feature = "defmt")]
            #[allow(clippy::if_same_then_else)]
            if connect {
                defmt::debug!("ethernet: connecting");
            } else {
                defmt::debug!("ethernet: disconnecting");
            }
            self.request_state = HostNotificationState::InProgress(HostNotification::Connect);
        }

        result.map(drop)
    }

    pub fn set_connection_speed(
        &mut self,
        download_bit_rate: u32,
        upload_bit_rate: u32,
    ) -> Result<()> {
        const NOTE_TYPE_CONNECTION_SPEED_CHANGE: u8 = 0x2A;

        if self.state == DeviceState::Disabled {
            #[cfg(feature = "defmt")]
            defmt::warn!("ethernet: device can't set connection speed while disabled",);
            return Err(UsbError::WouldBlock);
        }

        if let HostNotificationState::InProgress(_) = self.request_state {
            return Err(UsbError::WouldBlock);
        }

        let mut data = [0x00; 16];
        let mut writer = &mut data[..];
        writer.put_u8(REQ_TYPE_DEVICE_TO_HOST); // bmRequestType
        writer.put_u8(NOTE_TYPE_CONNECTION_SPEED_CHANGE); // bNotificationType
        writer.put_u16_le(0); // wValue
        writer.put_u16_le(u8::from(self.data_if).into()); // wIndex = interface
        writer.put_u16_le(0x08); // wLength
        writer.put_u32_le(download_bit_rate); // data - DLBitRate
        writer.put_u32_le(upload_bit_rate); // data - ULBitRate
        assert!(!writer.has_remaining_mut());

        let result = self.comm_ep.write(&data);

        if result.is_ok() {
            self.request_state =
                HostNotificationState::InProgress(HostNotification::Speed(Speed {
                    upload_bit_rate,
                    download_bit_rate,
                }));
            #[cfg(feature = "defmt")]
            defmt::debug!("ethernet: setting connection speed");
        }

        result.map(drop)
    }

    #[must_use]
    pub fn connection_speed(&self) -> Option<Speed> {
        self.connection_speed
    }

    #[must_use]
    pub fn state(&self) -> DeviceState {
        self.state
    }
}

fn mac_bytes_to_string(mac_address: [u8; 6]) -> String<12> {
    let mut s = String::new();
    for i in 0..12 {
        let n = (mac_address[i / 2] >> ((1 - i % 2) * 4)) & 0xF;
        match n {
            0x0..=0x9 => s.push(char::from(b'0' + n)),
            0xA..=0xF => s.push(char::from(b'A' + n - 0xA)),
            _ => unreachable!(),
        }
        .unwrap();
    }
    s
}
impl<'a, B: UsbBus> InBuf<'a, B> {
    /// Writes a single packet into the IN endpoint.
    pub fn write_datagram<F, R>(&mut self, len: u16, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        const OUT_HEADER_LEN: u16 = 28;

        if !self.can_write() {
            return Err(UsbError::WouldBlock);
        }

        if usize::from(len + OUT_HEADER_LEN) > self.buffer.capacity() {
            return Err(UsbError::BufferOverflow);
        }

        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);

        self.buffer.write(usize::from(OUT_HEADER_LEN), |mut buf| {
            buf.put_slice(SIG_NTH); // dwSignature
            buf.put_u16_le(0x0c); // wHeaderLength
            buf.put_u16_le(seq); // wSequence
            buf.put_u16_le(len + OUT_HEADER_LEN); // wBlockLength
            buf.put_u16_le(0x0c); // wNdpIndex

            // NDP
            buf.put_slice(SIG_NDP_NO_FCS); // dwSignature
            buf.put_u16_le(0x0010); // wLength
            buf.put_u16_le(0x0000); // wNextNdpIndex
            buf.put_u16_le(OUT_HEADER_LEN); // wDatagramIndex
            buf.put_u16_le(len); // wDatagramLength
            buf.put_u16_le(0x0000); // wZeroIndex
            buf.put_u16_le(0x0000); // wZeroLength

            assert!(!buf.has_remaining_mut());
            Ok((OUT_HEADER_LEN.into(), ()))
        })?;

        // Write the datagram
        let (_, result) = self
            .buffer
            .write(len.into(), |buf| Ok((len.into(), f(buf))))?;

        match self.write_packet() {
            Err(UsbError::WouldBlock) | Ok(()) => Ok(result),
            Err(e) => Err(e),
        }
    }

    fn can_write(&mut self) -> bool {
        match self.write_packet() {
            Ok(()) => self.buffer.is_empty(),
            Err(_) => false,
        }
    }

    fn write_packet(&mut self) -> Result<()> {
        let max_packet_size = self.write_ep.max_packet_size().into();

        if self.buffer.is_empty() {
            // No data to send
            Ok(())
        } else if !self.buffer.has_unread() {
            // Zero length packet
            match self.write_ep.write(&[]) {
                Ok(_) => {
                    self.buffer.clear();
                    Ok(())
                }
                Err(UsbError::WouldBlock) => return Err(UsbError::WouldBlock),
                Err(e) => {
                    self.buffer.clear();
                    Err(e)
                }
            }
        } else if self.buffer.unread() >= max_packet_size {
            // Full packet
            match self.buffer.read(max_packet_size, |data| {
                self.write_ep.write(data).map(|r| (max_packet_size, r))
            }) {
                Ok(_) | Err(UsbError::WouldBlock) => Err(UsbError::WouldBlock),
                Err(err) => {
                    self.buffer.clear();
                    return Err(err);
                }
            }
        } else {
            // Short packet
            let len = self.buffer.unread();
            match self
                .buffer
                .read(len, |data| self.write_ep.write(data).map(|r| (len, r)))
            {
                Ok(_) => {
                    self.buffer.clear();
                    Ok(())
                }
                Err(UsbError::WouldBlock) => return Err(UsbError::WouldBlock),
                Err(e) => {
                    self.buffer.clear();
                    Err(e)
                }
            }
        }
    }

    fn reset(&mut self) {
        self.next_seq = 0;
        self.buffer.clear();
    }
}

impl<'a, B: UsbBus> OutBuf<'a, B> {
    fn can_read(&mut self) -> bool {
        match self.read_packet() {
            Ok(()) => self.datagram_len.is_some(),
            Err(_) => false,
        }
    }

    fn read_datagram<R, F>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        if !self.can_read() {
            return Err(UsbError::WouldBlock);
        }

        let result = if let Some(datagram_len) = self.datagram_len {
            // Read the datagram
            self.buffer
                .read(datagram_len, |data| Ok((datagram_len, f(data))))?
                .1
        } else {
            return Err(UsbError::WouldBlock);
        };
        self.datagram_len = None;
        self.buffer.clear();

        match self.read_packet() {
            Err(UsbError::WouldBlock) | Ok(()) => Ok(result),
            Err(e) => Err(e),
        }
    }

    /// Reads a single packet from the OUT endpoint.
    pub fn read_packet(&mut self) -> Result<()> {
        const NTB_HEADER_LEN: usize = 12;
        const NDP_LEN: usize = 12;

        if self.datagram_len.is_some() {
            return Ok(());
        }

        let (read, ()) = self
            .buffer
            .write(self.read_ep.max_packet_size().into(), |data| {
                Ok((self.read_ep.read(data)?, ()))
            })?;

        if read == self.read_ep.max_packet_size().into() && !self.buffer.is_full() {
            return Err(UsbError::WouldBlock);
        }

        // Process NTB header
        let ndp_offset = match self.buffer.read(NTB_HEADER_LEN, |data| {
            let mut data: &[u8] = data;
            let sig = data.get_slice(4);
            if sig != SIG_NTH {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: received bad NTH sig.");
                return Err(UsbError::ParseError);
            }

            data.advance(6); // wHeaderLength, wSequence, wBlockLength
            let Some(ndp_idx) = data.get_u16_le().map(usize::from) else {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: NTH too short, unable to read ndp_idx");
                return Err(UsbError::ParseError);
            };
            assert!(!data.has_remaining());
            Ok((NTB_HEADER_LEN, ndp_idx - NTB_HEADER_LEN))
        }) {
            Ok((_, ndp_offset)) => ndp_offset,
            Err(UsbError::InvalidState) => {
                #[cfg(feature = "defmt")]
                defmt::error!("ntb: read NTB header too short NTB");
                self.buffer.clear();
                return Err(UsbError::ParseError);
            }
            Err(e) => {
                self.buffer.clear();
                return Err(e);
            }
        };

        // Process NTB Datagram Pointer

        match self.buffer.read(self.buffer.unread(), |data| {
            let Some(mut ntb_datagram_pointer) = data.get(ndp_offset..ndp_offset + NDP_LEN) else {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: NTB datagram pointer out of range or truncated");
                return Err(UsbError::ParseError);
            };

            // wdSignature
            let sig = ntb_datagram_pointer.get_slice(4);
            if sig != SIG_NDP_NO_FCS && sig != SIG_NDP_WITH_FCS {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: received bad NDP sig");
                return Err(UsbError::ParseError);
            }

            ntb_datagram_pointer.advance(4); // wLength, reserved

            let Some(datagram_index) = ntb_datagram_pointer.get_u16_le().map(usize::from) else {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: NTB too short, unable to read datagram_index");
                return Err(UsbError::ParseError);
            };
            let Some(datagram_len) = ntb_datagram_pointer.get_u16_le().map(usize::from) else {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: NTB too short, unable to read datagram_len");
                return Err(UsbError::ParseError);
            };

            if datagram_index == 0 || datagram_len == 0 {
                // empty, ignore. This is allowed by the spec, so don't warn.
                #[cfg(feature = "defmt")]
                defmt::debug!("ethernet: empty datagram");
                return Err(UsbError::WouldBlock);
            }

            let datagram_offset = datagram_index - NTB_HEADER_LEN;

            if data
                .get(datagram_offset..datagram_offset + datagram_len)
                .is_none()
            {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: NDP datagram pointer out of range");
                return Err(UsbError::ParseError);
            };

            // mark all data up to the datagram_offset as read
            Ok((datagram_offset, datagram_len))
        }) {
            Ok((_, len)) => {
                self.datagram_len = Some(len);
            }
            Err(e) => {
                self.buffer.clear();
                return Err(e);
            }
        };

        Ok(())
    }

    fn reset(&mut self) {
        self.buffer.clear();
        self.datagram_len = None;
    }
}

impl<B: UsbBus> UsbClass<B> for Ethernet<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        const CDC_PROTOCOL_NONE: u8 = 0x00;
        const CDC_PROTOCOL_NTB: u8 = 0x01;
        const CDC_SUBCLASS_NCM: u8 = 0x0d;
        const CDC_TYPE_ETHERNET: u8 = 0x0F;
        const CDC_TYPE_HEADER: u8 = 0x00;
        const CDC_TYPE_NCM: u8 = 0x1A;
        const CS_INTERFACE: u8 = 0x24;
        const USB_CLASS_CDC_DATA: u8 = 0x0a;

        // Interface Association Descriptor

        writer.iad(
            self.comm_if,
            2,
            USB_CLASS_CDC,
            CDC_SUBCLASS_NCM,
            CDC_PROTOCOL_NONE,
            None,
        )?;

        // Communication Class Interface (interface n)
        // Functional descriptors for the Communication Class Interface

        writer.interface(
            self.comm_if,
            USB_CLASS_CDC,
            CDC_SUBCLASS_NCM,
            CDC_PROTOCOL_NONE,
        )?;

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 3;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_HEADER); // bDescriptorSubtype
                buf.put_u16_le(0x0120); // bcdCDC (1.20)
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 11;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_ETHERNET); // bDescriptorSubtype
                buf.put_u8(self.mac_address_idx.into()); // iMACAddress
                buf.put_u32_le(0); // bmEthernetStatistics
                buf.put_u16_le(MAX_SEGMENT_SIZE); // wMaxSegmentSize
                buf.put_u16_le(0); // wNumberMCFilters
                buf.put_u8(0); // bNumberPowerFilters
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

        writer.write_with(CS_INTERFACE, |buf| {
            const LEN: usize = 4;
            if let Some(mut buf) = buf.get_mut(..LEN) {
                buf.put_u8(CDC_TYPE_NCM); // bDescriptorSubtype
                buf.put_u16_le(0x0100); // bcdCDC (1.00)
                buf.put_u8(0x00); // bmCapabilities - none
                assert!(!buf.has_remaining_mut());
                Ok(LEN)
            } else {
                Err(UsbError::BufferOverflow)
            }
        })?;

        // Endpoint descriptors for the Communication Class Interface

        writer.endpoint(&self.comm_ep)?;

        // Data Class Interface (interface n+1, alternate setting 0)
        // Functional descriptors for Data Class Interface (interface n+1, alternate setting 0)

        writer.interface_alt(
            self.data_if,
            0,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        // Data Class Interface (interface n+1, alternate setting 1)
        // Functional descriptors for Data Class Interface (interface n+1, alternate setting 1)

        writer.interface_alt(
            self.data_if,
            1,
            USB_CLASS_CDC_DATA,
            0x00,
            CDC_PROTOCOL_NTB,
            None,
        )?;

        // Endpoint descriptors for Data Class Interface (interface n+1, alternate setting 1)

        writer.endpoint(&self.in_buf.write_ep)?;
        writer.endpoint(&self.out_buf.read_ep)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("ethernet: configuration descriptors written");

        Ok(())
    }

    fn control_in(&mut self, transfer: ControlIn<B>) {
        const REQ_GET_NTB_PARAMETERS: u8 = 0x80;
        const REQ_GET_NTB_INPUT_SIZE: u8 = 0x85;

        let req = transfer.request();

        if req.recipient != control::Recipient::Interface {
            // Only handle interface requests
            return;
        }

        if req.index == u16::from(u8::from(self.data_if)) {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "ethernet: unhandled DATA_INTERFACE control_in {} {}",
                req.request_type,
                req.request
            );
            return;
        }

        if req.index != u16::from(u8::from(self.comm_if)) {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "ethernet: control_in unexpected interface {} - {} {}",
                req.index,
                req.request_type,
                req.request
            );
            return;
        }

        match (req.request_type, req.request) {
            (control::RequestType::Class, REQ_GET_NTB_PARAMETERS) => {
                #[cfg(feature = "defmt")]
                defmt::debug!("ethernet: REQ_GET_NTB_PARAMETERS");
                let _: Result<()> = transfer.accept(|data| {
                    const LEN: u16 = 28;
                    if let Some(mut data) = data.get_mut(..LEN.into()) {
                        data.put_u16_le(LEN); //wLength
                        data.put_u16_le(0x01); // bmNtbFormatsSupported - 16-bit NTB supported only
                        data.put_u32_le(NTB_MAX_SIZE); // dwNtbInMaxSize
                        data.put_u16_le(0x04); // wNdpInDivisor
                        data.put_u16_le(0x00); // wNdpInPayloadRemainder
                        data.put_u16_le(0x04); // wNdpInAlignment
                        data.put_u16_le(0x00); // wReserved
                        data.put_u32_le(NTB_MAX_SIZE); // dwNtbOutMaxSize
                        data.put_u16_le(0x04); // wNdpOutDivisor
                        data.put_u16_le(0x00); // wNdpOutPayloadRemainder
                        data.put_u16_le(0x04); // wNdpOutAlignment
                        data.put_u16_le(0x01); // wNtbOutMaxDatagrams
                        assert!(!data.has_remaining_mut());
                        Ok(LEN.into())
                    } else {
                        Err(UsbError::BufferOverflow)
                    }
                });
            }
            (control::RequestType::Class, REQ_GET_NTB_INPUT_SIZE) => {
                #[cfg(feature = "defmt")]
                defmt::debug!("ethernet: REQ_GET_NTB_INPUT_SIZE");
                let _: Result<()> = transfer.accept(|data| {
                    const LEN: usize = 4;

                    // We only support the minimum NTB maximum size so this can be a constant
                    if let Some(mut data) = data.get_mut(..LEN) {
                        data.put_u32_le(NTB_MAX_SIZE);
                        assert!(!data.has_remaining_mut()); // dwNtbInMaxSize
                        Ok(LEN)
                    } else {
                        Err(UsbError::BufferOverflow)
                    }
                });
            }
            _ => {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "ethernet: unhandled COMMUNICATION interface control_in {} {}",
                    req.request_type,
                    req.request
                );
            }
        }
    }

    fn control_out(&mut self, transfer: ControlOut<B>) {
        const REQ_SET_INTERFACE: u8 = 0x0B;
        const REQ_SET_NTB_INPUT_SIZE: u8 = 0x86;

        let req = transfer.request();

        if req.recipient != control::Recipient::Interface {
            // Only handle interface requests
            return;
        }

        if req.index == u16::from(u8::from(self.comm_if)) {
            if let (control::RequestType::Class, REQ_SET_NTB_INPUT_SIZE) =
                (req.request_type, req.request)
            {
                #[cfg(feature = "defmt")]
                defmt::debug!("ethernet: REQ_SET_NTB_INPUT_SIZE");
                // We only support the minimum NTB maximum size the value
                // will always be NTB_MAX_SIZE
                if let Some(ntb_input_size) = transfer.data().get_u32_le() {
                    if ntb_input_size != NTB_MAX_SIZE {
                        #[cfg(feature = "defmt")]
                        defmt::warn!(
                            "ncp: unexpected REQ_SET_NTB_INPUT_SIZE data {}",
                            transfer.data()
                        );
                    }
                } else {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("ncp: unexpected REQ_SET_NTB_INPUT_SIZE data too short");
                };

                let _: Result<()> = transfer.accept();
            } else {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "ethernet: unhandled COMMUNICATION interface control_out {} {}",
                    req.request_type,
                    req.request
                );
            }
            return;
        }

        if req.index == u16::from(u8::from(self.data_if)) {
            if let (control::RequestType::Standard, REQ_SET_INTERFACE) =
                (req.request_type, req.request)
            {
                #[cfg(feature = "defmt")]
                defmt::debug!("ethernet: REQ_SET_INTERFACE");
                if req.value == 0 {
                    transfer.accept().ok();
                    self.state = DeviceState::Disabled;
                    #[cfg(feature = "defmt")]
                    defmt::info!("ethernet: data interface disabled");
                    self.reset();
                } else if req.value == 1 {
                    #[cfg(feature = "defmt")]
                    defmt::info!("ethernet: data interface enabled");
                    self.state = DeviceState::Disconnected;
                    transfer.accept().ok();
                } else {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("SET_INTERFACE out of range {}", req.request);
                    transfer.reject().ok();
                }
            } else {
                #[cfg(feature = "defmt")]
                defmt::warn!(
                    "ethernet: unhandled DATA_INTERFACE control_out {} {}",
                    req.request_type,
                    req.request
                );
            }
            #[allow(clippy::needless_return)]
            return;
        }

        #[cfg(feature = "defmt")]
        defmt::warn!(
            "ethernet: control_out unexpected interface {} - {} {}",
            req.index,
            req.request_type,
            req.request
        );
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr != self.comm_ep.address() {
            return;
        }

        match self.request_state {
            HostNotificationState::Complete => {
                #[cfg(feature = "defmt")]
                defmt::warn!("ethernet: endpoint in completed when no request was in progress");
            }
            HostNotificationState::InProgress(HostNotification::Connect) => {
                #[cfg(feature = "defmt")]
                defmt::info!("ethernet: connected");
                self.state = DeviceState::Connected;
            }
            HostNotificationState::InProgress(HostNotification::Disconnect) => {
                #[cfg(feature = "defmt")]
                defmt::info!("ethernet: disconnected");
                self.state = DeviceState::Disconnected;
            }
            HostNotificationState::InProgress(HostNotification::Speed(cs)) => {
                #[cfg(feature = "defmt")]
                defmt::info!("ethernet: connection speed set");
                self.connection_speed = Some(cs);
            }
        }
        self.request_state = HostNotificationState::Complete;
    }

    fn get_string(&self, index: StringIndex, _lang_id: LangID) -> Option<&str> {
        if index == self.mac_address_idx {
            Some(&self.mac_address)
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("ethernet: unknown string index requested {}", index);
            None
        }
    }

    fn reset(&mut self) {
        #[cfg(feature = "defmt")]
        defmt::info!("ethernet: reset");
        self.in_buf.reset();
        self.out_buf.reset();
        self.state = DeviceState::Disabled;
        self.request_state = HostNotificationState::Complete;
        self.connection_speed = None;
    }
}

impl<'a, B: UsbBus> Device for Ethernet<'a, B> {
    type RxToken<'b> = EthernetRxToken<'a, 'b, B> where
    Self: 'b;
    type TxToken<'b> = EthernetTxToken<'a, 'b, B> where
    Self: 'b;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.state == DeviceState::Connected
            && self.in_buf.can_write()
            && self.out_buf.can_read()
        {
            Some((
                EthernetRxToken::new(&mut self.out_buf),
                EthernetTxToken::new(&mut self.in_buf),
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        if self.state == DeviceState::Connected && self.in_buf.can_write() {
            Some(EthernetTxToken::new(&mut self.in_buf))
        } else {
            None
        }
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = MAX_SEGMENT_SIZE.into();
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

pub struct EthernetRxToken<'a, 'b, B: UsbBus> {
    ethernet: &'b mut OutBuf<'a, B>,
}
impl<'a, 'b, B: UsbBus> EthernetRxToken<'a, 'b, B> {
    fn new(ethernet: &'b mut OutBuf<'a, B>) -> Self {
        Self { ethernet }
    }
}

impl<'a, 'b, B: UsbBus> phy::RxToken for EthernetRxToken<'a, 'b, B> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.ethernet.read_datagram(f).unwrap()
    }
}

pub struct EthernetTxToken<'a, 'b, B: UsbBus> {
    ethernet: &'b mut InBuf<'a, B>,
}
impl<'a, 'b, B: UsbBus> EthernetTxToken<'a, 'b, B> {
    fn new(ethernet: &'b mut InBuf<'a, B>) -> Self {
        Self { ethernet }
    }
}

impl<'a, 'b, B: UsbBus> phy::TxToken for EthernetTxToken<'a, 'b, B> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.ethernet
            .write_datagram(len.try_into().unwrap(), f)
            .unwrap()
    }
}
