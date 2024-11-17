use core::fmt::Write;
use core::str::from_utf8;
use defmt::debug;
use embedded_hal::digital::OutputPin;
use heapless::Vec;
use smoltcp::{iface::SocketHandle, socket::tcp::Socket};

enum HttpRequest<'a> {
    Get(&'a str),
    BadRequest,
    MethodNotAllowed,
    Post(&'a str),
}

enum HttpResponse {
    Ok,
    BadRequest,
    NotFound,
    MethodNotAllowed,
}

pub(crate) struct WebServer<'a, P: OutputPin> {
    socket_handle: SocketHandle,
    pin: P,
    buffer: &'a mut Vec<u8, 512>,
}

impl<'a, P: OutputPin> WebServer<'a, P> {
    pub fn new(socket_handle: SocketHandle, pin: P, buffer: &'a mut Vec<u8, 512>) -> Self {
        Self {
            socket_handle,
            pin,
            buffer,
        }
    }

    pub fn poll<'b, 'c: 'b>(&mut self, f: impl FnOnce(SocketHandle) -> &'b mut Socket<'c>) {
        let socket = f(self.socket_handle);
        if !socket.is_open() {
            socket.listen(80).unwrap();
            debug!("http: listening on 80");
        }

        if socket.can_recv() && socket.can_send() {
            let request = socket
                .recv(|buffer| (buffer.len(), request(buffer)))
                .unwrap();

            if let Some(request) = request {
                self.buffer.clear();
                let r = response(&request, self.buffer, &mut self.pin);

                match r {
                    HttpResponse::Ok => writeln!(socket, "HTTP/1.1 200 OK"),
                    HttpResponse::BadRequest => writeln!(socket, "HTTP/1.1 400 Bad Request"),
                    HttpResponse::NotFound => writeln!(socket, "HTTP/1.1 404 Not Found"),
                    HttpResponse::MethodNotAllowed => {
                        writeln!(socket, "HTTP/1.1 405 Method Not Allowed")
                    }
                }
                .unwrap();

                write!(
                    socket,
                    "Content-Type: text/html\nContent-Length: {}\n\n",
                    self.buffer.len()
                )
                .unwrap();
                socket.send_slice(self.buffer).unwrap();
                socket.close();
            }
        } else if socket.may_send() && !socket.may_recv() {
            debug!("http: socket close");
            socket.close();
        }
    }
}

fn response(
    request: &HttpRequest,
    content: &mut impl Write,
    led: &mut impl OutputPin,
) -> HttpResponse {
    match request {
        HttpRequest::Get(path) => get(path, content),
        HttpRequest::Post(path) => post(path, content, led),
        HttpRequest::BadRequest => HttpResponse::BadRequest,
        HttpRequest::MethodNotAllowed => HttpResponse::MethodNotAllowed,
    }
}

fn get(path: &str, content: &mut impl Write) -> HttpResponse {
    match path {
        "/" => index(content),
        _ => HttpResponse::NotFound,
    }
}

fn post(path: &str, content: &mut impl Write, led: &mut impl OutputPin) -> HttpResponse {
    match path {
        "/on" => on(content, led),
        "/off" => off(content, led),
        _ => HttpResponse::NotFound,
    }
}

fn index(content: &mut impl Write) -> HttpResponse {
    write!(
        content,
        "<!DOCTYPE html>
<html><head><script>
async function post(path) {{
    const response = await fetch(path, {{method: \"POST\", cache: \"no-cache\"}});
    return response;
}}
</script>
<meta name=\"viewport\" content=\"width=device-width, height=device-height, initial-scale=1\"></head>
<body><h1>Hello pico!</h1>
<p><button onclick=\"post('/on')\">LED on</button><button onclick=\"post('/off')\">LED off</button></p></body></html>"
    )
    .unwrap();
    HttpResponse::Ok
}

fn on(content: &mut impl Write, led: &mut impl OutputPin) -> HttpResponse {
    led.set_high().ok().unwrap();
    write!(
        content,
        "<!DOCTYPE html>
<html><body>on</body></html>"
    )
    .unwrap();
    HttpResponse::Ok
}

fn off(content: &mut impl Write, led: &mut impl OutputPin) -> HttpResponse {
    led.set_low().ok().unwrap();
    write!(
        content,
        "<!DOCTYPE html>
<html><body>off</body></html>"
    )
    .unwrap();
    HttpResponse::Ok
}

fn request(buffer: &mut [u8]) -> Option<HttpRequest> {
    if buffer.is_empty() {
        None
    } else if let Ok(s) = from_utf8(buffer) {
        debug!("http: recv data: {:?}", s);
        let mut lines = s.lines();
        let header = lines.next().unwrap();
        let mut words = header.split(' ');

        let request = match words.next().unwrap() {
            "GET" => Some(HttpRequest::Get(words.next().unwrap())),
            "POST" => Some(HttpRequest::Post(words.next().unwrap())),
            _ => Some(HttpRequest::MethodNotAllowed),
        };

        if !words.next().unwrap().starts_with("HTTP/1") {
            return Some(HttpRequest::BadRequest);
        }

        return request;
    } else {
        debug!("http: recv data: {:?}", buffer);
        Some(HttpRequest::BadRequest)
    }
}
