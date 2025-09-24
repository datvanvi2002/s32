use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::{gpio::*, peripherals::Peripherals};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::http::client::{Configuration as HttpConfig, EspHttpConnection};
use esp_idf_svc::nvs::{EspNvsPartition, NvsDefault};
use esp_idf_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi};
use serde_json::json;

use embedded_svc::http::client::Client;
use embedded_svc::http::Method;
use embedded_svc::io::{Read, Write};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use esp_idf_sys as _;
use mfrc522::comm::blocking::spi::SpiInterface;
use mfrc522::Mfrc522;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306,
};
use std::{thread, time::Duration};

// ================== Firebase Realtime Database ==================
const FIREBASE_DB_BASE: &str =
    "https://esp32-payment-45c6a-default-rtdb.asia-southeast1.firebasedatabase.app/";
// Nếu DB cần auth: đặt ID token hoặc database secret vào đây. Nếu public test: để None.
static FIREBASE_AUTH_TOKEN: Option<&str> = None;

fn bytes_to_hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        use std::fmt::Write as _;
        let _ = write!(s, "{:02X}", b);
    }
    s
}

fn print_hex_bytes(data: &[u8]) {
    for b in data {
        print!("{:02x} ", b);
    }
    println!();
}

const SSID: &str = "SSID";
const PASSWORD: &str = "PASS";

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    unsafe {
        let _ = esp_idf_sys::esp_task_wdt_deinit();
    }

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap(); //lấy quyền truy cập các ngoại vi

    //let pins = peripherals.pins; //,pin là tất cả các chân GPIO của chip
    let keys = [
        ['1', '2', '3', 'A'],
        ['4', '5', '6', 'B'],
        ['7', '8', '9', 'C'],
        ['*', '0', '#', 'D'],
    ];
    let mut key_pressed: Option<char> = None;

    let mut led = PinDriver::output(peripherals.pins.gpio2).unwrap(); // khởi tạo chân GPIO2 làm đầu ra
    
    let mut row_0 = PinDriver::output(peripherals.pins.gpio13).unwrap();
    let mut row_1 = PinDriver::output(peripherals.pins.gpio14).unwrap();
    let mut row_2 = PinDriver::output(peripherals.pins.gpio21).unwrap();
    let mut row_3 = PinDriver::output(peripherals.pins.gpio47).unwrap();

    let mut col_0 = PinDriver::input(peripherals.pins.gpio9.downgrade()).unwrap();
    col_0.set_pull(Pull::Up).unwrap();
    let mut col_1 = PinDriver::input(peripherals.pins.gpio10.downgrade()).unwrap();
    col_1.set_pull(Pull::Up).unwrap();
    let mut col_2 = PinDriver::input(peripherals.pins.gpio11.downgrade()).unwrap();
    col_2.set_pull(Pull::Up).unwrap();
    let mut col_3 = PinDriver::input(peripherals.pins.gpio12.downgrade()).unwrap();
    col_3.set_pull(Pull::Up).unwrap();

    ////////////////INIT RFID MODULE/////////////////////
    // Define GPIO pins for SPI
    let sclk = peripherals.pins.gpio36; // SCLK pin
    let mosi = peripherals.pins.gpio35; // MOSI pin
    let mut cs = peripherals.pins.gpio48; //sda pin
                                          // MISO is optional; set to None if not used
    let miso = Some(peripherals.pins.gpio37); // MISO pin

    // Add a reset pin (e.g., GPIO4; choose a free pin not conflicting with others)
    let mut rst_pin = PinDriver::output(peripherals.pins.gpio39).unwrap();
    // Perform hard reset RFID module
    rst_pin.set_low().unwrap();
    thread::sleep(Duration::from_millis(200));
    rst_pin.set_high().unwrap();
    thread::sleep(Duration::from_millis(200)); // Brief delay for stabilization
                                               //Configure SPI driver
    let spi_driver = SpiDriver::new(
        peripherals.spi3, // Use SPI3
        sclk,
        mosi,
        miso,
        &SpiDriverConfig::new(),
    )
    .unwrap();

    let config = SpiConfig::new()
        .baudrate(1_000_000u32.into())
        .data_mode(embedded_hal::spi::MODE_0);

    let mut spi = SpiDeviceDriver::new(spi_driver, Some(cs), &config)?;

    let delay = Delay::new(40000000);

    thread::sleep(Duration::from_millis(1000));
    let spi_interface = SpiInterface::new(spi);
    let mut rfid = Mfrc522::new(spi_interface).init().unwrap();

    thread::sleep(Duration::from_millis(1000));

    let sector_num = 0; // Sector to read (0-15 for MIFARE Classic 1K)

    /*Read version MFRC522*/
    log::info!("Scanning for RFID card...");
    let version = rfid
        .version()
        .map_err(|e| anyhow::anyhow!("Failed to read version: {:?}", e))?;
    log::info!("MFRC522 Version: 0x{:02x}", version);

    thread::sleep(Duration::from_millis(1000));

    if rfid.reqa().is_err() {
        log::warn!("RFID module may not be present or not responding!");
    } else {
        log::info!("RFID module initialized successfully!");
    }

    thread::sleep(Duration::from_millis(1000));

    /////////////////////////////////////////////////////////////////////////////////////
    ///////////// INIT OLED MODULE////////////////////////////////////////////////////

    log::info!("INIT: OLED I2C !");
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio18;

    let i2c_cfg = I2cConfig::new();
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_cfg)?;

    // Tạo interface & driver SSD1306/SSD1309
    let interface = I2CDisplayInterface::new(i2c);
    let mut disp: Ssd1306<_, DisplaySize128x64, BufferedGraphicsMode<_>> =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
    disp.init().unwrap();
    disp.clear(BinaryColor::Off).unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::new("ESP32-Payment !", Point::new(0, 16), style)
        .draw(&mut disp)
        .unwrap();

    disp.flush().unwrap();

    /////////////////////////////////
    log::info!("Init WIFI");
    let modem = peripherals.modem;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs_partition = EspNvsPartition::<NvsDefault>::take()?;
    let mut wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs_partition))?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    }))?;

    // Start + connect
    wifi.start()?;
    wifi.connect()?;
    log::info!("Đang kết nối tới Wi-Fi `{}` ...", SSID);

    loop {
        //LED RUN STATUS
        log::info!("Blinking LED 2...");
        led.set_high().unwrap(); // bật đèn LED
        thread::sleep(Duration::from_millis(1000)); // đợi 1 gi ây
        led.set_low().unwrap(); // tắt đèn LED
        thread::sleep(Duration::from_millis(1000));

        /***********************************************************************************************************/
        /********************************************** ĐỌC BÀN PHÍM MA TRẬN ***************************************/
        /***********************************************************************************************************/

        // Đọc bàn phím ma trận
        log::info!("Reading keypad...");
        key_pressed = None;
        for i in 0..4 {
            // Đặt tất cả hàng HIGH
            row_0.set_high().unwrap();
            row_1.set_high().unwrap();
            row_2.set_high().unwrap();
            row_3.set_high().unwrap();

            // Đặt hàng hiện tại LOW
            match i {
                0 => row_0.set_low().unwrap(),
                1 => row_1.set_low().unwrap(),
                2 => row_2.set_low().unwrap(),
                3 => row_3.set_low().unwrap(),
                _ => {}
            }
            // Đọc trạng thái các cột
            if col_0.is_low() {
                //log::info!("Phím nhấn ở hàng {}, cột 0", i);
                key_pressed = Some(keys[i][0]);
                log::info!("Key pressed: {}", keys[i][0]);
                break;
            } else if col_1.is_low() {
                //log::info!("Phím nhấn ở hàng {}, cột 1", i);
                key_pressed = Some(keys[i][1]);
                log::info!("Key pressed: {}", keys[i][1]);
                break;
            } else if col_2.is_low() {
                //log::info!("Phím nhấn ở hàng {}, cột 2", i);
                key_pressed = Some(keys[i][2]);
                log::info!("Key pressed: {}", keys[i][2]);
                break;
            } else if col_3.is_low() {
                //log::info!("Phím nhấn ở hàng {}, cột 3", i);
                key_pressed = Some(keys[i][3]);
                log::info!("Key pressed: {}", keys[i][3]);
                break;
            } else {
                // key_pressed = None; // Không có phím nào được nhấn
                log::info!("No key pressed in row {}", i);
            }
        }

        /***********************************************************************************************************/
        /********************************************** XỬ LÝ BÀN PHÍM MA TRẬN *************************************/
        /***********************************************************************************************************/
        if key_pressed == Some('A') {
            led.toggle().unwrap(); // Bật LED nếu phím 'A' được nhấn
            log::info!("Test handle matrix keyboard");
        }

        /***********************************************************************************************************/
        /********************************************** xử lý rfid *************************************************/
        /***********************************************************************************************************/
        // nếu đang đọc thẻ mà rút thẻ ra thì sẽ bị reset
        let mut uid_line_text = String::from("UID: -");
        if let Ok(atqa) = rfid.reqa() {
            log::info!("Card UID:  ");
            thread::sleep(Duration::from_millis(500));

            if let Ok(uid) = rfid.select(&atqa) {
                print_hex_bytes(uid.as_bytes());
                log::info!("{:02X?}", uid.as_bytes());
                thread::sleep(Duration::from_millis(500));

                log::info!("Reading sector: {:02X?}", sector_num);
                read_sector(&uid, sector_num, &mut rfid);

                rfid.hlta().unwrap();
                rfid.stop_crypto1().unwrap();

                let uid_bytes = uid.as_bytes();
                let uid_str = format!("{:02X?}", uid_bytes);

                uid_line_text = format!("UID: {}", uid_str);
            }
        } else {
            log::info!("No card detected.");
        }

        /***********************************************************************************************************/
        /********************************************** hiển thị OLED **********************************************/
        /***********************************************************************************************************/

        let key_text = match key_pressed {
            Some(c) => format!("Key: {}", c),
            None => "Key: -".to_string(),
        };
        //let ssid_text = format!("Wi-Fi: {}", SSID);

        let ssid_text = match wifi.get_configuration() {
            Ok(Configuration::Client(cfg)) => {
                let ok = wifi.sta_netif().get_ip_info().is_ok(); // có IP => ok
                format!(
                    "Wi-Fi: {} [{}]",
                    cfg.ssid.as_str(),
                    if ok { "OK" } else { "N/A" }
                )
            }
            _ => "Wi-Fi: - [N/A]".to_string(),
        };

        let y1 = 24; // dòng 1: SSID
        let y2 = 38; // dòng 2: phím
        let y3 = 52; // dòng 3: UID

        {
            use embedded_graphics::prelude::*;
            use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};

            // mỗi dòng cao 12px
            let h = 12u32;

            // Clear line 1
            Rectangle::new(Point::new(0, y1 - 10), Size::new(128, h))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut disp)
                .unwrap();

            // Clear line 2
            Rectangle::new(Point::new(0, y2 - 10), Size::new(128, h))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut disp)
                .unwrap();

            // Clear line 3
            Rectangle::new(Point::new(0, y3 - 10), Size::new(128, h))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut disp)
                .unwrap();
        }

        Text::new(&ssid_text, Point::new(0, y1), style)
            .draw(&mut disp)
            .unwrap();
        Text::new(&key_text, Point::new(0, y2), style)
            .draw(&mut disp)
            .unwrap();
        Text::new(&uid_line_text, Point::new(0, y3), style)
            .draw(&mut disp)
            .unwrap();

        disp.flush().unwrap();

        /***********************************************************************************************************/
        /********************************************** Gửi thông tin lên server ***********************************/
        /***********************************************************************************************************/

        static mut TICK: u32 = 0;
        unsafe {
            TICK = TICK.wrapping_add(1);
        }
        if unsafe { TICK % 10 } == 0 {
            // ~ mỗi 10 vòng (mỗi vòng ~1s theo blink)
            match firebase_get_i64("gpio36") {
                Ok(v) => log::info!("FB gpio36 = {}", v),
                Err(e) => log::error!("FB get gpio36 error: {}", e),
            }
            match firebase_get_i64("gpio5") {
                Ok(v) => {
                    log::info!("FB gpio5 = {}", v);
                    let _ = firebase_put_i64("gpio5", v + 1);
                }
                Err(e) => log::error!("FB get gpio5 error: {}", e),
            }
        }

        log::info!("From main.rs ESP32!");
        thread::sleep(Duration::from_millis(200));
    }
}

//read sector function
fn read_sector<E, COMM: mfrc522::comm::Interface<Error = E>>(
    uid: &mfrc522::Uid,
    sector: u8,
    rfid: &mut Mfrc522<COMM, mfrc522::Initialized>,
) {
    const AUTH_KEY: [u8; 6] = [0xFF; 6];

    let block_offset = sector * 4;
    rfid.mf_authenticate(uid, block_offset, &AUTH_KEY)
        .map_err(|_| "Auth failed")
        .unwrap();

    for abs_block in block_offset..block_offset + 4 {
        let data = rfid.mf_read(abs_block).map_err(|_| "Read failed").unwrap();
        log::info!("{:02X?}", data);
    }
}





// ================== Firebase helpers ==================
fn make_client() -> anyhow::Result<Client<EspHttpConnection>> {
    let cfg = HttpConfig {
        crt_bundle_attach: Some(esp_idf_sys::esp_crt_bundle_attach),
        ..Default::default()
    };
    let conn = EspHttpConnection::new(&cfg)?;
    Ok(Client::wrap(conn))
}
fn build_fb_url(path: &str) -> String {
    let p = path.trim().trim_start_matches('/');
    let mut url = format!("{}/{}.json", FIREBASE_DB_BASE.trim_end_matches('/'), p);
    if let Some(tok) = FIREBASE_AUTH_TOKEN {
        use core::fmt::Write as _;
        let _ = write!(url, "?auth={}", tok);
    }
    url
}

fn http_get(url: &str) -> anyhow::Result<String> {
    let mut client = make_client()?;

    let mut req = client.request(Method::Get, url, &[])?;
    let mut resp = req.submit()?;

    let status = resp.status();
    let mut buf: Vec<u8> = Vec::with_capacity(1024);
    let mut tmp = [0u8; 1024];

    loop {
        let n = resp.read(&mut tmp)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&tmp[..n]);
    }

    let body = String::from_utf8_lossy(&buf).to_string();
    if (200..300).contains(&status) {
        Ok(body)
    } else {
        anyhow::bail!("GET {} -> status {} body: {}", url, status, body)
    }
}

fn http_put_json(url: &str, json_body: &str) -> anyhow::Result<()> {
    let headers = &[("Content-Type", "application/json")];
    let mut client = make_client()?;
    let mut req = client.request(Method::Put, url, headers)?;
    let mut sent = 0;
    let data = json_body.as_bytes();
    while sent < data.len() {
        sent += req.write(&data[sent..])?;
    }
    req.flush()?;
    let mut resp = req.submit()?;
    let status = resp.status();

    if (200..300).contains(&status) {
        Ok(())
    } else {
        let mut buf: Vec<u8> = Vec::with_capacity(256);
        let mut tmp = [0u8; 256];
        loop {
            let n = resp.read(&mut tmp)?;
            if n == 0 {
                break;
            }
            buf.extend_from_slice(&tmp[..n]);
        }
        let body = String::from_utf8_lossy(&buf).to_string();
        anyhow::bail!("PUT {} -> status {} body: {}", url, status, body)
    }
}

fn http_patch_json(url: &str, json_body: &str) -> anyhow::Result<String> {
    let headers = &[("Content-Type", "application/json")];
    let mut client = make_client()?;
    let mut req = client.request(Method::Patch, url, headers)?;
    let data = json_body.as_bytes();
    let mut sent = 0;
    while sent < data.len() {
        sent += req.write(&data[sent..])?;
    }
    req.flush()?;
    let mut resp = req.submit()?;
    let status = resp.status();
    let mut buf: Vec<u8> = Vec::with_capacity(512);
    let mut tmp = [0u8; 256];
    loop {
        let n = resp.read(&mut tmp)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&tmp[..n]);
    }
    let body = String::from_utf8_lossy(&buf).to_string();
    if (200..300).contains(&status) {
        Ok(body)
    } else {
        anyhow::bail!("PATCH {} -> status {} body: {}", url, status, body)
    }
}

fn firebase_get_i64(path: &str) -> anyhow::Result<i64> {
    let url = build_fb_url(path);
    let body = http_get(&url)?;
    // giá trị có thể là "3" (string) hoặc 3 (number)
    let v: serde_json::Value = serde_json::from_str(&body)?;
    let n = match v {
        serde_json::Value::Number(num) => num.as_i64().unwrap_or(0),
        serde_json::Value::String(s) => s.parse::<i64>().unwrap_or(0),
        _ => 0,
    };
    Ok(n)
}

fn firebase_put_i64(path: &str, value: i64) -> anyhow::Result<()> {
    let url = build_fb_url(path);
    let body = serde_json::to_string(&value.to_string())?; // lưu dạng chuỗi cho compatible
    http_put_json(&url, &body)
}

// Put a string value to a node, e.g., "rfid/uid"
fn firebase_put_str(path: &str, s: &str) -> anyhow::Result<()> {
    let url = build_fb_url(path);
    let body = serde_json::to_string(&s)?;
    http_put_json(&url, &body)
}

fn http_post_json(url: &str, json_body: &str) -> anyhow::Result<String> {
    let headers = &[("Content-Type", "application/json")];

    let mut client = make_client()?;
    let mut req = client.request(Method::Post, url, headers)?;

    // Write all bytes
    let data = json_body.as_bytes();
    let mut sent = 0;
    while sent < data.len() {
        sent += req.write(&data[sent..])?;
    }

    req.flush()?;
    let mut resp = req.submit()?;
    let status = resp.status();

    let mut buf: Vec<u8> = Vec::with_capacity(512);
    let mut tmp = [0u8; 256];
    loop {
        let n = resp.read(&mut tmp)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&tmp[..n]);
    }
    let body = String::from_utf8_lossy(&buf).to_string();

    if (200..300).contains(&status) {
        Ok(body)
    } else {
        anyhow::bail!("POST {} -> status {} body: {}", url, status, body)
    }
}

fn firebase_post_json<T: serde::Serialize>(path: &str, value: &T) -> anyhow::Result<String> {
    let url = build_fb_url(path);
    let body = serde_json::to_string(value)?;
    http_post_json(&url, &body)
}

// Push RFID event and update current UID
fn push_rfid(uid_hex: &str) {
    // 1) Update current
    if let Err(e) = firebase_put_str("rfid/uid", uid_hex) {
        log::error!("PUT /rfid/uid error: {}", e);
    } else {
        log::info!("PUT /rfid/uid = {}", uid_hex);
    }

    // 2) Post event
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs() as i64;
    let evt = serde_json::json!({
        "uid": uid_hex,
        "ts": ts,
        "note": "scan"
    });
    match firebase_post_json("events/rfid", &evt) {
        Ok(ret) => log::info!("RFID event pushed: {}", ret),
        Err(e) => log::error!("push RFID event error: {}", e),
    }
}

fn firebase_patch_json<T: serde::Serialize>(path: &str, value: &T) -> anyhow::Result<String> {
    let url = build_fb_url(path);
    let body = serde_json::to_string(value)?;
    http_patch_json(&url, &body)
}

fn set_rfid_uid(uid_hex: &str) -> anyhow::Result<()> {
    let url = build_fb_url("rfid");
    let body = serde_json::to_string(&json!({ "uid": uid_hex }))?;
    http_put_json(&url, &body)
}

fn push_rfid_event(uid_hex: &str, note: &str) {
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs() as i64;

    let evt = json!({ "uid": uid_hex, "ts": ts, "note": note });

    match firebase_post_json("events/rfid", &evt) {
        Ok(ret) => log::info!("RFID event pushed: {}", ret),
        Err(e) => log::error!("push RFID event error: {}", e),
    }
}

// Chỉ đẩy khi UID khác lần trước & không rỗng
static mut LAST_UID_HEX: heapless::String<32> = heapless::String::new();
static mut LAST_UID_TS_US: i64 = 0;
fn now_us() -> i64 {
    unsafe { esp_idf_sys::esp_timer_get_time() }
}

fn push_rfid_if_new(uid_hex: &str) {
    if uid_hex.is_empty() {
        return;
    }
    let t = now_us();
    let mut should = false;
    unsafe {
        if uid_hex != LAST_UID_HEX || t - LAST_UID_TS_US > 500_000 {
            LAST_UID_HEX.clear();
            let _ = LAST_UID_HEX.push_str(uid_hex);
            LAST_UID_TS_US = t;
            should = true;
        }
    }
    if !should {
        return;
    }

    // 1) Update current UID
    if let Err(e) = set_rfid_uid(uid_hex) {
        log::error!("SET /rfid uid error: {}", e);
    } else {
        log::info!("SET /rfid uid = {}", uid_hex);
    }

    // 2) Append event
    let ts = (std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs()) as i64;
    let evt = json!({
        "uid": uid_hex,
        "ts": ts,
        "note": "scan"
    });
    if let Err(e) = firebase_post_json("events/rfid", &evt) {
        log::error!("POST events/rfid error: {}", e);
    }
}



