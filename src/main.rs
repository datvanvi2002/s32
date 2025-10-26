use core::fmt::Write as _;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Baseline,
    text::Text,
};

use embedded_svc::http::client::Client;
use embedded_svc::http::Method;
use embedded_svc::io::{Read, Write};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::{gpio::*, peripherals::Peripherals};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::http::client::{Configuration as HttpConfig, EspHttpConnection};
use esp_idf_svc::nvs::{EspNvsPartition, NvsDefault};
use esp_idf_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi};
use esp_idf_sys as _;
use mfrc522::comm::blocking::spi::SpiInterface;
use mfrc522::Mfrc522;
use serde_json::json;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306,
};
use std::time::Instant;
use std::{thread, time::Duration};

const SSID: &str = "July-VN";
const PASSWORD: &str = "10101010";

type MenuEntry = (String, i64);
struct UiMenu {
    cursor: usize,
    qty: Vec<u8>,
    win_top: usize,
    total: i64,
    last_key_ms: u64,
}

impl UiMenu {
    fn new(len: usize) -> Self {
        Self {
            cursor: 0,
            qty: vec![0; len],
            win_top: 0,
            total: 0,
            last_key_ms: 0,
        }
    }
}

#[derive(Copy, Clone)]
enum Key {
    A,
    B,
    C,
    D,
    Star,
    Hash,
    Digit(u8),
    None,
}

fn now_ms() -> u64 {
    use core::time::Duration;
    use std::time::SystemTime;
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or(Duration::from_millis(0))
        .as_millis() as u64
}

fn format_vnd(n: i64) -> String {
    let mut s = n.abs().to_string();
    let mut out = String::new();
    while s.len() > 3 {
        let t = s.split_off(s.len() - 3);
        if out.is_empty() {
            out = t;
        } else {
            out = format!("{},{}d", t, out);
        }
    }
    if out.is_empty() {
        out = s;
    } else {
        out = format!("{}{}d", s, out);
    }
    if n < 0 {
        out.insert(0, '-');
    }
    out
}
#[derive(Copy, Clone, PartialEq, Eq)]
enum Screen {
    Menu,
    Checkout,
    Status,
}

// ================== Firebase Realtime Database ==================
const FIREBASE_DB_BASE: &str =
    "https://esp32-rfid-rust-default-rtdb.asia-southeast1.firebasedatabase.app/";

const FIREBASE_DB_NS: &str = "esp32-rfid-rust-default-rtdb"; // <— instance/namespace RTDB
static FIREBASE_AUTH_TOKEN: Option<&str> = None;

//for emulator
//const FIREBASE_DB_BASE: &str = "http://192.168.0.100:9000"; // <— IP LAN PC + cổng DB emulator
// Cloud Functions base (emulator). Khi lên cloud, đổi dòng dưới:
//const CF_BASE: &str = "http://192.168.0.100:5001/esp32-payment-45c6a/asia-southeast1"; // <— IP LAN PC + cổng Functions

// LẤY từ `firebase functions:info`
const CF_STARTTX_URL: &str = "https://starttx-sr7jgo773a-as.a.run.app";
const CF_PROVIDE_URL: &str = "https://providecredentials-sr7jgo773a-as.a.run.app";
const CF_REQUEST_PHONE_URL: &str = "https://requestfromphone-sr7jgo773a-as.a.run.app";

// Mã thiết bị
const DEVICE_ID: &str = "dev-esp32-s3";

// ===== Logging helper: truncate long strings for serial logs
fn trunc(s: &str, max: usize) -> String {
    let mut t = s.to_string();
    if t.len() > max {
        t.truncate(max);
        t.push('…');
    }
    t
}
fn print_hex_bytes(data: &[u8]) {
    for b in data {
        print!("{:02x} ", b);
    }
    println!();
}

fn bytes_to_hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        use std::fmt::Write as _;
        let _ = write!(s, "{:02X}", b);
    }
    s
}

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

    /*Check reqa MFRC522*/
    log::info!("Starting reqa...");
    if let Err(e) = rfid.reqa() {
        log::error!("reqa failed: {:?}", e);
    } else {
        log::info!("reqa succeeded!");
    }

    /*Read version MFRC522*/
    log::info!("Scanning for RFID card...");
    let version = rfid
        .version()
        .map_err(|e| anyhow::anyhow!("Failed to read version: {:?}", e))?;
    log::info!("MFRC522 Version: 0x{:02x}", version);

    thread::sleep(Duration::from_millis(1000));

    /////////////////////////////////////////////////////////////////////////////////////
    ///////////// INIT OLED MODULE////////////////////////////////////////////////////

    log::info!("INIT: OLED I2C !");
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio18;

    let i2c_cfg = I2cConfig::new();
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_cfg)?;

    let interface = I2CDisplayInterface::new(i2c);
    let mut disp: Ssd1306<_, DisplaySize128x64, BufferedGraphicsMode<_>> =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
    disp.init().unwrap();
    disp.clear(BinaryColor::Off).unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
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

    thread::sleep(Duration::from_millis(5000));

    let mut co = CheckoutState::new(0);
    let mut uid_bytes: [u8; 10] = [0; 10];
    let mut screen = Screen::Menu;
    let mut co = CheckoutState::new(0);
    let mut tx_status = TxStatus::new(false, 0, 0, None);

    // ===== MENU STATE =====
    let mut menu_items: Vec<(String, i64)> = Vec::new();
    let mut menu_ids: Vec<String> = Vec::new();
    let mut ui = UiMenu::new(menu_items.len());
    ui_menu_sync_len(&mut ui, menu_items.len());
    let mut last_menu_fetch = std::time::Instant::now()
        .checked_sub(Duration::from_secs(3600))
        .unwrap_or(std::time::Instant::now());
    let mut menu_scroll_idx: usize = 0;
    let mut last_cmd_poll = std::time::Instant::now();
    loop {
        //LED RUN STATUS
        led.set_high().unwrap();
        thread::sleep(Duration::from_millis(10));
        led.set_low().unwrap();
        thread::sleep(Duration::from_millis(10));

        //read keypad
        key_pressed = None;
        for i in 0..4 {
            row_0.set_high().unwrap();
            row_1.set_high().unwrap();
            row_2.set_high().unwrap();
            row_3.set_high().unwrap();
            match i {
                0 => row_0.set_low().unwrap(),
                1 => row_1.set_low().unwrap(),
                2 => row_2.set_low().unwrap(),
                3 => row_3.set_low().unwrap(),
                _ => {}
            }
            if col_0.is_low() {
                key_pressed = Some(keys[i][0]);
                log::info!("Key pressed: {}", keys[i][0]);
                loop {
                    thread::sleep(Duration::from_millis(10));
                    if col_0.is_high() {
                        break;
                    }
                }
                break;
            } else if col_1.is_low() {
                key_pressed = Some(keys[i][1]);
                log::info!("Key pressed: {}", keys[i][1]);
                loop {
                    thread::sleep(Duration::from_millis(10));
                    if col_1.is_high() {
                        break;
                    }
                }
                break;
            } else if col_2.is_low() {
                key_pressed = Some(keys[i][2]);
                log::info!("Key pressed: {}", keys[i][2]);
                loop {
                    thread::sleep(Duration::from_millis(10));
                    if col_2.is_high() {
                        break;
                    }
                }
                break;
            } else if col_3.is_low() {
                key_pressed = Some(keys[i][3]);
                log::info!("Key pressed: {}", keys[i][3]);
                loop {
                    thread::sleep(Duration::from_millis(10));
                    if col_3.is_high() {
                        break;
                    }
                }
                break;
            } else {
                key_pressed = None;
            }
        }

        let key = match key_pressed {
            Some('A') => Key::A,
            Some('B') => Key::B,
            Some('*') => Key::Star,
            Some('#') => Key::Hash,
            Some('C') => Key::C,
            Some('D') => Key::D,
            Some(c) if c.is_ascii_digit() => Key::Digit(c as u8 - b'0'),
            _ => Key::None,
        };
        let key_opt: Option<Key> = Some(key);
        // nếu đang đọc thẻ mà rút thẻ ra thì sẽ bị reset
        let mut uid_line_text = String::from("UID: -");
        if let Ok(atqa) = rfid.reqa() {
            log::info!("Card UID:  ");
            thread::sleep(Duration::from_millis(50));
            if let Ok(uid) = rfid.select(&atqa) {
                let uid_bytes = uid.as_bytes();
                print_hex_bytes(uid_bytes);

                let uid_dec = uid_bytes_to_decimal_be(uid_bytes);
                let uid_fixed_10 = format!("{:0>10}", uid_dec);
                co.card_uid = Some(uid_fixed_10.clone());
                uid_line_text = format!("UID: {}", uid_dec);
                log::info!("Card UID (dec BE): {}", uid_dec);

                thread::sleep(Duration::from_millis(500));

                // println!("Reading sector: {}", sector_num);
                // read_sector(&uid, sector_num, &mut rfid);
                rfid.hlta().unwrap();
                rfid.stop_crypto1().unwrap();
            }
        }

        /***********************************************************************************************************/
        /********************************************** hiển thị OLED **********************************************/
        /***********************************************************************************************************/

        let old_len = menu_items.len();
        if menu_items.len() != old_len {
            let mut new_qty = vec![0u8; menu_items.len()];
            for i in 0..ui.qty.len().min(new_qty.len()) {
                new_qty[i] = ui.qty[i];
            }
            ui.qty = new_qty;
            if ui.cursor >= menu_items.len() {
                ui.cursor = menu_items.len().saturating_sub(1);
                ui.win_top = ui.cursor.saturating_sub(2);
            }
        }

        if last_menu_fetch.elapsed() > Duration::from_secs(60) || menu_items.is_empty() {
            match firebase_get_menu() {
                Ok((v, ids)) => {
                    menu_items = v;
                    menu_ids = ids;
                    ui_menu_sync_len(&mut ui, menu_items.len());
                    last_menu_fetch = std::time::Instant::now();
                }
                Err(e) => log::warn!("Could not fetch menu: {}", e),
            }
        }
        disp.flush().ok();

        //check commands từ điện thoại
        if last_cmd_poll.elapsed() > Duration::from_millis(5000) {
            match firebase_get_command_session(DEVICE_ID) {
                Ok(Some(sid)) => {
                    match firebase_get_session_total(&sid) {
                        Ok(total) => {
                            co.total = total as u32;
                            co.session_id = Some(sid);
                            co.pin_len = 0;
                            co.card_uid = None;
                            screen = Screen::Checkout; // bỏ qua bước chọn món
                        }
                        Err(e) => log::error!("get_session_total error: {:?}", e),
                    }
                }
                Ok(None) => {}
                Err(e) => log::warn!("poll commands error: {:?}", e),
            }
            last_cmd_poll = std::time::Instant::now();
        }

        match screen {
            Screen::Menu => {
                ui_menu_handle_key(&mut ui, key, menu_items.len());
                ui_menu_recompute_total(&mut ui, &menu_items);
                ui_menu_render(&mut disp, &menu_items, &ui);
                ui_menu_sync_len(&mut ui, menu_items.len());
                if let Some(Key::Hash) = key_opt {
                    match cf_start_tx(DEVICE_ID, &menu_ids, &ui.qty) {
                        Ok(resp) => {
                            co.total = resp.total as u32;
                            co.session_id = Some(resp.sessionId);
                            co.pin_len = 0;
                            co.card_uid = None;
                            screen = Screen::Checkout;
                        }
                        Err(e) => log::error!("startTx error: {:?}", e),
                    }
                }
            }
            Screen::Checkout => {
                draw_checkout_screen(&mut disp, key_opt, &mut co);
                if let Some(Key::Hash) = key_opt {
                    if let (Some(uid_str), Some(pin_str)) = (co_uid_str(&co), co_pin_str(&co)) {
                        if let Some(ref sid) = co.session_id {
                            match cf_provide_credentials(sid, &uid_str, &pin_str) {
                                Ok(resp) => {
                                    let success = resp.success;

                                    let balance = resp.balance.unwrap_or(0) as u32;
                                    let deducted = if success {
                                        resp.deducted.unwrap_or(co.total as i64) as u32
                                    } else {
                                        0
                                    };
                                    let reason = resp.reason.clone();
                                    tx_status = TxStatus::new(success, balance, deducted, reason);
                                    screen = Screen::Status;
                                }
                                Err(e) => {
                                    log::error!("provideCredentials error: {:?}", e);
                                    // Fallback: hiện màn hình thất bại để không bị đứng
                                    tx_status =
                                        TxStatus::new(false, 0, 0, Some("Error".to_string()));
                                    screen = Screen::Status;
                                }
                            }
                        } else {
                            log::warn!("No session_id; please startTx first");
                        }
                    }
                }
                if let Some(Key::Star) = key_opt {
                    co.pin_len = 0;
                    co.card_uid = None;
                    screen = Screen::Menu;
                }
            }
            Screen::Status => {
                let go_back = draw_status_screen(&mut disp, key_opt, &mut tx_status);
                if go_back.map_err(|e| anyhow::anyhow!("Display error: {:?}", e))? {
                    co.pin_len = 0;
                    co.card_uid = None;
                    screen = Screen::Menu;
                }
            }
        }
    }
}
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
        print_hex_bytes(&data);
    }
}

fn uid_bytes_to_decimal_be(uid: &[u8]) -> String {
    let mut val: u128 = 0;
    for &b in uid {
        val = (val << 8) | (b as u128);
    }
    val.to_string()
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
// fn build_fb_url(path: &str) -> String {
//     let p = path.trim().trim_start_matches('/');
//     let mut url = format!("{}/{}.json", FIREBASE_DB_BASE.trim_end_matches('/'), p);
//     if let Some(tok) = FIREBASE_AUTH_TOKEN {
//         use core::fmt::Write as _;
//         let _ = write!(url, "?auth={}", tok);
//     }
//     url
// }

fn build_fb_url(path: &str) -> String {
    let p = path.trim().trim_start_matches('/');
    let base = FIREBASE_DB_BASE.trim_end_matches('/');
    // Nếu là emulator (không chứa 'firebasedatabase.app') => cần ?ns=<instance>
    let is_emulator = !base.contains("firebasedatabase.app");
    let mut url = if is_emulator {
        format!("{}/{}.json?ns={}", base, p, FIREBASE_DB_NS)
    } else {
        format!("{}/{}.json", base, p)
    };
    if !is_emulator {
        if let Some(tok) = FIREBASE_AUTH_TOKEN {
            use core::fmt::Write as _;
            let _ = write!(url, "?auth={}", tok);
        }
    }
    url
}

fn http_get(url: &str) -> anyhow::Result<String> {
    //log::info!("HTTP GET {}", url);
    let mut client = make_client()?;

    let req = client.request(Method::Get, url, &[])?;
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
        //log::warn!("HTTP GET {} -> {} body: {}", url, status, body);
        Ok(body)
    } else {
        log::error!("HTTP GET {} -> {} body: {}", url, status, body);
        anyhow::bail!("GET {} -> status {} body: {}", url, status, body)
    }
}

fn http_put_json(url: &str, json_body: &str) -> anyhow::Result<()> {
    log::info!("HTTP PUT {}", url);
    log::debug!("payload: {}", trunc(json_body, 180));

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
        log::info!("HTTP PUT {} -> {} ({} bytes)", url, status, data.len());
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
        log::error!("HTTP PUT {} -> {} body: {}", url, status, body);
        anyhow::bail!("PUT {} -> status {} body: {}", url, status, body)
    }
}

fn http_patch_json(url: &str, json_body: &str) -> anyhow::Result<String> {
    log::info!("HTTP PATCH {}", url);
    log::debug!("payload: {}", trunc(json_body, 180));

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
        log::info!("HTTP PATCH {} -> {} ({} bytes)", url, status, buf.len());
        Ok(body)
    } else {
        log::error!(
            "HTTP PATCH {} -> {} body: {}",
            url,
            status,
            trunc(&body, 160)
        );
        anyhow::bail!("PATCH {} -> status {} body: {}", url, status, body)
    }
}

fn firebase_get_i64(path: &str) -> anyhow::Result<i64> {
    let url = build_fb_url(path);
    let body = http_get(&url)?;
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

fn firebase_put_str(path: &str, s: &str) -> anyhow::Result<()> {
    let url = build_fb_url(path);
    let body = serde_json::to_string(&s)?;
    http_put_json(&url, &body)
}

fn http_post_json(url: &str, json_body: &str) -> anyhow::Result<String> {
    let headers = &[("Content-Type", "application/json")];
    log::info!("HTTP POST {}", url);
    log::debug!("payload: {}", json_body);

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
        log::info!("HTTP POST {} -> {} ({} bytes)", url, status, buf.len());
        Ok(body)
    } else {
        log::error!(
            "HTTP POST {} -> {} body: {}",
            url,
            status,
            trunc(&body, 160)
        );
        anyhow::bail!("POST {} -> status {} body: {}", url, status, body)
    }
}
//emulator version
// fn build_cf_url(path: &str) -> String {
//     let p = path.trim().trim_start_matches('/');
//     let base = CF_BASE.trim_end_matches('/');
//     format!("{}/{}", base, p)
// }

#[derive(serde::Deserialize)]
struct CfStartTxResp {
    sessionId: String,
    #[serde(rename = "total_cents")]
    total: i64,
}

#[derive(serde::Deserialize, Debug)]
struct CfProvideResp {
    success: bool,
    balance: Option<i64>,
    deducted: Option<i64>,
    reason: Option<String>,
}

fn cf_start_tx(device_id: &str, ids: &[String], qty: &[u8]) -> anyhow::Result<CfStartTxResp> {
    let mut items = Vec::<serde_json::Value>::new();
    for (i, id) in ids.iter().enumerate() {
        let q = qty.get(i).copied().unwrap_or(0);
        if q > 0 {
            items.push(serde_json::json!({"id": id, "qty": q}));
        }
    }
    if items.is_empty() {
        anyhow::bail!("NO_ITEMS_SELECTED");
    }
    let payload = serde_json::json!({ "deviceId": device_id, "items": items });
    // DÙNG URL production
    let resp_txt = http_post_json(CF_STARTTX_URL, &payload.to_string())?;
    Ok(serde_json::from_str(&resp_txt)?)
}

fn cf_provide_credentials(session_id: &str, uid: &str, pin: &str) -> anyhow::Result<CfProvideResp> {
    let payload = serde_json::json!({ "sessionId": session_id, "uid": uid, "pin": pin });
    // DÙNG URL production
    match http_post_json(CF_PROVIDE_URL, &payload.to_string()) {
        Ok(resp_txt) => {
            // 2xx -> parse bình thường
            Ok(serde_json::from_str(&resp_txt)?)
        }
        Err(e) => {
            // Nếu server trả 4xx cùng JSON {success:false, reason,...} thì trích JSON từ thông báo lỗi
            let msg = format!("{}", e);
            // tìm JSON sau "body: "
            let reason_resp = if let Some(idx) = msg.find("body: ") {
                let body_txt = msg[idx + 6..].trim();
                // body có thể có dấu ngoặc JSON hợp lệ
                serde_json::from_str::<serde_json::Value>(body_txt).ok()
            } else {
                None
            };
            if let Some(v) = reason_resp {
                // map các field (nếu thiếu thì điền mặc định)
                let success = v.get("success").and_then(|x| x.as_bool()).unwrap_or(false);
                let balance = v.get("balance").and_then(|x| x.as_i64());
                let deducted = v.get("deducted").and_then(|x| x.as_i64());
                let reason = v
                    .get("reason")
                    .and_then(|x| x.as_str())
                    .map(|s| s.to_string());
                Ok(CfProvideResp {
                    success,
                    balance,
                    deducted,
                    reason,
                })
            } else {
                // Không parse được -> vẫn trả về kết quả thất bại để UI hiện "thông báo lỗi"
                Ok(CfProvideResp {
                    success: false,
                    balance: None,
                    deducted: None,
                    reason: Some(format!("NETWORK_ERROR: {}", msg)),
                })
            }
        }
    }
}

//emulator version
// fn cf_provide_credentials(session_id: &str, uid: &str, pin: &str) -> anyhow::Result<CfProvideResp> {
//     let payload = serde_json::json!({ "sessionId": session_id, "uid": uid, "pin": pin });
//     let url = build_cf_url("provideCredentials");
//     let headers = &[("Content-Type", "application/json")];

//     let mut client = make_client()?;
//     let mut req = client.request(Method::Post, &url, headers)?;
//     let data = payload.to_string().as_bytes().to_vec();
//     let mut sent = 0;
//     while sent < data.len() {
//         sent += req.write(&data[sent..])?;
//     }
//     req.flush()?;
//     let mut resp = req.submit()?;

//     let status = resp.status();
//     let mut buf: Vec<u8> = Vec::new();
//     let mut tmp = [0u8; 1024];
//     loop {
//         let n = resp.read(&mut tmp)?;
//         if n == 0 {
//             break;
//         }
//         buf.extend_from_slice(&tmp[..n]);
//     }
//     let body = String::from_utf8_lossy(&buf).to_string();

//     if let Ok(parsed) = serde_json::from_str::<CfProvideResp>(&body) {
//         if status == 200 {
//             log::info!("provideCredentials 200 OK: {:?}", parsed);
//         } else {
//             log::warn!("provideCredentials {} with JSON: {:?}", status, parsed);
//         }
//         return Ok(parsed);
//     }
//     anyhow::bail!("POST {} -> status {} body: {}", url, status, body)
//     // let txt = http_post_json(&url, &payload.to_string())?;
//     // Ok(serde_json::from_str(&txt)?)
// }

// ——— Phone-driven: poll /commands & đọc tổng từ /sessions
fn firebase_get_command_session(device_id: &str) -> anyhow::Result<Option<String>> {
    let url = build_fb_url(&format!("commands/{}", device_id));
    let body = http_get(&url)?;
    if body.trim() == "null" {
        return Ok(None);
    }
    let v: serde_json::Value = serde_json::from_str(&body)?;
    let ok = v
        .get("statusPhone")
        .and_then(|x| x.as_bool())
        .unwrap_or(false);
    let sid = v.get("sessionId").and_then(|x| x.as_str()).unwrap_or("");
    if ok && !sid.is_empty() {
        Ok(Some(sid.to_string()))
    } else {
        Ok(None)
    }
}
fn firebase_get_session_total(session_id: &str) -> anyhow::Result<i64> {
    let url = build_fb_url(&format!("sessions/{}", session_id));
    let v: serde_json::Value = serde_json::from_str(&http_get(&url)?)?;
    Ok(v.get("total_cents")
        .and_then(|x| x.as_i64())
        .or_else(|| v.get("total").and_then(|x| x.as_i64()))
        .unwrap_or(0))
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

/// Đọc danh sách món ăn từ /menu (Realtime Database)
/// Trả về Vec<(tên, giá)>
// fn firebase_get_menu() -> anyhow::Result<Vec<(String, i64)>> {
//     let url = build_fb_url("menu");
//     let body = http_get(&url)?; // dùng hàm có sẵn
//     let v: serde_json::Value = serde_json::from_str(&body)?;

//     let mut out: Vec<(String, i64)> = Vec::new();
//     if let serde_json::Value::Object(map) = v {
//         for (_key, item) in map {
//             if let serde_json::Value::Object(obj) = item {
//                 // name
//                 let name = obj
//                     .get("name")
//                     .and_then(|x| x.as_str())
//                     .unwrap_or("(no name)")
//                     .to_string();
//                 // price: cho phép number hoặc string
//                 let price = match obj.get("price") {
//                     Some(serde_json::Value::Number(n)) => n.as_i64().unwrap_or(0),
//                     Some(serde_json::Value::String(s)) => s.parse::<i64>().unwrap_or(0),
//                     _ => 0,
//                 };
//                 // available (nếu có) -> chỉ lấy những món đang bán
//                 let available = obj
//                     .get("available")
//                     .and_then(|x| x.as_bool())
//                     .unwrap_or(true);
//                 if available {
//                     out.push((name, price));
//                 }
//             }
//         }
//     }
//     // Sắp xếp theo tên cho dễ nhìn
//     out.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));
//     Ok(out)
// }
/// /menu -> ([(name, price)], [ids])
fn firebase_get_menu() -> anyhow::Result<(Vec<(String, i64)>, Vec<String>)> {
    let v: serde_json::Value = serde_json::from_str(&http_get(&build_fb_url("menu"))?)?;

    let mut tmp: Vec<(String, String, i64, bool)> = Vec::new(); // (id, name, price, enabled)
    if let serde_json::Value::Object(map) = v {
        for (id, item) in map {
            if let serde_json::Value::Object(obj) = item {
                let name = obj
                    .get("name")
                    .and_then(|x| x.as_str())
                    .unwrap_or("(no name)")
                    .to_string();
                let price = obj
                    .get("price_cents")
                    .and_then(|x| x.as_i64())
                    .or_else(|| obj.get("price").and_then(|x| x.as_i64()))
                    .unwrap_or_else(|| {
                        obj.get("price")
                            .and_then(|x| x.as_str())
                            .and_then(|s| s.parse().ok())
                            .unwrap_or(0)
                    });
                let enabled = obj
                    .get("enabled")
                    .and_then(|x| x.as_bool())
                    .or_else(|| obj.get("available").and_then(|x| x.as_bool()))
                    .unwrap_or(true);
                tmp.push((id, name, price, enabled));
            }
        }
    }
    tmp.sort_by(|a, b| a.0.cmp(&b.0)); // M1, M2, M3…

    let mut items = Vec::new();
    let mut ids = Vec::new();
    for (id, name, price, enabled) in tmp {
        if enabled {
            items.push((name, price));
            ids.push(id);
        }
    }
    Ok((items, ids))
}

fn ui_menu_handle_key(ui: &mut UiMenu, key: Key, items_len: usize) {
    const DEBOUNCE_MS: u64 = 100;
    let now = now_ms();
    if now - ui.last_key_ms < DEBOUNCE_MS {
        return;
    }
    ui.last_key_ms = now;
    const ROWS: usize = 3;

    let adjust_window = |ui: &mut UiMenu| {
        if ui.cursor < ui.win_top {
            ui.win_top = ui.cursor;
        }
        if ui.cursor >= ui.win_top + ROWS {
            ui.win_top = ui.cursor + 1 - ROWS;
        }
        let max_top = items_len.saturating_sub(ROWS);
        if ui.win_top > max_top {
            ui.win_top = max_top;
        }
    };

    match key {
        Key::A => {
            if items_len == 0 {
                return;
            }
            if ui.cursor > 0 {
                ui.cursor -= 1;
            }
            adjust_window(ui);
        }
        Key::B => {
            if items_len == 0 {
                return;
            }
            let last = items_len - 1;
            if ui.cursor < last {
                ui.cursor += 1;
            }
            adjust_window(ui);
        }
        Key::Star => {
            if items_len == 0 {
                return;
            }
            let q = &mut ui.qty[ui.cursor];
            *q = (*q + 1) % 10;
        }
        Key::Digit(_) => {
            if items_len == 0 {
                return;
            }
            let d = match key {
                Key::Digit(x) => x,
                _ => 0,
            };
            if d <= 9 {
                ui.qty[ui.cursor] = d;
            }
        }
        Key::None => {}
        Key::C | Key::D | Key::Hash => {}
    }
}
fn ui_menu_recompute_total(ui: &mut UiMenu, items: &[MenuEntry]) {
    let mut t = 0i64;
    for (i, (_, price)) in items.iter().enumerate() {
        t += *price * ui.qty[i] as i64;
    }
    ui.total = t;
}

fn ui_menu_render<D: DrawTarget<Color = BinaryColor>>(
    disp: &mut D,
    items: &[MenuEntry],
    ui: &UiMenu,
) {
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Rectangle::new(Point::new(0, 0), Size::new(128, 64))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(disp)
        .ok();

    Text::new("MENU:", Point::new(64, 15), style)
        .draw(disp)
        .ok();

    if items.is_empty() {
        Text::new("(Khong co mon)", Point::new(0, 50), style)
            .draw(disp)
            .ok();
    } else {
        for row in 0..3usize {
            let i = ui.win_top + row;
            if i >= items.len() {
                break;
            }
            let (name, price) = &items[i];
            let cursor = if i == ui.cursor { ">" } else { " " };

            let mut short = name.clone();
            if short.chars().count() > 8 {
                short = short.chars().take(8).collect::<String>() + "…";
            }
            let q = ui.qty[i];
            let qch = if q == 0 { '-' } else { (b'0' + q) as char };
            let line = format!(
                "{c}[{q}]{name:<8}{price}",
                c = cursor,
                q = qch,
                name = short,
                price = format_vnd(*price)
            );
            Text::new(&line, Point::new(0, 28 + (row as i32) * 12), style)
                .draw(disp)
                .ok();
        }
    }
}
fn ui_menu_sync_len(ui: &mut UiMenu, items_len: usize) {
    if ui.qty.len() != items_len {
        let mut new_qty = vec![0u8; items_len];
        let keep = core::cmp::min(ui.qty.len(), items_len);
        for i in 0..keep {
            new_qty[i] = ui.qty[i];
        }
        ui.qty = new_qty;

        if items_len == 0 {
            ui.cursor = 0;
            ui.win_top = 0;
        } else {
            if ui.cursor >= items_len {
                ui.cursor = items_len - 1;
            }
            if ui.win_top > ui.cursor {
                ui.win_top = ui.cursor;
            }
            if ui.cursor >= ui.win_top + 3 {
                ui.win_top = ui.cursor + 1 - 3;
            }
        }
    }
}

////////////2
#[inline]
fn co_has_uid(co: &CheckoutState) -> bool {
    co.card_uid.is_some() || co.uid_len == 10
}
#[derive(Clone, Debug)]
pub struct CheckoutState {
    pub total: u32,               // tổng tiền (VND)
    pub card_uid: Option<String>, // UID thẻ sau khi quẹt (tuỳ bạn đổi 4/7/10 bytes)
    pub pin: [u8; 6],             // buffer 6 số PIN
    pub pin_len: usize,           // số chữ số PIN đã nhập (0..=6)
    pub last_key_ms: u64,
    pub uid_input: [u8; 10],
    pub uid_len: u8,
    pub uid_manual: bool,
    pub session_id: Option<String>, // session từ server
}

impl CheckoutState {
    pub fn new(total: u32) -> Self {
        Self {
            total,
            card_uid: None,
            pin: [0; 6],
            pin_len: 0,
            last_key_ms: 0,
            uid_input: [0; 10],
            uid_len: 0,
            uid_manual: false,
            session_id: None,
        }
    }
    pub fn set_uid(&mut self, uid_dec: String) {
        self.card_uid = Some(uid_dec);
    }
    // helper: thêm 1 số PIN (0..9)
    pub fn push_pin(&mut self, d: u8) {
        if self.pin_len < 6 {
            self.pin[self.pin_len] = d;
            self.pin_len += 1;
        }
    }
    // helper: xóa 1 số PIN
    pub fn pop_pin(&mut self) {
        if self.pin_len > 0 {
            self.pin_len -= 1;
        }
    }
}
fn co_uid_str(co: &CheckoutState) -> Option<String> {
    // nếu nhập tay đủ 8 số
    if co.card_uid.is_none() && co.uid_len == 10 {
        return Some(
            co.uid_input[..10]
                .iter()
                .map(|d| char::from(b'0' + *d))
                .collect(),
        );
    }
    // nếu quẹt thẻ
    if let Some(uid) = &co.card_uid {
        return Some(uid.clone());
    }
    None
}
fn co_pin_str(co: &CheckoutState) -> Option<String> {
    if co.pin_len == 6 {
        Some(co.pin[..6].iter().map(|d| char::from(b'0' + *d)).collect())
    } else {
        None
    }
}
mod heapless_string {
    use core::fmt::{self, Write};
    pub struct String<const N: usize> {
        buf: [u8; N],
        len: usize,
    }
    impl<const N: usize> String<N> {
        pub fn new() -> Self {
            Self {
                buf: [0; N],
                len: 0,
            }
        }
        pub fn as_str(&self) -> &str {
            unsafe { core::str::from_utf8_unchecked(&self.buf[..self.len]) }
        }
        pub fn push_str(&mut self, s: &str) {
            let b = s.as_bytes();
            let can = core::cmp::min(b.len(), N.saturating_sub(self.len));
            self.buf[self.len..self.len + can].copy_from_slice(&b[..can]);
            self.len += can;
        }
    }
    impl<const N: usize> Write for String<N> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            self.push_str(s);
            Ok(())
        }
    }
}
#[inline]
fn ui_checkout_handle_key(co: &mut CheckoutState, key: Key) {
    // Debounce giống menu
    const DEBOUNCE_MS: u64 = 100;
    let now = now_ms();
    if now - co.last_key_ms < DEBOUNCE_MS {
        return;
    }
    co.last_key_ms = now;

    const UID_MAX: u8 = 10;
    const PIN_MAX: u8 = 6;
    //ở case nhập số
    // ban đầu có thể nhập UID (8 số dec) rồi mới nhập PIN (6 số dec)
    // hoặc quẹt thẻ rồi mới nhập PIN
    match key {
        // C: xoá 1 số cuối nếu có
        Key::C => {
            if co.pin_len > 0 {
                co.pin_len -= 1;
            } else if co.card_uid.is_none() && co.uid_len > 0 {
                co.uid_len -= 1;
            }
        }
        // D: Xóa hết PIN và UID (hủy)
        Key::D => {
            co.pin_len = 0;
            co.uid_len = 0;
            co.card_uid = None;
            co.uid_manual = false;
            co.uid_input = [0; 10];
        }
        Key::Digit(d) => {
            if !co_has_uid(co) {
                if co.uid_len < UID_MAX {
                    co.uid_input[co.uid_len as usize] = d;
                    co.uid_len += 1;
                    co.uid_manual = true;

                    if co.uid_len == UID_MAX {
                        let mut val: u32 = 0;
                        for i in 0..8 {
                            val = val
                                .saturating_mul(10)
                                .saturating_add(co.uid_input[i] as u32);
                        }
                        let b0 = ((val >> 24) & 0xFF) as u8;
                        let b1 = ((val >> 16) & 0xFF) as u8;
                        let b2 = ((val >> 8) & 0xFF) as u8;
                        let b3 = (val & 0xFF) as u8;

                        co.card_uid = Some(format!("{:02X}{:02X}{:02X}{:02X}", b0, b1, b2, b3));
                    }
                }
            } else {
                if co.pin_len < PIN_MAX as usize {
                    co.pin[co.pin_len as usize] = d;
                    co.pin_len += 1;
                }
            }
        }
        _ => {}
    }
}
#[inline]
fn clear_area<D: DrawTarget<Color = BinaryColor>>(
    disp: &mut D,
    rect: Rectangle,
) -> Result<(), D::Error> {
    rect.into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(disp)
}

// ------------------------------ VẼ MÀN HÌNH 2 ------------------------------
const LCD_W: i32 = 128; // màn 128x64 0.96"
const PAD_R: i32 = 2; //offset phải

pub fn draw_checkout_screen<D: DrawTarget<Color = BinaryColor>>(
    disp: &mut D,
    key: Option<Key>,
    co: &mut CheckoutState,
) -> Result<(), D::Error> {
    let font = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    clear_area(disp, Rectangle::new(Point::new(0, 0), Size::new(128, 64)))?;
    // Header
    Text::new("Total: ", Point::new(0, 10), font).draw(disp)?;
    if let Some(k) = key {
        ui_checkout_handle_key(co, k); // xử lý bên trong màn hình, có debounce
    }

    //let _price = co.total;
    let price_str = format!("{}d", co.total);
    let cw = font.font.character_size.width as i32;
    let x_right = LCD_W - PAD_R;
    let x_price = x_right - (price_str.chars().count() as i32) * cw;
    Text::new(&price_str, Point::new(x_price, 10), font).draw(disp)?;

    let uid_line = heapless_string::String::<20>::new();

    let uid_str = if co.uid_manual && co.uid_len > 0 {
        let mut s = String::with_capacity(10);
        for i in 0..(co.uid_len as usize) {
            s.push(char::from(b'0' + co.uid_input[i]));
        }
        while s.len() < 10 {
            s.push('_');
        }
        format!("UID: {}", s)
    } else if let Some(uid) = &co.card_uid {
        format!("UID: {}", uid)
    } else {
        "UID: __________".to_string()
    };

    Text::with_baseline(&uid_str, Point::new(0, 26), font, Baseline::Top).draw(disp)?;

    //Text::new(uid_line.as_str(), Point::new(0, 26), font).draw(disp)?;

    // --- Ô PIN (6 kí tự) ---
    let pin_mask: String = std::iter::repeat('*').take(co.pin_len as usize).collect();
    let pin_label = if co.card_uid.is_some() {
        "PIN: "
    } else {
        "PIN: (waiting...)"
    };
    Text::with_baseline(pin_label, Point::new(0, 38), font, Baseline::Top).draw(disp)?;
    Text::with_baseline(&pin_mask, Point::new(30, 38), font, Baseline::Top).draw(disp)?;

    // --- Gợi ý phím ---
    Text::new("D:Rescan *:Menu #:Pay", Point::new(0, 58), font).draw(disp)?;

    Ok(())
}
///////////////////////////////// XỬ LÝ TRẠNG THÁI GIAO DỊCH  /////////////////////////////
pub struct TxStatus {
    pub success: bool, // true = giao dịch thành công
    pub balance: u32,  // số dư tài khoản
    pub deducted: u32, // số tiền bị trừ
    pub last_key_ms: u64,
    pub reason: Option<String>,
}

impl TxStatus {
    pub fn new(success: bool, balance: u32, deducted: u32, reason: Option<String>) -> Self {
        Self {
            success,
            balance,
            deducted,
            reason,
            last_key_ms: 0,
        }
    }
}

#[inline]
fn ui_txstatus_handle_key(st: &mut TxStatus, key: Key) -> bool {
    const DEBOUNCE_MS: u64 = 100;
    let now = now_ms();
    if now - st.last_key_ms < DEBOUNCE_MS {
        return false;
    }
    st.last_key_ms = now;

    match key {
        Key::Hash => true, // back to menu
        _ => false,
    }
}
pub fn draw_status_screen<D: DrawTarget<Color = BinaryColor>>(
    disp: &mut D,
    key: Option<Key>,
    st: &mut TxStatus,
) -> Result<bool, D::Error> {
    let mut go_back = false;
    if let Some(k) = key {
        if ui_txstatus_handle_key(st, k) {
            go_back = true;
        }
    }
    clear_area(disp, Rectangle::new(Point::new(0, 0), Size::new(128, 64)))?;

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title = if st.success { "Success" } else { "Failed" };

    Text::with_baseline(title, Point::new(49, 4), style, Baseline::Top).draw(disp)?;
    if !st.success {
        st.deducted = 0;
    }
    let bal = format!("Balance:   {}d", st.balance);
    let ded = format!("Deducted: -{}d", st.deducted);
    if st.success {
        Text::with_baseline(
            &format!("Deducted: -{}d", st.deducted),
            Point::new(0, 16),
            style,
            Baseline::Top,
        )
        .draw(disp)?;
        Text::with_baseline(
            &format!("Balance:   {}d", st.balance),
            Point::new(0, 30),
            style,
            Baseline::Top,
        )
        .draw(disp)?;
    } else {
        Text::with_baseline("Reason:", Point::new(0, 16), style, Baseline::Top).draw(disp)?;
        let reason = st.reason.clone().unwrap_or_else(|| "UNKNOWN".to_string());
        Text::with_baseline(&reason, Point::new(0, 30), style, Baseline::Top).draw(disp)?;
    }
    Text::with_baseline("[#] Back to menu", Point::new(12, 50), style, Baseline::Top).draw(disp)?;
    Ok(go_back)
}
