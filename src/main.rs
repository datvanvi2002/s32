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
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};

use esp_idf_sys as _;
use mfrc522::comm::blocking::spi::SpiInterface;
use mfrc522::Mfrc522;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306,
};
use std::{thread, time::Duration};

// ======== MODEL MENU + UI ========
#[derive(Clone)]
struct MenuItem {
    name: String,
    price: i64, // VND
}
// ======== UI chọn món dùng A/B/* =========
struct UiMenu {
    cursor: usize,    // đang trỏ item nào
    qty: Vec<u8>,     // số lượng cho từng món (0..9)
    win_top: usize,   // item đầu trong "cửa sổ" 3 dòng
    total: i64,       // tổng tiền
    last_key_ms: u64, // debounce
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

fn now_ms() -> u64 {
    use core::time::Duration;
    use std::time::SystemTime;
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or(Duration::from_millis(0))
        .as_millis() as u64
}

fn format_vnd(n: i64) -> String {
    // 45000 -> "45,000đ"
    let mut s = n.abs().to_string();
    let mut out = String::new();
    while s.len() > 3 {
        let t = s.split_off(s.len() - 3);
        if out.is_empty() {
            out = t;
        } else {
            out = format!("{},{}", t, out);
        }
    }
    if out.is_empty() {
        out = s;
    } else {
        out = format!("{},{}", s, out);
    }
    if n < 0 {
        out.insert(0, '-');
    }
    out.push('đ');
    out
}

#[derive(Copy, Clone, Debug)]
enum Key {
    A,
    B,
    Star,
    None,
}

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

    // ===== MENU STATE =====

    let mut menu_items: Vec<(String, i64)> = firebase_get_menu().unwrap_or_default();
    let mut ui_menu = UiMenu::new(menu_items.len());
    let mut last_menu_fetch = std::time::Instant::now()
        .checked_sub(core::time::Duration::from_secs(3600))
        .unwrap_or(std::time::Instant::now());

    let mut menu_scroll_idx: usize = 0;
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
        //log::info!("Blinking LED 2...");
        led.set_high().unwrap(); // bật đèn LED
        thread::sleep(Duration::from_millis(5)); // đợi 1 gi ây
        led.set_low().unwrap(); // tắt đèn LED
        thread::sleep(Duration::from_millis(5));

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
            // let key = match key_pressed {
            //     Some('A') => Key::A,
            //     Some('B') => Key::B,
            //     Some('*') => Key::Star,
            //     _ => Key::None,
            // };
        }

        /***********************************************************************************************************/
        /********************************************** XỬ LÝ BÀN PHÍM MA TRẬN *************************************/
        /***********************************************************************************************************/
        if key_pressed == Some('A') {
            led.toggle().unwrap(); // Bật LED nếu phím 'A' được nhấn
            log::info!("Test handle matrix keyboard");
        }
        let key = match key_pressed {
            Some('A') => Key::A,
            Some('B') => Key::B,
            Some('*') => Key::Star,
            _ => Key::None,
        };
        // Cập nhật UI chọn món
        ui_menu_handle_key(&mut ui_menu, key, menu_items.len());
        ui_menu_recompute_total(&mut ui_menu, &menu_items);
        if last_menu_fetch.elapsed() > Duration::from_secs(30) || menu_items.is_empty() {
            match firebase_get_menu() {
                Ok(v) => {
                    // resize qty để giữ số lượng cũ nếu còn
                    let old_len = menu_items.len();
                    menu_items = v;
                    if menu_items.len() != old_len {
                        let mut new_qty = vec![0u8; menu_items.len()];
                        for i in 0..ui_menu.qty.len().min(new_qty.len()) {
                            new_qty[i] = ui_menu.qty[i];
                        }
                        ui_menu.qty = new_qty;

                        if ui_menu.cursor >= menu_items.len() {
                            ui_menu.cursor = menu_items.len().saturating_sub(1);
                            ui_menu.win_top = ui_menu.cursor.saturating_sub(2);
                        }
                    }
                    last_menu_fetch = std::time::Instant::now();
                    log::info!("Menu items fetched: {}", menu_items.len());
                }
                Err(e) => log::warn!("Could not fetch menu: {}", e),
            }
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

        ui_menu_render(&mut disp, &menu_items, &ui_menu);
        disp.flush().unwrap();

        /***********************************************************************************************************/
        /********************************************** Gửi thông tin lên server ***********************************/
        /***********************************************************************************************************/

        // static mut TICK: u32 = 0;
        // unsafe {
        //     TICK = TICK.wrapping_add(1);
        // }
        // if unsafe { TICK % 10 } == 0 {
        //     // ~ mỗi 10 vòng (mỗi vòng ~1s theo blink)
        //     match firebase_get_i64("gpio36") {
        //         Ok(v) => log::info!("FB gpio36 = {}", v),
        //         Err(e) => log::error!("FB get gpio36 error: {}", e),
        //     }
        //     match firebase_get_i64("gpio5") {
        //         Ok(v) => {
        //             log::info!("FB gpio5 = {}", v);
        //             let _ = firebase_put_i64("gpio5", v + 1);
        //         }
        //         Err(e) => log::error!("FB get gpio5 error: {}", e),
        //     }
        // }

        //log::info!("From main.rs ESP32!");
        thread::sleep(Duration::from_millis(20));

        // ====================== HIỂN THỊ MENU ======================
        // Cứ mỗi 30s (hoặc lần đầu trống) thì tải lại danh sách món
        if last_menu_fetch.elapsed() > Duration::from_secs(30) || menu_items.is_empty() {
            match firebase_get_menu() {
                Ok(v) => {
                    menu_items = v;
                    last_menu_fetch = std::time::Instant::now();
                    log::info!("Menu items fetched: {}", menu_items.len());
                    menu_scroll_idx = 0; // reset scroll
                }
                Err(e) => {
                    log::warn!("Could not fetch menu: {}", e);
                }
            }
        }

        // Vẽ vùng MENU ở nửa dưới màn hình (y từ ~40px)
        // Clear vùng hiển thị menu (3 dòng, mỗi dòng ~12px)
        let menu_y = 40i32;
        let line_h = 12u32;
        for i in 0..3 {
            embedded_graphics::primitives::Rectangle::new(
                Point::new(0, menu_y + (i as i32) * (line_h as i32) - 10),
                Size::new(128, line_h),
            )
            .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
                BinaryColor::Off,
            ))
            .draw(&mut disp)
            .ok();
        }

        // Header "MENU"
        let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        embedded_graphics::text::Text::new("MENU:", Point::new(0, menu_y - 2), style)
            .draw(&mut disp)
            .ok();

        // In tối đa 3 món theo dạng "Tên ... 45k"
        if !menu_items.is_empty() {
            for i in 0..3 {
                let idx = (menu_scroll_idx + i) % menu_items.len();
                let (name, price) = &menu_items[idx];

                // Rút gọn tên cho vừa 128px (FONT_6X10 ~6px/char => ~21 ký tự)
                let mut name_short = name.clone();
                if name_short.chars().count() > 16 {
                    name_short = name_short.chars().take(16).collect::<String>() + "…";
                }
                let price_text = format!("{}d", price); // hoặc định dạng "45,000đ"
                let line = format!("{:<17}{}", name_short, price_text);

                embedded_graphics::text::Text::new(
                    &line,
                    Point::new(0, menu_y + 12 * (i as i32) + 10),
                    style,
                )
                .draw(&mut disp)
                .ok();
            }

            // Cuộn mỗi vài vòng lặp (~tuỳ tốc độ loop của bạn). Ví dụ:
            static mut MENU_TICK: u32 = 0;
            unsafe {
                MENU_TICK = MENU_TICK.wrapping_add(1);
                if MENU_TICK % 20 == 0 {
                    // đổi 20 tuỳ ý để cuộn chậm/nhanh
                    menu_scroll_idx = (menu_scroll_idx + 1) % menu_items.len();
                }
            }
        }

        disp.flush().ok();
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

/// Đọc danh sách món ăn từ /menu (Realtime Database)
/// Trả về Vec<(tên, giá)>
/// Đọc danh sách món ăn từ /menu (Realtime Database).
/// Trả về Vec<(tên, giá)>
fn firebase_get_menu() -> anyhow::Result<Vec<(String, i64)>> {
    let url = build_fb_url("menu");
    let body = http_get(&url)?;
    let v: serde_json::Value = serde_json::from_str(&body)?;

    let mut out: Vec<(String, i64)> = Vec::new();
    if let serde_json::Value::Object(map) = v {
        for (_key, item) in map {
            if let serde_json::Value::Object(obj) = item {
                let name = obj
                    .get("name")
                    .and_then(|x| x.as_str())
                    .unwrap_or("(no name)")
                    .to_string();
                let price = match obj.get("price") {
                    Some(serde_json::Value::Number(n)) => n.as_i64().unwrap_or(0),
                    Some(serde_json::Value::String(s)) => s.parse::<i64>().unwrap_or(0),
                    _ => 0,
                };
                let available = obj
                    .get("available")
                    .and_then(|x| x.as_bool())
                    .unwrap_or(true);
                if available {
                    out.push((name, price));
                }
            }
        }
    }
    // Sắp xếp theo tên cho dễ nhìn
    out.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));
    Ok(out)
}
fn ui_menu_handle_key(ui: &mut UiMenu, key: Key, items_len: usize) {
    const DEBOUNCE_MS: u64 = 100;
    let now = now_ms();
    if now - ui.last_key_ms < DEBOUNCE_MS {
        return;
    }
    ui.last_key_ms = now;

    match key {
        Key::A => {
            if items_len == 0 {
                return;
            }
            ui.cursor = if ui.cursor == 0 {
                items_len - 1
            } else {
                ui.cursor - 1
            };
            if ui.cursor < ui.win_top {
                ui.win_top = ui.cursor;
            }
        }
        Key::B => {
            if items_len == 0 {
                return;
            }
            ui.cursor = (ui.cursor + 1) % items_len;
            if ui.cursor >= ui.win_top + 3 {
                ui.win_top = ui.cursor + 1 - 3;
            }
        }
        Key::Star => {
            if items_len == 0 {
                return;
            }
            let q = &mut ui.qty[ui.cursor];
            *q = (*q + 1) % 10; // 0..9
        }
        Key::None => {}
    }
}

fn ui_menu_recompute_total(ui: &mut UiMenu, items: &[(String, i64)]) {
    let mut t = 0i64;
    for (i, (_, price)) in items.iter().enumerate() {
        t += *price * ui.qty[i] as i64;
    }
    ui.total = t;
}

fn ui_menu_render<D: DrawTarget<Color = BinaryColor>>(
    disp: &mut D,
    items: &[(String, i64)],
    ui: &UiMenu,
) {
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Xoá vùng dưới (tuỳ chiều cao OLED của bạn; ở đây xoá từ y=36 trở xuống)
    Rectangle::new(Point::new(0, 36), Size::new(128, 28))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(disp)
        .ok();

    Text::new("MENU:", Point::new(0, 38), style).draw(disp).ok();

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
            if short.chars().count() > 14 {
                short = short.chars().take(14).collect::<String>() + "…";
            }
            let q = ui.qty[i];
            let qch = if q == 0 { '-' } else { (b'0' + q) as char };
            let line = format!(
                "{c} [{q}] {name:<15}{price}",
                c = cursor,
                q = qch,
                name = short,
                price = format_vnd(*price)
            );
            Text::new(&line, Point::new(0, 50 + (row as i32) * 12), style)
                .draw(disp)
                .ok();
        }
    }

    let status = format!("A↑ B↓ *=SL | Total {}", format_vnd(ui.total));
    Text::new(&status, Point::new(0, 62), style).draw(disp).ok();
}
