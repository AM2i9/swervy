#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    efuse::Efuse,
    gpio::Io,
    prelude::*,
    spi::{master::Spi, SpiBitOrder, SpiMode},
};
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};

extern crate alloc;

const PAIR_WITH_ME: [u8; 4] = [0xff, 0x68, 0x69, 0x3F];
const HELLO: [u8; 6] = [0xff, 0x48, 0x45, 0x4C, 0x4C, 0x4F];

#[entry]
fn main() -> ! {
    esp_alloc::heap_allocator!(72 * 1024);

    #[allow(unused)]
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        esp_wifi::EspWifiInitFor::Wifi,
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&_init, wifi).unwrap();

    let mut spi = Spi::new(peripherals.SPI2, 100.Hz(), SpiMode::Mode3)
        .with_pins(
            io.pins.gpio4, // clk, blue (purple)
            io.pins.gpio5, // mosi, orange (green)
            io.pins.gpio6, // miso, brown, needs pullup resistor (blue)
            io.pins.gpio7, // cs, yellow
        )
        .with_bit_order(SpiBitOrder::LSBFirst, SpiBitOrder::LSBFirst);

    let remote_address = Efuse::get_mac_address();
    let mut recv_address: [u8; 6] = [0; 6];

    log::info!("[ESP-NOW] Waiting for pairing packet...");

    loop {
        // pairing loop

        // listen for "pair with me" packet on multicast address
        let r = esp_now.receive();
        if let Some(r) = r {
            log::info!("[ESP-NOW] Recieved pairing message!");
            // log::info!("DST: {:?} | SRC: {:?} | DATA: {:?}", r.info.dst_address, r.info.src_address, r.get_data());

            // add peer
            if r.info.dst_address == BROADCAST_ADDRESS
                && r.len > 3
                && r.get_data()[..4] == PAIR_WITH_ME
                && !esp_now.peer_exists(&r.info.src_address)
            {
                esp_now
                    .add_peer(PeerInfo {
                        peer_address: r.info.src_address,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    })
                    .unwrap();
                // send peer a "hello" packet
                recv_address = r.info.src_address;
                let status = esp_now.send(&r.info.src_address, &HELLO).unwrap().wait();
                match status {
                    Ok(_) => log::info!("[ESP-NOW] Sent HELLO, waiting for response..."),
                    Err(e) => log::error!("[ESP-NOW] Failed to send HELLO: {:?}", e),
                }
            }
            // wait for peer to send back a "hello" packet to this address
            else if r.info.dst_address == remote_address
            && esp_now.peer_exists(&r.info.src_address)
            && r.get_data() == HELLO
            {
                log::info!("[ESP-NOW] HELLO packet recieved!");
                break;
            }
        }
    }

    delay.delay(10.millis());
    log::info!("[ESP-NOW] Polling started.");

    loop {
        delay.delay(10.millis());

        // For some reason the controller I have will not respond to configuration mode and
        // programmatically setting the controller to analog mode
        // will have to do manually for now

        // Poll controller
        let mut buffer = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

        if let Err(e) = spi.transfer(&mut buffer) {
            log::error!("[CONT] Error polling controller: {:?}", e);
            continue;
        }

        let header = if buffer[0] == 0xff && buffer[1] == 0x73 {
            [0xff, 0x00]
            
        } else {
            [0xff, 0xff]
        };
        let state: &[u8] = &buffer[3..];
        // println!("{:?}", state);

        // send controller state over esp-now broadcast
        if let Err(e) = esp_now.send(&recv_address, &[&header, state].concat()).unwrap().wait() {
            log::error!("[ESP-NOW] Error sending broadcast: {:?}", e);
        };
    }
}
