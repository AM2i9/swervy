#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Io, prelude::*, spi::{master::Spi, SpiBitOrder, SpiMode}};
use esp_println::println;

extern crate alloc;
use core::mem::MaybeUninit;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

#[entry]
fn main() -> ! {
    #[allow(unused)]
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    init_heap();

    esp_println::logger::init_logger_from_env();

    // let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    // let _init = esp_wifi::init(
    //     esp_wifi::EspWifiInitFor::Wifi,
    //     timg0.timer0,
    //     esp_hal::rng::Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    // )
    // .unwrap();

    let mut spi = Spi::new(
        peripherals.SPI2,
        100.Hz(),
        SpiMode::Mode3
    )
    .with_pins(
        io.pins.gpio4, // clk, blue (purple)
        io.pins.gpio5, // mosi, orange (green)
        io.pins.gpio6, // miso, brown, needs pullup resistor (blue)
        io.pins.gpio7 // cs, yellow
    ).with_bit_order(
        SpiBitOrder::LSBFirst,
        SpiBitOrder::LSBFirst
    );

    loop {

        delay.delay(10.millis());

        // For some reason the controller I have will not respond to configuration mode and
        // programmatically setting the controller to analog mode
        // will have to do manually for now

        // Poll controller
        let mut buffer = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        
        if let Err(e) = spi.transfer(&mut buffer) {
            log::error!("Error polling controller: {:?}", e);
            continue;
        }

        if buffer[0] == 0xff && buffer[1] == 0x73 {
            let state = &buffer[3..];
            println!("{:?}", state);
        }
    }
}
