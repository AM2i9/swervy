#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::{Io, Level, NoPin, Output}, prelude::*, spi::{master::Spi, SpiBitOrder, SpiMode}};
use esp_println::println;
use pscontroller_rs::{dualshock::ControlDS, Device, PlayStationPort};
// use pscontroller_rs::PlayStationPort;

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
        250.kHz(),
        SpiMode::Mode3
    )
    .with_pins(
        io.pins.gpio4, // clk, blue
        io.pins.gpio5, // mosi, orange
        io.pins.gpio6, // miso, brown, needs pullup resistor
        io.pins.gpio7 // cs, yellow
    ).with_bit_order(
        SpiBitOrder::LSBFirst,
        SpiBitOrder::LSBFirst
    );

    
    // let mut cs = Output::new(io.pins.gpio7, Level::Low);
    // let mut psp = PlayStationPort::new(spi, Some(Output::new(io.pins.gpio7, Level::Low)));
    // psp.enable_pressure().unwrap();
    loop {
        
        let mut command = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        // psp.enable_pressure().unwrap();

        // println!("c: {:?}", command);
        {
            let data = spi.transfer(&mut command);

            println!("{:?}", data.unwrap());
        }

        // let controller = match psp.read_input(None) {
        //     Err(_) => {
        //         println!("\rError reading controller");
        //         continue;
        //     },
        //     Ok(x) => x,
        // };

        // match controller {
        //     Device::None => {
        //         println!("Missing.");
        //     },
        //     Device::Mouse(x) => {
        //         println!("Mouse: X:{:02}, Y{:02}, L:{}, R:{}",
        //             x.x,
        //             x.y,
        //             x.buttons.left(),
        //             x.buttons.right());
        //     }
        //     Device::Classic(x) => {
        //         println!("Classic - Start? {0}, Square? {1}",
        //             x.buttons.start(),
        //             x.buttons.square());

        //         if x.buttons.start() && x.buttons.select() {
        //             psp.enable_pressure().unwrap();
        //         }
        //     },
        //     Device::AnalogJoystick(x) => {
        //         println!("Analog - Start? {0} - R:{1:02x},{2:02x}, L:{3:02x},{4:02x}",
        //             x.buttons.start(),
        //             x.rx,
        //             x.ry,
        //             x.lx,
        //             x.ly);
        //     },
        //     Device::DualShock(x) => {
        //         println!("DualShock - Start? {0} - R:{1:02x},{2:02x}, L:{3:02x},{4:02x}",
        //             x.buttons.start(),
        //             x.rx,
        //             x.ry,
        //             x.lx,
        //             x.ly);
        //     },
        //     Device::DualShock2(x) => {
        //         println!("DualShock2 - Start? {0} - R:{1:02x},{2:02x} - X Pressure:{3:02x}",
        //             x.buttons.start(),
        //             x.rx,
        //             x.ry,
        //             x.pressures[6]);
        //     },
        //     Device::JogCon(x) => {
        //         println!("JogCon - Buttons: {0:08b}, Wheel: {1}", x.buttons.bits(), x.jog_position())
        //     }
        //     Device::NegCon(x) => {
        //         println!("NegCon- Buttons: {0:08b}, Twist: {1}, I:  {2}", x.buttons.bits(), x.twist, x.switchi)
        //     }
        //     Device::GunCon(x) => {
        //         println!("\rGunCon - Trigger: {0}, A: {3}, B:{4}, X:{1} Y:{2}                    ",
        //             x.buttons.trigger(),
        //             x.x(),
        //             x.y(),
        //             x.buttons.a(),
        //             x.buttons.b())
        //     },
        //     Device::ConfigurationMode => {
        //         println!("Somehow we got stuck where we shouldn't be");
        //     },
        //     _ => println!("Unimplemented"),
        // }

        delay.delay(10.millis());
    }
}
