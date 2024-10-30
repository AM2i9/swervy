#![no_std]
#![no_main]

extern crate alloc;

mod encoder;
mod esc;
mod pid;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use encoder::MuxedEncoder;
use esc::BrushlessESC;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::efuse::Efuse;
use esp_hal::i2c::I2c;
use esp_hal::ledc::{self, Ledc, LowSpeed};
use esp_hal::rng::Rng;
use esp_hal::time;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{gpio::Io, peripherals::I2C0, prelude::*, Async};
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::EspWifiInitFor;
use log::{error, info, warn};
use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const PAIR_WITH_ME: [u8; 4] = [0xff, 0x68, 0x69, 0x3F];
const HELLO: [u8; 6] = [0xff, 0x48, 0x45, 0x4C, 0x4C, 0x4F];

#[main]
async fn main(spawner: Spawner) -> ! {
    // allocate heap memory for wifi
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    // Setup multiplxed encoders on I2C bus

    let i2c = I2c::new_with_timeout_async(
        peripherals.I2C0,
        io.pins.gpio9,
        io.pins.gpio10,
        1.kHz(),
        Some(10),
    );
    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<I2C0, Async>>> = StaticCell::new();
    let i2cbus: &mut Mutex<NoopRawMutex, I2c<'_, I2C0, Async>> = I2C_BUS.init(Mutex::new(i2c));

    let mut encoder_a =
        MuxedEncoder::new(I2cDevice::new(i2cbus), encoder::EncoderChannel::Zero, 0x06);

    let mut encoder_b =
        MuxedEncoder::new(I2cDevice::new(i2cbus), encoder::EncoderChannel::Two, 0x06);

    let mut encoder_c =
        MuxedEncoder::new(I2cDevice::new(i2cbus), encoder::EncoderChannel::Three, 0x06);

    let mut encoder_d =
        MuxedEncoder::new(I2cDevice::new(i2cbus), encoder::EncoderChannel::Four, 0x06);

    // Configure PWM escs

    let mut ledc = Ledc::new(peripherals.LEDC);

    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty14Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(ledc::channel::Number::Channel0, io.pins.gpio3);
    let mut motor0 = BrushlessESC::new(&mut channel0, &lstimer0, 2000, 1000, 1000);

    let mut channel1 = ledc.get_channel(ledc::channel::Number::Channel1, io.pins.gpio8);
    let mut motor1 = BrushlessESC::new(&mut channel1, &lstimer0, 2000, 1000, 1000);

    let mut channel2 = ledc.get_channel(ledc::channel::Number::Channel2, io.pins.gpio18);
    let mut motor2 = BrushlessESC::new(&mut channel2, &lstimer0, 2000, 1000, 1000);

    let mut channel3 = ledc.get_channel(ledc::channel::Number::Channel3, io.pins.gpio15);
    let mut motor3 = BrushlessESC::new(&mut channel3, &lstimer0, 2000, 1000, 1000);

    let mut channel4 = ledc.get_channel(ledc::channel::Number::Channel4, io.pins.gpio7);
    let mut motor4 = BrushlessESC::new(&mut channel4, &lstimer0, 2000, 1000, 1000);

    let mut channel5 = ledc.get_channel(ledc::channel::Number::Channel5, io.pins.gpio6);
    let mut motor5 = BrushlessESC::new(&mut channel5, &lstimer0, 2000, 1000, 1000);

    let mut channel6 = ledc.get_channel(ledc::channel::Number::Channel6, io.pins.gpio5);
    let mut motor6 = BrushlessESC::new(&mut channel6, &lstimer0, 2000, 1000, 1000);

    let mut channel7 = ledc.get_channel(ledc::channel::Number::Channel7, io.pins.gpio4);
    let mut motor7 = BrushlessESC::new(&mut channel7, &lstimer0, 2000, 1000, 1000);

    // Wifi AP setup

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let _init = esp_wifi::init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&_init, wifi).unwrap();

    // init embussy
    {
        let timg1 = TimerGroup::new(peripherals.TIMG1);
        esp_hal_embassy::init(timg1.timer0);
    }

    let controller_address = Efuse::get_mac_address();
    let mut remote_address: [u8; 6] = [0; 6];

    loop {
        // pair loop
        info!("[ESP-NOW] Pairing loop started...");
        loop {
            // send "pair with me"
            esp_now.send(&BROADCAST_ADDRESS, &PAIR_WITH_ME);

            let r = esp_now.receive();

            if let Some(r) = r {
                if r.info.dst_address == controller_address && r.get_data() == HELLO {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();

                    remote_address = r.info.src_address;
                    info!("[ESP-NOW] Got HELLO");
                    
                    // send back hello to remote
                    esp_now.send(&remote_address, &HELLO);
                    break;
                }
            }
        }

        info!("[ESP-NOW] Paired with {:?}", remote_address);

        info!("[ESC] Arming...");
        motor0.disable();
        motor1.disable();
        motor2.disable();
        motor3.disable();
        motor4.disable();
        motor5.disable();
        motor6.disable();
        motor7.disable();
        Timer::after_millis(500).await;
        motor0.arm_sig();
        motor1.arm_sig();
        motor2.arm_sig();
        motor3.arm_sig();
        motor4.arm_sig();
        motor5.arm_sig();
        motor6.arm_sig();
        motor7.arm_sig();
        Timer::after_millis(500).await;
        motor0.set_throttle_pct(50);
        motor1.set_throttle_pct(50);
        motor2.set_throttle_pct(50);
        motor3.set_throttle_pct(50);
        motor4.set_throttle_pct(50);
        motor5.set_throttle_pct(50);
        motor6.set_throttle_pct(50);
        motor7.set_throttle_pct(50);
        Timer::after_millis(1000).await;
        info!("[ESC] Ready!");

        // drive loop
        let mut last_rec_time = time::now();
        info!("[ESP-NOW] Remote loop started");
        loop {
            let r = esp_now.receive();

            if let Some(r) = r {
                // log::info!("DST: {:?} | SRC: {:?} | DATA: {:?}", r.info.dst_address, r.info.src_address, r.get_data());
                if r.info.src_address == remote_address
                    && r.info.dst_address == controller_address
                {
                    last_rec_time = time::now();
                    let state = r.get_data();

                    if state[..2] == [0xff, 0x00] {
                        // get joystick data
                        let rx = state[6];
                        let ry = state[7];
                        let lx = state[4];
                        let ly = state[5];

                        log::info!("RX: {} | RY: {} | LX: {} | LY: {}", rx, ry, lx, ly);
                    } else {
                        warn!("[ESP-NOW] No joystick data");
                    }

                    // do swerve module stuff
                    
                }
            }

            // check for timeout of remote signals
            if time::now()
                .checked_duration_since(last_rec_time)
                .unwrap()
                .to_millis()
                > 5000
            {
                error!("[ESP-NOW] Remote timeout!");
                break;
            }
        }

        info!("[ESP-NOw] Connection closed");
        esp_now.remove_peer(&remote_address);
        remote_address = [0_u8; 6];

        info!("[ESC] Disabling motors");
        motor0.disable();
        motor1.disable();
        motor2.disable();
        motor3.disable();
        motor4.disable();
        motor5.disable();
        motor6.disable();
        motor7.disable();
    }
}
