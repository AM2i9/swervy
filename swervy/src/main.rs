#![no_std]
#![no_main]

extern crate alloc;

mod encoder;
mod esc;
mod module;
mod pid;
mod util;

use core::f32::consts::PI;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use encoder::MuxedEncoder;
use esc::BrushlessESC;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::efuse::Efuse;
use esp_hal::i2c::master as esp_i2c;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, Ledc, LowSpeed};
use esp_hal::rng::Rng;
use esp_hal::time::{self, RateExtU32};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_hal_embassy::main;
use esp_println::println;
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use log::{debug, error, info, warn};
use module::SwerveModule;
use pid::{PIDConstants, PIDController};
use static_cell::StaticCell;

const PAIR_WITH_ME: [u8; 4] = [0xff, 0x68, 0x69, 0x3F];
const HELLO: [u8; 6] = [0xff, 0x48, 0x45, 0x4C, 0x4C, 0x4F];

#[main]
async fn main(spawner: Spawner) -> ! {
    // allocate heap memory for wifi
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::logger::init_logger_from_env();

    // Setup multiplxed encoders on I2C bus

    let i2c: esp_i2c::I2c<Async> = esp_i2c::I2c::new(
        peripherals.I2C0,
        esp_i2c::Config::default(), // might need 10khz
    )
    .unwrap()
    .with_sda(peripherals.GPIO17)
    .with_scl(peripherals.GPIO16)
    .into_async();

    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, esp_i2c::I2c<Async>>> = StaticCell::new();
    let i2cbus: &mut Mutex<NoopRawMutex, esp_i2c::I2c<'_, Async>> = I2C_BUS.init(Mutex::new(i2c));

    let mut encoder_a = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Zero,
        0b0000110,
        None,
    );

    let mut encoder_b = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Two,
        0b0000110,
        None,
    );

    let mut encoder_c = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Four,
        0b0000110,
        None,
    );

    let mut encoder_d = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Six,
        0b0000110,
        None,
    );

    // Configure PWM escs

    let mut ledc = Ledc::new(peripherals.LEDC);

    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty14Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    let mut channel0 = ledc.channel(ledc::channel::Number::Channel0, peripherals.GPIO3);
    let mut motor0 = BrushlessESC::new(&mut channel0, &lstimer0, 2000, 1000, 1000);

    let mut channel1 = ledc.channel(ledc::channel::Number::Channel1, peripherals.GPIO8);
    let mut motor1 = BrushlessESC::new(&mut channel1, &lstimer0, 2000, 1000, 1000);

    let mut channel2 = ledc.channel(ledc::channel::Number::Channel2, peripherals.GPIO18);
    let mut motor2 = BrushlessESC::new(&mut channel2, &lstimer0, 2000, 1000, 1000);

    let mut channel3 = ledc.channel(ledc::channel::Number::Channel3, peripherals.GPIO15);
    let mut motor3 = BrushlessESC::new(&mut channel3, &lstimer0, 2000, 1000, 1000);

    let mut channel4 = ledc.channel(ledc::channel::Number::Channel4, peripherals.GPIO7);
    let mut motor4 = BrushlessESC::new(&mut channel4, &lstimer0, 2000, 1000, 1000);

    let mut channel5 = ledc.channel(ledc::channel::Number::Channel5, peripherals.GPIO6);
    let mut motor5 = BrushlessESC::new(&mut channel5, &lstimer0, 2000, 1000, 1000);

    let mut channel6 = ledc.channel(ledc::channel::Number::Channel6, peripherals.GPIO5);
    let mut motor6 = BrushlessESC::new(&mut channel6, &lstimer0, 2000, 1000, 1000);

    let mut channel7 = ledc.channel(ledc::channel::Number::Channel7, peripherals.GPIO4);
    let mut motor7 = BrushlessESC::new(&mut channel7, &lstimer0, 2000, 1000, 1000);

    // Wifi AP setup

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let _init = esp_wifi::init(
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

    let controller_address = Efuse::read_base_mac_address();
    let mut remote_address: [u8; 6] = [0; 6];

    // let mut test_dev = I2cDevice::new(i2cbus);

    let mut steer1_pid = PIDController::new(PIDConstants {
        kp: 1.9,
        ki: 0.2,
        kd: 0.2,
    });
    let mut steer2_pid = PIDController::new(PIDConstants {
        kp: 0.1,
        ki: 0.0,
        kd: 0.1,
    });
    let mut steer3_pid = PIDController::new(PIDConstants {
        kp: 0.1,
        ki: 0.0,
        kd: 0.1,
    });
    let mut steer4_pid = PIDController::new(PIDConstants {
        kp: 0.1,
        ki: 0.0,
        kd: 0.1,
    });

    let mut module1 = SwerveModule::new(motor1, motor0, encoder_a, steer1_pid);
    let mut module2 = SwerveModule::new(motor3, motor2, encoder_b, steer2_pid);
    let mut module3 = SwerveModule::new(motor5, motor4, encoder_c, steer3_pid);
    let mut module4 = SwerveModule::new(motor7, motor6, encoder_d, steer4_pid);
    
    loop {
        // pair loop
        info!("[ESP-NOW] Pairing loop started...");
        loop {
            // send "pair with me"
            esp_now.send(&BROADCAST_ADDRESS, &PAIR_WITH_ME);

            let r = esp_now.receive();

            if let Some(r) = r {
                if r.info.dst_address == controller_address && r.data() == HELLO {
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

        info!("[ENC] Zeroing encoders...");
        module1.zero().await;
        module2.zero().await;
        module3.zero().await;
        module4.zero().await;

        info!("[ESC] Arming...");
        module1.disable();
        module2.disable();
        module3.disable();
        module4.disable();
        Timer::after_millis(500).await;
        module1.arm();
        module2.arm();
        module3.arm();
        module4.arm();
        Timer::after_millis(1000).await;
        module1.center_throttles();
        module2.center_throttles();
        module3.center_throttles();
        module4.center_throttles();
        Timer::after_millis(2000).await;
        info!("[ESC] Ready!");

        // drive loop
        let mut last_rec_time = time::now();
        info!("[ESP-NOW] Remote loop started");
        loop {
            let r = esp_now.receive();

            let mut rx: f32 = 123.0;
            let mut ry: f32 = 123.0;
            let mut lx: f32 = 123.0;
            let mut ly: f32 = 123.0;

            if let Some(r) = r {
                // log::info!("DST: {:?} | SRC: {:?} | DATA: {:?}", r.info.dst_address, r.info.src_address, r.get_data());
                if r.info.src_address == remote_address && r.info.dst_address == controller_address
                {
                    last_rec_time = time::now();
                    let state = r.data();

                    if state[..2] == [0xff, 0x00] {
                        // get joystick data
                        rx = state[4] as f32;
                        ry = state[5] as f32;
                        lx = state[6] as f32;
                        ly = state[7] as f32;

                        // log::info!("RX: {} | RY: {} | LX: {} | LY: {}", rx, ry, lx, ly);
                    } else {
                        warn!("[ESP-NOW] No joystick data");
                    }

                    // info!("encoders: 1 {enc_1_angle:?} | 2 {enc_2_angle:?} | 3 {enc_3_angle:?} | 4 {enc_4_angle:?}");

                    let steer_mag =
                        libm::sqrtf(libm::powf(rx - 122.0, 2.0) + libm::powf(ry - 123.0, 2.0))
                            / 123.0;

                    let throttle: f32 = (ly - 123.0) / 127.0;
                    let mut steer_angle = libm::atan2f(ry - 123.0, rx - 123.0);

                    // If output of atan2f is negative angle (pi->2pi), make it positive
                    if steer_angle < 0.0 {
                        steer_angle += 2.0 * PI;
                    }

                    publish_value!("steer_angle", steer_angle);
                    publish_value!("throttle", throttle);

                    module1.set_angle(steer_angle);
                    module2.set_angle(steer_angle);
                    module3.set_angle(steer_angle);
                    module4.set_angle(steer_angle);
                    
                    publish_value!("steer_angle_1", module1.get_angle().await);
                    publish_value!("steer_angle_2", module2.get_angle().await);
                    publish_value!("steer_angle_3", module3.get_angle().await);
                    publish_value!("steer_angle_4", module4.get_angle().await);

                    publish_value!("module1_setpoint", module1.get_setpoint());
                    publish_value!("module2_setpoint", module2.get_setpoint());
                    publish_value!("module3_setpoint", module3.get_setpoint());
                    publish_value!("module4_setpoint", module4.get_setpoint());

                    module1.set_wheel_speed(throttle);
                    module2.set_wheel_speed(throttle);
                    module3.set_wheel_speed(throttle);
                    module4.set_wheel_speed(throttle);

                    // Execute motion
                    module1.periodic().await;
                    module2.periodic().await;
                    module3.periodic().await;
                    module4.periodic().await;
                }
            }

            // check for timeout of remote signals
            if time::now()
                .checked_duration_since(last_rec_time)
                .unwrap()
                .to_millis()
                > 1000
            {
                error!("[ESP-NOW] Remote timeout!");
                break;
            }
        }

        info!("[ESC] Disabling modules");
        module1.disable();
        module2.disable();
        module3.disable();
        module4.disable();

        info!("[ESP-NOw] Connection closed");
        esp_now.remove_peer(&remote_address);
        remote_address = [0_u8; 6];
    }
}
