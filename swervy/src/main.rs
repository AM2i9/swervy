#![no_std]
#![no_main]

extern crate alloc;

mod encoder;
mod esc;
mod pid;

use core::f32::consts::PI;

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
use pid::{PIDConstants, PIDController};
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

const MAX_THROTTLE: f32 = 6.0;

fn signum(num: f32) -> f32 {
    if num > 0.0 {
        1.0
    } else if num < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[main]
async fn main(spawner: Spawner) -> ! {
    // allocate heap memory for wifi
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    // Setup multiplxed encoders on I2C bus

    let i2c: I2c<I2C0, Async> = I2c::new_with_timeout_async(
        peripherals.I2C0,
        io.pins.gpio17,
        io.pins.gpio16,
        10.kHz(),
        None,
    );
    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<I2C0, Async>>> = StaticCell::new();
    let i2cbus: &mut Mutex<NoopRawMutex, I2c<'_, I2C0, Async>> = I2C_BUS.init(Mutex::new(i2c));

    let mut encoder_a = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Zero,
        0b0000110,
        None
    );

    let mut encoder_b = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Two,
        0b0000110,
        None
    );

    let mut encoder_c = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Four,
        0b0000110,
        None
    );

    let mut encoder_d = MuxedEncoder::new(
        I2cDevice::new(i2cbus),
        encoder::EncoderChannel::Six,
        0b0000110,
        None
    );

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

    // let mut test_dev = I2cDevice::new(i2cbus);

    let mut steer1_pid = PIDController::new(PIDConstants {
        kp: 2.0,
        ki: 0.0,
        kd: 1.0,
    });
    let mut steer2_pid = PIDController::new(PIDConstants {
        kp: 2.0,
        ki: 0.0,
        kd: 1.0,
    });
    let mut steer3_pid = PIDController::new(PIDConstants {
        kp: 2.0,
        ki: 0.0,
        kd: 1.0,
    });
    let mut steer4_pid = PIDController::new(PIDConstants {
        kp: 2.0,
        ki: 0.0,
        kd: 1.0,
    });

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

        info!("[ENC] Zeroing encoders...");
        {
            let encoder_a_off = encoder_a.get_raw_angle().await.unwrap_or(0);
            let encoder_b_off = encoder_b.get_raw_angle().await.unwrap_or(0);
            let encoder_c_off = encoder_c.get_raw_angle().await.unwrap_or(0);
            let encoder_d_off = encoder_d.get_raw_angle().await.unwrap_or(0);

            encoder_a.set_offset(Some(encoder_a_off));
            encoder_b.set_offset(Some(encoder_b_off));
            encoder_c.set_offset(Some(encoder_c_off));
            encoder_d.set_offset(Some(encoder_d_off));
            info!("[ENC] Zeroed! A: {} | B: {} | C: {} | D: {}", encoder_a_off, encoder_b_off, encoder_c_off, encoder_d_off);
        }

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

            let mut rx: f32 = 123.0;
            let mut ry: f32 = 123.0;
            let mut lx: f32 = 123.0;
            let mut ly: f32 = 123.0;

            if let Some(r) = r {
                // log::info!("DST: {:?} | SRC: {:?} | DATA: {:?}", r.info.dst_address, r.info.src_address, r.get_data());
                if r.info.src_address == remote_address && r.info.dst_address == controller_address
                {
                    last_rec_time = time::now();
                    let state = r.get_data();

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

                    // TODO: Find a better failure mode for encoder errors
                    let mut enc_1_angle = encoder_a.get_angle_radians().await.unwrap_or(0.0);
                    let mut enc_2_angle = encoder_b.get_angle_radians().await.unwrap_or(0.0);
                    let mut enc_3_angle = encoder_c.get_angle_radians().await.unwrap_or(0.0);
                    let mut enc_4_angle = encoder_d.get_angle_radians().await.unwrap_or(0.0);

                    // info!("encoders: 1 {enc_1_angle:?} | 2 {enc_2_angle:?} | 3 {enc_3_angle:?} | 4 {enc_4_angle:?}");

                    let steer_mag =
                        libm::sqrtf(libm::powf(rx - 122.0, 2.0) + libm::powf(ry - 123.0, 2.0)) / 123.0;

                    let mut throttle: f32 = (ly - 123.0) / 127.0;

                    // see if its quicker to get to the angle or to get to the angle +180
                    // if |M - S| >= 179, new S = (S + 180) % 360, motor_dir = -1
                    let mut steer_angle = libm::atan2f(ry - 123.0, rx - 123.0);
                    
                    // If output of atan2f is negative angle (pi->2pi), make it positive
                    if steer_angle < 0.0 {
                        steer_angle += 2.0 * PI;
                    }

                    // see if its quicker to go clockwise or counterclockwise to reach angle
                    // if 359 - S <= M go clockwise else go counterclockwise (depends on direction of encoder)
                    // if libm::fabsf(enc_1_angle - steer_angle) > PI {
                    //     enc_1_angle = (enc_1_angle + steer_angle) - (2.0 * PI);
                    // }
                    
                    // if libm::fabsf(enc_2_angle - steer_angle) > PI {
                    //     enc_2_angle = (enc_2_angle + steer_angle) - (2.0 * PI);
                    // }
                    // if libm::fabsf(enc_3_angle - steer_angle) > PI {
                    //     enc_3_angle = (enc_3_angle + steer_angle) - (2.0 * PI);
                    // }
                    // if libm::fabsf(enc_4_angle - steer_angle) > PI {
                    //     enc_4_angle = (enc_4_angle + steer_angle) - (2.0 * PI);
                    // }

                    // we are NOT using the full range of these motors
                    let drive_speed = ((steer_mag * throttle) * (MAX_THROTTLE * 1.0)) as u8;
                    // let drive_out = 49 + drive_speed;

                    // let _ = motor1.set_throttle_pct(drive_out);
                    // let _ = motor3.set_throttle_pct(drive_out);
                    // let _ = motor5.set_throttle_pct(drive_out);
                    // let _ = motor7.set_throttle_pct(drive_out);
                    steer1_pid.setpoint(steer_angle);
                    steer2_pid.setpoint(steer_angle);
                    steer3_pid.setpoint(steer_angle);
                    steer4_pid.setpoint(steer_angle);

                    let timestamp = time::now();

                    let steer_1_pid_out = steer1_pid.calculate(enc_1_angle, Some(timestamp));
                    let steer_2_pid_out = steer2_pid.calculate(enc_2_angle, Some(timestamp));
                    let steer_3_pid_out = steer3_pid.calculate(enc_3_angle, Some(timestamp));
                    let steer_4_pid_out = steer4_pid.calculate(enc_4_angle, Some(timestamp));
                    let steer1_out = 50.0 - ((3.0 * signum(steer_1_pid_out)) + steer_1_pid_out);
                    let steer2_out = 50.0 - ((3.0 * signum(steer_2_pid_out)) + steer_2_pid_out);
                    let steer3_out = 50.0 - ((3.0 * signum(steer_3_pid_out)) + steer_3_pid_out);
                    let steer4_out = 50.0 - ((3.0 * signum(steer_4_pid_out)) + steer_4_pid_out);

                    // info!("drive_out: {} | steer_angle: {} | throttle: {} | 1: {} | 2: {} | 3: {} | 4: {}", enc_1_angle, steer_angle, throttle, 10.0 * signum(steer_1_pid_out), steer_1_pid_out, steer1_out, steer4_out);

                    let _ = motor0.set_throttle_pct(steer1_out as u8);
                    let _ = motor2.set_throttle_pct(steer2_out as u8);
                    let _ = motor4.set_throttle_pct(steer3_out as u8);
                    let _ = motor6.set_throttle_pct(steer4_out as u8);
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

        info!("[ESP-NOw] Connection closed");
        esp_now.remove_peer(&remote_address);
        remote_address = [0_u8; 6];

        info!("[ESC] Disabling motors");
        let _ = motor0.disable();
        let _ = motor1.disable();
        let _ = motor2.disable();
        let _ = motor3.disable();
        let _ = motor4.disable();
        let _ = motor5.disable();
        let _ = motor6.disable();
        let _ = motor7.disable();
    }
}
