#![no_std]
#![no_main]

extern crate alloc;

mod encoder;
mod esc;
mod pid;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket, Config, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources,
    StaticConfigV4,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use encoder::MuxedEncoder;
use esc::BrushlessESC;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::i2c::I2c;
use esp_hal::ledc::{self, Ledc, LowSpeed};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{gpio::Io, peripherals::I2C0, prelude::*, Async};
use esp_wifi::wifi::{
    AccessPointConfiguration, Configuration, WifiApDevice, WifiController, WifiDevice, WifiEvent,
    WifiState,
};
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

#[main]
async fn main(spawner: Spawner) -> ! {
    // allocate heap memory for wifi
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    // Setup multiplxed encoders on I2C bus

    let i2c = I2c::new_with_timeout_async(peripherals.I2C0, io.pins.gpio9, io.pins.gpio10, 1.kHz(), Some(10));
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

    let mut channel0 = ledc.get_channel(ledc::channel::Number::Channel0, io.pins.gpio4);
    let mut motor0 = BrushlessESC::new(&mut channel0, &lstimer0, 2000, 1000, 1000);

    let mut channel1 = ledc.get_channel(ledc::channel::Number::Channel1, io.pins.gpio5);
    let mut motor1 = BrushlessESC::new(&mut channel1, &lstimer0, 2000, 1000, 1000);

    let mut channel2 = ledc.get_channel(ledc::channel::Number::Channel2, io.pins.gpio6);
    let mut motor2 = BrushlessESC::new(&mut channel2, &lstimer0, 2000, 1000, 1000);

    let mut channel3 = ledc.get_channel(ledc::channel::Number::Channel3, io.pins.gpio7);
    let mut motor3 = BrushlessESC::new(&mut channel3, &lstimer0, 2000, 1000, 1000);

    let mut channel4 = ledc.get_channel(ledc::channel::Number::Channel4, io.pins.gpio15);
    let mut motor4 = BrushlessESC::new(&mut channel4, &lstimer0, 2000, 1000, 1000);

    let mut channel5 = ledc.get_channel(ledc::channel::Number::Channel5, io.pins.gpio18);
    let mut motor5 = BrushlessESC::new(&mut channel5, &lstimer0, 2000, 1000, 1000);

    let mut channel6 = ledc.get_channel(ledc::channel::Number::Channel6, io.pins.gpio8);
    let mut motor6 = BrushlessESC::new(&mut channel6, &lstimer0, 2000, 1000, 1000);

    let mut channel7 = ledc.get_channel(ledc::channel::Number::Channel7, io.pins.gpio3);
    let mut motor7 = BrushlessESC::new(&mut channel7, &lstimer0, 2000, 1000, 1000);

    // Wifi AP setup

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = esp_wifi::init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiApDevice).unwrap();

    // init embussy
    {
        let timg1 = TimerGroup::new(peripherals.TIMG1);
        esp_hal_embassy::init(timg1.timer0);
    }

    let config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Address::from_bytes(&[192, 168, 2, 1])),
        dns_servers: Default::default(),
    });

    let seed = 1234; // bleh

    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiApDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();

    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 1536];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("[WIFI] AP up!");

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_millis(500)));

    info!("[SOCK] Created TCP Socket");

    loop {
        info!("[SOCK] Waiting for connection");
        let r = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 6969,
            })
            .await;

        if let Err(e) = r {
            error!("[SOCK] Connection error: {e:?}");
            continue;
        } else {
            info!("[SOCK] Client connected.");
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

        // socket connection loop
        loop {
            let mut buffer = [0; 1536];
            match socket.read(&mut buffer).await {
                Ok(0) => {
                    warn!("[SOCK] read EOF");
                    break;
                }
                Ok(len) => {
                    // Socket API:
                    // [0] - Incomming start byte should be 0x2f, returns 0x2e on return
                    // [1] - Motor 1 speed %
                    // [2] - Motor 2 speed %
                    // [3] - Motor 3 speed %
                    // [4] - Motor 4 speed %
                    // [5] - Motor 5 speed %
                    // [6] - Motor 6 speed %
                    // [7] - Motor 7 speed %
                    // [8] - Motor 8 speed %
                    // [9-11] - Ending bytes '\r\n' or 0x0D0A
                    info!("{}", buffer[0]);
                    if buffer[0] == 0x2f && &buffer[9..11] == b"\r\n" {
                        // Set motor speeds
                        motor0.set_throttle_pct(buffer[1]);
                        motor1.set_throttle_pct(buffer[2]);
                        motor2.set_throttle_pct(buffer[3]);
                        motor3.set_throttle_pct(buffer[4]);
                        motor4.set_throttle_pct(buffer[5]);
                        motor5.set_throttle_pct(buffer[6]);
                        motor6.set_throttle_pct(buffer[7]);
                        motor7.set_throttle_pct(buffer[8]);
                    }
                }
                Err(e) => {
                    error!("[SOCK] read error: {:?}", e);
                    break;
                }
            };

            

            let encoder_val_a = encoder_a.get_angle().await;
            let encoder_val_b = encoder_b.get_angle().await;
            let encoder_val_c = encoder_c.get_angle().await;
            let encoder_val_d = encoder_d.get_angle().await;

            
            // Send encoder values
            match socket
                .write(&[
                    0x2e,
                    encoder_val_a.unwrap_or(0),
                    encoder_val_b.unwrap_or(0),
                    encoder_val_c.unwrap_or(0),
                    encoder_val_d.unwrap_or(0),
                    0x0D,
                    0x0A,
                ])
                .await
            {
                Ok(_) => {}
                Err(err) => {
                    error!("[SOCK] Socket write error: {err:?}",);
                    break;
                }
            };

        }

        warn!("[SOCK] Socket loop exit");

        info!("[ESC] Disabling motors");
        motor0.disable();
        motor1.disable();
        motor2.disable();
        motor3.disable();
        motor4.disable();
        motor5.disable();
        motor6.disable();
        motor7.disable();

        // For some reason this freezes and idk why
        // let r = socket.flush().await;

        // if let Err(e) = r {
        //     error!("[SOCK] Flush error: {e:?}");
        // }

        Timer::after(Duration::from_millis(1000)).await;

        socket.close();
        Timer::after(Duration::from_millis(1000)).await;

        socket.abort();
        info!("[SOCK] Socket closed");
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!(
        "[WIFI] Device capabilities: {:?}",
        controller.get_capabilities()
    );
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::ApStarted {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::ApStop).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        // For some reason, the ability to add a password to the AP has not been implemented yet
        // shouldn't matter cus im getting rid of this soon anyway
        //TODO: Hide AP name?
        //TODO: Change connection limit
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::AccessPoint(AccessPointConfiguration {
                ssid: "i have no internet don't connect".try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start().await.unwrap();
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) {
    stack.run().await
}
