use embedded_hal::pwm::SetDutyCycle;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::LowSpeed;
use esp_hal::ledc::{
        channel::{self, config::PinConfig, Channel},
        timer::TimerIFace,
    };

/// Struct implementation of a standard PWM Brushless ESC
pub struct BrushlessESC<'a> {
    pwm_channel: &'a mut Channel<'a, LowSpeed>,
    max_duty: u16,
    min_duty: u16,
    arm_duty: u16,
}

impl <'a> BrushlessESC<'a> {
    pub fn new(
        pwm_channel: &'a mut Channel<'a, LowSpeed>,
        timer: &'a dyn TimerIFace<LowSpeed>,
        max_micros: u32,
        min_micros: u32,
        arm_micros: u32,
    ) -> Self {
        pwm_channel
            .configure(channel::config::Config {
                timer,
                duty_pct: 0,
                pin_config: PinConfig::PushPull,
            })
            .unwrap();

        let output_max_duty = pwm_channel.max_duty_cycle() as u32;

        // trying to avoid floating point math because computers suck
        // uses weird math that might not work over like 1MHz (or lower idk)
        // but i'm never going to use that high a frequency in this project
        let freq = timer.frequency();

        let period_micros = 1_000_000 / freq;

        let max_duty_pct: u32 = max_micros * 10000 / period_micros;
        let min_duty_pct: u32 = min_micros * 10000 / period_micros;
        let arm_duty_pct: u32 = arm_micros * 10000 / period_micros;

        let max_duty = (output_max_duty * max_duty_pct / 10000) as u16;
        let min_duty = (output_max_duty * min_duty_pct / 10000) as u16;
        let arm_duty = (output_max_duty * arm_duty_pct / 10000) as u16;

        Self {
            pwm_channel,
            max_duty,
            min_duty,
            arm_duty,
        }
    }

    pub fn set_throttle_pct(&mut self, pct: u8) -> Result<(), channel::Error> {
        let duty = self.min_duty + ((self.max_duty - self.min_duty) / (100)) * (pct as u16);
        self.pwm_channel.set_duty_cycle(duty)
    }

    pub fn arm_sig(&mut self) -> Result<(), channel::Error> {
        self.pwm_channel.set_duty_cycle(self.arm_duty)
    }

    pub fn disable(&mut self) -> Result<(), channel::Error> {
        self.pwm_channel.set_duty_cycle_fully_off()
    }
}
