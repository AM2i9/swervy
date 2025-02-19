use core::f32::consts::PI;

use esp_hal::time;

use crate::{
    encoder::MuxedEncoder,
    esc::{BrushlessESC, ESCB},
    pid::PIDController,
    util::deadzone,
};

pub struct SwerveModule<'a> {
    drive: BrushlessESC<'a>,
    steer: ESCB<'a>,
    encoder: MuxedEncoder<'a>,
    pid: PIDController,
    wheelspeed: f32,
}

impl<'a> SwerveModule<'a> {
    pub fn new(
        drive: BrushlessESC<'a>,
        steer: ESCB<'a>,
        encoder: MuxedEncoder<'a>,
        pid: PIDController,
    ) -> Self {
        Self {
            drive,
            steer,
            encoder,
            pid,
            wheelspeed: 0.0,
        }
    }

    pub async fn get_angle(&mut self) -> f32 {
        return self.encoder.get_angle_radians().await.unwrap_or(0.0);
    }

    pub async fn zero(&mut self) {
        self.encoder.set_offset(None);
        let angle = self.encoder.get_raw_angle().await.unwrap_or(0);
        self.encoder.set_offset(Some(angle));
    }

    pub fn set_angle(&mut self, angle: f32) {
        self.pid.setpoint(angle);
    }

    pub fn get_setpoint(&self) -> f32 {
        self.pid.setpoint
    }

    pub fn set_wheel_speed(&mut self, speed: f32) {
        self.wheelspeed = speed;
    }

    pub fn disable(&mut self) {
        self.drive.disable();
        self.steer.disable();
    }

    pub fn arm(&mut self) {
        self.drive.arm_sig();
    }

    pub fn center_throttles(&mut self) {
        self.drive.set_throttle_pct(50);
        self.steer.set_throttle_pct(0.0);
    }

    pub async fn periodic(&mut self) {
        let timestamp = time::now();
        let mut measurement = self.get_angle().await;
        let mut drive_dir = 1.0;

        // Optimize steering and drive direction
        {
            let mut diff = libm::fabsf(measurement - self.pid.setpoint);
            if diff > PI {
                measurement -= PI;
                diff = libm::fabsf(measurement - self.pid.setpoint);
            }
            if diff > (PI / 2.0) {
                drive_dir *= -1.0;
                measurement -= PI;
            }

            // Add a manual deadzone to the measurement to prevent ocillation when close
            measurement = deadzone(
                measurement,
                self.pid.setpoint - 0.1,
                self.pid.setpoint + 0.1,
                Some(self.pid.setpoint),
            );
        }

        let pid_out = self.pid.calculate(measurement, Some(timestamp));
        let steer_out = pid_out;

        self.drive
            .set_throttle_pct((50.0 + (20.0 * self.wheelspeed * drive_dir)) as u8);
        self.steer.set_throttle_pct(steer_out);
    }
}
