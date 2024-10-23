use fugit::Instant;


#[derive(Default)]
pub struct PIDConstants{
    pub kp: f32,
    pub ki: f32,
    pub kd: f32
}

/// Basic PID controller implementation
pub struct PIDController {
    constants: PIDConstants,
    pub setpoint: f32,
    integral: f32,
    prev_time: Option<Instant<u64, 1, 1_000_000>>,
    prev_err: f32,
}

impl PIDController {
    pub fn new(constants: PIDConstants) -> Self {
        PIDController{
            constants,
            setpoint: 0.0,
            integral: 0.0,
            prev_time: None,
            prev_err: 0.0
        }
    }

    pub fn setpoint(&mut self, value: f32) {
        self.setpoint = value;
    }

    pub fn calculate(&mut self, measurement: f32, micros: Option<Instant<u64, 1, 1_000_000>>) -> f32 {

        let err = measurement - self.setpoint;

        let p = err * self.constants.kp;
        
        let (d, i) = if let (Some(then), Some(now)) = (self.prev_time, micros) {
                let diff = now - then;
                self.prev_time = Some(now);

                // this type cast may (will definitely) lead to problems sometime in the future
                // but right now i couldn't give more of a fuck
                let time_err = diff.to_micros() as f32;

                (
                    // D
                    ((err - self.prev_err)/time_err) * self.constants.kd,
                    //I
                    self.integral + (err * self.constants.ki * time_err)
                )
        } else {
            (0.0, 0.0)
        };

        self.prev_err = err;

        p + d + i


    }
}

