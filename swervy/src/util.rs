pub fn signum(num: f32) -> f32 {
    if num > 0.0 {
        1.0
    } else if num < 0.0 {
        -1.0
    } else {
        0.0
    }
}

pub fn mag_min(a: f32, b: f32) -> f32 {
    if libm::fabsf(a) < libm::fabsf(b) {
        a
    } else {
        b
    }
}

pub fn deadzone(x: f32, min: f32, max: f32, zero: Option<f32>) -> f32 {
    if min < x && x < max {
        zero.unwrap_or(0.0)
    } else {
        x
    }
}
