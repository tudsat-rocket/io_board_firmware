use crate::high_current_out::{HcoController, HighCurrentOutput, Level};
use embassy_time::{Duration, Ticker};

// in the future there could be angle offset calibration added here
pub struct CalibServo {
    min_us: u16,
    max_us: u16,
    /// total angle in deci degrees
    total_angle: u16,
}

impl CalibServo {
    fn new_270_deg() -> Self {
        Self {
            min_us: 500,
            max_us: 2500,
            total_angle: 2700,
        }
    }
    /// angle in deci degrees to pwm micro seconds
    fn unilateral_angle(&self, angle_deci: u16) -> u16 {
        let angle = angle_deci / 10;
        defmt::info!("angle: {}", angle);
        debug_assert!(angle_deci <= self.total_angle);
        let angle_deci = angle_deci.clamp(0, self.total_angle);
        debug_assert!(self.min_us < self.max_us);
        let span = self.max_us - self.min_us;

        let pwm = self.min_us + u16::try_from(((span as u32) * (angle_deci as u32)) / self.total_angle as u32).unwrap();
        debug_assert!(self.min_us <= pwm && pwm <= self.max_us);
        pwm
    }

    /// angle in deci degrees, centered around midpoint (positive or negative)
    fn bilateral_angle(&self, angle_deci: i16) -> u16 {
        let half = (self.total_angle / 2) as i16;
        debug_assert!(angle_deci >= -half && angle_deci <= half);
        let angle_deci = angle_deci.clamp(-half, half);
        let unilateral = (half + angle_deci) as u16;
        self.unilateral_angle(unilateral)
    }
}

#[embassy_executor::task]
pub async fn run_ereg(mut hco: HcoController) {
    let servo = CalibServo::new_270_deg();

    // NOTE: when using the 3pin header set (hco4 to signal) (hco3 to power)
    let pwm = HighCurrentOutput::_3;
    let power = HighCurrentOutput::_4;
    hco.set_level(power, Level::High);

    let mut ticker = Ticker::every(Duration::from_secs(5));

    loop {
        hco.set_pwm_micros(pwm, servo.bilateral_angle(-1300));
        defmt::info!("low");
        ticker.next().await;
        hco.set_pwm_micros(pwm, servo.bilateral_angle(1300));
        defmt::info!("high");
        ticker.next().await;
    }
}
