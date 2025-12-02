// use defmt::todo;
// use embassy_stm32::adc::{self, Adc, AdcChannel, AnyAdcChannel, SampleTime};
// use embassy_stm32::peripherals::ADC1;
// use embassy_time::{Duration, with_timeout};
//
// pub async fn adc_read<I: adc::Instance>(adc: Adc<'static, I>, ch: AnyAdcChannel<ADC1>) {
//     let vref = adc.enable_vref();
//     adc.set_sample_time(SampleTime::CYCLES7_5);
//     adc.read(ch, SampleTime::CYCLES7_5);
//     todo!();
//     // ch.degrade_adc()
// }
