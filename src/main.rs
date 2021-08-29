#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate nb;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use hal::time::Hertz;
use nb::block;
use rt::entry;
use hal::i2c::Config;
use sh::hprintln;
use hdc1080_arm_rs::Hdc1080;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    //let buf: [u8; 2] = [0,0];
    let mut delay = dp.TIM14.delay(&mut rcc);
    let mut delay_local = dp.TIM16.delay(&mut rcc);

    let mut led = gpioc.pc6.into_push_pull_output();
    let sda = gpioa.pa10.into_open_drain_output();
    let scl = gpioa.pa9.into_open_drain_output();
    hprintln!("Pins delagated");
    let mut timer = dp.TIM17.timer(&mut rcc);

    hprintln!("timer start");

    let conf:Config = Config::new(100.khz());//with_timing(0x2020_151b)

    delay_local.delay(20.ms());
    let mut i2c = dp
        .I2C1
        .i2c(sda, scl, conf, &mut rcc);//new(100.khz())
    let mut dev = Hdc1080::new(i2c, delay).unwrap();
    hprintln!("device created");

    delay_local.delay(20.ms());
    hprintln!("man id {}", dev.get_man_id().unwrap());


    timer.start(50.ms());
    loop {
        led.toggle().unwrap();
        delay_local.delay(20.ms());
        hprintln!("temperature {}", dev.temperature().unwrap());

        // match i2c.write(0x40, &[0xff]) {
        //     Ok(_) => hprintln!("ok{:?}",buf).unwrap(),
        //     Err(err) => hprintln!("error: {:?}", err).unwrap(),
        // }

        //led.set_low().unwrap();

        block!(timer.wait()).unwrap();
    }
}