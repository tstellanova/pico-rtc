#![no_std]
#![no_main]

extern crate rv3028c7_rtc;

use chrono::{Duration, NaiveTime};
use core::ops::Add;
use rv3028c7_rtc::{RV3028, DateTimeAccess, NaiveDateTime, NaiveDate};

use embedded_hal::blocking::i2c::{Write, Read, WriteRead};

use cortex_m::delay::Delay;

/// Tested with HS-probe
/// https://github.com/probe-rs/hs-probe
/// Connected to rp2040 in (Rpi Pico form)

// use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;
use rp2040_hal::{
    Clock,
    fugit::RateExtU32,
    gpio::{FunctionI2C, Pin, PullUp},
    pac,
    // pac::I2C0,
    rtc::{self,  RealTimeClock},
};


/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
      .ok()
      .unwrap();

    // let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ).unwrap();
    // clocks.rtc_clock.configure_clock(&pac.XOSC, 46875.Hz()).unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut sys_rtc = hal::rtc::RealTimeClock::new(
        pac.RTC,
        clocks.rtc_clock,
        &mut pac.RESETS,
        rtc::DateTime {
            year: 2023,
            month: 12,
            day: 8,
            day_of_week: rtc::DayOfWeek::Friday,
            hour: 15,
            minute: 45,
            second: 0,
        },
    ).unwrap();

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio5.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Create a new instance of the RV3028 driver
    let mut rtc = RV3028::new(i2c);

    // use the set_datetime method to ensure all the timekeeping registers on
    // the rtc are aligned to the same values
    let dt_sys = rpi_rtc_datetime_to_duration(&sys_rtc.now().unwrap());
    rtc.set_datetime(&dt_sys).unwrap();


    // Note that all RTC durations have a minimum uncertainty in the
    // first period duration: this is between 244 microseconds and 15 milliseconds.
    // Add to that some variation in how the host (linux) platform
    // polls for RTC countdown timer completion.

    info!("\r\n==== ONE SHOTS ====");
    test_one_shot_duration(&mut rtc, &Duration::microseconds(488) , &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::milliseconds(15 * (1000/15)), &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::seconds(1), &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::seconds(2), &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::seconds(3), &mut delay, &mut sys_rtc).unwrap();
    // test_one_shot_duration(&mut rtc, &Duration::minutes(1)).unwrap();

    info!("\r\n==== PERIODICS ====");
    test_periodic_duration(&mut rtc, &Duration::microseconds(488), &mut delay, &mut sys_rtc).unwrap();
    test_periodic_duration(&mut rtc, &Duration::milliseconds(15 * (1000/15)), &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::seconds(1), &mut delay, &mut sys_rtc).unwrap();
    test_one_shot_duration(&mut rtc, &Duration::seconds(2), &mut delay, &mut sys_rtc).unwrap();
    test_periodic_duration(&mut rtc, &Duration::seconds(3), &mut delay, &mut sys_rtc).unwrap();
    // test_periodic_duration(&mut rtc, &Duration::minutes(1)).unwrap();


    loop {
        cortex_m::asm::wfi();
    }
}


fn rpi_rtc_datetime_to_duration(sys_dt: &rtc::DateTime) -> chrono::NaiveDateTime {
    NaiveDateTime::new(
        NaiveDate::from_ymd_opt(
            sys_dt.year as i32, sys_dt.month as u32, sys_dt.day as u32).unwrap(),
        NaiveTime::from_hms_opt(
            sys_dt.hour as u32, sys_dt.minute as u32 , sys_dt.second as u32).unwrap()
    )

}

fn test_one_shot_duration<I2C,E>(
    rtc: &mut RV3028<I2C>, duration: &Duration, delay: &mut Delay, sys_rtc: &mut RealTimeClock) -> Result<Duration, E>
    where
      I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
      E: core::fmt::Debug
{
    rtc.clear_all_int_out_bits()?;
    rtc.toggle_countdown_timer(false)?;
    rtc.check_and_clear_countdown()?;
    let estimated_duration = rtc.config_countdown_timer(duration, false, false)?;
    // account for supposed maximum uncertainty in RTC durations
    let expected_sleep =
      if estimated_duration.le(&Duration::microseconds(4096 * 245)){
          estimated_duration.add(Duration::microseconds(245) )
      }
      else {
          estimated_duration.add(Duration::milliseconds(16) )
      };

    let start_time =    rpi_rtc_datetime_to_duration( &sys_rtc.now().unwrap());
    rtc.toggle_countdown_timer(true)?;
    delay.delay_ms(estimated_duration.num_milliseconds().try_into().unwrap());

    let actual = loop {
        let remain = rtc.get_countdown_value()?;
        if 0 == remain {
            let triggered = rtc.check_and_clear_countdown()?;
            if !triggered { println!("Counter zero but PERIODIC_TIMER_FLAG untriggered!!")}
            let end_time = rpi_rtc_datetime_to_duration(&sys_rtc.now().unwrap());
            let delta = end_time - start_time;
            break delta;
        }
        else {
            // println!("remain: {}", remain);
            // 15.625 ms uncertainty
            delay.delay_ms(1);
            // std::thread::sleep(Duration::milliseconds(1).to_std().unwrap());
        }
    };

    let delta_total = actual - estimated_duration ;
    println!("oneshot duration {} us , delta: {} us",
             duration.num_microseconds(), delta_total.num_microseconds());
    // println!("< actual {} expected {} requested {} delta {}", actual, estimated_duration, duration, delta_total);
    Ok(actual)
}

fn test_periodic_duration<I2C,E>(
    rtc: &mut RV3028<I2C>, duration: &Duration, delay: &mut Delay, sys_rtc: &mut RealTimeClock) -> Result<(), E>
    where
      I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
      E: core::fmt::Debug
{
    rtc.clear_all_int_out_bits()?;
    rtc.toggle_countdown_timer(false)?;
    rtc.check_and_clear_countdown()?;

    let estimated_duration = rtc.config_countdown_timer(duration, true, false)?;
    // we don't adjust for the first duration uncertainty, assuming it will average out
    let expected_sleep =    estimated_duration;
    // println!("> periodic {} sleep {} ", duration, expected_sleep);

    // place to store the sum of all measured countdown durations
    let mut sum_actual = Duration::zero();
    const NUM_ITERATIONS: usize = 10;

    // start the countdown repeating
    let mut start_time = rpi_rtc_datetime_to_duration(&sys_rtc.now().unwrap()); // Utc::now().naive_utc();
    rtc.toggle_countdown_timer(true)?;
    for _i in 0..NUM_ITERATIONS {
        delay.delay_ms(estimated_duration.num_milliseconds().try_into().unwrap());

        let actual = loop {
            let triggered = rtc.check_and_clear_countdown()?;
            if triggered {
                let end_time = rpi_rtc_datetime_to_duration( &sys_rtc.now().unwrap());
                let delta = end_time - start_time;
                start_time = end_time; //reset timer for next event
                break delta;
            }
        };

        // println!("actual {} expected {}", actual, duration);
        sum_actual = sum_actual + actual;
    }

    let avg_actual = sum_actual / NUM_ITERATIONS as i32;
    let delta_total = avg_actual - estimated_duration ;
    println!("periodic duration {} us, avg_delta: {} us",
             duration.num_microseconds(), delta_total.num_microseconds());


    Ok(())
}

