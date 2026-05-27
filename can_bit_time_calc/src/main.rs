use std::fmt;
use std::fmt::{Display, Formatter};
use std::io::{self, Write};
use std::num::{NonZeroU8, NonZeroU16};

fn main() {
    println!("Welcome to blindly trusting the embassy devs");

    let clock_hz = prompt_user_for_frequency(
        "Enter Peripheral Clock Frequency (e.g., 45MHz, 8000000, 16 kHz): ",
        45_000_000, // default
    );

    // 2. Get the Requested CAN Bus Bitrate
    let bitrate_bps = prompt_user_for_frequency(
        "Enter Target CAN Bitrate (e.g., 1Mbps, 500kHz, 1000000): ",
        1_000_000, // default
    );

    println!("--------------------------------------------");

    let res = calc_can_timings(Hertz(clock_hz), bitrate_bps);

    if res.is_err() {
        println!("Could now calculate bit timings.");
        _ = res.inspect_err(|e| eprintln!("{e}"));
    } else {
        println!("Bit timings:\n{}", res.unwrap())
    }
}

fn prompt_user_for_frequency(prompt_text: &str, default_val: u32) -> u32 {
    loop {
        print!("{}", prompt_text);
        let _ = io::stdout().flush(); // Ensure the prompt prints before waiting on input

        let mut input = String::new();
        if io::stdin().read_line(&mut input).is_err() {
            println!("Error reading input. Using default: {}", default_val);
            return default_val;
        }

        let trimmed = input.trim().to_lowercase();

        // Handle empty input by applying the default value safely
        if trimmed.is_empty() {
            println!("Using default: {}", default_val);
            return default_val;
        }

        match parse_units(&trimmed) {
            Some(value) => return value,
            None => {
                println!("❌ Invalid input format.");
                println!(
                    "💡 Help: You can enter raw numbers (45000000) or metric suffixes like 'khz', 'mhz', 'bps', 'kbps', 'mbps'."
                );
            }
        }
    }
}

fn parse_units(raw_input: &str) -> Option<u32> {
    // Strip trailing common symbols/units to find the numeric baseline string
    let clean_numeric: String = raw_input
        .chars()
        .take_while(|c| c.is_numeric() || *c == '.')
        .collect();

    let value_f: f32 = clean_numeric.parse().ok()?;

    // Evaluate unit scale adjustments based on typical context matching strings
    let multiplier =
        if raw_input.contains("mhz") || raw_input.contains("mbps") || raw_input.contains("m") {
            1_000_000.0
        } else if raw_input.contains("khz") || raw_input.contains("kbps") || raw_input.contains("k")
        {
            1_000.0
        } else {
            1.0 // Defaults directly to raw Hz / bps base scalar
        };

    Some((value_f * multiplier) as u32)
}

/// Shared struct to represent bit timings used by calc_can_timings.
#[derive(Clone, Copy, Debug)]
pub struct NominalBitTiming {
    /// Value by which the oscillator frequency is divided for generating the bit time quanta. The bit
    /// time is built up from a multiple of this quanta. Valid values are 1 to 512.
    pub prescaler: NonZeroU16,
    /// Valid values are 1 to 128.
    pub seg1: NonZeroU8,
    /// Valid values are 1 to 255.
    pub seg2: NonZeroU8,
    /// Valid values are 1 to 128.
    pub sync_jump_width: NonZeroU8,
}

impl Display for NominalBitTiming {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "NominalBitTiming {{ prescaler: {}, seg1: {}, seg2: {}, sync_jump_width: {} }}",
            self.prescaler.get(),
            self.seg1.get(),
            self.seg2.get(),
            self.sync_jump_width.get()
        )
    }
}

#[derive(Debug)]
pub enum TimingCalcError {
    /// Bitrate is lower than 1000
    BitrateTooLow {
        /// The set bitrate
        bitrate: u32,
    },
    /// No solution possible
    NoSolution {
        /// The sum of BS1 and BS2
        bs1_bs2_sum: u8,
    },
    /// Prescaler is not 1 < prescaler < 1024
    InvalidPrescaler {
        /// The calculated prescaler value
        prescaler: u32,
    },
    /// BS1 or BS2 are not in the range 0 < BSx < BSx_MAX
    BSNotInRange {
        /// The value of BS1
        bs1: u8,
        /// The value of BS2
        bs2: u8,
    },
    /// Final bitrate doesn't match the requested bitrate
    NoMatch {
        /// The requested bitrate
        requested: u32,
        /// The calculated bitrate
        final_calculated: u32,
    },
    /// core::num::NonZeroUxx::new error
    CoreNumNew,
}

impl Display for TimingCalcError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::BitrateTooLow { bitrate } => {
                write!(f, "Bitrate ({bitrate}) is too low (must be at least 1000)")
            }
            Self::NoSolution { bs1_bs2_sum } => {
                write!(
                    f,
                    "No timing solution possible with BS1 + BS2 sum of {bs1_bs2_sum}"
                )
            }
            Self::InvalidPrescaler { prescaler } => {
                write!(
                    f,
                    "Calculated prescaler value ({prescaler}) is invalid; must be between 1 and 1024"
                )
            }
            Self::BSNotInRange { bs1, bs2 } => {
                write!(
                    f,
                    "Bit segment values out of range: BS1 = {bs1}, BS2 = {bs2} (must be 0 < BSx < BSx_MAX)"
                )
            }
            Self::NoMatch {
                requested,
                final_calculated,
            } => {
                write!(
                    f,
                    "Calculated final bitrate ({final_calculated} bps) does not match the requested bitrate ({requested} bps)"
                )
            }
            Self::CoreNumNew => {
                write!(
                    f,
                    "Failed to create a NonZero integer instance via core::num"
                )
            }
        }
    }
}

#[derive(Eq, PartialEq, Ord, PartialOrd, Clone, Copy, Debug, Default)]
pub struct Hertz(pub u32);

impl Display for Hertz {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} Hz", self.0)
    }
}

/// Calculate nominal CAN bit timing based on CAN bitrate and periphial clock frequency
pub fn calc_can_timings(
    periph_clock: Hertz,
    can_bitrate: u32,
) -> Result<NominalBitTiming, TimingCalcError> {
    const BS1_MAX: u8 = 16;
    const BS2_MAX: u8 = 8;
    const MAX_SAMPLE_POINT_PERMILL: u16 = 900;

    let periph_clock = periph_clock.0;

    if can_bitrate < 1000 {
        return Err(TimingCalcError::BitrateTooLow {
            bitrate: can_bitrate,
        });
    }

    // Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    //      CAN in Automation, 2003
    //
    // According to the source, optimal quanta per bit are:
    //   Bitrate        Optimal Maximum
    //   1000 kbps      8       10
    //   500  kbps      16      17
    //   250  kbps      16      17
    //   125  kbps      16      17
    let max_quanta_per_bit: u8 = if can_bitrate >= 1_000_000 { 10 } else { 17 };

    // Computing (prescaler * BS):
    //   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    //   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    // let:
    //   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    //   PRESCALER_BS = PRESCALER * BS
    // ==>
    //   PRESCALER_BS = PCLK / BITRATE
    let prescaler_bs = periph_clock / can_bitrate;

    // Searching for such prescaler value so that the number of quanta per bit is highest.
    let mut bs1_bs2_sum = max_quanta_per_bit - 1;
    while (prescaler_bs % (1 + bs1_bs2_sum) as u32) != 0 {
        if bs1_bs2_sum <= 2 {
            return Err(TimingCalcError::NoSolution { bs1_bs2_sum }); // No solution
        }
        bs1_bs2_sum -= 1;
    }

    let prescaler = prescaler_bs / (1 + bs1_bs2_sum) as u32;
    if (prescaler < 1) || (prescaler > 1024) {
        return Err(TimingCalcError::InvalidPrescaler { prescaler }); // No solution
    }

    // Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    // We need to find such values so that the sample point is as close as possible to the optimal value,
    // which is 87.5%, which is 7/8.
    //
    //   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    //   {{bs2 -> (1 + bs1)/7}}
    //
    // Hence:
    //   bs2 = (1 + bs1) / 7
    //   bs1 = (7 * bs1_bs2_sum - 1) / 8
    //
    // Sample point location can be computed as follows:
    //   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    //
    // Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    //   - With rounding to nearest
    //   - With rounding to zero
    let mut bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8; // Trying rounding to nearest first
    let mut bs2 = bs1_bs2_sum - bs1;
    core::assert!(bs1_bs2_sum > bs1);

    let sample_point_permill = 1000 * ((1 + bs1) / (1 + bs1 + bs2)) as u16;
    if sample_point_permill > MAX_SAMPLE_POINT_PERMILL {
        // Nope, too far; now rounding to zero
        bs1 = (7 * bs1_bs2_sum - 1) / 8;
        bs2 = bs1_bs2_sum - bs1;
    }

    // Check is BS1 and BS2 are in range
    if (bs1 < 1) || (bs1 > BS1_MAX) || (bs2 < 1) || (bs2 > BS2_MAX) {
        return Err(TimingCalcError::BSNotInRange { bs1, bs2 });
    }

    let calculated = periph_clock / (prescaler * (1 + bs1 + bs2) as u32);
    // Check if final bitrate matches the requested
    if can_bitrate != calculated {
        return Err(TimingCalcError::NoMatch {
            requested: can_bitrate,
            final_calculated: calculated,
        });
    }

    // One is recommended by DS-015, CANOpen, and DeviceNet
    let sync_jump_width = core::num::NonZeroU8::new(1).ok_or(TimingCalcError::CoreNumNew)?;

    let seg1 = core::num::NonZeroU8::new(bs1).ok_or(TimingCalcError::CoreNumNew)?;
    let seg2 = core::num::NonZeroU8::new(bs2).ok_or(TimingCalcError::CoreNumNew)?;
    let nz_prescaler =
        core::num::NonZeroU16::new(prescaler as u16).ok_or(TimingCalcError::CoreNumNew)?;

    Ok(NominalBitTiming {
        sync_jump_width,
        prescaler: nz_prescaler,
        seg1,
        seg2,
    })
}
