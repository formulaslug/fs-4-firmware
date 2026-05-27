#![no_std]
#![no_main]

// Bare minimum deps
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    can::frame::Header,
    gpio::{Level, Output, Speed},
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// CAN deps
use embassy_stm32::can;
use embassy_stm32::can::frame::Envelope;
use embassy_stm32::can::*; // This is the module
use embassy_stm32::peripherals::FDCAN1; // THis is just the peripheral struct
use embassy_stm32::{Config, bind_interrupts};
use embedded_can::{Id, StandardId};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    // CAN configuration on C092 (likely the same on all FDCAN peripherals)
    let mut hscan = can::CanConfigurator::new(p.FDCAN1, p.PB0, p.PB1, Irqs); // RX, TX
    hscan.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::reject_all(), // Reject all incoming messages at hardware level to avoid interrupts
    );
    hscan.set_bitrate(2_000_000);
    let mut hscan = hscan.start(can::OperatingMode::NormalOperationMode);

    loop {
        info!("Loop");

        let frame = can::frame::Frame::new(
            Header::new(Id::Standard(unwrap!(StandardId::new(0x120))), 8, false),
            &[0; 8],
        )
        .unwrap();

        _ = hscan.write(&frame).await; // If the TX FIFO is full, then this will return the frame, but usually you don't care.
    }
}
