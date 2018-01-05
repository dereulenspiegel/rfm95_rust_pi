extern crate futures;
extern crate rf95pi;
extern crate tokio_core;

use tokio_core::reactor::Core;
use futures::stream::Stream;
use futures::Future;

use rf95pi::{Bandwidth, Channel, CodingRate, DioFunction, LoraMode, PacketStream, RF95,
             SpreadingFactor};

const DIO_BCM_PIN: u64 = 4;

pub fn main() {
    let mut module = RF95::new(
        "/dev/spidev0.0",
        Bandwidth::Bw125,
        CodingRate::Cr8,
        SpreadingFactor::Sf12,
    ).unwrap();
    let version = module.chip_version().unwrap();
    println!("Lora module version: {}", version.as_str());

    module.set_channel(Channel::Ch10).unwrap();
    module.set_coding_rate(CodingRate::Cr5).unwrap();
    module.set_spreading_factor(SpreadingFactor::Sf7).unwrap();
    module.set_bandwidth(Bandwidth::Bw125).unwrap();
    module.set_mode(LoraMode::Rx).unwrap();
    // module.set_dio_mapping(DioFunction::RxDone).unwrap();

    let mut core = Core::new().unwrap();

    let packets = PacketStream::new(DIO_BCM_PIN, &mut module, &core.handle()).unwrap();

    let done = packets.for_each(|lora_pkt| {
        println!("Received LoRa packet {:?}", lora_pkt);
        Ok(())
    });
    println!("Waiting for LoRa packets");
    core.run(done).unwrap();
    println!("Done receiving packets");
}
