extern crate base64;
extern crate spidev;
extern crate sysfs_gpio;
extern crate time;

extern crate futures;
extern crate tokio_core;

use sysfs_gpio::{Direction, Edge, Pin, PinValueStream};

use spidev::{SPI_MODE_0, Spidev, SpidevOptions};

use std::io;
use std::vec::Vec;
use std::time::Duration;
use std::io::{ErrorKind, Read, Write};
use std::sync::{Arc, Mutex};
use std::fmt::{Debug, Formatter};
use base64::display::Base64Display;

use time::{now, Tm};

use futures::prelude::*;
use tokio_core::reactor::Handle;

const RST_BCM_PIN: u64 = 17;
const CS_BCM_PIN: u64 = 25;

#[derive(Copy, Clone)]
pub enum LoraRegister {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    // reserved : 0x02 - 0x05
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,
    RegFifoAddrPtr = 0x0D,
    RegFifoTxBaseAddr = 0x0E,
    RegFifoRxBaseAddr = 0x0F,
    RegFifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegRxHeaderCntValueMsb = 0x14,
    RegRxHeaderCntValueLsb = 0x15,
    RegRxPacketCntValueMsb = 0x16,
    RegRxPacketCntValueLsb = 0x17,
    RegModemStat = 0x18,
    RegPktSnrValue = 0x19,
    RegPktRssiValue = 0x1A,
    RegRssiValue = 0x1B,
    RegHopChannel = 0x1C,
    RegModemConfig1 = 0x1D,
    RegModemConfig2 = 0x1E,
    RegSymbTimeoutLsb = 0x1F,
    RegPreambleMsb = 0x20,
    RegPreambleLsb = 0x21,
    RegPayloadLength = 0x22,
    RegMaxPayloadLength = 0x23,
    RegHopPeriod = 0x24,
    RegFifoRxByteAddr = 0x25,
    RegModemConfig3 = 0x26,
    // reserved : 0x27-0x3F
    RegBroadcastAdrs = 0x34,
    RegDioMapping1 = 0x40,
    RegDioMapping2 = 0x41,
    RegVersion = 0x42,
    // unused 0x44
    RegTcxo = 0x4B,
    RegPaDac = 0x4D,
    RegFormerTemp = 0x5B,
    RegAgcRef = 0x61,
    RegAgcThresh1 = 0x62,
    RegAgcThresh2 = 0x63,
    RegAgcThresh3 = 0x64,
}

impl LoraRegister {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone, Debug)]
pub enum FskOokRegister {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegBitRateMsb = 0x02,
    RegBitRateLsb = 0x03,
    RegFdevMsb = 0x04,
    RegFdevLsb = 0x05,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,
    RegRxConfig = 0x0D,
    RegRssiConfig = 0x0E,
    RegRssiCollision = 0x0F,
    RegRssiThresh = 0x10,
    RegRssiValue = 0x11,
    RegRxBw = 0x12,
    RegAfcBw = 0x13,
    RegOokPeak = 0x14,
    RegOokFix = 0x15,
    RegOokAvg = 0x16,
    RegAfcFei = 0x1A,
    RegAfcMsb = 0x1B,
    RegAfcLsb = 0x1C,
    RegFeiMsb = 0x1D,
    RegFeiLsb = 0x1E,
    RegPreambleDetect = 0x1F,
    RegRxTimeout1 = 0x20,
    RegRxTimeout2 = 0x21,
    RegRxTimeout3 = 0x22,
    RegRxDelay = 0x23,
    RegOsc = 0x24,
    RegPreambleMsb = 0x25,
    RegPreambleLsb = 0x26,
    RegSyncConfig = 0x27,
    RegSyncValue1 = 0x28,
    RegSyncValue2 = 0x29,
    RegSyncValue3 = 0x2A,
    RegSyncValue4 = 0x2B,
    RegSyncValue5 = 0x2C,
    RegSyncValue6 = 0x2D,
    RegSyncValue7 = 0x2E,
    RegSyncValue8 = 0x2F,
    RegPacketconfig1 = 0x30,
    RegPacketConfig2 = 0x31,
    RegPayloadLength = 0x32,
    RegNodeAdrs = 0x33,
    RegBroadcastAdrs = 0x34,
    RegFifoThresh = 0x35,
    RegSeqConfig1 = 0x36,
    RegSeqConfig2 = 0x37,
    RegTimerResol = 0x38,
    RegTimer1Coef = 0x39,
    RegTimer2Coef = 0x3A,
    RegImageCal = 0x3B,
    RegTemp = 0x3C,
    RegLowBat = 0x3D,
    RegIrqFlags1 = 0x3E,
    RegIrqFlags2 = 0x3F,
    RegDioMapping1 = 0x40,
    RegDioMapping2 = 0x41,
    RegVersion = 0x42,
    RegPllHop = 0x44,
    RegTxco = 0x4B,
    RegPaDac = 0x4D,
    RegFormerTemp = 0x5B,
    RegBitRateFrac = 0x5D,
    RegAgcRef = 0x61,
    RegAgcThresh1 = 0x62,
    RegAgcThresh2 = 0x63,
    RegAgcThresh3 = 0x64,
}

impl FskOokRegister {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum Channel {
    Ch10 = 0xD84CCC,
    Ch11 = 0xD86000,
    Ch12 = 0xD87333,
    Ch13 = 0xD88666,
    Ch14 = 0xD89999,
    Ch15 = 0xD8ACCC,
    Ch16 = 0xD8C000,
    Ch17 = 0xD90000,
}

impl Channel {
    pub fn msb(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        ((chan_as_u8 >> 16) & 0x000000FF) as u8
    }

    pub fn mid(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        ((chan_as_u8 >> 8) & 0x000000FF) as u8
    }

    pub fn lsb(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        (chan_as_u8 & 0x000000FF) as u8
    }
}

#[derive(Copy, Clone)]
pub enum Bandwidth {
    Bw125 = 0x07,
    Bw250 = 0x08,
    Bw500 = 0x09,
}

impl Bandwidth {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum CodingRate {
    Cr5 = 0x01,
    Cr6 = 0x02,
    Cr7 = 0x03,
    Cr8 = 0x04,
}

impl CodingRate {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum SpreadingFactor {
    Sf7 = 0x07,
    Sf8 = 0x08,
    Sf9 = 0x09,
    Sf10 = 0x0A,
    Sf11 = 0x0B,
    Sf12 = 0x0C,
}

impl SpreadingFactor {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum LoraMode {
    Sleep = 0x80,
    Standby = 0x81,
    Tx = 0x83,
    Rx = 0x85,
    RxSingle = 0x86,
    CadDetection = 0x87,
    StandbyOokFsk = 0xC1,
}

impl LoraMode {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum DioFunction {
    RxDone = 0x00,
    TxDone = 0x01,
}

impl DioFunction {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

trait ToFlag {
    fn flag_enabled(&self, f: IrqFlagMasks) -> bool;
}

#[derive(Copy, Clone)]
pub enum IrqFlagMasks {
    CadDetected = 0x01,
    FhssChangeChannel = 0x02,
    CadDone = 0x04,
    TxDone = 0x08,
    ValidHeader = 0x10,
    PayloadCrcError = 0x20,
    RxDone = 0x40,
    RxTimeout = 0x80,
}

impl IrqFlagMasks {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

impl ToFlag for u8 {
    fn flag_enabled(&self, f: IrqFlagMasks) -> bool {
        self & f.as_u8() != 0
    }
}

pub enum RF95EventType {
    None,
    DataReceived,
    DataSent,
    ErrorPinConfig,
    ErrorTimedOut,
    ErrorWrongCrc,
    ErrorCommBus,
}

#[derive(Copy, Clone, Debug)]
pub enum RF95Type {
    Unknown = 0x00,
    SX1272 = 0x22,
    SX1276 = 0x12,
}

impl RF95Type {
    pub fn as_str(&self) -> &str {
        match self {
            &RF95Type::SX1272 => "SX1272",
            &RF95Type::SX1276 => "SX1276",
            &RF95Type::Unknown => "Unknown",
        }
    }
}

pub struct LoraPacket {
    payload: Vec<u8>,
    rssi: i16,
    snr: i16,
    crc: bool,
    rx_time: Tm,
}

impl Debug for LoraPacket {
    fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
        let hex_payload = Base64Display::standard(self.payload.as_slice());
        write!(
            f,
            "LoRa Packet [payload: {}, rssi: {}, snr: {}, crc: {}, rx_time: {:?} ]",
            hex_payload, self.rssi, self.snr, self.crc, self.rx_time
        )
    }
}

trait Hw {
    fn reset(&mut self) -> io::Result<()>;
    fn reset_high(&mut self) -> io::Result<()>;
    fn select(&mut self) -> io::Result<()>;
    fn unselect(&mut self) -> io::Result<()>;
    fn read_register(&mut self, reg: LoraRegister) -> io::Result<u8>;
    fn write_register(&mut self, reg: LoraRegister, val: u8) -> io::Result<()>;
}

struct HwSpi {
    rst_pin: Pin,
    css_pin: Pin,
    spi_dev: Spidev,
}

impl HwSpi {
    fn new(spi_path: &str, rst: u64, css: u64) -> io::Result<HwSpi> {
        let rst_pin = Pin::new(rst);
        match rst_pin.export() {
            Ok(()) => (),
            Err(_e) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Can't export reset pin",
                ))
            }
        }
        match rst_pin.set_direction(Direction::Out) {
            Ok(()) => (),
            Err(e) => {
                println!("Failed to set pin direction for reset {:?}", e);
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Can't set direction of reset pin",
                ));
            }
        }

        let css_pin = Pin::new(css);
        match css_pin.export() {
            Ok(()) => (),
            Err(_e) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Can't export chip select pin",
                ));
            }
        }
        match css_pin.set_direction(Direction::Out) {
            Ok(()) => (),
            Err(e) => {
                println!("Failed to set pin direction for css {:?}", e);
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Can't set direction of chip select pin",
                ));
            }
        }

        let mut spi_dev = match Spidev::open(spi_path) {
            Ok(v) => v,
            Err(e) => {
                println!("Failed to open SPI device: {}", e);
                return Err(io::Error::new(ErrorKind::Other, "Can't open SPI device"));
            }
        };

        let spi_options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(5_000_000)
            .mode(SPI_MODE_0)
            .lsb_first(false)
            .build();

        match spi_dev.configure(&spi_options) {
            Err(e) => return Err(e),
            Ok(()) => (),
        };

        return Ok(HwSpi {
            rst_pin: rst_pin,
            css_pin: css_pin,
            spi_dev: spi_dev,
        });
    }
}

impl Hw for HwSpi {
    fn select(&mut self) -> io::Result<()> {
        match self.css_pin.set_value(0) {
            Ok(()) => Ok(()),
            Err(_) => Err(io::Error::new(ErrorKind::Other, "Can't select chip")),
        }
    }

    fn unselect(&mut self) -> io::Result<()> {
        match self.css_pin.set_value(1) {
            Ok(()) => Ok(()),
            Err(_) => Err(io::Error::new(ErrorKind::Other, "Can't unselect chip")),
        }
    }

    fn reset(&mut self) -> io::Result<()> {
        match self.rst_pin.set_value(1) {
            Ok(()) => (),
            Err(_) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to set reset pin high",
                ))
            }
        }
        std::thread::sleep(Duration::from_millis(100));
        match self.rst_pin.set_value(0) {
            Ok(()) => (),
            Err(_) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to set reset pin low",
                ))
            }
        }
        std::thread::sleep(Duration::from_millis(100));
        return Ok(());
    }

    fn reset_high(&mut self) -> io::Result<()> {
        match self.rst_pin.set_value(0) {
            Ok(()) => (),
            Err(_) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to set reset pin low",
                ))
            }
        }
        std::thread::sleep(Duration::from_millis(100));
        match self.rst_pin.set_value(1) {
            Ok(()) => (),
            Err(_) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to set reset pin high",
                ))
            }
        }
        std::thread::sleep(Duration::from_millis(100));
        return Ok(());
    }

    fn read_register(&mut self, reg: LoraRegister) -> io::Result<u8> {
        match self.select() {
            Ok(()) => (),
            Err(_e) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to select device",
                ))
            }
        }

        let tx = [reg.as_u8() & 0x7F];
        let mut rx = [0_u8; 1];

        self.spi_dev.write(&tx).unwrap();
        self.spi_dev.read(&mut rx).unwrap();

        match self.unselect() {
            Ok(()) => (),
            Err(_e) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to unselect chip",
                ))
            }
        }

        return Ok(rx[0]);
    }

    fn write_register(&mut self, reg: LoraRegister, val: u8) -> io::Result<()> {
        match self.select() {
            Ok(()) => (),
            Err(_e) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to select device",
                ))
            }
        }

        let tx = [reg.as_u8() | 0x80, val];

        self.spi_dev.write(&tx).unwrap();

        match self.unselect() {
            Ok(()) => Ok(()),
            Err(_e) => {
                return Err(std::io::Error::new(
                    ErrorKind::Other,
                    "Failed to unselect chip",
                ))
            }
        }
    }
}

pub struct RF95 {
    ch: Channel,
    bw: Bandwidth,
    cr: CodingRate,
    sf: SpreadingFactor,
    current_mode: LoraMode,
    crc_check_enabled: bool,
    implicit_header_enabled: bool,
    pwr_db: u8,
    chip_type: RF95Type,

    hw: Arc<Mutex<HwSpi>>,
}

impl RF95 {
    pub fn new(
        spi_path: &str,
        bw: Bandwidth,
        cr: CodingRate,
        sf: SpreadingFactor,
    ) -> io::Result<RF95> {
        let mut rfhw = match HwSpi::new(spi_path, RST_BCM_PIN, CS_BCM_PIN) {
            Ok(v) => v,
            Err(e) => return Err(e),
        };

        match rfhw.unselect() {
            Ok(()) => (),
            Err(e) => return Err(e),
        }

        let mut rf95 = RF95 {
            ch: Channel::Ch10,
            bw,
            cr,
            sf,
            current_mode: LoraMode::Standby,
            chip_type: RF95Type::Unknown,
            hw: Arc::new(Mutex::new(rfhw)),
            crc_check_enabled: false,
            implicit_header_enabled: false,
            pwr_db: 0,
        };

        rf95.chip_type = rf95.read_version()?;

        Ok(rf95)
    }

    pub fn chip_version(&mut self) -> io::Result<RF95Type> {
        return Ok(self.chip_type);
    }

    fn read_version(&mut self) -> io::Result<RF95Type> {
        // TODO have real error handling here
        let mut rfhw = self.hw.lock().unwrap();
        rfhw.reset()?;
        let v = rfhw.read_register(LoraRegister::RegVersion)?;
        if v == 0x22 {
            return Ok(RF95Type::SX1272);
        } else {
            rfhw.reset_high()?;
            let v = rfhw.read_register(LoraRegister::RegVersion)?;
            if v == 0x12 {
                return Ok(RF95Type::SX1276);
            }
        }
        return Err(io::Error::new(
            ErrorKind::Other,
            "Invalid Transceiver version",
        ));
    }

    fn read_register(&mut self, reg: LoraRegister) -> io::Result<u8> {
        match self.hw.lock() {
            Ok(mut rfhw) => rfhw.read_register(reg),
            Err(_) => Err(io::Error::new(
                ErrorKind::Other,
                "Failed to acquire hardware lock",
            )),
        }
    }

    fn write_register(&mut self, reg: LoraRegister, val: u8) -> io::Result<()> {
        match self.hw.lock() {
            Ok(mut rfhw) => rfhw.write_register(reg, val),
            Err(_) => Err(io::Error::new(
                ErrorKind::Other,
                "Failed to acquire hardware lock",
            )),
        }
    }

    pub fn set_mode(&mut self, m: LoraMode) -> io::Result<()> {
        self.current_mode = m;
        self.write_register(LoraRegister::RegOpMode, m.as_u8())
    }

    pub fn get_packet_snr(&mut self) -> io::Result<i16> {
        match self.read_register(LoraRegister::RegPktSnrValue) {
            Ok(val) => Ok((val as i16) / 4),
            Err(e) => Err(e),
        }
    }

    pub fn get_packet_rssi(&mut self) -> io::Result<i16> {
        match self.read_register(LoraRegister::RegRssiValue) {
            Ok(val) => Ok((val as i16) - 137),
            Err(e) => Err(e),
        }
    }

    pub fn get_rssi(&mut self) -> io::Result<i16> {
        match self.read_register(LoraRegister::RegRssiValue) {
            Ok(val) => Ok((val as i16) - 137),
            Err(e) => Err(e),
        }
    }

    pub fn set_output_power(&mut self, pwr: u8) -> io::Result<()> {
        let mut pwr = pwr;
        if pwr > 20 {
            pwr = 20;
        }
        self.pwr_db = pwr;
        let out = (pwr - 2) & 0x0F;
        let mut tmp = self.read_register(LoraRegister::RegPaConfig)?;
        tmp |= 0x80;
        tmp &= 0xF0;
        tmp |= out & 0x0F;
        self.write_register(LoraRegister::RegPaConfig, tmp)
    }

    pub fn set_dio_mapping(&mut self, df: DioFunction) -> io::Result<()> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };

        let prev_mode = rfhw.read_register(LoraRegister::RegOpMode)?;
        rfhw.write_register(LoraRegister::RegOpMode, LoraMode::StandbyOokFsk.as_u8())?;
        let mut tmp = rfhw.read_register(LoraRegister::RegDioMapping1)?;
        tmp &= 0x3F;
        if df.as_u8() == DioFunction::TxDone.as_u8() {
            tmp |= 0x40;
        }
        rfhw.write_register(LoraRegister::RegDioMapping1, tmp)?;
        rfhw.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    pub fn set_channel(&mut self, ch: Channel) -> io::Result<()> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };

        let prev_mode = rfhw.read_register(LoraRegister::RegOpMode)?;
        rfhw.write_register(LoraRegister::RegOpMode, LoraMode::Sleep.as_u8())?;

        self.ch = ch;
        rfhw.write_register(LoraRegister::RegFrfMsb, ch.msb())?;
        rfhw.write_register(LoraRegister::RegFrfMid, ch.mid())?;
        rfhw.write_register(LoraRegister::RegFrfLsb, ch.lsb())?;

        rfhw.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    pub fn set_spreading_factor(&mut self, sf: SpreadingFactor) -> io::Result<()> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };
        let prev_mode = rfhw.read_register(LoraRegister::RegOpMode)?;
        rfhw.write_register(LoraRegister::RegOpMode, LoraMode::Sleep.as_u8())?;

        self.sf = sf;
        let mut tmp = rfhw.read_register(LoraRegister::RegModemConfig2)?;
        tmp &= 0x0F;
        tmp |= sf.as_u8() << 4;
        rfhw.write_register(LoraRegister::RegModemConfig2, tmp)?;

        rfhw.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> io::Result<()> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };
        let prev_mode = rfhw.read_register(LoraRegister::RegOpMode)?;
        rfhw.write_register(LoraRegister::RegOpMode, LoraMode::Sleep.as_u8())?;

        self.bw = bw;
        let mut tmp = rfhw.read_register(LoraRegister::RegModemConfig1)?;
        tmp &= 0x0F;
        tmp |= bw.as_u8() << 4;
        rfhw.write_register(LoraRegister::RegModemConfig1, tmp)?;
        rfhw.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    pub fn set_coding_rate(&mut self, cr: CodingRate) -> io::Result<()> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };
        let prev_mode = rfhw.read_register(LoraRegister::RegOpMode)?;
        rfhw.write_register(LoraRegister::RegOpMode, LoraMode::Sleep.as_u8())?;

        self.cr = cr;
        let mut tmp = rfhw.read_register(LoraRegister::RegModemConfig1)?;
        tmp &= 0xF1;
        tmp |= cr.as_u8() << 1;
        rfhw.write_register(LoraRegister::RegModemConfig1, tmp)?;
        rfhw.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    pub fn set_main_parameters(
        &mut self,
        ch: Channel,
        bw: Bandwidth,
        cr: CodingRate,
        sf: SpreadingFactor,
    ) -> io::Result<()> {
        self.set_channel(ch)?;
        self.set_bandwidth(bw)?;
        self.set_coding_rate(cr)?;
        self.set_spreading_factor(sf)
    }

    pub fn enable_crc_check(&mut self, en: bool) -> io::Result<()> {
        let mut tmp = self.read_register(LoraRegister::RegModemConfig2)?;
        if en {
            tmp |= 1 << 2;
        } else {
            tmp &= !(1 << 2);
        }
        self.crc_check_enabled = en;
        self.write_register(LoraRegister::RegModemConfig2, tmp)
    }

    pub fn enable_implicit_header(&mut self, en: bool) -> io::Result<()> {
        let mut tmp = self.read_register(LoraRegister::RegModemConfig1)?;
        if en {
            tmp |= 0x01;
        } else {
            tmp &= !0x01;
        }
        self.implicit_header_enabled = en;
        self.write_register(LoraRegister::RegModemConfig1, tmp)
    }

    pub fn set_sync_word(&mut self, sync_word: u8) -> io::Result<()> {
        self.write_register(LoraRegister::RegBroadcastAdrs, sync_word)
    }

    pub fn set_rx_fifo_base_addr(&mut self, base_addr: u8) -> io::Result<()> {
        self.write_register(LoraRegister::RegFifoRxBaseAddr, base_addr)
    }

    pub fn set_tx_fifo_base_addr(&mut self, base_addr: u8) -> io::Result<()> {
        self.write_register(LoraRegister::RegFifoTxBaseAddr, base_addr)
    }

    pub fn read_rx_buffer(&mut self) -> io::Result<Vec<u8>> {
        let mut rfhw = match self.hw.lock() {
            Ok(v) => v,
            Err(_) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to acquire hardware lock",
                ))
            }
        };

        let rx_fifo_addr = rfhw.read_register(LoraRegister::RegFifoRxCurrentAddr)?;
        let rx_fifo_byte_cnt = rfhw.read_register(LoraRegister::RegRxNbBytes)?;
        rfhw.write_register(LoraRegister::RegFifoAddrPtr, rx_fifo_addr)?;

        let mut fifo_buf: Vec<u8> = Vec::with_capacity(rx_fifo_byte_cnt as usize);

        for _i in 0..rx_fifo_byte_cnt {
            match rfhw.read_register(LoraRegister::RegFifo) {
                Ok(buf) => fifo_buf.push(buf),
                Err(e) => return Err(e),
            }
        }
        Ok(fifo_buf)
    }

    pub fn receive_packet(&mut self) -> io::Result<LoraPacket> {
        let irq_flags = self.read_register(LoraRegister::RegIrqFlags)?;
        self.write_register(LoraRegister::RegIrqFlags, 0x40)?;
        if irq_flags.flag_enabled(IrqFlagMasks::PayloadCrcError) {
            self.write_register(LoraRegister::RegIrqFlags, 0x20)?;
            return Err(io::Error::new(ErrorKind::InvalidData, "CRC failed"));
        }

        let pkt_buf = self.read_rx_buffer()?;
        let pkt_snr = self.get_packet_snr()?;
        let pkt_rssi = self.get_packet_rssi()?;
        let rssi = self.get_rssi()?;

        Ok(LoraPacket {
            payload: pkt_buf,
            snr: pkt_snr,
            rssi: pkt_rssi,
            crc: true,
            rx_time: now(),
        })
    }
}

pub struct PacketStream<'a> {
    receiver: &'a mut RF95,
    rx_pin: Pin,
    value_stream: PinValueStream,
}

impl<'a> PacketStream<'a> {
    pub fn new(
        rx_pin_num: u64,
        receiver: &'a mut RF95,
        handle: &Handle,
    ) -> io::Result<PacketStream<'a>> {
        let rx_pin = Pin::new(rx_pin_num);
        match rx_pin.export() {
            Ok(()) => (),
            Err(_e) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to export DI0 interrupt pin",
                ))
            }
        }

        match rx_pin.set_direction(Direction::In) {
            Ok(()) => (),
            Err(_e) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to configure DI0 interrupt as input",
                ))
            }
        }

        match rx_pin.set_edge(Edge::RisingEdge) {
            Ok(()) => (),
            Err(_e) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to set edge for DI0 interrupt pin",
                ))
            }
        }

        let value_stream = match rx_pin.get_value_stream(handle) {
            Ok(stream) => stream,
            Err(_e) => {
                return Err(io::Error::new(
                    ErrorKind::Other,
                    "Failed to create value stream for RX IRQ pin",
                ))
            }
        };

        return Ok(PacketStream {
            rx_pin: rx_pin,
            receiver: receiver,
            value_stream: value_stream,
        });
    }
}

impl<'a> Stream for PacketStream<'a> {
    type Item = LoraPacket;
    type Error = io::Error;

    fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
        match self.value_stream.poll() {
            Ok(Async::Ready(Some(0))) => Ok(Async::NotReady),

            Ok(Async::Ready(Some(_))) => {
                let pkt = match self.receiver.receive_packet() {
                    Ok(pkt) => pkt,
                    Err(_e) => LoraPacket {
                        crc: false,
                        payload: Vec::new(),
                        rssi: 0,
                        snr: 0,
                        rx_time: now(),
                    },
                };
                Ok(Async::Ready(Some(pkt)))
            }

            Ok(Async::Ready(None)) => Ok(Async::Ready(None)),

            Ok(Async::NotReady) => Ok(Async::NotReady),

            Err(e) => Err(io::Error::new(
                ErrorKind::Other,
                "Unknown error polling RX IRQ pin",
            )),
        }
    }
}
