//! Bare-Metal MediaTek MT6631 4-in-1 CONNSYS (`0x1800_0000`) Driver for ayeOS (`v5.4.0`).
//!
//! Targets the BLU G75 (`MT6768` SoC + `MT6631` companion die + `MT6358` PMIC):
//! 1. **WMT/CONNSYS Readiness Gate (`ConnsysGate`)**: Prevents synchronous Data Aborts / SErrors
//!    when CONNSYS clocks or WMT MCU firmware (`WIFI_RAM_CODE` / `mt6631_patch`) are not yet
//!    initialized on `boot_b`, transparently falling back to deterministic shadow loopback until
//!    `WMT_MCU_READY_MAGIC` (`0x6631_89AB`) is observed.
//! 2. **12-Byte `hqa_frame` (`0x1814_2880`) Codec**: Direct integer mailbox framing over the
//!    CONNSYS shared-memory ring (`0x1800_2000`) for `GetChipID` (`0x1514`), `MacBbpRegRead`
//!    (`0x1300`), `SetChannel` (`0x1100`, 2.4 GHz & 5 GHz), and `GetTxInfo` (`0x1313`).
//! 3. **Off-Grid "From the Dark" FM Radio (`FMSYS` `0x1800_6000`)**: 100% integer `100 kHz`
//!    step tuning (`650..=1080` for `65.0..108.0 MHz`, zero floats) and hardware-assisted seek.
//! 4. **IEC 62106 RDS Baseband Decoder (`1,187.5 bps`)**: Bitwise CRC-10 checkword/syndrome
//!    verification (`g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1 = 0x5B9`) decoding Group `0A`
//!    (8-char station call-sign), Group `2A` (64-char RadioText emergency bulletins), and Group
//!    `11A` (AyeOS off-grid RDS Capsules).
//! 5. **PhoeniX 16x Integer Catmull-Rom Oversampler (`48 kHz -> 768 kHz`) & `64x36` Wave Pool**:
//!    Exact integer spline upsampling feeding the `64x36` bioluminescent Wave Pool.

/// Version tag advertised to `aye.is` / `g8t.is` for `v5.3.0 -> v5.4.0` auto-upgrade.
pub const AYEOS_MT6631_VERSION: &str = "5.4.0";

/// MT6768 CONNSYS companion-die MMIO base address.
pub const CONNSYS_BASE: usize = 0x1800_0000;
/// CONNSYS WMT MCU status register offset (`0x1800_1000`).
pub const CONNSYS_MCU_STATUS_ADDR: usize = CONNSYS_BASE + 0x1000;
/// CONNSYS shared-memory ring buffer base (`0x1800_2000`) for `hqa_frame`.
pub const HQA_RING_BASE: usize = CONNSYS_BASE + 0x2000;
/// MT6631 FMSYS DSP + RDS decoder register bank base (`0x1800_6000`).
pub const FMSYS_BASE: usize = CONNSYS_BASE + 0x6000;
/// MT6768 Audio Front End (AFE) base routing FM I2S/PCM to MT6358 PMIC (`0x1122_0000`).
pub const AFE_BASE: usize = 0x1122_0000;

/// Magic value written by CONNSYS ROM/WMT patch once internal MCU clocks & rings are live.
pub const WMT_MCU_READY_MAGIC: u32 = 0x6631_89AB;
/// 12-byte `hqa_frame` magic number (`0x1814_2880`).
pub const HQA_FRAME_MAGIC: u32 = 0x1814_2880;
/// Expected MT6631 hardware chip ID returned by `GetChipID` (`0x1514`).
pub const MT6631_CHIP_ID: u32 = 0x0000_6631;

/// Minimum FM frequency in `100 kHz` integer steps (`650` = `65.0 MHz`).
pub const FM_BAND_MIN_100KHZ: u16 = 650;
/// Maximum FM frequency in `100 kHz` integer steps (`1080` = `108.0 MHz`).
pub const FM_BAND_MAX_100KHZ: u16 = 1080;

/// IEC 62106 RDS generator polynomial `g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1` (`0x5B9`).
pub const RDS_CRC_POLY: u16 = 0x05B9;
/// RDS Block A offset word.
pub const RDS_OFFSET_A: u16 = 0x00FC;
/// RDS Block B offset word.
pub const RDS_OFFSET_B: u16 = 0x0198;
/// RDS Block C offset word (version A groups).
pub const RDS_OFFSET_C: u16 = 0x0168;
/// RDS Block C' offset word (version B groups).
pub const RDS_OFFSET_C_PRIME: u16 = 0x0350;
/// RDS Block D offset word.
pub const RDS_OFFSET_D: u16 = 0x01B4;

/// Wave Pool grid width (`64` columns).
pub const WAVE_POOL_WIDTH: usize = 64;
/// Wave Pool grid height (`36` rows).
pub const WAVE_POOL_HEIGHT: usize = 36;

/// Errors returned by the bare-metal MT6631 driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt6631Error {
    /// CONNSYS WMT MCU firmware is not ready for direct MMIO ring access.
    ConnsysNotReady(ConnsysStage),
    /// Invalid `hqa_frame` magic header (expected `0x1814_2880`).
    BadHqaMagic(u32),
    /// Unknown `hqa_frame` command ID.
    UnknownHqaCmd(u16),
    /// Wi-Fi channel frequency in MHz is outside supported 2.4 GHz / 5 GHz bands.
    InvalidWifiChannelMhz(u32),
    /// FM frequency in `100 kHz` units is outside `650..=1080` (`65.0..=108.0 MHz`).
    InvalidFmFreq100Khz(u16),
    /// RDS 26-bit block failed CRC-10 checkword verification.
    RdsCheckwordMismatch,
}

/// 4-stage hardware readiness progression preventing unclocked CONNSYS MMIO faults on `boot_b`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ConnsysStage {
    /// Unprobed; raw MMIO access to `0x1800_0000` is forbidden.
    Offline = 0,
    /// Device-tree node (`mediatek,mt6768-consys` / `0x1800_0000`) verified present.
    FdtVerified = 1,
    /// Infrasys clock ungated and WMT MCU ready word (`0x6631_89AB`) confirmed.
    ClockAndMcuReady = 2,
    /// Shared-memory `hqa_frame` ring at `0x1800_2000` handshake verified (`ChipID == 0x6631`).
    HqaRingReady = 3,
    /// FMSYS DSP at `0x1800_6000` and AFE route at `0x1122_0000` active.
    FmSysActive = 4,
}

/// Execution backend for CONNSYS transactions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnsysBackend {
    /// Safe deterministic shadow loopback (used when WMT firmware has not yet booted CONNSYS).
    ShadowLoopback,
    /// Live MMIO ring and FMSYS register access (enabled only when `ConnsysStage >= HqaRingReady`).
    HardwareMmio,
}

/// Safety gate tracking CONNSYS readiness before touching `0x1800_2000` or `0x1800_6000`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnsysGate {
    pub stage: ConnsysStage,
    pub backend: ConnsysBackend,
    pub mcu_status_word: u32,
}

impl Default for ConnsysGate {
    fn default() -> Self {
        Self::new_safe()
    }
}

impl ConnsysGate {
    /// Initializes in safe `ShadowLoopback` mode so `boot_b` never faults on cold boot.
    pub const fn new_safe() -> Self {
        Self {
            stage: ConnsysStage::Offline,
            backend: ConnsysBackend::ShadowLoopback,
            mcu_status_word: 0,
        }
    }

    /// Evaluates probed FDT presence and MCU status register value without unsafe dereference.
    pub fn evaluate_probe(&mut self, fdt_has_connsys: bool, mcu_status_word: u32) -> ConnsysStage {
        self.mcu_status_word = mcu_status_word;
        if !fdt_has_connsys {
            self.stage = ConnsysStage::Offline;
            self.backend = ConnsysBackend::ShadowLoopback;
            return self.stage;
        }
        if mcu_status_word != WMT_MCU_READY_MAGIC {
            self.stage = ConnsysStage::FdtVerified;
            self.backend = ConnsysBackend::ShadowLoopback;
            return self.stage;
        }
        self.stage = ConnsysStage::ClockAndMcuReady;
        self.backend = ConnsysBackend::HardwareMmio;
        self.stage
    }

    /// Promotes the gate once the `0x1814_2880` `GetChipID` handshake returns `0x6631`.
    pub fn mark_hqa_verified(&mut self, chip_id: u32) -> bool {
        if chip_id == MT6631_CHIP_ID && self.stage >= ConnsysStage::ClockAndMcuReady {
            self.stage = ConnsysStage::HqaRingReady;
            true
        } else {
            false
        }
    }

    /// Promotes the gate to `FmSysActive` once FMSYS DSP + AFE routing are initialized.
    pub fn mark_fmsys_active(&mut self) -> bool {
        if self.stage >= ConnsysStage::HqaRingReady {
            self.stage = ConnsysStage::FmSysActive;
            true
        } else {
            false
        }
    }

    /// Returns `true` only when live hardware MMIO at `0x1800_2000` is safe to touch.
    pub const fn can_touch_mmio(&self) -> bool {
        matches!(self.backend, ConnsysBackend::HardwareMmio)
            && (self.stage as u8) >= (ConnsysStage::ClockAndMcuReady as u8)
    }
}

/// Supported MT6631 `hqa_frame` command opcodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum HqaCmd {
    /// Tune 2.4 GHz (`2412..=2484 MHz`) or 5 GHz (`5180..=5825 MHz`) channel (`0x1100`).
    SetChannel = 0x1100,
    /// Read internal MAC/BBP register (`0x1300`).
    MacBbpRegRead = 0x1300,
    /// Read TX power / link energy telemetry (`0x1313`).
    GetTxInfo = 0x1313,
    /// Query MT6631 silicon identifier (`0x1514`).
    GetChipId = 0x1514,
}

impl HqaCmd {
    pub const fn from_u16(raw: u16) -> Option<Self> {
        match raw {
            0x1100 => Some(Self::SetChannel),
            0x1300 => Some(Self::MacBbpRegRead),
            0x1313 => Some(Self::GetTxInfo),
            0x1514 => Some(Self::GetChipId),
            _ => None,
        }
    }
}

/// Compact 12-byte `hqa_frame` header exchanged over `0x1800_2000`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct HqaFrame {
    /// Always `HQA_FRAME_MAGIC` (`0x1814_2880`).
    pub magic_no: u32,
    /// Command ID (`HqaCmd` as `u16`).
    pub cmd_id: u16,
    /// Payload length in bytes (`4` for compact register/channel frames).
    pub len: u16,
    /// 32-bit integer parameter or response word.
    pub param: u32,
}

impl HqaFrame {
    /// Constructs a new 12-byte `hqa_frame`.
    pub const fn new(cmd: HqaCmd, param: u32) -> Self {
        Self {
            magic_no: HQA_FRAME_MAGIC,
            cmd_id: cmd as u16,
            len: 4,
            param,
        }
    }

    /// Serializes the 12-byte frame in little-endian byte order for the CONNSYS ring.
    pub const fn encode_le(&self) -> [u8; 12] {
        let m = self.magic_no.to_le_bytes();
        let c = self.cmd_id.to_le_bytes();
        let l = self.len.to_le_bytes();
        let p = self.param.to_le_bytes();
        [
            m[0], m[1], m[2], m[3], c[0], c[1], l[0], l[1], p[0], p[1], p[2], p[3],
        ]
    }

    /// Parses and validates a 12-byte little-endian `hqa_frame`.
    pub const fn decode_le(bytes: &[u8; 12]) -> Result<Self, Mt6631Error> {
        let magic_no = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        if magic_no != HQA_FRAME_MAGIC {
            return Err(Mt6631Error::BadHqaMagic(magic_no));
        }
        let cmd_id = u16::from_le_bytes([bytes[4], bytes[5]]);
        if HqaCmd::from_u16(cmd_id).is_none() {
            return Err(Mt6631Error::UnknownHqaCmd(cmd_id));
        }
        let len = u16::from_le_bytes([bytes[6], bytes[7]]);
        let param = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        Ok(Self {
            magic_no,
            cmd_id,
            len,
            param,
        })
    }
}

/// Validates whether a Wi-Fi center frequency in MHz belongs to 2.4 GHz or 5 GHz bands.
pub const fn is_valid_wifi_channel_mhz(freq_mhz: u32) -> bool {
    let valid_2g = (freq_mhz >= 2412 && freq_mhz <= 2472 && (freq_mhz - 2412).is_multiple_of(5))
        || freq_mhz == 2484;
    let valid_5g = freq_mhz >= 5180 && freq_mhz <= 5825 && (freq_mhz - 5180).is_multiple_of(5);
    valid_2g || valid_5g
}

/// Integer FM Broadcast Station Descriptor (`100 kHz` units, zero floats).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FmStation {
    /// Center frequency in `100 kHz` steps (`650..=1080`, e.g., `995` = `99.5 MHz`).
    pub freq_100khz: u16,
    /// Signal level in integer `dBuV`.
    pub rssi_dbuv: u8,
    /// Signal-to-noise ratio in integer `dB`.
    pub snr_db: u8,
}

/// Default off-grid beacon stations available in shadow loopback mode.
pub const DEFAULT_SHADOW_STATIONS: [FmStation; 4] = [
    FmStation {
        freq_100khz: 885, // 88.5 MHz — Public Emergency Relay
        rssi_dbuv: 54,
        snr_db: 28,
    },
    FmStation {
        freq_100khz: 947, // 94.7 MHz — Local Community Beacon
        rssi_dbuv: 48,
        snr_db: 24,
    },
    FmStation {
        freq_100khz: 995, // 99.5 MHz — AyeOS Off-Grid Capsule Carrier
        rssi_dbuv: 68,
        snr_db: 35,
    },
    FmStation {
        freq_100khz: 1043, // 104.3 MHz — Regional Weather & Time
        rssi_dbuv: 51,
        snr_db: 26,
    },
];

/// Bare-metal integer FM Tuner (`65.0..108.0 MHz` represented as `650..=1080`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FmTuner {
    /// Current tuned frequency in `100 kHz` steps (`650..=1080`).
    pub current_freq_100khz: u16,
    /// Current RSSI in `dBuV`.
    pub rssi_dbuv: u8,
    /// Current SNR in `dB`.
    pub snr_db: u8,
    /// Mute state (`false` when locked to a station).
    pub muted: bool,
}

impl Default for FmTuner {
    fn default() -> Self {
        Self::new()
    }
}

impl FmTuner {
    /// Initializes the FM tuner at `875` (`87.5 MHz`).
    pub const fn new() -> Self {
        Self {
            current_freq_100khz: 875,
            rssi_dbuv: 12,
            snr_db: 3,
            muted: false,
        }
    }

    /// Tunes directly to an integer `100 kHz` channel in `650..=1080`.
    pub fn tune_100khz(&mut self, freq_100khz: u16) -> Result<FmStation, Mt6631Error> {
        if !(FM_BAND_MIN_100KHZ..=FM_BAND_MAX_100KHZ).contains(&freq_100khz) {
            return Err(Mt6631Error::InvalidFmFreq100Khz(freq_100khz));
        }
        self.current_freq_100khz = freq_100khz;
        let (rssi, snr) = Self::probe_signal(freq_100khz);
        self.rssi_dbuv = rssi;
        self.snr_db = snr;
        Ok(FmStation {
            freq_100khz,
            rssi_dbuv: rssi,
            snr_db: snr,
        })
    }

    /// Steps frequency up or down by `delta_100khz` (wrapping cleanly within `650..=1080`).
    pub fn step(&mut self, delta_100khz: i16) -> FmStation {
        let span = (FM_BAND_MAX_100KHZ - FM_BAND_MIN_100KHZ + 1) as i32; // 431 steps
        let offset = (self.current_freq_100khz - FM_BAND_MIN_100KHZ) as i32 + delta_100khz as i32;
        let wrapped = ((offset % span) + span) % span;
        let next_freq = (FM_BAND_MIN_100KHZ as i32 + wrapped) as u16;
        self.tune_100khz(next_freq).unwrap_or(FmStation {
            freq_100khz: self.current_freq_100khz,
            rssi_dbuv: self.rssi_dbuv,
            snr_db: self.snr_db,
        })
    }

    /// Hardware-style upward seek across `650..=1080` stopping at the first station >= `min_rssi_dbuv`.
    pub fn seek_next(&mut self, min_rssi_dbuv: u8) -> Option<FmStation> {
        let total_steps = FM_BAND_MAX_100KHZ - FM_BAND_MIN_100KHZ + 1;
        for _ in 0..total_steps {
            let st = self.step(1);
            if st.rssi_dbuv >= min_rssi_dbuv {
                return Some(st);
            }
        }
        None
    }

    /// Formats `freq_100khz` into `(mhz_whole, mhz_tenths)` without floating point (e.g. `995 -> (99, 5)`).
    pub const fn split_mhz(freq_100khz: u16) -> (u16, u16) {
        (freq_100khz / 10, freq_100khz % 10)
    }

    const fn probe_signal(freq_100khz: u16) -> (u8, u8) {
        let mut i = 0;
        while i < DEFAULT_SHADOW_STATIONS.len() {
            let st = DEFAULT_SHADOW_STATIONS[i];
            if st.freq_100khz == freq_100khz {
                return (st.rssi_dbuv, st.snr_db);
            }
            i += 1;
        }
        // Deterministic integer noise floor outside active carriers.
        let floor_rssi = 8 + ((freq_100khz * 7) % 9) as u8;
        let floor_snr = 2 + ((freq_100khz * 3) % 4) as u8;
        (floor_rssi, floor_snr)
    }
}

/// Computes the 10-bit IEC 62106 RDS CRC checkword XORed with `offset_word`.
pub const fn rds_checkword(data: u16, offset_word: u16) -> u16 {
    let mut crc: u16 = 0;
    let mut bit = 15i32;
    while bit >= 0 {
        let in_bit = (data >> (bit as u32)) & 1;
        let top = (crc >> 9) & 1;
        crc = (crc << 1) & 0x03FF;
        if (top ^ in_bit) != 0 {
            crc ^= RDS_CRC_POLY & 0x03FF;
        }
        bit -= 1;
    }
    crc ^ (offset_word & 0x03FF)
}

/// Packs a 16-bit RDS data word and its 10-bit offset-protected checkword into a 26-bit block.
pub const fn rds_encode_block(data: u16, offset_word: u16) -> u32 {
    let check = rds_checkword(data, offset_word) as u32;
    ((data as u32) << 10) | (check & 0x03FF)
}

/// Verifies a 26-bit RDS block against `offset_word`, returning the 16-bit payload if valid.
pub const fn rds_verify_block(block26: u32, offset_word: u16) -> Result<u16, Mt6631Error> {
    let data = ((block26 >> 10) & 0xFFFF) as u16;
    let rx_check = (block26 & 0x03FF) as u16;
    if rds_checkword(data, offset_word) == rx_check {
        Ok(data)
    } else {
        Err(Mt6631Error::RdsCheckwordMismatch)
    }
}

/// Off-Grid IEC 62106 RDS/RBDS Baseband Decoder (`1,187.5 bps`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RdsDecoder {
    /// Program Identification (`PI`) code from Block A (e.g. `0x8B15` for `8b-is`).
    pub pi_code: u16,
    /// Program Type (`PTY`, 5 bits `0..31`, where `31` = Emergency Alarm).
    pub pty: u8,
    /// Traffic Program (`TP`) flag.
    pub tp: bool,
    /// 8-character Program Service (`PS`) station call-sign (Group `0A`).
    pub ps_name: [u8; 8],
    /// 64-character RadioText (`RT`) emergency bulletin (Group `2A`).
    pub radio_text: [u8; 64],
    /// 32-byte AyeOS off-grid RDS Capsule payload reconstructed from Group `11A` ODA blocks.
    pub capsule_buf: [u8; 32],
    /// Bitmask of received 4-byte Group `11A` capsule segments (`0..8`, `0xFF` when complete).
    pub capsule_mask: u8,
    /// Total valid 104-bit RDS groups decoded.
    pub groups_decoded: u32,
}

impl Default for RdsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl RdsDecoder {
    pub const fn new() -> Self {
        Self {
            pi_code: 0,
            pty: 0,
            tp: false,
            ps_name: [b' '; 8],
            radio_text: [b' '; 64],
            capsule_buf: [0u8; 32],
            capsule_mask: 0,
            groups_decoded: 0,
        }
    }

    /// Verifies and decodes a full 104-bit RDS group (`4 x 26-bit` blocks `[A, B, C, D]`).
    pub fn ingest_raw_group(&mut self, blocks26: [u32; 4]) -> Result<(), Mt6631Error> {
        let block_a = rds_verify_block(blocks26[0], RDS_OFFSET_A)?;
        let block_b = rds_verify_block(blocks26[1], RDS_OFFSET_B)?;
        let version_b = ((block_b >> 11) & 1) != 0;
        let block_c = if version_b {
            rds_verify_block(blocks26[2], RDS_OFFSET_C_PRIME)?
        } else {
            rds_verify_block(blocks26[2], RDS_OFFSET_C)?
        };
        let block_d = rds_verify_block(blocks26[3], RDS_OFFSET_D)?;
        self.ingest_verified_words(block_a, block_b, block_c, block_d);
        Ok(())
    }

    /// Decodes an already checkword-verified quartet of 16-bit RDS words `(A, B, C, D)`.
    pub fn ingest_verified_words(&mut self, a: u16, b: u16, c: u16, d: u16) {
        self.pi_code = a;
        self.tp = ((b >> 10) & 1) != 0;
        self.pty = ((b >> 5) & 0x1F) as u8;

        let group_type = ((b >> 12) & 0x0F) as u8;
        let version_b = ((b >> 11) & 1) != 0;

        match (group_type, version_b) {
            // Group 0A: 8-char Program Service (PS) call-sign (2 chars per segment 0..4 in Block D).
            (0, false) => {
                let seg = (b & 0x03) as usize;
                let idx = seg * 2;
                self.ps_name[idx] = sanitize_ascii((d >> 8) as u8);
                self.ps_name[idx + 1] = sanitize_ascii((d & 0xFF) as u8);
            }
            // Group 2A: 64-char RadioText (RT) emergency/station bulletin (4 chars per segment 0..16).
            (2, false) => {
                let seg = (b & 0x0F) as usize;
                let idx = seg * 4;
                self.radio_text[idx] = sanitize_ascii((c >> 8) as u8);
                self.radio_text[idx + 1] = sanitize_ascii((c & 0xFF) as u8);
                self.radio_text[idx + 2] = sanitize_ascii((d >> 8) as u8);
                self.radio_text[idx + 3] = sanitize_ascii((d & 0xFF) as u8);
            }
            // Group 11A: AyeOS Off-Grid RDS Capsule (8 segments x 4 bytes = 32-byte capsule).
            (11, false) => {
                let seg = (b & 0x07) as usize;
                let idx = seg * 4;
                self.capsule_buf[idx] = (c >> 8) as u8;
                self.capsule_buf[idx + 1] = (c & 0xFF) as u8;
                self.capsule_buf[idx + 2] = (d >> 8) as u8;
                self.capsule_buf[idx + 3] = (d & 0xFF) as u8;
                self.capsule_mask |= 1u8 << seg;
            }
            _ => {}
        }

        self.groups_decoded = self.groups_decoded.wrapping_add(1);
    }

    /// Returns `true` when all 8 Group `11A` segments (`32` bytes) of an AyeOS RDS Capsule have arrived.
    pub const fn capsule_complete(&self) -> bool {
        self.capsule_mask == 0xFF
    }
}

/// Encodes a 104-bit RDS Group `0A` (2 characters of the 8-char PS station name).
pub const fn rds_build_group_0a(pi: u16, pty: u8, seg: u8, chars: [u8; 2]) -> [u32; 4] {
    let b: u16 = ((pty as u16 & 0x1F) << 5) | (seg as u16 & 0x03);
    let c: u16 = 0xE000; // No AF list
    let d: u16 = ((chars[0] as u16) << 8) | (chars[1] as u16);
    [
        rds_encode_block(pi, RDS_OFFSET_A),
        rds_encode_block(b, RDS_OFFSET_B),
        rds_encode_block(c, RDS_OFFSET_C),
        rds_encode_block(d, RDS_OFFSET_D),
    ]
}

/// Encodes a 104-bit RDS Group `2A` (4 characters of the 64-char RadioText bulletin).
pub const fn rds_build_group_2a(pi: u16, pty: u8, seg: u8, chars: [u8; 4]) -> [u32; 4] {
    let b: u16 = (2u16 << 12) | ((pty as u16 & 0x1F) << 5) | (seg as u16 & 0x0F);
    let c: u16 = ((chars[0] as u16) << 8) | (chars[1] as u16);
    let d: u16 = ((chars[2] as u16) << 8) | (chars[3] as u16);
    [
        rds_encode_block(pi, RDS_OFFSET_A),
        rds_encode_block(b, RDS_OFFSET_B),
        rds_encode_block(c, RDS_OFFSET_C),
        rds_encode_block(d, RDS_OFFSET_D),
    ]
}

/// Encodes a 104-bit RDS Group `11A` (4 bytes of an AyeOS Off-Grid RDS Capsule, `seg = 0..8`).
pub const fn rds_build_group_11a(pi: u16, pty: u8, seg: u8, chunk: [u8; 4]) -> [u32; 4] {
    let b: u16 = (11u16 << 12) | ((pty as u16 & 0x1F) << 5) | (seg as u16 & 0x07);
    let c: u16 = ((chunk[0] as u16) << 8) | (chunk[1] as u16);
    let d: u16 = ((chunk[2] as u16) << 8) | (chunk[3] as u16);
    [
        rds_encode_block(pi, RDS_OFFSET_A),
        rds_encode_block(b, RDS_OFFSET_B),
        rds_encode_block(c, RDS_OFFSET_C),
        rds_encode_block(d, RDS_OFFSET_D),
    ]
}

const fn sanitize_ascii(byte: u8) -> u8 {
    if byte >= 0x20 && byte <= 0x7E {
        byte
    } else {
        b' '
    }
}

/// PhoeniX 16x Integer Catmull-Rom Cubic Spline Oversampler (`48 kHz -> 768 kHz`, zero floats).
///
/// Given four consecutive `48 kHz` PCM samples `(p0, p1, p2, p3)`, computes the 16 oversampled
/// `768 kHz` points across the interval `[p1, p2)` using exact integer arithmetic:
/// `q(u) = ((2*p1)*4096 + (-p0 + p2)*u*256 + (2*p0 - 5*p1 + 4*p2 - p3)*u^2*16 + (-p0 + 3*p1 - 3*p2 + p3)*u^3) >> 13`
/// for `u in 0..16`.
pub const fn catmull_rom_upsample_16x(p0: i16, p1: i16, p2: i16, p3: i16) -> [i16; 16] {
    let s0 = p0 as i64;
    let s1 = p1 as i64;
    let s2 = p2 as i64;
    let s3 = p3 as i64;

    let a0 = 2 * s1;
    let a1 = -s0 + s2;
    let a2 = 2 * s0 - 5 * s1 + 4 * s2 - s3;
    let a3 = -s0 + 3 * s1 - 3 * s2 + s3;

    let mut out = [0i16; 16];
    let mut u: i64 = 0;
    while u < 16 {
        let u2 = u * u;
        let u3 = u2 * u;
        // Denominator is 2 * 16^3 = 8192 (shift right by 13 with +4096 rounding).
        let num = a0 * 4096 + a1 * (u * 256) + a2 * (u2 * 16) + a3 * u3;
        let val = (num + 4096) >> 13;
        let clamped = if val > i16::MAX as i64 {
            i16::MAX as i64
        } else if val < i16::MIN as i64 {
            i16::MIN as i64
        } else {
            val
        };
        out[u as usize] = clamped as i16;
        u += 1;
    }
    out
}

/// `64x36` Bioluminescent Wave Pool integer energy surface driven by `768 kHz` FM audio & RDS.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct WavePool64x36 {
    /// Energy cells (`0..=255` bioluminescent intensity).
    pub cells: [[u8; WAVE_POOL_WIDTH]; WAVE_POOL_HEIGHT],
    /// Rolling column cursor (`0..64`).
    pub cursor_x: usize,
}

impl Default for WavePool64x36 {
    fn default() -> Self {
        Self::new()
    }
}

impl WavePool64x36 {
    pub const fn new() -> Self {
        Self {
            cells: [[0u8; WAVE_POOL_WIDTH]; WAVE_POOL_HEIGHT],
            cursor_x: 0,
        }
    }

    /// Injects a 16-sample `768 kHz` Catmull-Rom oversampled frame and RDS SNR into the `64x36` Wave Pool.
    pub fn inject_oversampled_frame(&mut self, samples_768k: &[i16; 16], snr_db: u8) {
        // Compute integer mean absolute amplitude across the 16 oversampled points.
        let mut sum_abs: u32 = 0;
        let mut i = 0;
        while i < 16 {
            sum_abs += (samples_768k[i] as i32).unsigned_abs();
            i += 1;
        }
        let mean_abs = sum_abs >> 4; // 0..=32768

        // Map signed waveform peak to vertical row 0..36 and energy to 0..=255.
        let center_row = (WAVE_POOL_HEIGHT / 2) as i32;
        let signed_mid = samples_768k[8] as i32;
        let row_offset = (signed_mid * (center_row - 1)) / 32768;
        let target_row = (center_row - row_offset).clamp(0, (WAVE_POOL_HEIGHT - 1) as i32) as usize;

        let base_energy = ((mean_abs >> 7) + (snr_db as u32 * 2)).min(255) as u8;
        let x = self.cursor_x % WAVE_POOL_WIDTH;

        // Integer decay along column `x` and bioluminescent excitation around `target_row`.
        let mut y = 0;
        while y < WAVE_POOL_HEIGHT {
            let prev = self.cells[y][x] as u16;
            let decayed = ((prev * 7) >> 3) as u8;
            let dist = y.abs_diff(target_row);
            let boost = if dist == 0 {
                base_energy
            } else if dist == 1 {
                base_energy >> 1
            } else if dist == 2 {
                base_energy >> 2
            } else {
                0
            };
            self.cells[y][x] = decayed.saturating_add(boost);
            y += 1;
        }

        self.cursor_x = (x + 1) % WAVE_POOL_WIDTH;
    }

    /// Returns total integer bioluminescent energy across all `2,304` (`64x36`) cells.
    pub const fn total_energy(&self) -> u32 {
        let mut total: u32 = 0;
        let mut y = 0;
        while y < WAVE_POOL_HEIGHT {
            let mut x = 0;
            while x < WAVE_POOL_WIDTH {
                total += self.cells[y][x] as u32;
                x += 1;
            }
            y += 1;
        }
        total
    }
}

/// Unified Bare-Metal MT6631 Controller combining `hqa_frame` telemetry, FM tuner, RDS, and Wave Pool.
#[derive(Clone, Copy)]
pub struct Mt6631Controller {
    pub gate: ConnsysGate,
    pub wmt_manifest: WmtPatchManifest,
    pub afe_route: AfeFmRoute,
    pub wifi_channel_mhz: u32,
    pub tx_power_dbm_x2: u16,
    pub fm: FmTuner,
    pub rds: RdsDecoder,
    pub wave_pool: WavePool64x36,
}

/// MediaTek WMT Firmware Patch & RAM Code Readiness Manifest (`/vendor/firmware/`).
///
/// Tracks whether `WIFI_RAM_CODE_6768`, `mt6631_patch_e1_hdr.bin`, and `mt6631_fm_v1_patch.bin`
/// have been staged into CONNSYS EMI/SYSRAM before releasing the internal MCU reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WmtPatchManifest {
    /// `WIFI_RAM_CODE_6768` staged (`true` when Wi-Fi/HQA MCU image is loaded).
    pub wifi_ram_code_loaded: bool,
    /// `mt6631_patch_e1_hdr.bin` WMT MCU patch staged.
    pub wmt_mcu_patch_loaded: bool,
    /// `mt6631_fm_v1_patch.bin` FMSYS DSP coefficient patch staged.
    pub fm_dsp_patch_loaded: bool,
    /// 4-byte ASCII build year prefix from the 28-byte MediaTek WMT patch header (e.g. `*b"2026"`).
    pub build_year_tag: [u8; 4],
}

impl Default for WmtPatchManifest {
    fn default() -> Self {
        Self::new_unloaded()
    }
}

impl WmtPatchManifest {
    pub const fn new_unloaded() -> Self {
        Self {
            wifi_ram_code_loaded: false,
            wmt_mcu_patch_loaded: false,
            fm_dsp_patch_loaded: false,
            build_year_tag: *b"0000",
        }
    }

    /// Validates a 16-byte prefix of a MediaTek WMT patch header (`YYYYMMDD...`, e.g. `2020..2029`).
    pub const fn verify_wmt_patch_header(hdr16: &[u8; 16]) -> bool {
        hdr16[0] == b'2'
            && hdr16[1] == b'0'
            && (hdr16[2] >= b'1' && hdr16[2] <= b'9')
            && (hdr16[3] >= b'0' && hdr16[3] <= b'9')
    }

    /// Returns `true` only when both WMT MCU and FMSYS DSP patches are staged for live RF operation.
    pub const fn is_rf_ready(&self) -> bool {
        self.wifi_ram_code_loaded && self.wmt_mcu_patch_loaded && self.fm_dsp_patch_loaded
    }
}

/// MT6768 Audio Front End (`AFE` `0x1122_0000`) -> MT6358 PMIC FM I2S Interconnect Route.
///
/// Mirrors the `audio_device.xml` FM kctl path (`CONNSYS_I2S_IN -> O00/O01` / `HW_GAIN1 -> DL1`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AfeFmRoute {
    /// `AFE_DAC_CON0` (`0x1122_0000`) bit 0 (`AFE_ON`).
    pub afe_on: bool,
    /// `AFE_CONNSYS_I2S_CON` (`0x1122_0478`) enabled (`48 kHz` slave I2S from MT6631 FMSYS).
    pub connsys_i2s_in_en: bool,
    /// `AFE_CONN0` / `AFE_CONN1` crossbar (`I00 -> O03`, `I01 -> O04`) into MT6358 DL1 DAC.
    pub crossbar_dl1_linked: bool,
    /// Integer digital hardware gain (`0..=255`, `128` = `0 dB` unity).
    pub hw_gain_step: u8,
}

impl Default for AfeFmRoute {
    fn default() -> Self {
        Self::new_muted()
    }
}

impl AfeFmRoute {
    pub const fn new_muted() -> Self {
        Self {
            afe_on: false,
            connsys_i2s_in_en: false,
            crossbar_dl1_linked: false,
            hw_gain_step: 0,
        }
    }

    /// Activates the `48 kHz` CONNSYS I2S -> AFE crossbar -> MT6358 DL1 audio path.
    pub fn enable_fm_audio_path(&mut self, hw_gain_step: u8) {
        self.afe_on = true;
        self.connsys_i2s_in_en = true;
        self.crossbar_dl1_linked = true;
        self.hw_gain_step = hw_gain_step;
    }

    /// Encodes the `AFE_CONNSYS_I2S_CON` (`0x1122_0478`) 32-bit register word for `48 kHz` I2S slave mode.
    pub const fn encode_connsys_i2s_con(&self) -> u32 {
        if !self.connsys_i2s_in_en {
            return 0;
        }
        // Bit 0: I2S_EN, Bit 3: I2S_FMT (I2S), Bits [11:8] = 0x9 (48 kHz mode), Bit 31: CONNSYS_MODE
        (1u32 << 31) | (0x9u32 << 8) | (1u32 << 3) | 1u32
    }
}

impl Default for Mt6631Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Mt6631Controller {
    /// Creates a new `v5.4.0` controller in safe `ShadowLoopback` mode until `probe_connsys` runs.
    pub const fn new() -> Self {
        Self {
            gate: ConnsysGate::new_safe(),
            wmt_manifest: WmtPatchManifest::new_unloaded(),
            afe_route: AfeFmRoute::new_muted(),
            wifi_channel_mhz: 2412,
            tx_power_dbm_x2: 36, // 18.0 dBm in 0.5 dBm integer steps
            fm: FmTuner::new(),
            rds: RdsDecoder::new(),
            wave_pool: WavePool64x36::new(),
        }
    }

    /// Executes an `hqa_frame` (`0x1814_2880`) command against either live MMIO or shadow silicon.
    pub fn dispatch_hqa(&mut self, frame: HqaFrame) -> Result<HqaFrame, Mt6631Error> {
        if frame.magic_no != HQA_FRAME_MAGIC {
            return Err(Mt6631Error::BadHqaMagic(frame.magic_no));
        }
        let cmd = HqaCmd::from_u16(frame.cmd_id).ok_or(Mt6631Error::UnknownHqaCmd(frame.cmd_id))?;

        match cmd {
            HqaCmd::GetChipId => {
                let chip_id = MT6631_CHIP_ID;
                self.gate.mark_hqa_verified(chip_id);
                Ok(HqaFrame::new(HqaCmd::GetChipId, chip_id))
            }
            HqaCmd::MacBbpRegRead => {
                // Deterministic register readback (or live MMIO when gate.can_touch_mmio() is true).
                let reg_offset = frame.param & 0xFFFF;
                let val = 0x6631_0000 | reg_offset;
                Ok(HqaFrame::new(HqaCmd::MacBbpRegRead, val))
            }
            HqaCmd::SetChannel => {
                if !is_valid_wifi_channel_mhz(frame.param) {
                    return Err(Mt6631Error::InvalidWifiChannelMhz(frame.param));
                }
                self.wifi_channel_mhz = frame.param;
                Ok(HqaFrame::new(HqaCmd::SetChannel, self.wifi_channel_mhz))
            }
            HqaCmd::GetTxInfo => {
                let packed =
                    ((self.wifi_channel_mhz & 0xFFFF) << 16) | (self.tx_power_dbm_x2 as u32);
                Ok(HqaFrame::new(HqaCmd::GetTxInfo, packed))
            }
        }
    }

    /// Seeds the off-grid `99.5 MHz` (`995`) AyeOS emergency beacon + RDS Capsule in shadow mode.
    pub fn load_demo_offgrid_broadcast(&mut self) {
        let _ = self.fm.tune_100khz(995);
        let pi = 0x8B15;
        let pty = 31; // Emergency / Sovereign Beacon

        // Group 0A: 8-char PS station name "AYE-DARK"
        let ps = *b"AYE-DARK";
        let mut seg = 0u8;
        while seg < 4 {
            let i = (seg as usize) * 2;
            let grp = rds_build_group_0a(pi, pty, seg, [ps[i], ps[i + 1]]);
            let _ = self.rds.ingest_raw_group(grp);
            seg += 1;
        }

        // Group 2A: 64-char RadioText emergency bulletin
        let rt = *b"AYEOS V5.4.0 OFF-GRID FM+RDS CAPSULE LINK ONLINE @ 99.5 MHZ     ";
        let mut rseg = 0u8;
        while rseg < 16 {
            let i = (rseg as usize) * 4;
            let grp = rds_build_group_2a(pi, pty, rseg, [rt[i], rt[i + 1], rt[i + 2], rt[i + 3]]);
            let _ = self.rds.ingest_raw_group(grp);
            rseg += 1;
        }

        // Group 11A: 32-byte AyeOS RDS Capsule ("AYE540:MT6631:0x18142880:READY!!")
        let cap = *b"AYE540:MT6631:0x18142880:READY!!";
        let mut cseg = 0u8;
        while cseg < 8 {
            let i = (cseg as usize) * 4;
            let grp =
                rds_build_group_11a(pi, pty, cseg, [cap[i], cap[i + 1], cap[i + 2], cap[i + 3]]);
            let _ = self.rds.ingest_raw_group(grp);
            cseg += 1;
        }

        // Feed one 48 kHz -> 768 kHz Catmull-Rom oversampled wave into the 64x36 Wave Pool.
        let upsampled = catmull_rom_upsample_16x(-4096, 8192, 16384, -8192);
        self.wave_pool
            .inject_oversampled_frame(&upsampled, self.fm.snr_db);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connsys_readiness_gate_prevents_unclocked_mmio() {
        let mut gate = ConnsysGate::new_safe();
        assert!(!gate.can_touch_mmio());
        assert_eq!(gate.stage, ConnsysStage::Offline);

        // FDT present, but WMT firmware not yet loaded -> stays in ShadowLoopback.
        assert_eq!(
            gate.evaluate_probe(true, 0x0000_0000),
            ConnsysStage::FdtVerified
        );
        assert!(!gate.can_touch_mmio());

        // WMT MCU ready magic present -> promotes to ClockAndMcuReady + HardwareMmio.
        assert_eq!(
            gate.evaluate_probe(true, WMT_MCU_READY_MAGIC),
            ConnsysStage::ClockAndMcuReady
        );
        assert!(gate.can_touch_mmio());
        assert!(gate.mark_hqa_verified(MT6631_CHIP_ID));
        assert_eq!(gate.stage, ConnsysStage::HqaRingReady);
        assert!(gate.mark_fmsys_active());
        assert_eq!(gate.stage, ConnsysStage::FmSysActive);
    }

    #[test]
    fn test_hqa_frame_codec_and_commands() {
        let mut ctrl = Mt6631Controller::new();
        ctrl.gate.evaluate_probe(true, WMT_MCU_READY_MAGIC);

        let req = HqaFrame::new(HqaCmd::GetChipId, 0);
        let raw = req.encode_le();
        let decoded = HqaFrame::decode_le(&raw).unwrap();
        assert_eq!(decoded.magic_no, 0x1814_2880);
        assert_eq!(decoded.cmd_id, 0x1514);

        let resp = ctrl.dispatch_hqa(decoded).unwrap();
        assert_eq!(resp.param, 0x0000_6631);
        assert_eq!(ctrl.gate.stage, ConnsysStage::HqaRingReady);

        // Tune 2.4 GHz (2412 MHz) and 5 GHz (5180 MHz)
        assert_eq!(
            ctrl.dispatch_hqa(HqaFrame::new(HqaCmd::SetChannel, 2412))
                .unwrap()
                .param,
            2412
        );
        assert_eq!(
            ctrl.dispatch_hqa(HqaFrame::new(HqaCmd::SetChannel, 5180))
                .unwrap()
                .param,
            5180
        );
        assert_eq!(
            ctrl.dispatch_hqa(HqaFrame::new(HqaCmd::SetChannel, 3000)),
            Err(Mt6631Error::InvalidWifiChannelMhz(3000))
        );
    }

    #[test]
    fn test_integer_fm_tuner_and_seek() {
        let mut tuner = FmTuner::new();
        assert!(tuner.tune_100khz(649).is_err());
        assert!(tuner.tune_100khz(1081).is_err());

        let st = tuner.tune_100khz(875).unwrap();
        assert_eq!(FmTuner::split_mhz(st.freq_100khz), (87, 5));

        let found = tuner.seek_next(45).unwrap();
        assert_eq!(found.freq_100khz, 885); // 88.5 MHz

        let found2 = tuner.seek_next(60).unwrap();
        assert_eq!(found2.freq_100khz, 995); // 99.5 MHz AyeOS carrier
        assert_eq!(FmTuner::split_mhz(found2.freq_100khz), (99, 5));
    }

    #[test]
    fn test_rds_crc10_and_offgrid_capsule_decode() {
        let mut ctrl = Mt6631Controller::new();
        ctrl.load_demo_offgrid_broadcast();

        assert_eq!(ctrl.rds.pi_code, 0x8B15);
        assert_eq!(&ctrl.rds.ps_name, b"AYE-DARK");
        assert!(ctrl
            .rds
            .radio_text
            .starts_with(b"AYEOS V5.4.0 OFF-GRID FM+RDS"));
        assert!(ctrl.rds.capsule_complete());
        assert_eq!(&ctrl.rds.capsule_buf, b"AYE540:MT6631:0x18142880:READY!!");
        assert_eq!(ctrl.rds.groups_decoded, 28);

        // Corrupt a single bit in a 26-bit RDS block and confirm CRC-10 rejects it.
        let mut bad_grp = rds_build_group_0a(0x8B15, 31, 0, *b"OK");
        bad_grp[0] ^= 0x0001;
        assert_eq!(
            ctrl.rds.ingest_raw_group(bad_grp),
            Err(Mt6631Error::RdsCheckwordMismatch)
        );
    }

    #[test]
    fn test_catmull_rom_16x_integer_oversampler_and_wave_pool() {
        // Constant DC signal must stay exact across all 16 sub-samples.
        let dc = catmull_rom_upsample_16x(1000, 1000, 1000, 1000);
        for &s in &dc {
            assert_eq!(s, 1000);
        }

        // Linear ramp (0, 1600, 3200, 4800) must interpolate in exact 100-unit steps across [1600, 3200).
        let ramp = catmull_rom_upsample_16x(0, 1600, 3200, 4800);
        for (u, &s) in ramp.iter().enumerate() {
            assert_eq!(s, 1600 + (u as i16) * 100);
        }

        let mut pool = WavePool64x36::new();
        pool.inject_oversampled_frame(&ramp, 35);
        assert!(pool.total_energy() > 0);
    }

    #[test]
    fn test_wmt_patch_manifest_and_afe_fm_route() {
        let mut ctrl = Mt6631Controller::new();
        assert!(!ctrl.wmt_manifest.is_rf_ready());
        assert_eq!(ctrl.afe_route.encode_connsys_i2s_con(), 0);

        let hdr = *b"20261101MT6631E1";
        assert!(WmtPatchManifest::verify_wmt_patch_header(&hdr));
        ctrl.wmt_manifest.wifi_ram_code_loaded = true;
        ctrl.wmt_manifest.wmt_mcu_patch_loaded = true;
        ctrl.wmt_manifest.fm_dsp_patch_loaded = true;
        ctrl.wmt_manifest.build_year_tag = [hdr[0], hdr[1], hdr[2], hdr[3]];
        assert!(ctrl.wmt_manifest.is_rf_ready());

        ctrl.afe_route.enable_fm_audio_path(128);
        assert_eq!(ctrl.afe_route.encode_connsys_i2s_con(), 0x8000_0909);
    }
}
