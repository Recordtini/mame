// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/******************************************************************************


    Philips CD-i consoles and games
    -------------------------------

    Preliminary MAME driver by Ryan Holtz
    Help provided by CD-i Fan


*******************************************************************************

STATUS:

  CD-i:
- The SLAVE MCU cannot be low-level emulated until there is proper /DTACK
  support in the 68k core. A2/A1 and D7..D0 are hooked up to Port C bits 1/0
  and Port A respectively, the Read/Write signal is sent to Port D bit 7, and
  /DTACK is received from Port B bit 6. The MCU therefore has the capability to
  pull /DTACK high on a data read in order to tell the 68k to hold off until
  data is ready.

- There is currently a lack of documentation on any of the chips used for
  audio in any of the CD-i models. The CDIC, which was used on Mono-I boards,
  is partially emulated thanks to information provided by CD-i Fan, the author
  of CD-i Emu. Desired documentation includes:
  * GSX38KG307CE46, "ATTEX"
  * Philips IMS66490, "CDIC" ADPCM decoder
  * PC85010 DSP

TODO:

- Screen clocks are a hack right now; they should be exactly CLOCK_A/2. However, the
  MCD-212 documentation states in both tables and timing diagrams that vertical retrace
  has an additional half-line even in non-interlaced mode, which cannot be represented
  in the current screen-timing framework. The input clock has been adjusted downward
  to factor out this half-line, resulting in the expected 50Hz exactly in PAL mode.

- Proper abstraction of the 68070's internal devices (UART, DMA, Timers, etc.)

- Mono-I: Full emulation of the CDIC, as well as the SERVO and SLAVE MCUs

- Mono-II: SERVO and SLAVE I/O device hookup
- Mono-II: DSP56k hookup

*******************************************************************************/

#include "emu.h"
#include "cdi.h"

#define PL_MPEG_IMPLEMENTATION
#include "pl_mpeg.h"

#include "cpu/m6805/m6805.h"
#include "imagedev/cdromimg.h"
#include "machine/timekpr.h"
#include "sound/cdda.h"
#include "sound/mpeg_audio.h"

#include "emupal.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#include "cdrom.h"

#include "cdi.lh"

#include <array>

namespace
{
constexpr uint32_t DVC_VMPEG_ROM_BASE = 0xe40000;
constexpr uint32_t DVC_VMPEG_ROM_MASK = 0x1ffff;
constexpr size_t DVC_PLM_BUFFER_CAPACITY = 2 * 1024 * 1024;
constexpr size_t DVC_AUDIO_ES_BUFFER_CAPACITY = 8 * 1024 * 1024;
constexpr uint32_t DVC_DMA_FALLBACK_MAX_BYTES = 0x2000;
constexpr uint32_t DVC_DMA_AUDIO_SECTOR_BYTES = 2356;
constexpr uint32_t DVC_AUDIO_OUTPUT_CHUNK_SAMPLES = 1152;
constexpr uint32_t DVC_AUDIO_TIMER_GRANULARITY_SAMPLES = 64;
constexpr uint32_t DVC_AUDIO_DAC_TARGET_SAMPLES = DVC_AUDIO_OUTPUT_CHUNK_SAMPLES * 8;
constexpr uint32_t DVC_AUDIO_DAC_MIN_FILL_SAMPLES = DVC_AUDIO_OUTPUT_CHUNK_SAMPLES * 4;
constexpr uint32_t DVC_AUDIO_START_PREBUFFER_SAMPLES = DVC_AUDIO_OUTPUT_CHUNK_SAMPLES * 8;
constexpr uint32_t DVC_AUDIO_RESUME_PREBUFFER_SAMPLES = DVC_AUDIO_OUTPUT_CHUNK_SAMPLES * 4;
constexpr size_t DVC_AUDIO_MAX_FRAMES_PER_DECODE = 4;
constexpr size_t DVC_VIDEO_MAX_FRAMES_PER_DECODE = 2;
constexpr size_t DVC_VIDEO_QUEUE_TARGET = 2;

constexpr uint16_t DVC_FMA_ISR_EOI = 0x0001;
constexpr uint16_t DVC_FMA_ISR_CSU = 0x0002;
constexpr uint16_t DVC_FMA_ISR_UPD = 0x0004;
constexpr uint16_t DVC_FMA_ISR_UNF = 0x0008;
constexpr uint16_t DVC_FMA_ISR_DEC = 0x0010;
constexpr uint16_t DVC_FMA_ISR_POLL = 0x0100;

constexpr uint16_t DVC_FMV_ISR_PIC = 0x0004;
constexpr uint16_t DVC_FMV_ISR_EOD = 0x0008;
constexpr uint16_t DVC_FMV_ISR_NDAT = 0x0020;
constexpr uint16_t DVC_FMV_ISR_DCL = 0x0080;
constexpr uint16_t DVC_FMV_ISR_TIM = 0x0100;
constexpr uint16_t DVC_FMV_ISR_ESI = 0x0200;
constexpr uint16_t DVC_FMV_ISR_EII = 0x0400;
constexpr uint16_t DVC_FMV_ISR_PAI = 0x1000;
constexpr uint16_t DVC_FMV_ISR_VCUP = 0x2000;

constexpr uint8_t SCC_DMA_CSR_COC = 0x80;
constexpr uint8_t SCC_DMA_SEQ_MAC_INC = 0x04;
constexpr uint8_t SCC_DMA_SEQ_DAC_INC = 0x01;

uint16_t dvc_frame_period_90khz(double rate_hz)
{
	return rate_hz > 0.0 ? uint16_t(90000.0 / rate_hz) : 0;
}

uint16_t dvc_rate_code(double rate_hz)
{
	if (rate_hz >= 55.0)
		return 7;
	if (rate_hz >= 45.0)
		return 6;
	if (rate_hz >= 29.5)
		return 5;
	if (rate_hz >= 29.0)
		return 4;
	if (rate_hz >= 24.5)
		return 3;
	if (rate_hz >= 23.9)
		return 2;
	if (rate_hz >= 23.0)
		return 1;
	return 0;
}

uint16_t dvc_reduced_timestamp(int64_t timestamp)
{
	return uint16_t((uint64_t(timestamp) >> 7) & 0x7fff);
}

int dvc_peek_l2_frame_bytes(const std::vector<uint8_t> &buffer, size_t bytes, int bitpos)
{
	static constexpr int bitrate_tab[16] = {
		0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0
	};
	static constexpr int srate_tab[4] = { 44100, 48000, 32000, 0 };

	if ((bitpos < 0) || (bitpos & 7))
		return -1;

	const size_t offset = size_t(bitpos >> 3);
	if ((offset + 4) > bytes)
		return -1;

	const uint32_t header =
		(uint32_t(buffer[offset + 0]) << 24) |
		(uint32_t(buffer[offset + 1]) << 16) |
		(uint32_t(buffer[offset + 2]) << 8) |
		uint32_t(buffer[offset + 3]);

	if ((header & 0xfff00000U) != 0xfff00000U)
		return -1;

	const int version_id = (header >> 19) & 0x3;
	const int layer = (header >> 17) & 0x3;
	const int bitrate_index = (header >> 12) & 0xf;
	const int srate_index = (header >> 10) & 0x3;
	const int padding = (header >> 9) & 0x1;

	if ((version_id != 3) || (layer != 2) || (bitrate_index <= 0) || (bitrate_index >= 15) || (srate_index >= 3))
		return -1;

	return ((144000 * bitrate_tab[bitrate_index]) / srate_tab[srate_index]) + padding;
}

int dvc_count_complete_l2_frames(const std::vector<uint8_t> &buffer, size_t bytes, int bitpos, int max_frames)
{
	int frames = 0;
	int scan_bitpos = bitpos;
	while (frames < max_frames)
	{
		const int frame_bytes = dvc_peek_l2_frame_bytes(buffer, bytes, scan_bitpos);
		if (frame_bytes <= 0)
			break;
		if ((scan_bitpos + frame_bytes * 8) > int(bytes * 8U))
			break;
		scan_bitpos += frame_bytes * 8;
		frames++;
	}
	return frames;
}

uint32_t dvc_scan_dma_fallback_bytes(address_space &program, uint32_t memory_address, bool video, uint8_t stream_filter)
{
	uint32_t total_bytes = 0;
	bool matched_stream_packet = false;

	for (unsigned int packet = 0; packet < 16 && total_bytes < DVC_DMA_FALLBACK_MAX_BYTES; packet++)
	{
		const uint32_t address = (memory_address + total_bytes) & 0x00ffffff;
		const uint8_t b0 = program.read_byte(address + 0);
		const uint8_t b1 = program.read_byte(address + 1);
		const uint8_t b2 = program.read_byte(address + 2);
		const uint8_t b3 = program.read_byte(address + 3);

		if ((b0 != 0x00) || (b1 != 0x00) || (b2 != 0x01))
			break;

		uint32_t packet_bytes = 0;
		const bool is_audio_pes = ((b3 & 0xf0) == 0xc0);
		const bool is_video_pes = ((b3 & 0xf0) == 0xe0);
		const bool is_matching_stream = (is_audio_pes || is_video_pes) && ((b3 & 0x0f) == stream_filter);

		if (b3 == 0xba)
		{
			// MPEG-1 pack headers are a fixed 12 bytes.
			packet_bytes = 12;
		}
		else if (b3 == 0xb9)
		{
			packet_bytes = 4;
		}
		else if ((b3 == 0xbb) || (b3 == 0xbc) || (b3 == 0xbd) || (b3 == 0xbe) || (b3 == 0xbf)
			|| ((b3 & 0xf0) == 0xc0) || ((b3 & 0xf0) == 0xe0))
		{
			const uint8_t b4 = program.read_byte(address + 4);
			const uint8_t b5 = program.read_byte(address + 5);
			packet_bytes = 6 + ((uint32_t(b4) << 8) | uint32_t(b5));
		}
		else
		{
			break;
		}

		if (!packet_bytes)
			break;

		total_bytes += packet_bytes;
		total_bytes = (total_bytes + 1) & ~1U;

		// Audio DMA on Mono-I appears to fetch one selected PES payload at a time.
		// If we greedily consume multiple matching packets from one zero-count burst,
		// the next DMA starts partway through data we've already fed and the audio
		// audibly doubles/stutters. Keep any leading system headers, but stop after
		// the first packet for the active stream.
		if (is_matching_stream)
		{
			matched_stream_packet = true;
			break;
		}

		// If we've already consumed the selected stream packet, don't run into the
		// following one on the same guessed DMA burst.
		if (matched_stream_packet)
			break;
	}

	if (!total_bytes)
		total_bytes = 2;

	total_bytes = std::min(total_bytes, DVC_DMA_FALLBACK_MAX_BYTES);
	return total_bytes;
}

void dvc_set_bits(uint64_t &value, unsigned start, unsigned width, uint64_t bits)
{
	const uint64_t mask = ((uint64_t(1) << width) - 1) << start;
	value = (value & ~mask) | ((bits << start) & mask);
}
} // anonymous namespace

// TODO: NTSC system clock is 30.2098 MHz; additional 4.9152 MHz XTAL provided for UART
#define CLOCK_A 30_MHz_XTAL

#define LOG_DVC             (1U << 1)
#define LOG_QUIZARD_READS   (1U << 2)
#define LOG_QUIZARD_WRITES  (1U << 3)
#define LOG_QUIZARD_OTHER   (1U << 4)
#define LOG_UART            (1U << 5)

#define VERBOSE         (LOG_DVC)
#include "logmacro.h"

#define ENABLE_UART_PRINTING (0)

/*************************
*      Memory maps       *
*************************/

void cdi_state::cdimono1_mem(address_map &map)
{
	map(0x000000, 0xffffff).rw(FUNC(cdi_state::bus_error_r), FUNC(cdi_state::bus_error_w));
	map(0x000000, 0x07ffff).rw(FUNC(cdi_state::plane_r<0>), FUNC(cdi_state::plane_w<0>)).share("plane0");
	map(0x200000, 0x27ffff).rw(FUNC(cdi_state::plane_r<1>), FUNC(cdi_state::plane_w<1>)).share("plane1");
	map(0x300000, 0x303bff).rw(m_cdic, FUNC(cdicdic_device::ram_r), FUNC(cdicdic_device::ram_w));
#if ENABLE_UART_PRINTING
	map(0x301400, 0x301403).r(m_maincpu, FUNC(scc68070_device::uart_loopback_enable));
#endif
	map(0x303c00, 0x303fff).rw(m_cdic, FUNC(cdicdic_device::regs_r), FUNC(cdicdic_device::regs_w));
	map(0x310000, 0x317fff).rw(m_slave_hle, FUNC(cdislave_hle_device::slave_r), FUNC(cdislave_hle_device::slave_w));
	map(0x318000, 0x31ffff).noprw();
	map(0x320000, 0x323fff).rw("mk48t08", FUNC(timekeeper_device::read), FUNC(timekeeper_device::write)).umask16(0xff00);    /* nvram (only low bytes used) */
	map(0x400000, 0x47ffff).r(FUNC(cdi_state::main_rom_r));
	map(0x4fffe0, 0x4fffff).m(m_mcd212, FUNC(mcd212_device::map));
	map(0x500000, 0x57ffff).ram();
	map(0xd00000, 0xdfffff).ram(); // DVC RAM block 1
	map(0xe00000, 0xe7ffff).rw(FUNC(cdi_state::dvc_r), FUNC(cdi_state::dvc_w));
	map(0xe80000, 0xefffff).rw(FUNC(cdi_state::dvc_ram2_r), FUNC(cdi_state::dvc_ram2_w)); // DVC RAM block 2
}

void cdi_state::cdimono2_mem(address_map &map)
{
	map(0x000000, 0x07ffff).ram().share("plane0");
	map(0x200000, 0x27ffff).ram().share("plane1");
#if ENABLE_UART_PRINTING
	map(0x301400, 0x301403).r(m_maincpu, FUNC(scc68070_device::uart_loopback_enable));
#endif
	map(0x320000, 0x323fff).rw("mk48t08", FUNC(timekeeper_device::read), FUNC(timekeeper_device::write)).umask16(0xff00);    /* nvram (only low bytes used) */
	map(0x400000, 0x47ffff).r(FUNC(cdi_state::main_rom_r));
	map(0x4fffe0, 0x4fffff).m(m_mcd212, FUNC(mcd212_device::map));
}

void cdi_state::cdi910_mem(address_map &map)
{
	map(0x000000, 0x07ffff).ram().share("plane0");
	map(0x180000, 0x1fffff).rom().region("maincpu", 0); // boot vectors point here
	map(0x200000, 0x27ffff).ram().share("plane1");
#if ENABLE_UART_PRINTING
	map(0x301400, 0x301403).r(m_maincpu, FUNC(scc68070_device::uart_loopback_enable));
#endif
	map(0x320000, 0x323fff).rw("mk48t08", FUNC(timekeeper_device::read), FUNC(timekeeper_device::write)).umask16(0xff00);    /* nvram (only low bytes used) */
	map(0x4fffe0, 0x4fffff).m(m_mcd212, FUNC(mcd212_device::map));
	map(0x500000, 0xffffff).noprw();
}


/*************************
*      Input ports       *
*************************/

static INPUT_PORTS_START( cdi )
	PORT_START("MOUSEX")
	PORT_BIT(0xffff, 0x000, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(2)

	PORT_START("MOUSEY")
	PORT_BIT(0xffff, 0x000, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(2)

	PORT_START("MOUSEBTN")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_CODE(MOUSECODE_BUTTON1) PORT_NAME("Button 1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_CODE(MOUSECODE_BUTTON2) PORT_NAME("Button 2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_CODE(MOUSECODE_BUTTON3) PORT_NAME("Button 3")
	PORT_BIT(0xf8, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("DEBUGLAY")
	PORT_CONFNAME(0x01, 0x01, "Debug Backdrop / EV")
	PORT_CONFSETTING(0x01, DEF_STR(On))
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFNAME(0x02, 0x02, "Debug Plane A")
	PORT_CONFSETTING(0x02, DEF_STR(On))
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFNAME(0x04, 0x04, "Debug Plane B")
	PORT_CONFSETTING(0x04, DEF_STR(On))
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFNAME(0x08, 0x08, "Debug Cursor")
	PORT_CONFSETTING(0x08, DEF_STR(On))
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFNAME(0x10, 0x00, "Debug EV Force On Top")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x10, DEF_STR(On))
	PORT_CONFNAME(0x20, 0x00, "Debug Raw EV Only")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x20, DEF_STR(On))
	PORT_CONFNAME(0x40, 0x00, "Debug Replace Plane A With EV")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x40, DEF_STR(On))
	PORT_CONFNAME(0x80, 0x00, "Debug EV Ignore Alpha")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x80, DEF_STR(On))
	PORT_CONFNAME(0x100, 0x000, "Debug Swap A/B Transparency")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x100, DEF_STR(On))
	PORT_CONFNAME(0x200, 0x000, "Debug Replace Plane B With EV")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x200, DEF_STR(On))
	PORT_CONFNAME(0x400, 0x000, "Debug A Weight-Black Yields")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x400, DEF_STR(On))
	PORT_CONFNAME(0x800, 0x000, "Debug B Weight-Black Yields")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x800, DEF_STR(On))
	PORT_CONFNAME(0x1000, 0x000, "Debug Force A Front")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x1000, DEF_STR(On))
	PORT_CONFNAME(0x2000, 0x000, "Debug Force B Front")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x2000, DEF_STR(On))
	PORT_CONFNAME(0x4000, 0x000, "Debug Log SHOW_NT")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x4000, DEF_STR(On))
	PORT_CONFNAME(0x8000, 0x000, "Debug A Matte0 Yields")
	PORT_CONFSETTING(0x000, DEF_STR(Off))
	PORT_CONFSETTING(0x8000, DEF_STR(On))
	PORT_CONFNAME(0x10000, 0x0000, "Debug A Matte1 Yields")
	PORT_CONFSETTING(0x0000, DEF_STR(Off))
	PORT_CONFSETTING(0x10000, DEF_STR(On))
	PORT_CONFNAME(0x20000, 0x0000, "Debug B Matte0 Yields")
	PORT_CONFSETTING(0x0000, DEF_STR(Off))
	PORT_CONFSETTING(0x20000, DEF_STR(On))
	PORT_CONFNAME(0x40000, 0x0000, "Debug B Matte1 Yields")
	PORT_CONFSETTING(0x0000, DEF_STR(Off))
	PORT_CONFSETTING(0x40000, DEF_STR(On))
	PORT_CONFNAME(0x80000, 0x00000, "Debug Replace Dominant Plane")
	PORT_CONFSETTING(0x00000, DEF_STR(Off))
	PORT_CONFSETTING(0x80000, DEF_STR(On))
	PORT_CONFNAME(0x100000, 0x000000, "Debug Replace Sparse Plane")
	PORT_CONFSETTING(0x000000, DEF_STR(Off))
	PORT_CONFSETTING(0x100000, DEF_STR(On))
	PORT_CONFNAME(0x200000, 0x000000, "Debug Log Plane Stats")
	PORT_CONFSETTING(0x000000, DEF_STR(Off))
	PORT_CONFSETTING(0x200000, DEF_STR(On))
INPUT_PORTS_END

static INPUT_PORTS_START( cdimono2 )
INPUT_PORTS_END

static INPUT_PORTS_START( quizard )
	PORT_START("P0")
	PORT_DIPNAME( 0x07, 0x05, "Settings" )
	PORT_DIPSETTING(    0x00, "1 Coin, 0 Bonus Limit, 0 Bonus Number" )
	PORT_DIPSETTING(    0x01, "2 Coins, 0 Bonus Limit, 0 Bonus Number" )
	PORT_DIPSETTING(    0x02, "1 Coin, 2 Bonus Limit, 1 Bonus Number" )
	PORT_DIPSETTING(    0x03, "1 Coin, 3 Bonus Limit, 1 Bonus Number" )
	PORT_DIPSETTING(    0x04, "1 Coin, 5 Bonus Limit, 1 Bonus Number" )
	PORT_DIPSETTING(    0x05, "1 Coin, 5 Bonus Limit, 2 Bonus Number" )
	PORT_DIPSETTING(    0x06, "1 Coin, 10 Bonus Limit, 2 Bonus Number" )
	PORT_DIPSETTING(    0x07, "2 Coins, 4 Bonus Limit, 1 Bonus Number" )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0xc8, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("P1")
	PORT_BIT( 0x1f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_SERVICE1 )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Player 1 A")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("Player 1 B")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("Player 1 C")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("Player 2 A")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("Player 2 B")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME("Player 2 C")
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


/***************************
*  Machine Initialization  *
***************************/

void cdi_state::machine_start()
{
	m_dvc_timer = timer_alloc(FUNC(cdi_state::dvc_timer_tick), this);
	m_dvc_video_timer = timer_alloc(FUNC(cdi_state::dvc_video_tick), this);
	m_dvc_audio_timer = timer_alloc(FUNC(cdi_state::dvc_audio_tick), this);

	save_item(NAME(m_dvc_fma_command));
	save_item(NAME(m_dvc_fma_status));
	save_item(NAME(m_dvc_fma_interrupt_status));
	save_item(NAME(m_dvc_fma_interrupt_enable));
	save_item(NAME(m_dvc_fma_interrupt_vector));
	save_item(NAME(m_dvc_fma_stream));
	save_item(NAME(m_dvc_fma_dsp_addr));
	save_item(NAME(m_dvc_fma_dsp_enable));
	save_item(NAME(m_dvc_fma_dclk));

	save_item(NAME(m_dvc_fmv_interrupt_status));
	save_item(NAME(m_dvc_fmv_interrupt_enable));
	save_item(NAME(m_dvc_fmv_interrupt_vector));
	save_item(NAME(m_dvc_fmv_system_command));
	save_item(NAME(m_dvc_fmv_video_command));
	save_item(NAME(m_dvc_fmv_system_control));
	save_item(NAME(m_dvc_fmv_timer_compare));
	save_item(NAME(m_dvc_fmv_frame_rate));
	save_item(NAME(m_dvc_fmv_decoder_command));
	save_item(NAME(m_dvc_fmv_video_data_input_command));
	save_item(NAME(m_dvc_fmv_stream));
	save_item(NAME(m_dvc_fmv_y_offset));
	save_item(NAME(m_dvc_fmv_x_offset));
	save_item(NAME(m_dvc_fmv_y_active));
	save_item(NAME(m_dvc_fmv_x_active));
	save_item(NAME(m_dvc_fmv_y_display));
	save_item(NAME(m_dvc_fmv_x_display));
	save_item(NAME(m_dvc_fmv_window_height));
	save_item(NAME(m_dvc_fmv_window_width));
	save_item(NAME(m_dvc_fmv_decoder_offset_y));
	save_item(NAME(m_dvc_fmv_decoder_offset_x));
	save_item(NAME(m_dvc_fmv_program));
	save_item(NAME(m_dvc_fmv_demux_timestamp));
	save_item(NAME(m_dvc_fmv_last_decoded_timestamp));
	save_item(NAME(m_dvc_image_width));
	save_item(NAME(m_dvc_image_height));
	save_item(NAME(m_dvc_image_rt));
	save_item(NAME(m_dvc_fmv_program_ram));
	save_item(NAME(m_dvc_ram2));

	save_item(NAME(m_dvc_decoder_enabled));
	save_item(NAME(m_dvc_playback_active));
	save_item(NAME(m_dvc_video_visible));
	save_item(NAME(m_dvc_video_show_pending));
	save_item(NAME(m_dvc_fma_started));
	save_item(NAME(m_dvc_fma_pending_stream_change));
	save_item(NAME(m_dvc_audio_output_active));
	save_item(NAME(m_dvc_audio_output_started_once));
	save_item(NAME(m_dvc_audio_last_decoded_valid));
	save_item(NAME(m_dvc_audio_output_level));
	save_item(NAME(m_dvc_audio_last_decoded));
	save_item(NAME(m_dvc_audio_empty_ticks));
	save_item(NAME(m_dvc_fmv_register_update_latch));
	save_item(NAME(m_dvc_fmv_register_update_scroll));
	save_item(NAME(m_dvc_mpeg_ram_enabled));
	save_item(NAME(m_cdic_irq_pending));
	save_item(NAME(m_dvc_mpeg_ram_enable_count));
	save_item(NAME(m_dvc_dma_preview_count));
	save_item(NAME(m_dvc_video_present_accum));
	save_item(NAME(m_dvc_audio_sample_rate));
	save_item(NAME(m_dvc_audio_dac_queued_samples));
	save_item(NAME(m_dvc_audio_dma_last_mac));
	save_item(NAME(m_dvc_audio_dma_span_hint));
	save_item(NAME(m_dvc_audio_es_bytes));
	save_item(NAME(m_dvc_audio_es_bitpos));
	save_item(NAME(m_dvc_audio_mpeg_header));
	save_item(NAME(m_dvc_frame_rate_hz));
	save_item(NAME(m_dvc_audio_dac_last_tick));
	save_item(NAME(m_dvc_dclk_base));

	machine().save().register_postload(save_prepost_delegate(FUNC(cdi_state::dvc_restore_state), this));
}

void cdi_state::machine_reset()
{
	uint16_t *src = &m_main_rom[0];
	uint16_t *dst = &m_plane_ram[0][0];
	memcpy(dst, src, 0x8);
	m_cdic_irq_pending = false;
	dvc_reset();
}

void quizard_state::machine_start()
{
	cdi_state::machine_start();

	save_item(NAME(m_boot_press));

	m_boot_timer = timer_alloc(FUNC(quizard_state::boot_press_tick), this);

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(9600);
}

void quizard_state::machine_reset()
{
	cdi_state::machine_reset();

	m_boot_press = false;
	m_boot_timer->adjust(attotime::from_seconds(13), 1);
	m_mcu_p3 = 0x05; // RTS|RXD
}


/***************************
*  Wait-State Handling     *
***************************/

template<int Channel>
uint16_t cdi_state::plane_r(offs_t offset, uint16_t mem_mask)
{
	m_maincpu->eat_cycles(m_mcd212->ram_dtack_cycle_count<Channel>());
	return m_plane_ram[Channel][offset];
}

template<int Channel>
void cdi_state::plane_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	m_maincpu->eat_cycles(m_mcd212->ram_dtack_cycle_count<Channel>());
	COMBINE_DATA(&m_plane_ram[Channel][offset]);
}

uint16_t cdi_state::main_rom_r(offs_t offset)
{
	m_maincpu->eat_cycles(m_mcd212->rom_dtack_cycle_count());
	return m_main_rom[offset];
}


/**********************
*  BERR Handling      *
**********************/

uint16_t cdi_state::bus_error_r(offs_t offset)
{
	if(!machine().side_effects_disabled())
	{
		m_maincpu->set_buserror_details(offset*2, true, m_maincpu->get_fc());
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	}
	return 0xff;
}

void cdi_state::bus_error_w(offs_t offset, uint16_t data)
{
	if(!machine().side_effects_disabled())
	{
		m_maincpu->set_buserror_details(offset*2, false, m_maincpu->get_fc());
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	}
}

void cdi_state::cdic_dvc_irq_w(int state)
{
	m_cdic_irq_pending = bool(state);

	const bool dvc_pending = ((m_dvc_fma_interrupt_status & m_dvc_fma_interrupt_enable) != 0)
		|| ((m_dvc_fmv_interrupt_status & m_dvc_fmv_interrupt_enable) != 0);

	m_maincpu->in4_w((m_cdic_irq_pending || dvc_pending) ? ASSERT_LINE : CLEAR_LINE);
}


/**********************
*  Quizard Protection *
**********************/

TIMER_CALLBACK_MEMBER(quizard_state::boot_press_tick)
{
	m_boot_press = (bool)param;
	if (m_boot_press)
		m_boot_timer->adjust(attotime::from_msec(250), 0);
}

uint8_t quizard_state::mcu_button_press()
{
	return (uint8_t)m_boot_press;
}

void quizard_state::mcu_rtsn_from_cpu(int state)
{
	LOGMASKED(LOG_UART, "MCU receiving RTSN from CPU: %d\n", state);
}

void quizard_state::mcu_rx_from_cpu(uint8_t data)
{
	LOGMASKED(LOG_UART, "MCU receiving %02x from CPU\n", data);

	transmit_register_setup(data);
}

uint8_t quizard_state::mcu_p0_r()
{
	const uint8_t data = m_inputs[0]->read();
	LOGMASKED(LOG_QUIZARD_READS, "%s: MCU Port 0 Read (%02x)\n", machine().describe_context(), data);
	return data;
}

uint8_t quizard_state::mcu_p1_r()
{
	uint8_t data = m_inputs[1]->read();
	if (BIT(~m_inputs[0]->read(), 4))
		data &= ~(1 << 4);
	LOGMASKED(LOG_QUIZARD_READS, "%s: MCU Port 1 Read (%02x)\n", machine().describe_context(), data);
	return data;
}

uint8_t quizard_state::mcu_p2_r()
{
	const uint8_t data = m_inputs[2]->read();
	LOGMASKED(LOG_QUIZARD_READS, "%s: MCU Port 2 Read (%02x)\n", machine().describe_context(), data);
	return data;
}

uint8_t quizard_state::mcu_p3_r()
{
	LOGMASKED(LOG_QUIZARD_READS, "%s: MCU Port 3 Read (%02x)\n", machine().describe_context(), m_mcu_p3);
	return m_mcu_p3;
}

void quizard_state::mcu_p0_w(uint8_t data)
{
	LOGMASKED(LOG_QUIZARD_WRITES, "%s: MCU Port 0 Write (%02x)\n", machine().describe_context(), data);
}

void quizard_state::mcu_p1_w(uint8_t data)
{
	LOGMASKED(LOG_QUIZARD_WRITES, "%s: MCU Port 1 Write (%02x)\n", machine().describe_context(), data);
}

void quizard_state::mcu_p2_w(uint8_t data)
{
	LOGMASKED(LOG_QUIZARD_WRITES, "%s: MCU Port 2 Write (%02x)\n", machine().describe_context(), data);
}

void quizard_state::mcu_p3_w(uint8_t data)
{
	LOGMASKED(LOG_QUIZARD_WRITES, "%s: MCU Port 3 Write (%02x)\n", machine().describe_context(), data);
	rx_w(BIT(data, 1));
	m_maincpu->uart_ctsn(BIT(data, 6));
}

/*************************
*     DVC cartridge      *
*************************/

void cdi_state::dvc_reset_demux(dvc_demux_state &state)
{
	state = dvc_demux_state();
}

void cdi_state::dvc_restore_state()
{
	dvc_rebuild_external_video();
	dvc_update_irq_timer();
	dvc_update_video_timer();
	dvc_update_audio_timer();
	dvc_update_irq();
	dvc_log_video_state("reset", true);
}

void cdi_state::dvc_apply_video_show_mode()
{
	const uint8_t mode = (m_dvc_video_visible && (m_dvc_video_show_mode == DVC_SHOW_NT))
		? uint8_t(mcd212_device::EXTERNAL_VIDEO_BETWEEN_PLANES)
		: uint8_t(mcd212_device::EXTERNAL_VIDEO_BACKDROP);
	m_mcd212->set_external_video_mode(mode);
}

void cdi_state::dvc_update_irq_timer()
{
	if (!m_dvc_timer)
		return;

	const uint32_t ticks = (uint32_t(m_dvc_fmv_timer_compare) + 1U) * 8U;
	const attotime period = attotime::from_ticks(ticks ? ticks : 1U, 45000);
	m_dvc_timer->adjust(period, 0, period);
}

void cdi_state::dvc_update_video_timer()
{
	if (!m_dvc_video_timer)
		return;

	if (!m_dvc_playback_active)
	{
		m_dvc_video_present_accum = 0;
		m_dvc_video_timer->adjust(attotime::never);
		return;
	}

	const uint32_t frame_period_90khz = std::max<uint32_t>(1U, dvc_frame_period_90khz(m_dvc_frame_rate_hz > 0.0 ? m_dvc_frame_rate_hz : 25.0));
	const attotime period = attotime::from_ticks(frame_period_90khz, 90000);

	// Decode happens more often than display. If we restart the presenter timer
	// on every new packet/frame decode, it can be pushed out indefinitely and
	// the display gets stuck on the first seeded frame.
	if (!m_dvc_video_timer->enabled() || (m_dvc_video_timer->period() != period))
		m_dvc_video_timer->adjust(period, 0, period);
}

void cdi_state::dvc_update_audio_timer()
{
	if (!m_dvc_audio_timer)
		return;

	if (!m_dvc_audio_sample_rate)
	{
		m_dvc_audio_timer->adjust(attotime::never);
		return;
	}

	const attotime period = attotime::from_ticks(DVC_AUDIO_TIMER_GRANULARITY_SAMPLES, m_dvc_audio_sample_rate);
	if (!m_dvc_audio_timer->enabled() || (m_dvc_audio_timer->period() != period))
		m_dvc_audio_timer->adjust(period, 0, period);
}

void cdi_state::dvc_update_audio_dac_fill()
{
	if (!m_dvc_audio_sample_rate)
	{
		m_dvc_audio_dac_queued_samples = 0;
		m_dvc_audio_dac_last_tick = 0;
		return;
	}

	const u64 current_tick = machine().time().as_ticks(m_dvc_audio_sample_rate);
	if (!m_dvc_audio_dac_last_tick)
	{
		m_dvc_audio_dac_last_tick = current_tick;
		return;
	}

	if (current_tick > m_dvc_audio_dac_last_tick)
	{
		const u64 elapsed = current_tick - m_dvc_audio_dac_last_tick;
		if (elapsed >= m_dvc_audio_dac_queued_samples)
			m_dvc_audio_dac_queued_samples = 0;
		else
			m_dvc_audio_dac_queued_samples -= uint32_t(elapsed);
		m_dvc_audio_dac_last_tick = current_tick;
	}
}

void cdi_state::dvc_flush_audio_output(size_t max_samples)
{
	if (!m_dvc_audio_output_active || !m_dvc_audio_sample_rate)
		return;

	dvc_update_audio_dac_fill();

	size_t available_samples = std::min(m_dvc_audio_pcm[0].size(), m_dvc_audio_pcm[1].size());
	if (!available_samples)
		return;

	if (max_samples)
		available_samples = std::min(available_samples, max_samples);
	if (!available_samples)
		return;

	std::vector<int16_t> interleaved(available_samples * 2);
	for (size_t index = 0; index < available_samples; index++)
	{
		interleaved[index * 2 + 0] = m_dvc_audio_pcm[0].front();
		interleaved[index * 2 + 1] = m_dvc_audio_pcm[1].front();
		m_dvc_audio_pcm[0].pop_front();
		m_dvc_audio_pcm[1].pop_front();
	}

	m_dvc_audio_output_level[0] = interleaved[(available_samples - 1) * 2 + 0];
	m_dvc_audio_output_level[1] = interleaved[(available_samples - 1) * 2 + 1];

	dmadac_sound_device *dac_list[2] = { m_dmadac[0], m_dmadac[1] };
	dmadac_transfer(dac_list, 2, 1, 2, available_samples, interleaved.data());
	m_dvc_audio_dac_queued_samples += uint32_t(available_samples);
}

TIMER_CALLBACK_MEMBER(cdi_state::dvc_timer_tick)
{
	const uint32_t debug = m_debug_layers ? uint32_t(m_debug_layers->read()) : uint32_t(mcd212_device::DEBUG_LAYER_ALL);
	m_mcd212->set_debug_layer_mask(uint8_t(debug & uint32_t(mcd212_device::DEBUG_LAYER_ALL)));
	m_mcd212->set_debug_video_mask(uint32_t(debug >> 4));
	dvc_raise_fmv_irq(DVC_FMV_ISR_TIM);
	if (m_dvc_fma_started)
		dvc_raise_fma_irq(DVC_FMA_ISR_POLL);
	if (m_dvc_fmv_register_update_latch && m_dvc_fmv_register_update_scroll)
	{
		m_dvc_fmv_register_update_latch = false;
		dvc_raise_fmv_irq(DVC_FMV_ISR_VCUP | DVC_FMV_ISR_DCL);
	}
}

TIMER_CALLBACK_MEMBER(cdi_state::dvc_video_tick)
{
	const uint32_t debug = m_debug_layers ? uint32_t(m_debug_layers->read()) : uint32_t(mcd212_device::DEBUG_LAYER_ALL);
	m_mcd212->set_debug_layer_mask(uint8_t(debug & uint32_t(mcd212_device::DEBUG_LAYER_ALL)));
	m_mcd212->set_debug_video_mask(uint16_t((debug >> 4) & 0x7ff));
	if (!m_dvc_playback_active)
		return;

	if (m_dvc_video_queue.empty())
	{
		dvc_raise_fmv_irq(DVC_FMV_ISR_NDAT);
		return;
	}

	dvc_present_next_frame();
}

TIMER_CALLBACK_MEMBER(cdi_state::dvc_audio_tick)
{
	const uint32_t debug = m_debug_layers ? uint32_t(m_debug_layers->read()) : uint32_t(mcd212_device::DEBUG_LAYER_ALL);
	m_mcd212->set_debug_layer_mask(uint8_t(debug & uint32_t(mcd212_device::DEBUG_LAYER_ALL)));
	m_mcd212->set_debug_video_mask(uint16_t((debug >> 4) & 0x7ff));
	const uint32_t current_dclk = uint32_t(machine().time().as_ticks(45000) - m_dvc_dclk_base);
	m_dvc_fma_dclk = current_dclk;
	constexpr size_t chunk_samples = DVC_AUDIO_OUTPUT_CHUNK_SAMPLES;
	size_t available_samples = std::min(m_dvc_audio_pcm[0].size(), m_dvc_audio_pcm[1].size());
	const size_t resume_threshold = m_dvc_audio_output_started_once ? DVC_AUDIO_RESUME_PREBUFFER_SAMPLES : DVC_AUDIO_START_PREBUFFER_SAMPLES;
	bool started_output = false;

	// MiSTer enables the FMA DSP when the stream start time is reached; the
	// audio output FIFO fill level is a separate local concern.
	if (m_dvc_fma_started
		&& !m_dvc_fma_dsp_enable
		&& m_dvc_audio_demux_state.system_clock_reference_start_time_valid
		&& int64_t(current_dclk) >= m_dvc_audio_demux_state.system_clock_reference_start_time)
	{
		m_dvc_fma_dsp_enable = 1;
		LOGMASKED(LOG_DVC, "%s: DVC audio DSP enable dclk=%u start=%d\n",
			machine().describe_context(),
			unsigned(current_dclk),
			int(m_dvc_audio_demux_state.system_clock_reference_start_time));
	}

	if (!m_dvc_audio_output_active
		&& m_dvc_fma_started
		&& m_dvc_audio_demux_state.system_clock_reference_start_time_valid
		&& int64_t(current_dclk) >= m_dvc_audio_demux_state.system_clock_reference_start_time)
	{
		if (available_samples >= resume_threshold)
		{
			m_dvc_audio_output_active = true;
			m_dvc_audio_output_started_once = true;
			m_dvc_audio_empty_ticks = 0;
			started_output = true;
			LOGMASKED(LOG_DVC, "%s: DVC audio output start queued=%u\n",
				machine().describe_context(),
				unsigned(available_samples));
		}
	}

	if (!m_dvc_audio_sample_rate)
		return;

	dvc_update_audio_dac_fill();

	// MiSTer's audio decoder runs continuously behind a FIFO; our software model
	// only gets new PCM when we explicitly ask pl_mpeg for more. If the local
	// FIFO is running low but compressed bytes are already buffered, opportunisti-
	// cally decode again here instead of waiting for another DMA burst.
	if (m_dvc_audio_decoder)
	{
		const size_t queued_before = available_samples;
		const size_t buffered_bytes = (m_dvc_audio_es_bitpos >= 0 && m_dvc_audio_es_bytes >= size_t(m_dvc_audio_es_bitpos >> 3))
			? (m_dvc_audio_es_bytes - size_t(m_dvc_audio_es_bitpos >> 3))
			: 0;
		const int complete_frames = dvc_count_complete_l2_frames(m_dvc_audio_es, m_dvc_audio_es_bytes, m_dvc_audio_es_bitpos, 4);
		const size_t desired_pcm = std::max<size_t>(resume_threshold, DVC_AUDIO_DAC_TARGET_SAMPLES);
		if ((queued_before < desired_pcm)
			&& (buffered_bytes >= 4)
			&& (complete_frames >= (m_dvc_audio_output_active ? 1 : 2)))
		{
			dvc_decode_audio();
			available_samples = std::min(m_dvc_audio_pcm[0].size(), m_dvc_audio_pcm[1].size());
			dvc_update_audio_dac_fill();
		}
	}

	if (!available_samples)
	{
		if (!m_dvc_audio_output_active)
			return;

		if (m_dvc_audio_empty_ticks != 0xffff)
			m_dvc_audio_empty_ticks++;
		// Let the DAC's own FIFO drain naturally instead of synthesizing held
		// samples into the stream. The native decoder path already fixed the
		// compressed-data side; remaining clicks are more likely to come from
		// these artificial software filler blocks than from true starvation.
		return;
	}

	m_dvc_audio_empty_ticks = 0;
	if (!m_dvc_audio_output_active)
		return;

	const size_t target_fill = started_output ? DVC_AUDIO_DAC_TARGET_SAMPLES : DVC_AUDIO_DAC_MIN_FILL_SAMPLES;
	if (m_dvc_audio_dac_queued_samples < target_fill)
	{
		const size_t missing = target_fill - m_dvc_audio_dac_queued_samples;
		const size_t flush_samples = std::min(available_samples, std::max(chunk_samples, missing));
		if (flush_samples)
			dvc_flush_audio_output(flush_samples);
	}
}

uint8_t cdi_state::dvc_iack_r()
{
	const bool fma_pending = (m_dvc_fma_interrupt_status & m_dvc_fma_interrupt_enable) != 0;
	const bool fmv_pending = (m_dvc_fmv_interrupt_status & m_dvc_fmv_interrupt_enable) != 0;

	if (fma_pending)
		return m_dvc_fma_interrupt_vector & 0xff;
	if (fmv_pending)
		return (m_dvc_fmv_interrupt_vector >> 3) & 0xff;
	return 0xff;
}

uint8_t cdi_state::cdimono1_iack4_r()
{
	const bool fma_pending = (m_dvc_fma_interrupt_status & m_dvc_fma_interrupt_enable) != 0;
	const bool fmv_pending = (m_dvc_fmv_interrupt_status & m_dvc_fmv_interrupt_enable) != 0;

	if (fma_pending || fmv_pending)
		return dvc_iack_r();

	return m_cdic->intack_r();
}

void cdi_state::dvc_reset()
{
	LOGMASKED(LOG_DVC, "%s: DVC reset\n", machine().describe_context());
	m_dvc_dclk_base = machine().time().as_ticks(45000);

	m_dvc_fma_command = 0;
	m_dvc_fma_status = 0;
	m_dvc_fma_interrupt_status = 0;
	m_dvc_fma_interrupt_enable = 0;
	m_dvc_fma_interrupt_vector = 0;
	m_dvc_fma_stream = 0;
	m_dvc_fma_dsp_addr = 0;
	m_dvc_fma_dsp_enable = 0;
	m_dvc_fma_dclk = 0;

	m_dvc_fmv_interrupt_status = 0;
	m_dvc_fmv_interrupt_enable = 0;
	m_dvc_fmv_interrupt_vector = 0;
	m_dvc_fmv_system_command = 0;
	m_dvc_fmv_video_command = 0;
	m_dvc_fmv_system_control = 0;
	m_dvc_fmv_timer_compare = 56 - 1;
	m_dvc_fmv_frame_rate = 0;
	m_dvc_fmv_decoder_command = 0;
	m_dvc_fmv_video_data_input_command = 0;
	m_dvc_fmv_stream = 0;
	m_dvc_fmv_y_offset = 0;
	m_dvc_fmv_x_offset = 0;
	m_dvc_fmv_y_active = 0;
	m_dvc_fmv_x_active = 0;
	m_dvc_fmv_y_display = 0;
	m_dvc_fmv_x_display = 0;
	m_dvc_fmv_window_height = 0;
	m_dvc_fmv_window_width = 0;
	m_dvc_fmv_decoder_offset_y = 0;
	m_dvc_fmv_decoder_offset_x = 0;
	m_dvc_fmv_program = 0;
	m_dvc_image_width = 0;
	m_dvc_image_height = 0;
	m_dvc_image_rt = 0;
	m_dvc_fmv_program_ram.fill(0);
	m_dvc_ram2.fill(0);

	m_dvc_decoder_enabled = false;
	m_dvc_playback_active = false;
	m_dvc_video_visible = false;
	m_dvc_video_show_pending = false;
	m_dvc_fmv_program_end_seen = false;
	m_dvc_fma_started = false;
	m_dvc_fma_pending_stream_change = false;
	m_dvc_audio_output_active = false;
	m_dvc_audio_output_started_once = false;
	m_dvc_audio_last_decoded_valid = false;
	m_dvc_fmv_register_update_latch = false;
	m_dvc_fmv_register_update_scroll = false;
	m_dvc_mpeg_ram_enabled = false;
	m_dvc_mpeg_ram_enable_count = 0;
	m_dvc_dma_preview_count = 0;
	m_dvc_video_present_accum = 0;
	m_dvc_audio_sample_rate = 0;
	m_dvc_audio_dac_queued_samples = 0;
	m_dvc_audio_dma_last_mac = 0;
	m_dvc_audio_dma_span_hint = 0;
	m_dvc_audio_output_level[0] = 0;
	m_dvc_audio_output_level[1] = 0;
	m_dvc_audio_last_decoded[0] = 0;
	m_dvc_audio_last_decoded[1] = 0;
	m_dvc_audio_empty_ticks = 0;
	m_dvc_audio_dac_last_tick = 0;
	m_dvc_frame_rate_hz = 25.0;
	m_dvc_fmv_demux_timestamp = 0;
	m_dvc_fmv_last_decoded_timestamp = 0;
	m_dvc_audio_pcm[0].clear();
	m_dvc_audio_pcm[1].clear();
	m_dvc_audio_output_active = false;
	m_dvc_audio_output_started_once = false;

	m_dmadac[0]->enable(0);
	m_dmadac[1]->enable(0);

	dvc_reset_video_decoder();
	dvc_reset_audio_decoder();
	dvc_rebuild_external_video();
	dvc_update_irq_timer();
	dvc_update_video_timer();
	dvc_update_audio_timer();
	dvc_update_irq();
}

void cdi_state::dvc_reset_video_decoder()
{
	LOGMASKED(LOG_DVC, "%s: DVC reset video decoder\n", machine().describe_context());
	if (m_dvc_video_plm)
	{
		plm_video_destroy(m_dvc_video_plm);
		m_dvc_video_plm = nullptr;
		m_dvc_video_buffer = nullptr;
	}

	m_dvc_video_queue.clear();
	m_dvc_display_frame = dvc_video_frame();
	m_dvc_image_width = 0;
	m_dvc_image_height = 0;
	m_dvc_image_rt = 0;
	m_dvc_fmv_program_end_seen = false;
	m_dvc_fmv_video_data_input_command &= ~0x4000;
	dvc_reset_demux(m_dvc_video_demux_state);

	m_dvc_video_buffer = plm_buffer_create_with_capacity(DVC_PLM_BUFFER_CAPACITY);
	m_dvc_video_plm = m_dvc_video_buffer ? plm_video_create_with_buffer(m_dvc_video_buffer, TRUE) : nullptr;
}

void cdi_state::dvc_reset_audio_decoder()
{
	LOGMASKED(LOG_DVC, "%s: DVC reset audio decoder\n", machine().describe_context());

	dvc_reset_demux(m_dvc_audio_demux_state);
	m_dvc_audio_decoder.reset();
	if (m_dvc_audio_es.size() != DVC_AUDIO_ES_BUFFER_CAPACITY)
		m_dvc_audio_es.resize(DVC_AUDIO_ES_BUFFER_CAPACITY);
	m_dvc_audio_es_bytes = 0;
	m_dvc_audio_es_bitpos = 0;
	m_dvc_audio_mpeg_header = 0;
	m_dvc_audio_pcm[0].clear();
	m_dvc_audio_pcm[1].clear();
	m_dvc_audio_sample_rate = 0;
	m_dvc_audio_dac_queued_samples = 0;
	m_dvc_audio_dma_last_mac = 0;
	m_dvc_audio_dma_span_hint = 0;
	m_dvc_audio_output_active = false;
	m_dvc_audio_output_started_once = false;
	m_dvc_audio_last_decoded_valid = false;
	m_dvc_audio_output_level[0] = 0;
	m_dvc_audio_output_level[1] = 0;
	m_dvc_audio_last_decoded[0] = 0;
	m_dvc_audio_last_decoded[1] = 0;
	m_dvc_audio_empty_ticks = 0;
	m_dvc_audio_dac_last_tick = 0;
	m_dmadac[0]->enable(0);
	m_dmadac[1]->enable(0);
	dvc_update_audio_timer();
	// VMPEG feeds the MPEG audio decoder a byte-stream FIFO; keeping the
	// native decoder byte-aligned between frames avoids false sync at random
	// bit offsets inside the payload.
	m_dvc_audio_decoder = std::make_unique<mpeg_audio>(m_dvc_audio_es.data(), mpeg_audio::L2, false, 8);
}

void cdi_state::dvc_log_video_state(const char *reason, bool force)
{
	const uint32_t signature =
		(m_dvc_video_visible ? 0x00000001 : 0x00000000) |
		(m_dvc_video_show_pending ? 0x00000002 : 0x00000000) |
		(m_dvc_playback_active ? 0x00000004 : 0x00000000) |
		(m_dvc_decoder_enabled ? 0x00000008 : 0x00000000) |
		(uint32_t(m_dvc_video_show_mode & 0x03) << 4) |
		(uint32_t(std::min<size_t>(m_dvc_video_queue.size(), 0xff)) << 8) |
		(uint32_t(m_dvc_fmv_system_command) << 16);

	if (!force
		&& (signature == m_dvc_last_video_log_signature)
		&& (m_dvc_display_frame.width == m_dvc_last_video_log_frame_w)
		&& (m_dvc_display_frame.height == m_dvc_last_video_log_frame_h))
	{
		return;
	}

	m_dvc_last_video_log_signature = signature;
	m_dvc_last_video_log_frame_w = m_dvc_display_frame.width;
	m_dvc_last_video_log_frame_h = m_dvc_display_frame.height;

	LOGMASKED(LOG_DVC, "%s: DVC video state [%s] vis=%d show_pending=%d mode=%u play=%d dec=%d queue=%u frame=%ux%u sys=%04x vid=%04x ev_pending=%d ev_active=%d ev_dirty=%d icm_ev=%d\n",
		machine().describe_context(),
		reason,
		m_dvc_video_visible ? 1 : 0,
		m_dvc_video_show_pending ? 1 : 0,
		unsigned(m_dvc_video_show_mode),
		m_dvc_playback_active ? 1 : 0,
		m_dvc_decoder_enabled ? 1 : 0,
		unsigned(m_dvc_video_queue.size()),
		m_dvc_display_frame.width,
		m_dvc_display_frame.height,
		m_dvc_fmv_system_command,
		m_dvc_fmv_video_command,
		m_mcd212->external_video_pending_enabled() ? 1 : 0,
		m_mcd212->external_video_active_enabled() ? 1 : 0,
		m_mcd212->external_video_dirty() ? 1 : 0,
		m_mcd212->external_video_icm_enabled() ? 1 : 0);
}

void cdi_state::dvc_update_irq()
{
	const bool fma_pending = (m_dvc_fma_interrupt_status & m_dvc_fma_interrupt_enable) != 0;
	const bool fmv_pending = (m_dvc_fmv_interrupt_status & m_dvc_fmv_interrupt_enable) != 0;
	LOGMASKED(LOG_DVC, "%s: DVC IRQ update fma=%04x/%04x fmv=%04x/%04x -> %s\n",
		machine().describe_context(),
		m_dvc_fma_interrupt_status,
		m_dvc_fma_interrupt_enable,
		m_dvc_fmv_interrupt_status,
		m_dvc_fmv_interrupt_enable,
		(fma_pending || fmv_pending) ? "ASSERT" : "CLEAR");
	cdic_dvc_irq_w(m_cdic_irq_pending ? ASSERT_LINE : CLEAR_LINE);
}

void cdi_state::dvc_raise_fmv_irq(uint16_t bits)
{
	if (!bits)
		return;

	LOGMASKED(LOG_DVC, "%s: DVC raise FMV IRQ %04x\n", machine().describe_context(), bits);
	m_dvc_fmv_interrupt_status |= bits;
	dvc_update_irq();
}

void cdi_state::dvc_raise_fma_irq(uint16_t bits)
{
	if (!bits)
		return;

	LOGMASKED(LOG_DVC, "%s: DVC raise FMA IRQ %04x\n", machine().describe_context(), bits);
	m_dvc_fma_interrupt_status |= bits;
	dvc_update_irq();
}

void cdi_state::dvc_handle_fmv_command(uint16_t data)
{
	LOGMASKED(LOG_DVC, "%s: DVC FMV system command %04x\n", machine().describe_context(), data);
	m_dvc_fmv_system_command = data;

	if (data & 0x2000)
	{
		m_dvc_decoder_enabled = false;
		m_dvc_playback_active = false;
		m_dvc_video_visible = false;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_HIDDEN;
		dvc_reset_video_decoder();
		dvc_rebuild_external_video();
	}

	if (data & 0x0100)
	{
		m_dvc_playback_active = false;
		m_dvc_video_visible = false;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_HIDDEN;
		dvc_reset_video_decoder();
		dvc_rebuild_external_video();
	}

	if (data & 0x1000)
		m_dvc_decoder_enabled = true;

	if (data & 0x8000)
		dvc_handle_dma_transfer(true);

	if (data & 0x0008)
	{
		m_dvc_playback_active = true;
		m_dvc_decoder_enabled = true;
		if (m_dvc_display_frame.pixels.empty() && !m_dvc_video_queue.empty())
			dvc_present_next_frame();
	}

	if (data & 0x0010)
	{
		m_dvc_playback_active = false;
		m_dvc_video_visible = false;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_HIDDEN;
		dvc_rebuild_external_video();
	}

	if (data & 0x0020)
	{
		m_dvc_playback_active = true;
		if (m_dvc_display_frame.pixels.empty() && !m_dvc_video_queue.empty())
			dvc_present_next_frame();
	}

	if (data & 0x0040)
	{
		m_dvc_playback_active = false;
		dvc_present_next_frame();
	}

	if (data & 0x0080)
	{
		m_dvc_playback_active = false;
		m_dvc_video_visible = false;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_HIDDEN;
		dvc_rebuild_external_video();
	}

	dvc_update_video_timer();
	dvc_log_video_state("fmv system cmd", true);
}

void cdi_state::dvc_handle_fmv_video_command(uint16_t data)
{
	LOGMASKED(LOG_DVC, "%s: DVC FMV video command %04x\n", machine().describe_context(), data);
	m_dvc_fmv_video_command = data;

	const bool show_t = (data & 0x0020) && !(data & 0x0200) && !(data & 0x0400);
	const bool show_nt = (data & 0x0200) != 0;
	const bool show_pending_nt = (data & 0x0400) != 0;
	const bool hide_only = (data & 0x0100) && !show_t && !show_nt && !show_pending_nt;

	if (hide_only)
	{
		m_dvc_video_visible = false;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_HIDDEN;
		dvc_rebuild_external_video();
	}

	if (show_t)
	{
		m_dvc_video_visible = true;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_T;
		dvc_rebuild_external_video();
	}

	if (show_nt)
	{
		m_dvc_video_visible = true;
		m_dvc_video_show_pending = false;
		m_dvc_video_show_mode = DVC_SHOW_NT;
		dvc_rebuild_external_video();
	}

	if (show_pending_nt)
	{
		m_dvc_video_show_pending = true;
		m_dvc_video_show_mode = DVC_SHOW_NT;
	}

	if (data & 0x0008)
	{
		m_dvc_fmv_register_update_latch = true;
		m_dvc_fmv_register_update_scroll = BIT(data, 2);
	}

	dvc_log_video_state("fmv video cmd", true);
}

void cdi_state::dvc_handle_fma_command(uint16_t data)
{
	LOGMASKED(LOG_DVC, "%s: DVC FMA command %04x\n", machine().describe_context(), data);
	m_dvc_fma_command = data;

	if (data & 0x0001)
	{
		m_dvc_fma_started = false;
		m_dvc_fma_dsp_enable = 0;
		m_dvc_fma_status = 0;
		m_dvc_audio_output_active = false;
		m_dvc_audio_output_started_once = false;
		dvc_reset_audio_decoder();
	}

	if (data & 0x0002)
		m_dvc_fma_started = true;

	if (data & 0x8000)
	{
		m_dvc_fma_started = true;
		m_dvc_fma_status &= ~0x08;
		dvc_handle_dma_transfer(false);
	}
}

void cdi_state::dvc_handle_dma_transfer(bool video)
{
	auto &dma = m_maincpu->dma().channel[1];
	address_space &program = m_maincpu->space(AS_PROGRAM);
	uint32_t memory_address = dma.memory_address_counter & 0x00fffffe;
	uint32_t device_address = dma.device_address_counter;
	const bool increment_memory = (dma.sequence_control & SCC_DMA_SEQ_MAC_INC) != 0;
	const bool increment_device = (dma.sequence_control & SCC_DMA_SEQ_DAC_INC) != 0;
	const uint8_t stream_filter = video ? (m_dvc_fmv_stream & 0x0f) : (m_dvc_fma_stream & 0x0f);
	uint32_t transfer_words = dma.transfer_counter ? (uint32_t(dma.transfer_counter) + 1) : 1;

	// VMPEG audio bursts consistently program exact compressed payload sizes in
	// the DMA transfer counter (for example 0x0480 -> 2304 bytes). Consuming
	// "count + 1" words on every nonzero FMA transfer pulls two stray bytes from
	// the next ring slot into the MPEG audio elementary stream and causes tiny
	// recurring clicks. Keep the zero-count final-word fallback for the SCC68070
	// quirk, but treat nonzero FMA counts as exact payload words.
	if (!video && dma.transfer_counter && increment_memory && !increment_device)
		transfer_words = uint32_t(dma.transfer_counter);

	LOGMASKED(LOG_DVC, "%s: DVC DMA %s mac=%06x dac=%06x count=%04x words=%u seq=%02x\n",
		machine().describe_context(),
		video ? "video" : "audio",
		memory_address,
		device_address,
		dma.transfer_counter,
		transfer_words,
		dma.sequence_control);

	if (!video && increment_memory && !increment_device)
	{
		if (m_dvc_audio_dma_last_mac)
		{
			const uint32_t delta = (memory_address >= m_dvc_audio_dma_last_mac)
				? (memory_address - m_dvc_audio_dma_last_mac)
				: (m_dvc_audio_dma_last_mac - memory_address);

			if (delta == DVC_DMA_AUDIO_SECTOR_BYTES)
				m_dvc_audio_dma_span_hint = DVC_DMA_AUDIO_SECTOR_BYTES;
			else if (delta == 2304)
				m_dvc_audio_dma_span_hint = 2304;
		}

		m_dvc_audio_dma_last_mac = memory_address;
	}

	if (video && (m_dvc_fmv_interrupt_status & DVC_FMV_ISR_NDAT))
	{
		m_dvc_fmv_interrupt_status &= ~DVC_FMV_ISR_NDAT;
		dvc_update_irq();
	}

	if (video && m_dvc_dma_preview_count < 16)
	{
		const uint16_t w0 = program.read_word((memory_address + 0x0) & 0x00fffffe);
		const uint16_t w1 = program.read_word((memory_address + 0x2) & 0x00fffffe);
		const uint16_t w2 = program.read_word((memory_address + 0x4) & 0x00fffffe);
		const uint16_t w3 = program.read_word((memory_address + 0x6) & 0x00fffffe);
		const uint16_t w4 = program.read_word((memory_address + 0x8) & 0x00fffffe);
		const uint16_t w5 = program.read_word((memory_address + 0xa) & 0x00fffffe);
		const uint16_t w6 = program.read_word((memory_address + 0xc) & 0x00fffffe);
		const uint16_t w7 = program.read_word((memory_address + 0xe) & 0x00fffffe);
		LOGMASKED(LOG_DVC, "%s: DVC DMA preview mac=%06x [%04x %04x %04x %04x %04x %04x %04x %04x]\n",
			machine().describe_context(),
			memory_address,
			w0, w1, w2, w3, w4, w5, w6, w7);
		m_dvc_dma_preview_count++;
	}

	// The MCD251 docs describe a DMA request/ack FIFO path, and MiSTer keeps
	// VMPEG requesting until the transfer completes. MAME's SCC68070 model
	// currently leaves VMPEG DMA bursts at a visible count of 0, so fall back
	// to pulling one contiguous MPEG packet span from RAM when that happens.
	if ((dma.transfer_counter == 0) && increment_memory && !increment_device)
	{
		uint32_t fallback_bytes = dvc_scan_dma_fallback_bytes(program, memory_address, video, stream_filter);
		transfer_words = fallback_bytes / 2;
		LOGMASKED(LOG_DVC, "%s: DVC DMA %s using zero-count fallback bytes=%u words=%u\n",
			machine().describe_context(),
			video ? "video" : "audio",
			fallback_bytes,
			transfer_words);
	}
	// MiSTer's SCC68070 DMA engine treats the transfer counter as "remaining words - 1",
	// so a count of 0 still performs one final 16-bit transfer before completion.
	for (uint32_t remaining = transfer_words; remaining > 0; remaining--)
	{
		dvc_feed_word(video, program.read_word(memory_address));

		if (increment_memory)
			memory_address = (memory_address + 2) & 0x00fffffe;
		if (increment_device)
			device_address = (device_address + 2) & 0x00fffffe;
	}

	dma.memory_address_counter = memory_address;
	dma.device_address_counter = device_address;
	dma.transfer_counter = 0;
	dma.channel_status |= SCC_DMA_CSR_COC;

	if (video)
		dvc_decode_video();
	else
		dvc_decode_audio();
}

void cdi_state::dvc_feed_word(bool video, uint16_t data)
{
	const uint8_t bytes[2] = { uint8_t(data >> 8), uint8_t(data) };
	dvc_feed_bytes(video, bytes, 2);
}

void cdi_state::dvc_feed_bytes(bool video, const uint8_t *data, size_t bytes)
{
	if (video && !m_dvc_video_plm)
		dvc_reset_video_decoder();
	if (!video && !m_dvc_audio_decoder)
		dvc_reset_audio_decoder();

	if (!data || !bytes)
		return;

	for (size_t index = 0; index < bytes; index++)
		dvc_process_demux_byte(video, data[index]);
}

void cdi_state::dvc_process_demux_byte(bool video, uint8_t data)
{
	dvc_demux_state &demux = video ? m_dvc_video_demux_state : m_dvc_audio_demux_state;
	plm_buffer_t *const video_buffer = video ? m_dvc_video_buffer : nullptr;
	const uint8_t stream_filter = video ? (m_dvc_fmv_stream & 0x0f) : (m_dvc_fma_stream & 0x0f);
	const uint32_t dclk = video ? 0 : m_dvc_fma_dclk;
	const bool packet_body = demux.packet_body;
	const bool end_of_packet = demux.packet_length_decreasing && (demux.packet_length == 1);

	demux.decoding_timestamp_updated = false;
	demux.event_program_end = false;
	if (demux.packet_length_decreasing && demux.packet_length)
		demux.packet_length--;

	switch (demux.phase)
	{
	case dvc_demux_state::PACK5:
		demux.phase = dvc_demux_state::IDLE;
		LOGMASKED(LOG_DVC, "%s: DVC %s pack scr=%08x\n",
			machine().describe_context(),
			video ? "FMV" : "FMA",
			uint32_t(demux.system_clock_reference));
		break;

	case dvc_demux_state::PACK4:
		demux.phase = dvc_demux_state::PACK5;
		dvc_set_bits(demux.system_clock_reference, 0, 7, data >> 1);
		break;

	case dvc_demux_state::PACK3:
		demux.phase = dvc_demux_state::PACK4;
		dvc_set_bits(demux.system_clock_reference, 7, 8, data);
		break;

	case dvc_demux_state::PACK2:
		demux.phase = dvc_demux_state::PACK3;
		dvc_set_bits(demux.system_clock_reference, 15, 7, data >> 1);
		break;

	case dvc_demux_state::PACK1:
		demux.phase = dvc_demux_state::PACK2;
		dvc_set_bits(demux.system_clock_reference, 22, 8, data);
		break;

	case dvc_demux_state::PACK0:
		demux.phase = dvc_demux_state::PACK1;
		dvc_set_bits(demux.system_clock_reference, 30, 3, data >> 1);
		break;

	case dvc_demux_state::PES8:
		demux.phase = dvc_demux_state::IDLE;
		demux.decoding_timestamp = demux.dts_present ? int64_t(demux.decoding_timestamp_temp) : int64_t(demux.presentation_timestamp);
		demux.decoding_timestamp_updated = true;
		if (!demux.system_clock_reference_start_time_valid)
		{
			demux.system_clock_reference_start_time_valid = true;
			demux.system_clock_reference_start_time = int64_t(dclk) + int64_t(demux.presentation_timestamp >> 1) - int64_t(demux.system_clock_reference >> 1);
		}
		LOGMASKED(LOG_DVC, "%s: DVC demux %s timestamp=%04x stream=%x\n",
			machine().describe_context(),
			video ? "FMV" : "FMA",
			dvc_reduced_timestamp(demux.decoding_timestamp),
			stream_filter);
		break;

	case dvc_demux_state::PES_DTS4:
		if (BIT(data, 0))
		{
			dvc_set_bits(demux.decoding_timestamp_temp, 0, 7, data >> 1);
			demux.packet_body = true;
			demux.phase = dvc_demux_state::PES8;
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES_DTS3:
		demux.phase = dvc_demux_state::PES_DTS4;
		dvc_set_bits(demux.decoding_timestamp_temp, 7, 8, data);
		break;

	case dvc_demux_state::PES_DTS2:
		if (BIT(data, 0))
		{
			demux.phase = dvc_demux_state::PES_DTS3;
			dvc_set_bits(demux.decoding_timestamp_temp, 15, 7, data >> 1);
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES_DTS1:
		demux.phase = dvc_demux_state::PES_DTS2;
		dvc_set_bits(demux.decoding_timestamp_temp, 22, 8, data);
		break;

	case dvc_demux_state::PES_DTS0:
		if ((data & 0xf1) == 0x11)
		{
			demux.phase = dvc_demux_state::PES_DTS1;
			dvc_set_bits(demux.decoding_timestamp_temp, 30, 3, data >> 1);
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES7:
		if (BIT(data, 0))
		{
			dvc_set_bits(demux.presentation_timestamp, 0, 7, data >> 1);
			if (demux.dts_present)
			{
				demux.phase = dvc_demux_state::PES_DTS0;
			}
			else
			{
				demux.packet_body = true;
				demux.phase = dvc_demux_state::PES8;
			}
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES6:
		demux.phase = dvc_demux_state::PES7;
		dvc_set_bits(demux.presentation_timestamp, 7, 8, data);
		break;

	case dvc_demux_state::PES5:
		if (BIT(data, 0))
		{
			demux.phase = dvc_demux_state::PES6;
			dvc_set_bits(demux.presentation_timestamp, 15, 7, data >> 1);
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES4:
		demux.phase = dvc_demux_state::PES5;
		dvc_set_bits(demux.presentation_timestamp, 22, 8, data);
		break;

	case dvc_demux_state::PES3:
		demux.phase = dvc_demux_state::PES2;
		break;

	case dvc_demux_state::PES2:
		if ((data & 0xf1) == 0x21)
		{
			dvc_set_bits(demux.presentation_timestamp, 30, 3, data >> 1);
			demux.dts_present = false;
			demux.phase = dvc_demux_state::PES4;
		}
		else if ((data & 0xf1) == 0x31)
		{
			dvc_set_bits(demux.presentation_timestamp, 30, 3, data >> 1);
			demux.dts_present = true;
			demux.phase = dvc_demux_state::PES4;
		}
		else if (data == 0x0f)
		{
			demux.packet_body = true;
			demux.phase = dvc_demux_state::IDLE;
		}
		else if ((data & 0xc0) == 0x40)
		{
			demux.phase = dvc_demux_state::PES3;
		}
		else if (data == 0xff)
		{
			demux.phase = dvc_demux_state::PES2;
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::PES1:
		demux.phase = dvc_demux_state::PES2;
		demux.packet_length = (demux.packet_length & 0xff00) | data;
		demux.packet_length_decreasing = true;
		break;

	case dvc_demux_state::PES0:
		demux.phase = dvc_demux_state::PES1;
		demux.packet_length = (demux.packet_length & 0x00ff) | (uint16_t(data) << 8);
		break;

	case dvc_demux_state::MAGIC_MATCH:
		if (data == 0xba)
		{
			demux.phase = dvc_demux_state::PACK0;
		}
		else if (!video && ((data & 0xf0) == 0xc0))
		{
			if ((data & 0x0f) == stream_filter)
				demux.phase = dvc_demux_state::PES0;
		}
		else if (video && ((data & 0xf0) == 0xe0))
		{
			if ((data & 0x0f) == stream_filter)
				demux.phase = dvc_demux_state::PES0;
		}
		else if (data == 0xb9)
		{
			demux.event_program_end = true;
			demux.phase = dvc_demux_state::IDLE;
			LOGMASKED(LOG_DVC, "%s: DVC demux %s program end\n",
				machine().describe_context(),
				video ? "FMV" : "FMA");
		}
		else
		{
			demux.phase = dvc_demux_state::IDLE;
		}
		break;

	case dvc_demux_state::MAGIC2:
		if (data == 0x01)
			demux.phase = dvc_demux_state::MAGIC_MATCH;
		else if (data == 0x00)
			demux.phase = dvc_demux_state::MAGIC2;
		else
			demux.phase = dvc_demux_state::IDLE;
		break;

	case dvc_demux_state::MAGIC0:
		demux.phase = (data == 0x00) ? dvc_demux_state::MAGIC2 : dvc_demux_state::IDLE;
		break;

	case dvc_demux_state::IDLE:
	default:
		if (!packet_body && data == 0x00)
			demux.phase = dvc_demux_state::MAGIC0;
		else
			demux.phase = dvc_demux_state::IDLE;
		break;
	}

	if (end_of_packet)
	{
		demux.packet_length_decreasing = false;
		demux.packet_body = false;
	}

	if (packet_body)
	{
		if (video && video_buffer)
		{
			plm_buffer_write(video_buffer, &data, 1);
		}
		else if (!video && (m_dvc_audio_es_bytes < m_dvc_audio_es.size()))
		{
			// Keep a large, fixed backing store so the MPEG audio decoder can
			// continue to reference the same base pointer while we slide
			// consumed bytes down in-place between bursts.
			if ((m_dvc_audio_es_bitpos >= 8192) && ((m_dvc_audio_es_bitpos & 7) == 0))
			{
				const size_t consumed_bytes = size_t(m_dvc_audio_es_bitpos >> 3);
				if (consumed_bytes && consumed_bytes <= m_dvc_audio_es_bytes)
				{
					const size_t remaining = m_dvc_audio_es_bytes - consumed_bytes;
					std::memmove(m_dvc_audio_es.data(), m_dvc_audio_es.data() + consumed_bytes, remaining);
					m_dvc_audio_es_bytes = uint32_t(remaining);
					m_dvc_audio_es_bitpos -= int(consumed_bytes * 8);
				}
			}

			if (m_dvc_audio_es_bytes < m_dvc_audio_es.size())
				m_dvc_audio_es[m_dvc_audio_es_bytes++] = data;
		}
	}

	if (video && demux.decoding_timestamp_updated)
		m_dvc_fmv_demux_timestamp = dvc_reduced_timestamp(demux.decoding_timestamp);
	if (demux.event_program_end)
	{
		if (video_buffer)
			plm_buffer_signal_end(video_buffer);

		if (video)
		{
			m_dvc_fmv_program_end_seen = true;
			dvc_raise_fmv_irq(DVC_FMV_ISR_EII);
		}
		else
		{
			m_dvc_fma_status |= 0x01;
			dvc_raise_fma_irq(DVC_FMA_ISR_EOI);
		}
	}
}

void cdi_state::dvc_decode_video()
{
	if (!m_dvc_video_plm)
		return;

	const size_t buffered_before = m_dvc_video_buffer ? plm_buffer_get_remaining(m_dvc_video_buffer) : 0;
	bool decoded_any = false;

	if (const double rate_hz = plm_video_get_framerate(m_dvc_video_plm); rate_hz > 0.0)
	{
		m_dvc_frame_rate_hz = rate_hz;
		m_dvc_image_rt = dvc_rate_code(rate_hz);
	}

	size_t decoded_frames = 0;
	while (m_dvc_video_queue.size() < DVC_VIDEO_QUEUE_TARGET && decoded_frames < DVC_VIDEO_MAX_FRAMES_PER_DECODE)
	{
		plm_frame_t *const frame = plm_video_decode(m_dvc_video_plm);
		if (!frame)
			break;

		LOGMASKED(LOG_DVC, "%s: DVC decoded video frame %dx%d\n", machine().describe_context(), frame->width, frame->height);

		dvc_video_frame queued;
		queued.width = frame->width;
		queued.height = frame->height;
		queued.pixels.resize(size_t(frame->width) * size_t(frame->height));

		std::vector<uint8_t> rgba(size_t(frame->width) * size_t(frame->height) * 4);
		plm_frame_to_rgba(frame, rgba.data(), frame->width * 4);
		for (size_t index = 0; index < queued.pixels.size(); index++)
		{
			const uint8_t *const pixel = &rgba[index * 4];
			queued.pixels[index] = rgb_t(0xff, pixel[0], pixel[1], pixel[2]);
		}

		m_dvc_image_width = frame->width;
		m_dvc_image_height = frame->height;
		m_dvc_fmv_last_decoded_timestamp = m_dvc_fmv_demux_timestamp;
		m_dvc_fmv_video_data_input_command |= 0x4000;
		m_dvc_video_queue.push_back(std::move(queued));
		decoded_any = true;
		decoded_frames++;
	}

	if (decoded_any)
	{
		m_dvc_fmv_interrupt_status &= ~DVC_FMV_ISR_NDAT;
		dvc_update_irq();
	}
	else if (m_dvc_decoder_enabled && m_dvc_video_queue.empty())
	{
		const size_t buffered_after = m_dvc_video_buffer ? plm_buffer_get_remaining(m_dvc_video_buffer) : 0;
		LOGMASKED(LOG_DVC, "%s: DVC FMV needs more data buffered_before=%u buffered_after=%u\n",
			machine().describe_context(),
			unsigned(buffered_before),
			unsigned(buffered_after));
		dvc_raise_fmv_irq(DVC_FMV_ISR_NDAT);
	}

	dvc_update_video_timer();
}

void cdi_state::dvc_decode_audio()
{
	if (!m_dvc_audio_decoder)
		return;

	const int limit_bits = int(m_dvc_audio_es_bytes * 8U);
	if ((m_dvc_audio_es_bitpos < 0) || (m_dvc_audio_es_bitpos > limit_bits))
	{
		LOGMASKED(LOG_DVC, "%s: DVC audio bitstream pointer out of range bitpos=%d limit=%d, resetting decoder state\n",
			machine().describe_context(),
			m_dvc_audio_es_bitpos,
			limit_bits);
		m_dvc_audio_es_bitpos = 0;
		m_dvc_audio_decoder->clear();
	}

	bool decoded_any = false;
	size_t decoded_samples = 0;
	size_t decoded_frames = 0;
	std::array<short, 1152 * 2> decoded{};
	for (;;)
	{
		if (decoded_frames >= DVC_AUDIO_MAX_FRAMES_PER_DECODE)
			break;
		const int start_bitpos = m_dvc_audio_es_bitpos;
		const int frame_bytes = dvc_peek_l2_frame_bytes(m_dvc_audio_es, m_dvc_audio_es_bytes, start_bitpos);
		if ((frame_bytes > 0) && ((start_bitpos + frame_bytes * 8) > limit_bits))
			break;
		int next_bitpos = m_dvc_audio_es_bitpos;
		int output_samples = 0;
		int sample_rate = 0;
		int channels = 0;
		const bool complete = m_dvc_audio_decoder->decode_buffer(
			next_bitpos,
			int(m_dvc_audio_es_bytes * 8U),
			decoded.data(),
			output_samples,
			sample_rate,
			channels);
		if (!complete || (next_bitpos <= start_bitpos) || !output_samples)
			break;
		if (next_bitpos > limit_bits)
		{
			LOGMASKED(LOG_DVC, "%s: DVC audio decoder advanced past buffered data bitpos=%d limit=%d, stopping decode\n",
				machine().describe_context(),
				next_bitpos,
				limit_bits);
			break;
		}

		// FMV audio is fed as a 44.1 kHz byte stream. If the decoder suddenly
		// "finds" a different sample rate mid-stream, that's almost certainly a
		// false sync inside the buffered payload rather than a real format
		// change, so rescan from the next byte instead of retuning playback.
		if ((sample_rate > 0) && m_dvc_audio_sample_rate && (uint32_t(sample_rate) != m_dvc_audio_sample_rate))
		{
			LOGMASKED(LOG_DVC, "%s: DVC audio resync on suspect sample rate %d Hz at bitpos=%d current=%u\n",
				machine().describe_context(),
				sample_rate,
				start_bitpos,
				unsigned(m_dvc_audio_sample_rate));
			m_dvc_audio_decoder->clear();
			m_dvc_audio_es_bitpos = std::min(limit_bits, start_bitpos + 8);
			continue;
		}

		if ((sample_rate > 0) && (m_dvc_audio_sample_rate != uint32_t(sample_rate)))
		{
			m_dvc_audio_sample_rate = uint32_t(sample_rate);
			m_dvc_audio_dac_queued_samples = 0;
			m_dvc_audio_dac_last_tick = machine().time().as_ticks(m_dvc_audio_sample_rate);
			LOGMASKED(LOG_DVC, "%s: DVC audio sample rate %u Hz\n",
				machine().describe_context(),
				unsigned(m_dvc_audio_sample_rate));
			m_dmadac[0]->enable(1);
			m_dmadac[0]->set_frequency(sample_rate);
			m_dmadac[0]->set_volume(0x100);
			m_dmadac[1]->enable(1);
			m_dmadac[1]->set_frequency(sample_rate);
			m_dmadac[1]->set_volume(0x100);
			dvc_update_audio_timer();
		}

		if ((start_bitpos & 7) == 0)
		{
			const size_t start_byte = size_t(start_bitpos >> 3);
			for (size_t offset = start_byte; (offset + 3) < m_dvc_audio_es_bytes && offset < (start_byte + 32); offset++)
			{
				if ((m_dvc_audio_es[offset + 0] == 0xff) && ((m_dvc_audio_es[offset + 1] & 0xe0) == 0xe0))
				{
					m_dvc_audio_mpeg_header =
						(uint32_t(m_dvc_audio_es[offset + 0]) << 24) |
						(uint32_t(m_dvc_audio_es[offset + 1]) << 16) |
						(uint32_t(m_dvc_audio_es[offset + 2]) << 8) |
						uint32_t(m_dvc_audio_es[offset + 3]);
					break;
				}
			}
		}

		if (channels <= 0)
			channels = 1;
		if (channels > 2)
			channels = 2;

		for (int index = 0; index < output_samples; index++)
		{
			const int16_t raw_left = decoded[index * channels + 0];
			const int16_t raw_right = (channels > 1) ? decoded[index * channels + 1] : raw_left;
			m_dvc_audio_pcm[0].push_back(raw_left);
			m_dvc_audio_pcm[1].push_back(raw_right);
			m_dvc_audio_last_decoded[0] = raw_left;
			m_dvc_audio_last_decoded[1] = raw_right;
			m_dvc_audio_last_decoded_valid = true;
		}

		m_dvc_audio_es_bitpos = next_bitpos;
		if ((m_dvc_audio_es_bitpos >= 16384) && ((m_dvc_audio_es_bitpos & 7) == 0))
		{
			const size_t consumed_bytes = size_t(m_dvc_audio_es_bitpos >> 3);
			if (consumed_bytes && consumed_bytes <= m_dvc_audio_es_bytes)
			{
				const size_t remaining = m_dvc_audio_es_bytes - consumed_bytes;
				std::memmove(m_dvc_audio_es.data(), m_dvc_audio_es.data() + consumed_bytes, remaining);
				m_dvc_audio_es_bytes = uint32_t(remaining);
				m_dvc_audio_es_bitpos -= int(consumed_bytes * 8);
			}
		}

		decoded_samples += size_t(output_samples);
		decoded_any = true;
		decoded_frames++;
	}

	if (decoded_any)
	{
		LOGMASKED(LOG_DVC, "%s: DVC decoded audio samples count=%u queued=%u\n",
			machine().describe_context(),
			unsigned(decoded_samples),
			unsigned(std::min(m_dvc_audio_pcm[0].size(), m_dvc_audio_pcm[1].size())));
		if (!(m_dvc_fma_status & 0x10))
		{
			m_dvc_fma_status |= 0x10;
			dvc_raise_fma_irq(DVC_FMA_ISR_DEC);
		}

		m_dvc_fma_status |= 0x04;
		dvc_raise_fma_irq(DVC_FMA_ISR_UPD);
		if (m_dvc_fma_pending_stream_change)
		{
			m_dvc_fma_pending_stream_change = false;
			dvc_raise_fma_irq(DVC_FMA_ISR_CSU);
		}

		if (m_dvc_audio_output_active)
		{
			dvc_update_audio_dac_fill();
			if (m_dvc_audio_dac_queued_samples < DVC_AUDIO_DAC_MIN_FILL_SAMPLES)
			{
				const size_t available = std::min(m_dvc_audio_pcm[0].size(), m_dvc_audio_pcm[1].size());
				const size_t missing = DVC_AUDIO_DAC_TARGET_SAMPLES - std::min<uint32_t>(m_dvc_audio_dac_queued_samples, DVC_AUDIO_DAC_TARGET_SAMPLES);
				if (available)
					dvc_flush_audio_output(std::min(available, std::max<size_t>(DVC_AUDIO_OUTPUT_CHUNK_SAMPLES, missing)));
			}
		}

	}
}

void cdi_state::dvc_present_next_frame()
{
	if (m_dvc_video_queue.empty())
		return;

	m_dvc_display_frame = std::move(m_dvc_video_queue.front());
	m_dvc_video_queue.pop_front();
	const bool last_program_frame =
		m_dvc_fmv_program_end_seen &&
		m_dvc_video_queue.empty() &&
		m_dvc_video_plm &&
		plm_video_has_ended(m_dvc_video_plm);
	uint32_t sample_luma_min = 0xff;
	uint32_t sample_luma_max = 0x00;
	uint32_t sample_nonblack = 0;
	if (!m_dvc_display_frame.pixels.empty())
	{
		const size_t step = std::max<size_t>(size_t(1), m_dvc_display_frame.pixels.size() / 256);
		for (size_t i = 0; i < m_dvc_display_frame.pixels.size(); i += step)
		{
			const uint32_t pixel = m_dvc_display_frame.pixels[i];
			const uint32_t r = (pixel >> 16) & 0xff;
			const uint32_t g = (pixel >> 8) & 0xff;
			const uint32_t b = pixel & 0xff;
			const uint32_t luma = (r * 77 + g * 150 + b * 29) >> 8;
			sample_luma_min = std::min(sample_luma_min, luma);
			sample_luma_max = std::max(sample_luma_max, luma);
			if ((r | g | b) != 0)
				sample_nonblack++;
		}
	}
	LOGMASKED(LOG_DVC, "%s: DVC present frame %dx%d queue=%u show_pending=%d sample_nonblack=%u luma=%u-%u\n",
		machine().describe_context(),
		m_dvc_display_frame.width,
		m_dvc_display_frame.height,
		unsigned(m_dvc_video_queue.size()),
		m_dvc_video_show_pending ? 1 : 0,
		sample_nonblack,
		sample_luma_min,
		sample_luma_max);

	if (m_dvc_video_show_pending)
	{
		m_dvc_video_show_pending = false;
		m_dvc_video_visible = true;
		m_dvc_video_show_mode = DVC_SHOW_NT;
	}
	if (m_dvc_fmv_register_update_latch && !m_dvc_fmv_register_update_scroll)
	{
		m_dvc_fmv_register_update_latch = false;
		dvc_raise_fmv_irq(DVC_FMV_ISR_VCUP | DVC_FMV_ISR_DCL);
	}

	dvc_log_video_state("present frame", true);
	dvc_rebuild_external_video();
	dvc_raise_fmv_irq(DVC_FMV_ISR_PIC);
	if (last_program_frame)
	{
		m_dvc_fmv_program_end_seen = false;
		dvc_raise_fmv_irq(DVC_FMV_ISR_EOD);
	}
}

void cdi_state::dvc_rebuild_external_video()
{
	const uint32_t debug = m_debug_layers ? uint32_t(m_debug_layers->read()) : uint32_t(mcd212_device::DEBUG_LAYER_ALL);
	m_mcd212->set_debug_layer_mask(uint8_t(debug & uint32_t(mcd212_device::DEBUG_LAYER_ALL)));
	m_mcd212->set_debug_video_mask(uint16_t((debug >> 4) & 0x7ff));
	m_mcd212->clear_external_video();
	LOGMASKED(LOG_DVC, "%s: DVC rebuild ext video visible=%d pixels=%u active=%dx%d window=%dx%d display=%dx%d offset=%dx%d crop=%dx%d\n",
		machine().describe_context(),
		m_dvc_video_visible ? 1 : 0,
		unsigned(m_dvc_display_frame.pixels.size()),
		m_dvc_fmv_x_active,
		m_dvc_fmv_y_active,
		m_dvc_fmv_window_width,
		m_dvc_fmv_window_height,
		m_dvc_fmv_x_display,
		m_dvc_fmv_y_display,
		m_dvc_fmv_x_offset,
		m_dvc_fmv_y_offset,
		m_dvc_fmv_decoder_offset_x,
		m_dvc_fmv_decoder_offset_y);
	dvc_log_video_state("rebuild ext video");

	if (!m_dvc_video_visible || m_dvc_display_frame.pixels.empty())
	{
		LOGMASKED(LOG_DVC, "%s: DVC ext video skip reason=%s\n",
			machine().describe_context(),
			!m_dvc_video_visible ? "hidden" : "no display frame");
		dvc_apply_video_show_mode();
		m_mcd212->set_external_video_enable(false);
		return;
	}

	bitmap_rgb32 &bitmap = m_mcd212->external_video();
	const int src_x = std::min<int>(m_dvc_fmv_decoder_offset_x, m_dvc_display_frame.width);
	const int src_y = std::min<int>(m_dvc_fmv_decoder_offset_y, m_dvc_display_frame.height);
	const int max_copy_w = int(m_dvc_display_frame.width) - src_x;
	const int max_copy_h = int(m_dvc_display_frame.height) - src_y;
	// Repeat Offender programs tiny DECWIN values while still using a normal
	// 384x280 active FMV area. Treat 0/1-sized DECWIN as "no explicit crop",
	// and scale the decoded source into the active output rectangle.
	const int source_w = std::max(0, std::min<int>((m_dvc_fmv_window_width > 1) ? m_dvc_fmv_window_width : max_copy_w, max_copy_w));
	const int source_h = std::max(0, std::min<int>((m_dvc_fmv_window_height > 1) ? m_dvc_fmv_window_height : max_copy_h, max_copy_h));
	// MiSTer's frame player uses the decoded/window dimensions for the actual
	// displayed picture area; the "active" registers don't appear to size the
	// final overlay. Horizontal units are half-resolution relative to the
	// MCD212's 768-pixel-wide backdrop plane, so expand width and X position.
	int draw_w = std::max(0, source_w * 2);
	int draw_h = std::max(0, source_h);
	// Titles like Repeat Offender leave Xd/Yd at zero and rely on Xo/Yo for
	// placement, while others program explicit display positions.
	// Both Xd and Xo are programmed in half-resolution horizontal units, so
	// either source must be expanded to the 768-pixel MCD212 backdrop space.
	int dest_x = m_dvc_fmv_x_display ? (int(m_dvc_fmv_x_display) * 2) : (int(m_dvc_fmv_x_offset) * 2);
	int dest_y = m_dvc_fmv_y_display ? int(m_dvc_fmv_y_display) : int(m_dvc_fmv_y_offset);

	// Some titles program a full active FMV area but still leave Xd at zero and
	// an Xo/Yo pair that would place the scaled window partly off-screen if
	// interpreted literally. When that happens, treat the active dimensions as
	// the intended display size and clamp the picture back onto the external
	// video plane instead of cropping off the right/bottom edges.
	if (!m_dvc_fmv_x_display && !m_dvc_fmv_y_display)
	{
		const int active_draw_w = std::max(0, int(m_dvc_fmv_x_active) * 2);
		const int active_draw_h = std::max(0, int(m_dvc_fmv_y_active));
		if ((dest_x + draw_w) > bitmap.width() && active_draw_w > 0 && active_draw_w <= bitmap.width())
		{
			draw_w = active_draw_w;
			dest_x = 0;
		}
		if (active_draw_h > 0 && active_draw_h <= bitmap.height())
		{
			const bool using_full_width = (draw_w == bitmap.width());
			const bool y_offscreen = (dest_y + draw_h) > bitmap.height();
			const bool active_is_full_height = (active_draw_h >= 240);
			if ((y_offscreen || (using_full_width && active_is_full_height)) && active_draw_h >= draw_h)
			{
				draw_h = active_draw_h;
				dest_y = 0;
			}
		}
	}

	if (draw_w > bitmap.width())
		draw_w = bitmap.width();
	if (draw_h > bitmap.height())
		draw_h = bitmap.height();
	dest_x = std::clamp(dest_x, 0, std::max(0, bitmap.width() - draw_w));
	dest_y = std::clamp(dest_y, 0, std::max(0, bitmap.height() - draw_h));
	LOGMASKED(LOG_DVC, "%s: DVC ext video map src=%dx%d+%d+%d draw=%dx%d dest=%d,%d\n",
		machine().describe_context(),
		source_w,
		source_h,
		src_x,
		src_y,
		draw_w,
		draw_h,
		dest_x,
		dest_y);

	if (source_w <= 0 || source_h <= 0 || draw_w <= 0 || draw_h <= 0
		|| dest_x >= bitmap.width() || dest_y >= bitmap.height())
	{
		LOGMASKED(LOG_DVC, "%s: DVC ext video skip reason=invalid map src=%dx%d draw=%dx%d dest=%d,%d bitmap=%dx%d\n",
			machine().describe_context(),
			source_w,
			source_h,
			draw_w,
			draw_h,
			dest_x,
			dest_y,
			bitmap.width(),
			bitmap.height());
		dvc_apply_video_show_mode();
		m_mcd212->set_external_video_enable(false);
		return;
	}

	for (int y = 0; y < draw_h && (dest_y + y) < bitmap.height(); y++)
	{
		uint32_t *const dst = &bitmap.pix(dest_y + y, dest_x);
		const int sample_y = src_y + ((y * source_h) / draw_h);
		const uint32_t *const src = &m_dvc_display_frame.pixels[size_t(sample_y) * m_dvc_display_frame.width];
		for (int x = 0; x < draw_w && (dest_x + x) < bitmap.width(); x++)
		{
			const int sample_x = src_x + ((x * source_w) / draw_w);
			dst[x] = 0xff000000 | (src[sample_x] & 0x00ffffff);
		}
	}

	dvc_apply_video_show_mode();
	m_mcd212->set_external_video_enable(true);
	dvc_log_video_state("rebuild ext video done", true);
}

uint16_t cdi_state::dvc_r(offs_t offset, uint16_t mem_mask)
{
	const uint32_t address = 0xe00000 + (offset << 1);
	const uint32_t current_dclk = uint32_t(machine().time().as_ticks(45000) - m_dvc_dclk_base);
	m_dvc_fma_dclk = current_dclk;

	if (m_dvc_rom.found() && address >= DVC_VMPEG_ROM_BASE && address <= 0xe7fffe)
	{
		const uint32_t rom_offset = 0x40000 + ((address - DVC_VMPEG_ROM_BASE) & DVC_VMPEG_ROM_MASK);
		const uint16_t data = (m_dvc_rom[rom_offset] << 8) | m_dvc_rom[rom_offset + 1];
		LOGMASKED(LOG_DVC, "%s: dvc_r: %08x = %04x & %04x\n", machine().describe_context(), address, data, mem_mask);
		return data;
	}

	if ((address & 0xffff) >= 0x4800 && (address & 0xffff) <= 0x7ffe)
	{
		const uint16_t data = m_dvc_fmv_program_ram[((address & 0xffff) - 0x4800) >> 1];
		LOGMASKED(LOG_DVC, "%s: dvc_r: %08x = %04x & %04x\n", machine().describe_context(), address, data, mem_mask);
		return data;
	}

	uint16_t data = 0;
	switch (address & 0xffff)
	{
	case 0x3000: data = m_dvc_fma_command; break;
	case 0x3002: data = 0x0200 | m_dvc_fma_status; break;
	case 0x3004: data = 0x0007; break;
	case 0x3006: data = 0x0900; break;
	case 0x3008: data = m_dvc_fma_stream & 0x000f; break;
	case 0x300a: data = m_dvc_fma_stream & 0x000f; break;
	case 0x300c: data = m_dvc_fma_interrupt_vector; break;
	case 0x300e: data = 0x0042; break;
	case 0x3010: data = current_dclk >> 16; break;
	case 0x3012: data = current_dclk & 0xffff; break;
	case 0x3014: data = uint16_t(m_dvc_audio_mpeg_header >> 16); break;
	case 0x3016: data = uint16_t(m_dvc_audio_mpeg_header & 0xffff); break;
	case 0x3018: data = m_dvc_fma_dsp_enable & 0x0001; break;
	case 0x301a:
		data = m_dvc_fma_interrupt_status;
		if (!machine().side_effects_disabled())
		{
			m_dvc_fma_interrupt_status = 0;
			dvc_update_irq();
		}
		break;
	case 0x301c: data = m_dvc_fma_interrupt_enable; break;
	case 0x3024: data = 0x0004; break;

	case 0x4002: data = m_dvc_image_width; break;
	case 0x4004: data = m_dvc_image_height; break;
	case 0x4006: data = m_dvc_image_rt; break;
	case 0x4052: data = m_dvc_image_width; break;
	case 0x4054: data = m_dvc_image_height; break;
	case 0x4056: data = m_dvc_image_rt; break;
	case 0x405e: data = 0x2000; break;
	case 0x4060: data = m_dvc_fmv_interrupt_enable; break;
	case 0x4062:
		data = m_dvc_fmv_interrupt_status;
		if (!machine().side_effects_disabled())
		{
			m_dvc_fmv_interrupt_status = 0;
			dvc_update_irq();
		}
		break;
	case 0x4064: data = m_dvc_fmv_timer_compare; break;
	case 0x406c: data = m_dvc_fmv_y_offset; break;
	case 0x406e: data = m_dvc_fmv_x_offset; break;
	case 0x4070: data = m_dvc_fmv_y_active; break;
	case 0x4072: data = m_dvc_fmv_x_active; break;
	case 0x4074: data = m_dvc_fmv_y_display; break;
	case 0x4076: data = m_dvc_fmv_x_display; break;
	case 0x4078: data = m_dvc_fmv_window_height; break;
	case 0x407a: data = m_dvc_fmv_window_width; break;
	case 0x407c: data = m_dvc_fmv_decoder_offset_y; break;
	case 0x407e: data = m_dvc_fmv_decoder_offset_x; break;
	case 0x4088:
		data = m_dvc_fmv_decoder_command;
		if (m_dvc_decoder_enabled)
			data |= 0x0042;
		break;
	case 0x408c: data = m_dvc_fmv_video_data_input_command; break;
	case 0x40da: data = m_dvc_fmv_program; break;
	case 0x4098: data = current_dclk >> 6; break;
	case 0x409c: data = 0; break;
	case 0x409e: data = 0xfe96; break;
	case 0x40a0: data = m_dvc_fmv_last_decoded_timestamp; break;
	case 0x40a4: data = uint16_t(std::min<size_t>(31, m_dvc_video_queue.size())); break;
	case 0x40a8: data = dvc_frame_period_90khz(m_dvc_frame_rate_hz); break;
	case 0x40aa: data = dvc_frame_period_90khz(m_dvc_frame_rate_hz); break;
	case 0x40ac: data = m_dvc_fmv_frame_rate; break;
	case 0x40c0: data = m_dvc_fmv_system_command; break;
	case 0x40c2: data = m_dvc_fmv_video_command; break;
	case 0x40c4: data = m_dvc_fmv_stream & 0x000f; break;
	case 0x40c6: data = m_dvc_fmv_system_control; break;
	case 0x40dc: data = m_dvc_fmv_interrupt_vector; break;
	case 0x40e6: data = 0; break;
	default:
		LOGMASKED(LOG_DVC, "%s: dvc_r: %08x = 0000 & %04x\n", machine().describe_context(), address, mem_mask);
		return 0;
	}

	LOGMASKED(LOG_DVC, "%s: dvc_r: %08x = %04x & %04x\n", machine().describe_context(), address, data, mem_mask);
	return data;
}

void cdi_state::dvc_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	const uint32_t address = 0xe00000 + (offset << 1);

	LOGMASKED(LOG_DVC, "%s: dvc_w: %08x = %04x & %04x\n", machine().describe_context(), address, data, mem_mask);

	if (!m_dvc_mpeg_ram_enabled && m_dvc_mpeg_ram_enable_count < 0x40)
	{
		m_dvc_mpeg_ram_enable_count++;
		if (m_dvc_mpeg_ram_enable_count == 0x40)
		{
			LOGMASKED(LOG_DVC, "%s: DVC MPEG RAM enabled\n", machine().describe_context());
			m_dvc_mpeg_ram_enabled = true;
		}
	}

	switch (address & 0xffff)
	{
	case 0x3000:
		COMBINE_DATA(&m_dvc_fma_command);
		dvc_handle_fma_command(m_dvc_fma_command);
		break;

	case 0x3008:
	case 0x300a:
		COMBINE_DATA(&m_dvc_fma_stream);
		m_dvc_fma_stream &= 0x000f;
		m_dvc_fma_pending_stream_change = true;
		break;

	case 0x300c:
		COMBINE_DATA(&m_dvc_fma_interrupt_vector);
		break;

	case 0x301c:
		COMBINE_DATA(&m_dvc_fma_interrupt_enable);
		dvc_update_irq();
		break;

	case 0x3018:
		// MiSTer exposes RUN as the decoder's internal enable state. The host
		// writes here during setup, but playback actually starts from the
		// demux-timed SCR/PTS path rather than this register write.
		break;

	case 0x3022:
		COMBINE_DATA(&m_dvc_fma_dsp_addr);
		break;

	case 0x3024:
		break;

	case 0x4002: COMBINE_DATA(&m_dvc_image_width); break;
	case 0x4004: COMBINE_DATA(&m_dvc_image_height); break;
	case 0x4006: COMBINE_DATA(&m_dvc_image_rt); break;
	case 0x4060:
		COMBINE_DATA(&m_dvc_fmv_interrupt_enable);
		dvc_update_irq();
		break;
	case 0x4064:
		COMBINE_DATA(&m_dvc_fmv_timer_compare);
		dvc_update_irq_timer();
		break;
	case 0x40ae:
		dvc_update_irq_timer();
		break;
	case 0x406c: COMBINE_DATA(&m_dvc_fmv_y_offset); break;
	case 0x406e: COMBINE_DATA(&m_dvc_fmv_x_offset); break;
	case 0x4070: COMBINE_DATA(&m_dvc_fmv_y_active); break;
	case 0x4072: COMBINE_DATA(&m_dvc_fmv_x_active); break;
	case 0x4074: COMBINE_DATA(&m_dvc_fmv_y_display); break;
	case 0x4076: COMBINE_DATA(&m_dvc_fmv_x_display); break;
	case 0x4078: COMBINE_DATA(&m_dvc_fmv_window_height); break;
	case 0x407a: COMBINE_DATA(&m_dvc_fmv_window_width); break;
	case 0x407c: COMBINE_DATA(&m_dvc_fmv_decoder_offset_y); break;
	case 0x407e: COMBINE_DATA(&m_dvc_fmv_decoder_offset_x); break;
	case 0x4088: COMBINE_DATA(&m_dvc_fmv_decoder_command); break;
	case 0x408c: COMBINE_DATA(&m_dvc_fmv_video_data_input_command); break;
	case 0x40da:
		COMBINE_DATA(&m_dvc_fmv_program);
		LOGMASKED(LOG_DVC, "%s: DVC FMV program select %04x\n", machine().describe_context(), m_dvc_fmv_program);
		break;
	case 0x40ac: COMBINE_DATA(&m_dvc_fmv_frame_rate); break;
	case 0x40c0:
		COMBINE_DATA(&m_dvc_fmv_system_command);
		dvc_handle_fmv_command(m_dvc_fmv_system_command);
		break;
	case 0x40c2:
		COMBINE_DATA(&m_dvc_fmv_video_command);
		dvc_handle_fmv_video_command(m_dvc_fmv_video_command);
		break;
	case 0x40c4:
		COMBINE_DATA(&m_dvc_fmv_stream);
		m_dvc_fmv_stream &= 0x000f;
		break;
	case 0x40c6: COMBINE_DATA(&m_dvc_fmv_system_control); break;
	case 0x40dc: COMBINE_DATA(&m_dvc_fmv_interrupt_vector); break;
	case 0x40de:
		dvc_feed_word(true, data);
		dvc_decode_video();
		break;
	default:
		if ((address & 0xffff) >= 0x4800 && (address & 0xffff) <= 0x7ffe)
		{
			COMBINE_DATA(&m_dvc_fmv_program_ram[((address & 0xffff) - 0x4800) >> 1]);
			break;
		}
		break;
	}
}

uint16_t cdi_state::dvc_ram2_r(offs_t offset, uint16_t mem_mask)
{
	if (!m_dvc_mpeg_ram_enabled)
	{
		if (!machine().side_effects_disabled())
		{
			const uint32_t address = 0xe80000 + (offset << 1);
			m_maincpu->set_buserror_details(address, true, m_maincpu->get_fc());
			m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
			m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
		}
		return 0xff;
	}

	return m_dvc_ram2[offset];
}

void cdi_state::dvc_ram2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (!m_dvc_mpeg_ram_enabled)
	{
		if (!machine().side_effects_disabled())
		{
			const uint32_t address = 0xe80000 + (offset << 1);
			m_maincpu->set_buserror_details(address, false, m_maincpu->get_fc());
			m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
			m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
		}
		return;
	}

	COMBINE_DATA(&m_dvc_ram2[offset]);
}

/*************************
*       LCD screen       *
*************************/

static const uint16_t cdi220_lcd_char[20*22] =
{
	0x2000, 0x2000, 0x2000, 0x2000, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x8000, 0x8000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000, 0x0002, 0x0002, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x8000, 0x8000, 0x8000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0002, 0x0002, 0x0002, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x8000, 0x8000, 0x8000, 0x8000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0002, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x0000, 0x8000, 0x8000, 0x8000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0000, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x0000, 0x0000, 0x8000, 0x8000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0000, 0x0000, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0200, 0x0200, 0x0200, 0x0200,
	0x2000, 0x2000, 0x2000, 0x2000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0200, 0x0200, 0x0200, 0x0200,
	0x1000, 0x1000, 0x1000, 0x1000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0000, 0x0000, 0x0010, 0x0010, 0x0001, 0x0001, 0x0001, 0x0001, 0x0008, 0x0008, 0x0000, 0x0000, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0000, 0x0010, 0x0010, 0x0010, 0x0001, 0x0001, 0x0001, 0x0001, 0x0008, 0x0008, 0x0008, 0x0000, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0010, 0x0010, 0x0010, 0x0010, 0x0001, 0x0001, 0x0001, 0x0001, 0x0008, 0x0008, 0x0008, 0x0008, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0010, 0x0010, 0x0010, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0008, 0x0008, 0x0008, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0010, 0x0010, 0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000, 0x0008, 0x0008, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0400, 0x0400, 0x0400, 0x0400,
	0x1000, 0x1000, 0x1000, 0x1000, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0400, 0x0400, 0x0400, 0x0400
};

uint32_t cdi_state::screen_update_cdimono1_lcd(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	if (!m_slave_hle.found())
		return 0;

	for (int y = 0; y < 22; y++)
	{
		uint32_t *scanline = &bitmap.pix(y);

		for (int lcd = 0; lcd < 8; lcd++)
		{
			uint16_t data = (m_slave_hle->get_lcd_state()[lcd*2] << 8) |
							m_slave_hle->get_lcd_state()[lcd*2 + 1];
			for (int x = 0; x < 20; x++)
			{
				if (data & cdi220_lcd_char[y*20 + x])
				{
					scanline[(7 - lcd)*24 + x] = rgb_t::white();
				}
				else
				{
					scanline[(7 - lcd)*24 + x] = rgb_t::black();
				}
			}
		}
	}

	return 0;
}

/*************************
*    Machine Drivers     *
*************************/

// CD-i Mono-I system base
void cdi_state::cdimono1_base(machine_config &config)
{
	SCC68070(config, m_maincpu, CLOCK_A);
	m_maincpu->set_addrmap(AS_PROGRAM, &cdi_state::cdimono1_mem);
	m_maincpu->iack4_callback().set(FUNC(cdi_state::cdimono1_iack4_r));

	MCD212(config, m_mcd212, CLOCK_A, m_plane_ram[0], m_plane_ram[1]);
	m_mcd212->set_screen("screen");
	m_mcd212->int_callback().set(m_maincpu, FUNC(scc68070_device::int1_w));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_raw(960*(312*2-32)*50, 960, 0, 768, 312*2-32, 32, 312*2-32);
	screen.set_video_attributes(VIDEO_UPDATE_SCANLINE);
	screen.set_screen_update(m_mcd212, FUNC(mcd212_device::screen_update));

	SCREEN(config, m_lcd, SCREEN_TYPE_RASTER);
	m_lcd->set_refresh_hz(50);
	m_lcd->set_vblank_time(ATTOSECONDS_IN_USEC(0));
	m_lcd->set_size(192, 22);
	m_lcd->set_visarea(0, 192-1, 0, 22-1);
	m_lcd->set_screen_update(FUNC(cdi_state::screen_update_cdimono1_lcd));

	PALETTE(config, "palette").set_entries(0x100);

	config.set_default_layout(layout_cdi);

	// IMS66490 CDIC input clocks are 22.5792 MHz and 19.3536 MHz
	// DSP input clock is 7.5264 MHz
	CDI_CDIC(config, m_cdic, 45.1584_MHz_XTAL / 2);
	m_cdic->set_clock2(45.1584_MHz_XTAL * 3 / 7); // generated by PLL circuit incorporating 19.3575 MHz XTAL
	m_cdic->intreq_callback().set(FUNC(cdi_state::cdic_dvc_irq_w));

	CDI_SLAVE_HLE(config, m_slave_hle, 0);
	m_slave_hle->int_callback().set(m_maincpu, FUNC(scc68070_device::in2_w));

	CDROM(config, m_cdrom);
	m_cdrom->set_interface("cdrom");

	/* sound hardware */
	SPEAKER(config, "speaker", 2).front();

	dmadac_sound_device &cdic_dac_l(DMADAC(config, "dac1"));
	cdic_dac_l.add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	dmadac_sound_device &cdic_dac_r(DMADAC(config, "dac2"));
	cdic_dac_r.add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	DMADAC(config, m_dmadac[0]);
	m_dmadac[0]->add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	DMADAC(config, m_dmadac[1]);
	m_dmadac[1]->add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	MK48T08(config, "mk48t08");
}

// CD-i model 220 (Mono-II, NTSC)
void cdi_state::cdimono2(machine_config &config)
{
	SCC68070(config, m_maincpu, CLOCK_A);
	m_maincpu->set_addrmap(AS_PROGRAM, &cdi_state::cdimono2_mem);

	MCD212(config, m_mcd212, CLOCK_A, m_plane_ram[0], m_plane_ram[1]);
	m_mcd212->set_screen("screen");
	m_mcd212->int_callback().set(m_maincpu, FUNC(scc68070_device::int1_w));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_raw(14976000, 960, 0, 768, 312, 32, 312);
	screen.set_video_attributes(VIDEO_UPDATE_SCANLINE);
	screen.set_screen_update(m_mcd212, FUNC(mcd212_device::screen_update));

	SCREEN(config, m_lcd, SCREEN_TYPE_RASTER);
	m_lcd->set_refresh_hz(60);
	m_lcd->set_vblank_time(ATTOSECONDS_IN_USEC(0));
	m_lcd->set_size(192, 22);
	m_lcd->set_visarea(0, 192-1, 0, 22-1);
	m_lcd->set_screen_update(FUNC(cdi_state::screen_update_cdimono1_lcd));

	PALETTE(config, "palette").set_entries(0x100);

	config.set_default_layout(layout_cdi);

	M68HC05C8(config, m_servo, 4_MHz_XTAL);
	M68HC05C8(config, m_slave, 4_MHz_XTAL);

	CDROM(config, m_cdrom).set_interface("cdrom");
	SOFTWARE_LIST(config, "cd_list").set_original("cdi").set_filter("!DVC");
	SOFTWARE_LIST(config, "photocd_list").set_compatible("photo_cd");

	/* sound hardware */
	SPEAKER(config, "speaker", 2).front();

	dmadac_sound_device &cdic_dac_l(DMADAC(config, "dac1"));
	cdic_dac_l.add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	dmadac_sound_device &cdic_dac_r(DMADAC(config, "dac2"));
	cdic_dac_r.add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	DMADAC(config, m_dmadac[0]);
	m_dmadac[0]->add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	DMADAC(config, m_dmadac[1]);
	m_dmadac[1]->add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	MK48T08(config, "mk48t08");
}

void cdi_state::cdi910(machine_config &config)
{
	SCC68070(config, m_maincpu, CLOCK_A);
	m_maincpu->set_addrmap(AS_PROGRAM, &cdi_state::cdi910_mem);

	MCD212(config, m_mcd212, CLOCK_A, m_plane_ram[0], m_plane_ram[1]);
	m_mcd212->set_screen("screen");
	m_mcd212->int_callback().set(m_maincpu, FUNC(scc68070_device::int1_w));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_raw(14976000, 960, 0, 768, 312, 32, 312);
	screen.set_video_attributes(VIDEO_UPDATE_SCANLINE);
	screen.set_screen_update(m_mcd212, FUNC(mcd212_device::screen_update));

	SCREEN(config, m_lcd, SCREEN_TYPE_RASTER);
	m_lcd->set_refresh_hz(60);
	m_lcd->set_vblank_time(ATTOSECONDS_IN_USEC(0));
	m_lcd->set_size(192, 22);
	m_lcd->set_visarea(0, 192-1, 0, 22-1);
	m_lcd->set_screen_update(FUNC(cdi_state::screen_update_cdimono1_lcd));

	PALETTE(config, "palette").set_entries(0x100);

	config.set_default_layout(layout_cdi);

	M68HC05C8(config, m_servo, 4_MHz_XTAL);
	M68HC05C8(config, m_slave, 4_MHz_XTAL);

	CDROM(config, "cdrom").set_interface("cdrom");
	SOFTWARE_LIST(config, "cd_list").set_original("cdi").set_filter("!DVC");
	SOFTWARE_LIST(config, "photocd_list").set_compatible("photo_cd");

	/* sound hardware */
	SPEAKER(config, "speaker", 2).front();

	dmadac_sound_device &cdic_dac_l(DMADAC(config, "dac1"));
	cdic_dac_l.add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	dmadac_sound_device &cdic_dac_r(DMADAC(config, "dac2"));
	cdic_dac_r.add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	DMADAC(config, m_dmadac[0]);
	m_dmadac[0]->add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	DMADAC(config, m_dmadac[1]);
	m_dmadac[1]->add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	MK48T08(config, "mk48t08");
}

// CD-i Mono-I, with CD-ROM image device (MESS) and Software List (MESS)
void cdi_state::cdimono1(machine_config &config)
{
	cdimono1_base(config);

	m_slave_hle->read_mousex().set_ioport("MOUSEX");
	m_slave_hle->read_mousey().set_ioport("MOUSEY");
	m_slave_hle->read_mousebtn().set_ioport("MOUSEBTN");

	SOFTWARE_LIST(config, "cd_list").set_original("cdi");
	SOFTWARE_LIST(config, "photocd_list").set_compatible("photo_cd");
}

void quizard_state::quizard(machine_config &config)
{
	cdimono1_base(config);
	m_cdrom->add_region("cdrom");

	m_maincpu->set_addrmap(AS_PROGRAM, &quizard_state::cdimono1_mem);
	m_maincpu->uart_rtsn_callback().set(FUNC(quizard_state::mcu_rtsn_from_cpu));
	m_maincpu->uart_tx_callback().set(FUNC(quizard_state::mcu_rx_from_cpu));

	I8751(config, m_mcu, 11.0592_MHz_XTAL);
	m_mcu->port_in_cb<0>().set(FUNC(quizard_state::mcu_p0_r));
	m_mcu->port_in_cb<1>().set(FUNC(quizard_state::mcu_p1_r));
	m_mcu->port_in_cb<2>().set(FUNC(quizard_state::mcu_p2_r));
	m_mcu->port_in_cb<3>().set(FUNC(quizard_state::mcu_p3_r));
	m_mcu->port_out_cb<0>().set(FUNC(quizard_state::mcu_p0_w));
	m_mcu->port_out_cb<1>().set(FUNC(quizard_state::mcu_p1_w));
	m_mcu->port_out_cb<2>().set(FUNC(quizard_state::mcu_p2_w));
	m_mcu->port_out_cb<3>().set(FUNC(quizard_state::mcu_p3_w));

	m_slave_hle->read_mousebtn().set(FUNC(quizard_state::mcu_button_press));
}

void quizard_state::tra_callback()
{
	if (transmit_register_get_data_bit())
		m_mcu_p3 |= 1;
	else
		m_mcu_p3 &= ~1;
}

void quizard_state::rcv_complete()
{
	receive_register_extract();

	const uint8_t data = get_received_char();
	LOGMASKED(LOG_QUIZARD_OTHER, "%s: MCU transmitting %02x\n", machine().describe_context(), data);
	m_maincpu->uart_rx(data);
}

/*************************
*        Rom Load        *
*************************/

ROM_START( cdimono1 )
	ROM_REGION(0x80000, "maincpu", 0) // these roms need byteswapping
	ROM_SYSTEM_BIOS( 0, "mcdi200", "Magnavox CD-i 200" )
	ROMX_LOAD( "cdi200.rom", 0x000000, 0x80000, CRC(40c4e6b9) SHA1(d961de803c89b3d1902d656ceb9ce7c02dccb40a), ROM_BIOS(0) )
	ROM_SYSTEM_BIOS( 1, "pcdi220", "Philips CD-i 220 F2" )
	ROMX_LOAD( "cdi220b.rom", 0x000000, 0x80000, CRC(279683ca) SHA1(53360a1f21ddac952e95306ced64186a3fc0b93e), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS( 2, "pcdi220_alt", "Philips CD-i 220?" ) // doesn't boot
	ROMX_LOAD( "cdi220.rom", 0x000000, 0x80000, CRC(584c0af8) SHA1(5d757ab46b8c8fc36361555d978d7af768342d47), ROM_BIOS(2) )

	ROM_REGION(0x60000, "mpegs", 0)
	ROM_LOAD( "impega.rom", 0x00000, 0x40000, CRC(84d6f6aa) SHA1(02526482a0851ea2a7b582d8afaa8ef14a8bd914) )
	ROM_LOAD16_BYTE( "fmv ffd9 p7308 r4.1 vmpeg.bin", 0x40000, 0x10000, CRC(30ba9273) SHA1(d8adca0627b356ced6131b9458ac1175e43e6548) )
	ROM_LOAD16_BYTE( "fmv 4ba9 p7307 r4.1 vmpeg.bin", 0x40001, 0x10000, CRC(623edb1f) SHA1(4c6b11e28ad4c2f5c2e439f7910a783e0a79d1a9) )

	// The two MCU dumps below are taken from the cdi910. We still need dumps from a Mono-I board in case the revisions are different.
	ROM_REGION(0x2000, "servo", 0)
	ROM_LOAD( "zx405037p__cdi_servo_2.1__b43t__llek9215.mc68hc705c8a_withtestrom.7201", 0x0000, 0x2000, CRC(7a3af407) SHA1(fdf8d78d6a0df4a56b5b963d72eabd39fcec163f) BAD_DUMP )

	ROM_REGION(0x2000, "slave", 0)
	ROM_LOAD( "zx405042p__cdi_slave_2.0__b43t__zzmk9213.mc68hc705c8a_withtestrom.7206", 0x0000, 0x2000, CRC(688cda63) SHA1(56d0acd7caad51c7de703247cd6d842b36173079) BAD_DUMP )
ROM_END

ROM_START( cdi910 )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_SYSTEM_BIOS( 0, "cdi910", "CD-I 910-17P Mini-MMC" )
	ROMX_LOAD( "philips__cd-i_2.1__mb834200b-15__26b_aa__9224_z01.tc574200.7211", 0x000000, 0x80000, CRC(4ae3bee3) SHA1(9729b4ee3ce0c17172d062339c47b1ab822b222b), ROM_BIOS(0) | ROM_GROUPWORD | ROM_REVERSE )
	ROM_SYSTEM_BIOS( 1, "cdi910_alt", "alt" )
	ROMX_LOAD( "cdi910.rom", 0x000000, 0x80000, CRC(2f3048d2) SHA1(11c4c3e602060518b52e77156345fa01f619e793), ROM_BIOS(1) | ROM_GROUPWORD | ROM_REVERSE )

	ROM_REGION(0x2000, "servo", 0)
	ROM_LOAD( "zx405037p__cdi_servo_2.1__b43t__llek9215.mc68hc705c8a_withtestrom.7201", 0x0000, 0x2000, CRC(7a3af407) SHA1(fdf8d78d6a0df4a56b5b963d72eabd39fcec163f) )

	ROM_REGION(0x2000, "slave", 0)
	ROM_LOAD( "zx405042p__cdi_slave_2.0__b43t__zzmk9213.mc68hc705c8a_withtestrom.7206", 0x0000, 0x2000, CRC(688cda63) SHA1(56d0acd7caad51c7de703247cd6d842b36173079) )

	ROM_REGION(0x2000, "pals", 0)
	ROM_LOAD( "ti_portugal_206xf__tibpal20l8-15cnt__m7205n.7205.bin",      0x0000, 0x144, CRC(dd167e0d) SHA1(2ba82a4619d7a0f19e62e02a2841afd4d45d56ba) )
	ROM_LOAD( "ti_portugal_774_206xf__tibpal16l8-10cn_m7204n.7204.bin",    0x0000, 0x104, CRC(04e6bd37) SHA1(153d1a977291bedb7420484a9f889325dbd3628e) )
ROM_END

ROM_START( cdimono2 )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_SYSTEM_BIOS(0, "pcdi220", "Philips CD-i 220 F3")
	ROMX_LOAD( "philips__cdi-220_ph3_r1.2__mb834200b-15__02f_aa__9402_z04.tc574200-le._1.7211", 0x000000, 0x80000, CRC(17d723e7) SHA1(6c317a82e35d60ca5e7a74fc99f665055693169d), ROM_BIOS(0) | ROM_GROUPWORD | ROM_REVERSE )
	ROM_SYSTEM_BIOS(1, "pcdi210", "Philips CD-i 210 F2")
	ROMX_LOAD( "philips__cd-i_4.1_r1.1__mb834200b-15__10e_aa__9336_z01.7211", 0x000000, 0x80000, CRC(8453553f) SHA1(5ee4dc3e7eb4c3867ac9d04f1614908906af19fb), ROM_BIOS(1) | ROM_GROUPWORD | ROM_REVERSE )

	ROM_REGION(0x2000, "servo", 0)
	ROM_LOAD( "zc405351p__servo_cdi_4.1__0d67p__lluk9404.mc68hc705c8a.7490", 0x0000, 0x2000, CRC(2bc8e4e9) SHA1(8cd052b532fc052d6b0077261c12f800e8655bb1) )

	ROM_REGION(0x2000, "slave", 0)
	ROM_LOAD( "zc405352p__slave_cdi_4.1__0d67p__lltr9403.mc68hc705c8a.7206", 0x0000, 0x2000, CRC(5b19da07) SHA1(cf02d84977050c71e87a38f1249e83c43a93949b) )
ROM_END

ROM_START( cdi490a )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_SYSTEM_BIOS( 0, "cdi490", "CD-i 490" )
	ROMX_LOAD( "cdi490a.rom", 0x000000, 0x80000, CRC(e2f200f6) SHA1(c9bf3c4c7e4fe5cbec3fe3fc993c77a4522ca547), ROM_BIOS(0) | ROM_GROUPWORD | ROM_REVERSE  )

	ROM_REGION(0x60000, "mpegs", 0) // keep these somewhere
	ROM_LOAD( "impega.rom", 0x00000, 0x40000, CRC(84d6f6aa) SHA1(02526482a0851ea2a7b582d8afaa8ef14a8bd914) ) // 1ST AND 2ND HALF IDENTICAL
	// Philips CD-i - DVC card 22ER9141
	ROM_LOAD16_BYTE( "fmv ffd9 p7308 r4.1 vmpeg.bin", 0x40000, 0x10000, CRC(30ba9273) SHA1(d8adca0627b356ced6131b9458ac1175e43e6548) )
	ROM_LOAD16_BYTE( "fmv 4ba9 p7307 r4.1 vmpeg.bin", 0x40001, 0x10000, CRC(623edb1f) SHA1(4c6b11e28ad4c2f5c2e439f7910a783e0a79d1a9) )
ROM_END

ROM_START( gpi1200 )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD16_WORD_SWAP( "gpi-1200k-1313.bin", 0x000000, 0x80000, CRC(dbd41615) SHA1(83929617a5c01551ee961aeb685295fcc0810f54) )
ROM_END

ROM_START( cdibios ) // for the quizard sets
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_SYSTEM_BIOS( 0, "mcdi200", "Magnavox CD-i 200" )
	ROMX_LOAD( "cdi200.rom", 0x000000, 0x80000, CRC(40c4e6b9) SHA1(d961de803c89b3d1902d656ceb9ce7c02dccb40a), ROM_BIOS(0) )
	ROM_SYSTEM_BIOS( 1, "pcdi220", "Philips CD-i 220 F2" )
	ROMX_LOAD( "cdi220b.rom", 0x000000, 0x80000, CRC(279683ca) SHA1(53360a1f21ddac952e95306ced64186a3fc0b93e), ROM_BIOS(1) )

	// The MCU dump below is taken from the cdi910. We still need a dump from a Mono-I board SLAVE MCU in case the revisions are different.
	ROM_REGION(0x2000, "slave", 0)
	ROM_LOAD( "zx405042p__cdi_slave_2.0__b43t__zzmk9213.mc68hc705c8a_withtestrom.7206", 0x0000, 0x2000, CRC(688cda63) SHA1(56d0acd7caad51c7de703247cd6d842b36173079) BAD_DUMP )
ROM_END

/*  Quizard notes

    The MCU controls the protection sequence, which in turn controls the game display language.
    Each Quizard game (1,2,3,4) requires its own MCU, you can upgrade between revisions by changing
    just the CD, but not between games as a new MCU is required.

    MCU Notes:
    i8751 MCU dumps confirmed good on original hardware
    Italian language MCU for Quizard 1 is dumped
    German language MCUs for Quizard 1 through 4 are dumped
    Czech language MCU for Quizard 4 is dumped
    Alt. German language MCU for Quizard 2 is known to exist (DE 122 D3, not dumped)

*/


#define QUIZARD_BIOS_ROM \
	ROM_REGION(0x80000, "maincpu", 0) \
	ROM_LOAD( "cdi220b.rom", 0x000000, 0x80000, CRC(279683ca) SHA1(53360a1f21ddac952e95306ced64186a3fc0b93e) )

//********************************************************
//                     Quizard (1)
//********************************************************

#define QUIZARD1_CHD_10 \
	DISK_REGION( "cdrom" ) \
	DISK_IMAGE_READONLY( "quizard10", 0, SHA1(5715db50f0d5ffe06f47c0943f4bf0481ab6048e) ) // Dumped via BurnAtOnce 0.99.5, CHDMAN 0.163, TS-L633R drive

// CD-ROM printed 01/95
#define QUIZARD1_CHD_12 \
	DISK_REGION( "cdrom" ) \
	DISK_IMAGE_READONLY( "quizard12", 0, BAD_DUMP SHA1(6e41683b96b74e903040842aeb18437ad7813c82) )

#define QUIZARD1_CHD_17 \
	DISK_REGION( "cdrom" ) \
	DISK_IMAGE_READONLY( "quizard17", 0, BAD_DUMP SHA1(4bd698f076505b4e17be978481bce027eb47123b) )

#define QUIZARD1_CHD_18 \
	DISK_REGION( "cdrom" ) \
	DISK_IMAGE_READONLY( "quizard18", 0, BAD_DUMP SHA1(ede873b22957f2a707bbd3039e962ef2ca5aedbd) )

// MCU Type: Intel D8751H MCU
#define QUIZARD1_MCU_DE \
	ROM_REGION(0x1000, "mcu", 0) \
	ROM_LOAD( "de_11_d3.bin", 0x0000, 0x1000, CRC(95f45b6b) SHA1(51b34956539b1e2cf0306f243a970750f1e18d01) ) // German

#define QUIZARD1_MCU_IT \
	ROM_REGION(0x1000, "mcu", 0) \
	ROM_LOAD( "it_11_i2.bin", 0x0000, 0x1000, CRC(e00dc02c) SHA1(e4ef1ea47c242879a99c9d54cfc008ae99a651cb) ) // Italian

ROM_START( quizard )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_18
	QUIZARD1_MCU_DE
ROM_END

ROM_START( quizard_17 )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_17
	QUIZARD1_MCU_DE
ROM_END

ROM_START( quizard_12 )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_12
	QUIZARD1_MCU_DE
ROM_END

ROM_START( quizard_10 )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_10
	QUIZARD1_MCU_DE
ROM_END

ROM_START( quizardi )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_18
	QUIZARD1_MCU_IT
ROM_END

ROM_START( quizardi_17 )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_17
	QUIZARD1_MCU_IT
ROM_END

ROM_START( quizardi_12 )
	QUIZARD_BIOS_ROM
	QUIZARD1_CHD_12
	QUIZARD1_MCU_IT
ROM_END

//********************************************************
//                     Quizard 2
//********************************************************

ROM_START( quizard2 ) /* CD-ROM printed ??/?? */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard23", 0, BAD_DUMP SHA1(cd909d9a54275d6f2d36e03e83eea996e781b4d3) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "dn_122_d3.bin", 0x0000, 0x1000, CRC(d48063ea) SHA1(b512fa5e53f296a180340e09b53613dd1c0d38bc) ) // German language - DE 122 D3 known to exist
ROM_END

ROM_START( quizard2_22 )
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard22", 0, BAD_DUMP SHA1(03c8fdcf27ead6e221691111e8c679b551099543) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "dn_122_d3.bin", 0x0000, 0x1000, CRC(d48063ea) SHA1(b512fa5e53f296a180340e09b53613dd1c0d38bc) ) // German language - DE 122 D3 known to exist
ROM_END


//********************************************************
//                     Quizard 3
//********************************************************

ROM_START( quizard3 ) /* CD-ROM printed ??/?? */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard34", 0, BAD_DUMP SHA1(37ad49b72b5175afbb87141d57bc8604347fe032) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_132_d3.bin", 0x0000, 0x1000, CRC(8858251e) SHA1(2c1005a74bb6f0c2918dff4ab6326528eea48e1f) ) // German language
ROM_END

ROM_START( quizard3a ) /* CD-ROM printed ??/?? */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard34", 0, BAD_DUMP SHA1(37ad49b72b5175afbb87141d57bc8604347fe032) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_132_a1.bin", 0x0000, 0x1000, CRC(313ac673) SHA1(cb0ee7e9a6eaa5f4d000f5ea99b7ee4c440b31d1) ) // German language - earlier version of MCU code
ROM_END

ROM_START( quizard3_32 )
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard32", 0, BAD_DUMP SHA1(31e9fa2169aa44d799c37170b238134ab738e1a1) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_132_d3.bin", 0x0000, 0x1000, CRC(8858251e) SHA1(2c1005a74bb6f0c2918dff4ab6326528eea48e1f) ) // German language
ROM_END


//********************************************************
//                     Quizard 4
//********************************************************

ROM_START( quizard4 ) /* CD-ROM printed 09/98 */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard4r42", 0, BAD_DUMP SHA1(a5d5c8950b4650b8753f9119dc7f1ccaa2aa5442) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_142_d3.bin", 0x0000, 0x1000, CRC(77be0b40) SHA1(113b5c239480a2259f55e411ba8fb3972e6d4301) ) // German language
ROM_END

ROM_START( quizard4cz ) /* CD-ROM printed 09/98 */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard4r42", 0, BAD_DUMP SHA1(a5d5c8950b4650b8753f9119dc7f1ccaa2aa5442) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "ts142_cz1.bin", 0x0000, 0x1000, CRC(fdc1f457) SHA1(5169c4d2ea4073a854c3f619205161386c9af8af) ) // Czech language - works with all Quizard 4 versions
ROM_END

ROM_START( quizard4_41 )
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard4r41", 0, BAD_DUMP SHA1(2c0484c6545aac8e00b318328c6edce6f5dde43d) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_142_d3.bin", 0x0000, 0x1000, CRC(77be0b40) SHA1(113b5c239480a2259f55e411ba8fb3972e6d4301) ) // German language
ROM_END

ROM_START( quizard4_40 ) /* CD-ROM printed 07/97 */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizard4r40", 0, BAD_DUMP SHA1(288cc37a994e4f1cbd47aa8c92342879c6fc0b87) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "de_142_d3.bin", 0x0000, 0x1000, CRC(77be0b40) SHA1(113b5c239480a2259f55e411ba8fb3972e6d4301) ) // German language
ROM_END

// only the CD was dumped, MCU not available
ROM_START( quizardff ) /* CD-ROM printed 01/96 */
	QUIZARD_BIOS_ROM

	DISK_REGION( "cdrom" )
	DISK_IMAGE_READONLY( "quizardff", 0, SHA1(ac533040379c1350066e778e3a86d1beb11c6f71) )

	ROM_REGION(0x1000, "mcu", 0) // Intel D8751H MCU
	ROM_LOAD( "8751.bin", 0x0000, 0x1000, NO_DUMP )
ROM_END


/*************************
*      Game driver(s)    *
*************************/

/*    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     CLASS      INIT        COMPANY       FULLNAME */
// BIOS / System
CONS( 1991, cdimono1, 0,      0,      cdimono1, cdi,      cdi_state, empty_init, "Philips",    "CD-i (Mono-I) (PAL)",   MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
CONS( 1991, cdimono2, 0,      0,      cdimono2, cdimono2, cdi_state, empty_init, "Philips",    "CD-i (Mono-II) (NTSC)",   MACHINE_NOT_WORKING )
CONS( 1991, cdi910,   0,      0,      cdi910,   cdimono2, cdi_state, empty_init, "Philips",    "CD-i 910-17P Mini-MMC (PAL)",   MACHINE_NOT_WORKING )
CONS( 1991, cdi490a,  0,      0,      cdimono1, cdi,      cdi_state, empty_init, "Philips",    "CD-i 490",   MACHINE_NOT_WORKING )
CONS( 1995, gpi1200,  0,      0,      cdimono1, cdi,      cdi_state, empty_init, "Goldstar",   "GPi 1200",   MACHINE_NOT_WORKING )

// The Quizard games are retail CD-i units in a cabinet, with an additional JAMMA adapter and dongle for protection, hence being clones of the system.
/*    YEAR  NAME         PARENT    MACHINE        INPUT     DEVICE          INIT         MONITOR     COMPANY         FULLNAME */
GAME( 1995, cdibios,     0,        cdimono1,      quizard,  cdi_state,     empty_init,  ROT0,     "Philips",  "CD-i (Mono-I) (PAL) BIOS", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IS_BIOS_ROOT )

GAME( 1995, quizard,     cdibios,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.8, German, i8751 DE 11 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizard_17,  quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.7, German, i8751 DE 11 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizard_12,  quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.2, German, i8751 DE 11 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizard_10,  quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.0, German, i8751 DE 11 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizardi,    quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.8, Italian, i8751 IT 11 I2)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizardi_17, quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.7, Italian, i8751 IT 11 I2)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizardi_12, quizard,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard (v1.2, Italian, i8751 IT 11 I2)", MACHINE_IMPERFECT_SOUND )

GAME( 1995, quizard2,    cdibios,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 2 (v2.3, German, i8751 DN 122 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizard2_22, quizard2, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 2 (v2.2, German, i8751 DN 122 D3)", MACHINE_IMPERFECT_SOUND )

// Quizard 3 and 4 will hang after starting a game (CDIC issues?)
GAME( 1995, quizard3,    cdibios,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 3 (v3.4, German, i8751 DE 132 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1995, quizard3a,   quizard3, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 3 (v3.4, German, i8751 DE 132 A1)", MACHINE_IMPERFECT_SOUND )
GAME( 1996, quizard3_32, quizard3, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 3 (v3.2, German, i8751 DE 132 D3)", MACHINE_IMPERFECT_SOUND )

GAME( 1998, quizard4,    cdibios,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 4 Rainbow (v4.2, German, i8751 DE 142 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1998, quizard4cz,  quizard4, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 4 Rainbow (v4.2, Czech, i8751 TS142 CZ1)", MACHINE_IMPERFECT_SOUND )
GAME( 1998, quizard4_41, quizard4, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 4 Rainbow (v4.1, German, i8751 DE 142 D3)", MACHINE_IMPERFECT_SOUND )
GAME( 1997, quizard4_40, quizard4, quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard 4 Rainbow (v4.0, German, i8751 DE 142 D3)", MACHINE_IMPERFECT_SOUND )

GAME( 1996, quizardff,   cdibios,  quizard,       quizard,  quizard_state, empty_init,  ROT0, "TAB Austria",  "Quizard Fun and Fascination (French Edition V1 - 01/96)", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND )
