// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

    wxstar4000.cpp: WeatherSTAR 4000 cable head-end unit
    1990 Applied Microelectronics Institute (Amirix) / The Weather Channel
    Skeleton driver by R. Belmont

    ... (Header comments remain the same) ...

   TODO:
   - Implement detailed I/O registers for Data board PIOs, I/O board UART/Kbd comms
   - Implement shared memory access control/arbitration if needed (VME bus?)
   - Implement Data 8344 SDLC communication (satellite feed)
   - Implement audio tone generation
   - Verify clock speeds and timings (PTM, UART)
   - Map unknown address areas based on code execution
   - Verify screen parameters and VRAM mapping
   - Verify Bt471 register mapping (pixel mask, overlay addr vs data)
   - Implement vector acknowledge handlers if needed

***************************************************************************/

#include "emu.h"
// #include "emu/driver.h" // REMOVE THIS LINE
#include "cpu/m68000/m68010.h"
#include "cpu/mcs51/mcs51.h"
#include "machine/gen_latch.h"
#include "machine/icm7170.h"
#include "machine/nvram.h"
#include "machine/timer.h"
#include "video/bt47x.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"
#include "machine/6840ptm.h"
#include "machine/i8251.h"
#include "bus/rs232/rs232.h"
#include "bus/pc_kbd/keyboards.h" // Ensure this line uses the correct path

#define LOG_CPU_IO (1U << 1)
#define LOG_GFX_IO (1U << 2)
#define LOG_GFX_SUB_IO (1U << 3)
#define LOG_DATA_IO (1U << 4)
#define LOG_IO_IO (1U << 5)
#define LOG_IRQ (1U << 6)
#define LOG_WARN (1U << 7)
#define LOG_LATCH (1U << 8)

//#define VERBOSE (LOG_CPU_IO | LOG_GFX_IO | LOG_GFX_SUB_IO | LOG_DATA_IO | LOG_IO_IO | LOG_IRQ | LOG_WARN | LOG_LATCH)
#define VERBOSE (LOG_WARN | LOG_IRQ | LOG_LATCH)
#include "logmacro.h"

#define LOGCPU(x, ...)    LOGMASKED(LOG_CPU_IO, x, ##__VA_ARGS__)
#define LOGGFX(x, ...)    LOGMASKED(LOG_GFX_IO, x, ##__VA_ARGS__)
#define LOGGFXSUB(x, ...) LOGMASKED(LOG_GFX_SUB_IO, x, ##__VA_ARGS__)
#define LOGDATA(x, ...)   LOGMASKED(LOG_DATA_IO, x, ##__VA_ARGS__)
#define LOGIO(x, ...)     LOGMASKED(LOG_IO_IO, x, ##__VA_ARGS__)
#define LOGIRQ(x, ...)    LOGMASKED(LOG_IRQ, x, ##__VA_ARGS__)
#define LOGWARN(x, ...)   LOGMASKED(LOG_WARN, x, ##__VA_ARGS__)
#define LOGLATCH(x, ...)  LOGMASKED(LOG_LATCH, x, ##__VA_ARGS__)

// Device Tags
#define PTM_TAG "u51_ptm"
#define UART_TAG "u5_uart"
#define KBD_TAG "kbd"
#define RS232_TAG "serport"
#define GFX_LATCH_IN_TAG "gfxlatch_in"
#define DATA_LATCH_IN_TAG "datalatch_in"
#define DATA_LATCH_OUT_TAG "datalatch_out"
#define IO_LATCH_IN_TAG "iolatch_in"
#define IO_LATCH_OUT_TAG "iolatch_out"


namespace {

class wxstar4k_state : public driver_device
{
public:
	// Constructor - Base class must be initialized FIRST
	wxstar4k_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag), // Initialize base driver_device
		m_maincpu(*this, "maincpu"),
		m_gfxcpu(*this, "gfxcpu"),
		m_gfxsubcpu(*this, "gfxsubcpu"),
		m_datacpu(*this, "datacpu"),
		m_iocpu(*this, "iocpu"),
		m_mainram(*this, "mainram"),
		m_extram(*this, "extram"),
		m_vram(*this, "vram"),
		m_rtc(*this, "rtc"),
		m_ramdac(*this, "ramdac"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_nvram(*this, "eeprom"),
		m_ptm(*this, PTM_TAG),
		m_uart(*this, UART_TAG),
		m_rs232(*this, RS232_TAG),
		m_kbd(*this, KBD_TAG), // Initialize after base class
		m_gfx_latch_in(*this, GFX_LATCH_IN_TAG),
		m_data_latch_in(*this, DATA_LATCH_IN_TAG),
		m_data_latch_out(*this, DATA_LATCH_OUT_TAG),
		m_io_latch_in(*this, IO_LATCH_IN_TAG),
		m_io_latch_out(*this, IO_LATCH_OUT_TAG),
		m_diag_led(*this, "led%u", 0U),
		m_cpu_irq_vector(0),
		m_gfx_irq_vector(0),
		m_gfx_sub_p1(0xff),
		m_main_watchdog(0),
		m_io_kbd_data(0),
		m_io_kbd_status(0)
	{ }

	void wxstar4k(machine_config &config);

private:
	// Screen update
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	// Memory Maps
	void cpubd_main_map(address_map &map) ATTR_COLD;
	void vidbd_main_map(address_map &map) ATTR_COLD;
	void vidbd_sub_map(address_map &map) ATTR_COLD;
	void vidbd_sub_io_map(address_map &map) ATTR_COLD;
	void databd_main_map(address_map &map) ATTR_COLD;
	void databd_main_io_map(address_map &map) ATTR_COLD;
	void iobd_main_map(address_map &map) ATTR_COLD;
	void iobd_main_io_map(address_map &map) ATTR_COLD;

	// Driver overrides
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	// Devices
	required_device<m68010_device> m_maincpu;
	required_device<m68010_device> m_gfxcpu;
	required_device<mcs51_cpu_device> m_gfxsubcpu;
	required_device<mcs51_cpu_device> m_datacpu;
	required_device<mcs51_cpu_device> m_iocpu;
	required_shared_ptr<uint16_t> m_mainram;
	required_shared_ptr<uint16_t> m_extram;
	required_shared_ptr<uint16_t> m_vram;
	required_device<icm7170_device> m_rtc;
	required_device<bt471_device> m_ramdac;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<nvram_device> m_nvram;
	required_device<ptm6840_device> m_ptm;
	required_device<i8251_device> m_uart;
	required_device<rs232_port_device> m_rs232;
	required_device<at_keyboard_device> m_kbd; // Type name seems correct
	required_device<generic_latch_8_device> m_gfx_latch_in;
	required_device<generic_latch_8_device> m_data_latch_in;
	required_device<generic_latch_8_device> m_data_latch_out;
	required_device<generic_latch_8_device> m_io_latch_in;
	required_device<generic_latch_8_device> m_io_latch_out;

	output_finder<8> m_diag_led;

	// Internal state
	uint8_t m_cpu_irq_vector;
	uint8_t m_gfx_irq_vector;
	uint8_t m_gfx_sub_p1;
	uint16_t m_vram_addr_low = 0;
	uint16_t m_vram_addr_high = 0;
	uint8_t m_main_watchdog;
	uint8_t m_io_kbd_data;
	uint8_t m_io_kbd_status;

	// CPU Board I/O Handlers
	uint16_t buserr_r();
	uint16_t cpubd_io_status_r();
	void cpubd_io_control_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t cpubd_data_latch_r();
	void cpubd_data_latch_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void cpubd_gfx_irq6_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void cpubd_watchdog_reset_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	// Graphics Board 68010 I/O Handlers
	uint16_t vidbd_status_r();
	void vidbd_vme_addr_hi_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_irq_vector_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_latch_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_main_cpu_irq4_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t vidbd_gfx_control_r(offs_t offset);
	void vidbd_gfx_control_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	// Graphics Board 8051 I/O Handlers
	uint8_t vidbd_sub_vram_counter_low_r(offs_t offset);
	void vidbd_sub_vram_counter_low_w(offs_t offset, uint8_t data);
	uint8_t vidbd_sub_vram_counter_high_r(offs_t offset);
	void vidbd_sub_vram_counter_high_w(offs_t offset, uint8_t data);
	uint8_t vidbd_sub_latch_r();
	uint8_t vidbd_sub_p1_r();
	void vidbd_sub_p1_w(uint8_t data);
	uint8_t vidbd_sub_p3_r();
	void vidbd_sub_p3_w(uint8_t data);

	// Data Board 8344 I/O Handlers
	uint8_t databd_latch_in_r();
	void databd_latch_out_w(uint8_t data);
	uint8_t databd_pio1_r(offs_t offset);
	void databd_pio1_w(offs_t offset, uint8_t data);
	uint8_t databd_pio2_r(offs_t offset);
	void databd_pio2_w(offs_t offset, uint8_t data);

	// I/O Board 8031 Handlers
	uint8_t iobd_latch_in_r();
	void iobd_latch_out_w(uint8_t data);
	uint8_t iobd_kbd_data_r();
	uint8_t iobd_kbd_status_r();

	// Interrupt handling & Callbacks
	TIMER_DEVICE_CALLBACK_MEMBER(vblank_irq);
	void rtc_irq_w(int state);
	void ptm_irq_w(int state);
	void data_latch_irq_w(int state);
	void io_latch_irq_w(int state);
	void kbd_put_key(uint8_t scancode);
	void led_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
};

// ... (Rest of file remains the same) ...

// --- Video ---

uint32_t wxstar4k_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint32_t vram_addr_byte = 0;
	size_t total_vram_bytes = m_vram.bytes() * 2;

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		uint16_t *scanline = &bitmap.pix(y, cliprect.left());
		vram_addr_byte = y * 640;

		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			if (vram_addr_byte < total_vram_bytes)
			{
				uint32_t word_addr = vram_addr_byte / 2;
				uint16_t word_data = m_vram[word_addr];
				uint8_t pixel_data = (vram_addr_byte & 1) ? (word_data & 0x00ff) : ((word_data >> 8) & 0x00ff);
				*scanline++ = pixel_data;
			} else {
				*scanline++ = 0;
			}
			vram_addr_byte++;
		}
	}
	return 0;
}

// --- CPU Board ---

uint16_t wxstar4k_state::buserr_r()
{
	LOGWARN("%s: Bus error read access!\n", machine().describe_context());
	if (!machine().side_effects_disabled())
	{
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	}
	return 0xffff;
}

void wxstar4k_state::led_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		LOGCPU("LED: %02x\n", data & 0xff);
		for(int i=0; i<8; i++)
			m_diag_led[i] = BIT(data, i);
	}
}

void wxstar4k_state::cpubd_watchdog_reset_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGCPU("Watchdog reset write: %04X & %04X\n", data, mem_mask);
	m_main_watchdog = 0;
}

void wxstar4k_state::cpubd_gfx_irq6_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (ACCESSING_BITS_0_15 && offset == 0)
	{
		LOGIRQ("%s: Main CPU triggering GFX CPU IRQ 6\n", machine().describe_context());
		m_gfxcpu->set_input_line_and_vector(M68K_IRQ_6, ASSERT_LINE, m_gfx_irq_vector);
	}
	else
	{
		LOGCPU("Write to GFX IRQ trigger area offset %d, data %04X & %04X\n", offset, data, mem_mask);
	}
}

uint16_t wxstar4k_state::cpubd_io_status_r()
{
	uint16_t status = m_io_latch_out->pending_r() ? 1 : 0; // Bit 0 = data pending from IO board
	LOGCPU("%s: Read from I/O card status @ C00000 = %04X\n", machine().describe_context(), status);
	return status;
}

void wxstar4k_state::cpubd_io_control_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		uint8_t byte_data = data & 0xff;
		LOGLATCH("Write to I/O card control/data @ C00000: data %02X -> Latch In\n", byte_data);
		m_io_latch_in->write(byte_data);
	}
}

uint16_t wxstar4k_state::cpubd_data_latch_r()
{
	uint16_t data = m_data_latch_out->read();
	LOGLATCH("%s: Read byte from Data card latch @ C0A400 = %02X\n", machine().describe_context(), data & 0xff);
	return data;
}

void wxstar4k_state::cpubd_data_latch_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		uint8_t byte_data = data & 0xff;
		if (offset == 0 || offset == 0x100)
		{
			LOGLATCH("Write byte to Data card latch @ %08X: data %02X\n", 0xC0A000 + (offset*2), byte_data);
			m_data_latch_in->write(byte_data);
		}
		else if (offset == 0x300)
		{
			LOGCPU("Write byte to audio control latch 1 @ C0A600: data %02X\n", byte_data);
		}
		else if (offset == 0x400)
		{
			LOGCPU("Write byte to audio control latch 2 @ C0A800: data %02X\n", byte_data);
		}
	} else {
		LOGCPU("Write to Data card area offset %X: data %04X & %04X\n", offset*2, data, mem_mask);
	}
}


void wxstar4k_state::cpubd_main_map(address_map &map)
{
	map(0x000000, 0x1fffff).ram().share("mainram");
	map(0x200000, 0x3fffff).ram().share("extram");
	map(0x400000, 0xbfffff).r(FUNC(wxstar4k_state::buserr_r));
	map(0xc00000, 0xc00001).rw(FUNC(wxstar4k_state::cpubd_io_status_r), FUNC(wxstar4k_state::cpubd_io_control_w));
	map(0xc00002, 0xc001ff).noprw();
	map(0xc04000, 0xc041ff).noprw();
	map(0xc0a000, 0xc0a801).rw(FUNC(wxstar4k_state::cpubd_data_latch_r), FUNC(wxstar4k_state::cpubd_data_latch_w));
	map(0xc0a802, 0xfcffff).r(FUNC(wxstar4k_state::buserr_r));
	map(0xfd0000, 0xfd1fff).ram().share("eeprom");
	map(0xfd1fff, 0xfd1fff).w(FUNC(wxstar4k_state::led_w));
	map(0xfd8000, 0xfd800f).rw(m_ptm, FUNC(ptm6840_device::read), FUNC(ptm6840_device::write)).umask16(0x00ff); // PTM @ Guessed addr
	map(0xfdf000, 0xfdf007).w(FUNC(wxstar4k_state::cpubd_gfx_irq6_w));
	map(0xfdf008, 0xfdf009).w(FUNC(wxstar4k_state::cpubd_watchdog_reset_w));
	map(0xfdf00a, 0xfdffbf).r(FUNC(wxstar4k_state::buserr_r));
	map(0xfdffc0, 0xfdffdf).rw(m_rtc, FUNC(icm7170_device::read), FUNC(icm7170_device::write)).umask16(0x00ff);
	map(0xfdffe0, 0xfdffff).r(FUNC(wxstar4k_state::buserr_r));
	map(0xfe0000, 0xffffff).rom().region("maincpu", 0);
}

// --- Graphics Board 68010 ---

uint16_t wxstar4k_state::vidbd_status_r()
{
	uint16_t status = 0;
	if (!m_gfx_latch_in->pending_r()) status |= 0x0001;
	if (m_gfx_sub_p1 & 0x08) status |= 0x0002;
	LOGGFX("%s: Read GFX Status @ 200000 = %04X\n", machine().describe_context(), status);
	return status;
}
void wxstar4k_state::vidbd_vme_addr_hi_w(offs_t offset, uint16_t data, uint16_t mem_mask) { LOGGFX("Write GFX VME Addr Hi @ 200000: data %04X & %04X\n", data, mem_mask); }
void wxstar4k_state::vidbd_irq_vector_w(offs_t offset, uint16_t data, uint16_t mem_mask) { if (ACCESSING_BITS_0_7) { m_cpu_irq_vector = data & 0xff; LOGIRQ("GFX CPU set Main CPU IRQ4 vector to %02X\n", m_cpu_irq_vector); } }
void wxstar4k_state::vidbd_latch_w(offs_t offset, uint16_t data, uint16_t mem_mask) { if (ACCESSING_BITS_0_7) { LOGLATCH("Write GFX 8051 latch @ 200004: data %02X\n", data & 0xff); m_gfx_latch_in->write(data & 0xff); } }
void wxstar4k_state::vidbd_main_cpu_irq4_w(offs_t offset, uint16_t data, uint16_t mem_mask) { LOGIRQ("%s: GFX CPU triggering Main CPU IRQ 4\n", machine().describe_context()); m_maincpu->set_input_line_and_vector(M68K_IRQ_4, ASSERT_LINE, m_cpu_irq_vector); }
uint16_t wxstar4k_state::vidbd_gfx_control_r(offs_t offset) { LOGGFX("%s: Read GFX Control Reg @ %08X\n", machine().describe_context(), 0x300000 + offset*2); return 0; }
void wxstar4k_state::vidbd_gfx_control_w(offs_t offset, uint16_t data, uint16_t mem_mask) { LOGGFX("Write GFX Control Reg @ %08X: data %04X & %04X\n", 0x300000 + offset*2, data, mem_mask); }

void wxstar4k_state::vidbd_main_map(address_map &map)
{
	map(0x000000, 0x00ffff).rom().region("gfxcpu", 0);
	map(0x100000, 0x10ffff).ram();
	map(0x200000, 0x200001).rw(FUNC(wxstar4k_state::vidbd_status_r), FUNC(wxstar4k_state::vidbd_vme_addr_hi_w));
	map(0x200002, 0x200003).nopr().w(FUNC(wxstar4k_state::vidbd_irq_vector_w));
	map(0x200004, 0x200005).w(FUNC(wxstar4k_state::vidbd_latch_w));
	map(0x200006, 0x200007).w(FUNC(wxstar4k_state::vidbd_main_cpu_irq4_w));
	map(0x300000, 0x300003).rw(FUNC(wxstar4k_state::vidbd_gfx_control_r), FUNC(wxstar4k_state::vidbd_gfx_control_w));
	map(0x400000, 0x5fffff).ram().share("vram");
	map(0xe00000, 0xe1ffff).noprw();
}

// --- Graphics Board 8051 ---

uint8_t wxstar4k_state::vidbd_sub_vram_counter_low_r(offs_t offset) { return m_vram_addr_low & 0xff; }
void wxstar4k_state::vidbd_sub_vram_counter_low_w(offs_t offset, uint8_t data) { m_vram_addr_low = (m_vram_addr_low & 0xff00) | data; }
uint8_t wxstar4k_state::vidbd_sub_vram_counter_high_r(offs_t offset) { return (m_vram_addr_low >> 8) & 0xff; }
void wxstar4k_state::vidbd_sub_vram_counter_high_w(offs_t offset, uint8_t data) { m_vram_addr_low = (m_vram_addr_low & 0x00ff) | (uint16_t(data) << 8); }
uint8_t wxstar4k_state::vidbd_sub_latch_r() { uint8_t data = m_gfx_latch_in->read(); LOGLATCH("%s: GFX 8051 reads latch from 68010 = %02X\n", machine().describe_context(), data); return data; }

void wxstar4k_state::vidbd_sub_map(address_map &map) { map(0x0000, 0x1fff).rom().region("gfxsubcpu", 0); }
void wxstar4k_state::vidbd_sub_io_map(address_map &map)
{
	map(0x0000, 0x07ff).ram();
	map(0x0800, 0x0fff).noprw();
	map(0x1000, 0x17ff).rw(FUNC(wxstar4k_state::vidbd_sub_vram_counter_low_r), FUNC(wxstar4k_state::vidbd_sub_vram_counter_low_w));
	map(0x1800, 0x1fff).rw(FUNC(wxstar4k_state::vidbd_sub_vram_counter_high_r), FUNC(wxstar4k_state::vidbd_sub_vram_counter_high_w));
	map(0x2000, 0x27ff).r(FUNC(wxstar4k_state::vidbd_sub_latch_r));
	map(0x8000, 0x8000).mirror(0x4000).w(m_ramdac, FUNC(bt471_device::write)).select(0);
	map(0x8800, 0x8800).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read), FUNC(bt471_device::write)).select(1);
	map(0x9000, 0x9000).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read), FUNC(bt471_device::write)).select(2);
	map(0x9800, 0x9800).mirror(0x4000).r(m_ramdac, FUNC(bt471_device::read)).select(0);
	map(0xa000, 0xa000).mirror(0x4000).w(m_ramdac, FUNC(bt471_device::write)).select(4);
	map(0xa800, 0xa800).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read), FUNC(bt471_device::write)).select(5);
	map(0xb800, 0xb800).mirror(0x4000).r(m_ramdac, FUNC(bt471_device::read)).select(4);
	map(0xc000, 0xffff).noprw();
}
uint8_t wxstar4k_state::vidbd_sub_p1_r() { uint8_t data = m_gfx_sub_p1; if (!m_gfx_latch_in->pending_r()) data |= 0x01; LOGGFXSUB("Read P1 = %02X (internal latch %02X)\n", data, m_gfx_sub_p1); return data; }
void wxstar4k_state::vidbd_sub_p1_w(uint8_t data) { m_gfx_sub_p1 = data; }
uint8_t wxstar4k_state::vidbd_sub_p3_r() { return 0xff; }
void wxstar4k_state::vidbd_sub_p3_w(uint8_t data) { }

// --- Data Board 8344 ---

uint8_t wxstar4k_state::databd_latch_in_r() { uint8_t data = m_data_latch_in->read(); LOGLATCH("Data 8344 reads latch from main CPU = %02X\n", data); return data; }
void wxstar4k_state::databd_latch_out_w(uint8_t data) { LOGLATCH("Data 8344 writes latch to main CPU = %02X\n", data); m_data_latch_out->write(data); }
uint8_t wxstar4k_state::databd_pio1_r(offs_t offset){ LOGDATA("Read PIO1 offset %d\n", offset); return 0; }
void wxstar4k_state::databd_pio1_w(offs_t offset, uint8_t data){ LOGDATA("Write PIO1 offset %d = %02X\n", offset, data); }
uint8_t wxstar4k_state::databd_pio2_r(offs_t offset){ LOGDATA("Read PIO2 offset %d\n", offset); return 0; }
void wxstar4k_state::databd_pio2_w(offs_t offset, uint8_t data){ LOGDATA("Write PIO2 offset %d = %02X\n", offset, data); }

void wxstar4k_state::databd_main_map(address_map &map) { map(0x0000, 0x1fff).rom().region("datacpu", 0); }
void wxstar4k_state::databd_main_io_map(address_map &map)
{
	map(0x0000, 0x01ff).ram();
	map(0x0200, 0x0201).noprw();
	map(0x2000, 0x2000).r(FUNC(wxstar4k_state::databd_latch_in_r)); // Guessed addr
	map(0x2001, 0x2001).w(FUNC(wxstar4k_state::databd_latch_out_w)); // Guessed addr
	map(0x8000, 0x8005).rw(FUNC(wxstar4k_state::databd_pio1_r), FUNC(wxstar4k_state::databd_pio1_w));
	map(0x8100, 0x8105).rw(FUNC(wxstar4k_state::databd_pio2_r), FUNC(wxstar4k_state::databd_pio2_w));
}

// --- I/O Board 8031 ---

uint8_t wxstar4k_state::iobd_latch_in_r() { uint8_t data = m_io_latch_in->read(); LOGLATCH("I/O 8031 reads latch from main CPU = %02X\n", data); return data; }
void wxstar4k_state::iobd_latch_out_w(uint8_t data) { LOGLATCH("I/O 8031 writes latch to main CPU = %02X\n", data); m_io_latch_out->write(data); }
uint8_t wxstar4k_state::iobd_kbd_data_r() { uint8_t data = m_io_kbd_data; m_io_kbd_status &= ~1; LOGIO("I/O 8031 reads keyboard data = %02X\n", data); return data; }
uint8_t wxstar4k_state::iobd_kbd_status_r() { LOGIO("I/O 8031 reads keyboard status = %02X\n", m_io_kbd_status); return m_io_kbd_status; }

void wxstar4k_state::iobd_main_map(address_map &map) { map(0x0000, 0x7fff).rom().region("iocpu", 0); }
void wxstar4k_state::iobd_main_io_map(address_map &map)
{
	map(0x6000, 0x6000).r(FUNC(wxstar4k_state::iobd_latch_in_r)); // Guessed addr
	map(0x6001, 0x6001).w(FUNC(wxstar4k_state::iobd_latch_out_w)); // Guessed addr
	map(0x8000, 0x8000).rw(m_uart, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0x8001, 0x8001).rw(m_uart, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0x9000, 0x9000).r(FUNC(wxstar4k_state::iobd_kbd_data_r));
	map(0x9001, 0x9001).r(FUNC(wxstar4k_state::iobd_kbd_status_r));
}

// --- Machine Lifecycle ---

void wxstar4k_state::machine_start()
{
	m_diag_led.resolve();
	save_item(NAME(m_cpu_irq_vector));
	save_item(NAME(m_gfx_irq_vector));
	save_item(NAME(m_main_watchdog));
	save_item(NAME(m_vram_addr_low));
	save_item(NAME(m_vram_addr_high));
	save_item(NAME(m_gfx_sub_p1));
	save_item(NAME(m_io_kbd_data));
	save_item(NAME(m_io_kbd_status));
	m_nvram->set_base(memshare("eeprom")->ptr(), 0x2000);
}

void wxstar4k_state::machine_reset()
{
	uint16_t *ram = m_mainram.target();
	uint16_t *rom = (uint16_t *)memregion("maincpu")->base();
	ram[0] = rom[0]; ram[1] = rom[1]; // SP
	ram[2] = rom[2]; ram[3] = rom[3]; // PC
	m_cpu_irq_vector = 0;
	m_gfx_irq_vector = 0;
	m_main_watchdog = 0;
	m_vram_addr_low = 0;
	m_vram_addr_high = 0;
	m_gfx_sub_p1 = 0xff;
	m_io_kbd_data = 0;
	m_io_kbd_status = 0;
	m_gfxcpu->reset();
	m_gfxsubcpu->reset();
	m_datacpu->reset();
	m_iocpu->reset();
}

// --- Interrupt Handling ---

TIMER_DEVICE_CALLBACK_MEMBER(wxstar4k_state::vblank_irq)
{
	m_gfxcpu->set_input_line(M68K_IRQ_5, ASSERT_LINE);
	m_gfxcpu->set_input_line(M68K_IRQ_5, CLEAR_LINE);
	m_gfxsubcpu->set_input_line(MCS51_INT0_LINE, ASSERT_LINE);
}
void wxstar4k_state::rtc_irq_w(int state)
{
	LOGIRQ("RTC IRQ -> Main CPU IRQ 1 %s\n", state ? "Assert" : "Clear");
	m_maincpu->set_input_line(M68K_IRQ_1, state ? ASSERT_LINE : CLEAR_LINE);
}
void wxstar4k_state::ptm_irq_w(int state)
{
	LOGIRQ("PTM IRQ -> Main CPU IRQ 3 %s\n", state ? "Assert" : "Clear");
	m_maincpu->set_input_line_and_vector(M68K_IRQ_3, state ? ASSERT_LINE : CLEAR_LINE, m_cpu_irq_vector); // Assuming vector needed
}
void wxstar4k_state::data_latch_irq_w(int state)
{
	LOGIRQ("Data Latch IRQ -> Main CPU IRQ 5 %s\n", state ? "Assert" : "Clear");
	m_maincpu->set_input_line_and_vector(M68K_IRQ_5, state ? ASSERT_LINE : CLEAR_LINE, m_cpu_irq_vector);
}
void wxstar4k_state::io_latch_irq_w(int state)
{
	LOGIRQ("I/O Latch IRQ -> Main CPU IRQ 2 %s\n", state ? "Assert" : "Clear");
	m_maincpu->set_input_line_and_vector(M68K_IRQ_2, state ? ASSERT_LINE : CLEAR_LINE, m_cpu_irq_vector);
}
void wxstar4k_state::kbd_put_key(uint8_t scancode)
{
	LOGIO("Keyboard received scancode %02X\n", scancode);
	m_io_kbd_data = scancode;
	m_io_kbd_status |= 1;
	// TODO: Assert 8031 interrupt?
}

static INPUT_PORTS_START( wxstar4k )
	PORT_START("KEYBOARD") // Tag must match the AT_KEYBOARD device tag
	PORT_INCLUDE(at_keyboard)
INPUT_PORTS_END


// --- Machine Configuration ---

void wxstar4k_state::wxstar4k(machine_config &config)
{
	/* basic machine hardware */
	M68010(config, m_maincpu, XTAL(20'000'000)/2);
	m_maincpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::cpubd_main_map);

	NVRAM(config, "eeprom", nvram_device::DEFAULT_ALL_0);

	ICM7170(config, m_rtc, XTAL(32'768));
	m_rtc->irq().set(FUNC(wxstar4k_state::rtc_irq_w));

	PTM6840(config, m_ptm, XTAL(20'000'000)/80);
	m_ptm->set_external_clocks(XTAL(20'000'000)/80, 0, XTAL(20'000'000)/80);
	m_ptm->irq_callback().set(FUNC(wxstar4k_state::ptm_irq_w));

	/* Graphics board hardware */
	M68010(config, m_gfxcpu, XTAL(20'000'000)/2);
	m_gfxcpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::vidbd_main_map);

	I8051(config, m_gfxsubcpu, XTAL(12'000'000));
	m_gfxsubcpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::vidbd_sub_map);
	m_gfxsubcpu->set_addrmap(AS_IO, &wxstar4k_state::vidbd_sub_io_map);
	m_gfxsubcpu->port_in_cb<1>().set(FUNC(wxstar4k_state::vidbd_sub_p1_r));
	m_gfxsubcpu->port_out_cb<1>().set(FUNC(wxstar4k_state::vidbd_sub_p1_w));
	m_gfxsubcpu->port_in_cb<3>().set(FUNC(wxstar4k_state::vidbd_sub_p3_r));
	m_gfxsubcpu->port_out_cb<3>().set(FUNC(wxstar4k_state::vidbd_sub_p3_w));

	BT471(config, m_ramdac, 0);

	/* Data/Audio board hardware */
	I8344(config, m_datacpu, XTAL(7'372'800));
	m_datacpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::databd_main_map);
	m_datacpu->set_addrmap(AS_IO, &wxstar4k_state::databd_main_io_map);

	/* I/O board hardware */
    I8031(config, m_iocpu, XTAL(11'059'200));
    m_iocpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::iobd_main_map);
    m_iocpu->set_addrmap(AS_IO, &wxstar4k_state::iobd_main_io_map);

    I8251(config, m_uart, XTAL(11'059'200)/6);
    m_uart->txd_handler().set(m_rs232, FUNC(rs232_port_device::write_txd));
    m_uart->dtr_handler().set(m_rs232, FUNC(rs232_port_device::write_dtr));
    m_uart->rts_handler().set(m_rs232, FUNC(rs232_port_device::write_rts));

    RS232_PORT(config, m_rs232, default_rs232_devices, nullptr);
    m_rs232->rxd_handler().set(m_uart, FUNC(i8251_device::write_rxd));
    m_rs232->dsr_handler().set(m_uart, FUNC(i8251_device::write_dsr));
    m_rs232->cts_handler().set(m_uart, FUNC(i8251_device::write_cts));

    // Use the correct macro and tag for AT Keyboard
    AT_KEYBOARD(config, KBD_TAG, 0); // Instantiate device with the tag used in the finder
    m_kbd->keypress().set(FUNC(wxstar4k_state::kbd_put_key)); // Connect callback

    /* Latches for Inter-CPU Communication */
    GENERIC_LATCH_8(config, m_gfx_latch_in);
    GENERIC_LATCH_8(config, m_data_latch_in);
    GENERIC_LATCH_8(config, m_data_latch_out).data_pending_callback().set(FUNC(wxstar4k_state::data_latch_irq_w));
    GENERIC_LATCH_8(config, m_io_latch_in);
    GENERIC_LATCH_8(config, m_io_latch_out).data_pending_callback().set(FUNC(wxstar4k_state::io_latch_irq_w));

	/* video hardware */
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_raw(XTAL(20'000'000) * 2 / 3, 858, 0, 640, 262, 0, 240);
	m_screen->set_screen_update(FUNC(wxstar4k_state::screen_update));
	m_screen->set_palette(m_palette);

	PALETTE(config, m_palette).set_entries(256);

	TIMER(config, "vblank").configure_periodic(FUNC(wxstar4k_state::vblank_irq), m_screen->frame_period());

	/* sound hardware */
	// MACHINE_NO_SOUND flag set
}

// --- ROM Loading ---

ROM_START( wxstar4k )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "u79 rom.bin",  0x000001, 0x010000, CRC(11df2d70) SHA1(ac6cdb5290c90b043562464dc001fc5e3d26f7c6) )
	ROM_LOAD16_BYTE( "u80 rom.bin",  0x000000, 0x010000, CRC(23e15f22) SHA1(a630bda39c0beec7e7fc3834178ec8a6fece70c8) )

	ROM_REGION( 0x2000, "eeprom", ROMREGION_ERASEFF )
	ROM_LOAD( "u72 eeprom.bin", 0x000000, 0x002000, CRC(f775b4d6) SHA1(a0895177c381919f9bfd99ee35edde0dd5fa379c) )

	ROM_REGION(0x2000, "datacpu", 0)
	ROM_LOAD( "u12 rom.bin",  0x000000, 0x002000, CRC(f7d8432d) SHA1(0ff1dad65ecb4c3d8cb21feef56bbc6f06a2f712) )

	ROM_REGION(0x8000, "iocpu", 0)
	ROM_LOAD( "u11 rom.bin",  0x000000, 0x008000, CRC(f12cb28b) SHA1(3368f55717d8e9e7a06a4f241de02b7b2577b32b) )

	ROM_REGION(0x2000, "gfxsubcpu", 0)
	ROM_LOAD( "u13 rom.bin",  0x000000, 0x002000, CRC(667b0a2b) SHA1(d60bcc271a73633544b0cf2f80589b2e5670b705) )

	ROM_REGION(0x10000, "gfxcpu", 0)
	ROM_LOAD16_BYTE( "u42 rom low.bin", 0x000001, 0x008000, CRC(84038ca3) SHA1(b28a0d357d489fb06ff0d5d36ea11ebd1f9612a5) )
	ROM_LOAD16_BYTE( "u43 rom high.bin", 0x000000, 0x008000, CRC(6f2a7592) SHA1(1aa2394db42b6f28277e35a48a7cef348c213e05) )
ROM_END

} // anonymous namespace


//                                                                    YEAR  NAME      PARENT COMPAT MACHINE   INPUT     CLASS           INIT        COMPANY                                            FULLNAME            FLAGS
COMP( 1990, wxstar4k, 0,     0,     wxstar4k, wxstar4k, wxstar4k_state, empty_init, "Applied Microelectronics Institute/The Weather Channel", "WeatherSTAR 4000", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
