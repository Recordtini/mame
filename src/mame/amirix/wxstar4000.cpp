// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

    wxstar4000.cpp: WeatherSTAR 4000 cable head-end unit
    1990 Applied Microelectronics Institute (Amirix) / The Weather Channel
    Skeleton driver by R. Belmont

  This was used by cable companies in the US starting in 1990 to generate
  graphics and text for the local weather forecast during The Weather Channel's
  "Local on the 8s" segments.

 There are 4 PCBs on a VME backplane:

 - CPU board contains a 68010 CPU, RAM (2MB private + 2MB shared), and EEPROM (8KB)
 - Graphics board contains a 68010 CPU, an i8051 sub-CPU, RAM (64KB), VRAM (2MB), and RAMDAC (Bt471)
 - Data/Audio board contains an Intel P8344AH (i8051 + SDLC), RAM (512 bytes), PIOs, and tone generation.
 - I/O board contains an i8031 CPU, RAM (not specified?), an i8251A UART, and an AT keyboard interface.

   CPU board 68010 IRQs:
   IRQ1 = RTC (ICM7170) Interrupt. Autovectored.
   IRQ2 = I/O Card Incoming interrupt Request. Triggered when I/O card needs us to handle something. Needs vector.
   IRQ3 = Secondary Graphics Card Incoming interrupt request. Not used in single-card systems.
   IRQ4 = Primary Graphics Card Incoming interrupt request. Needs vector.
   IRQ5 = Data Card incoming interrupt request. Triggers when the Data card has something for us to do or handle. Needs vector.
   IRQ6 = Unused.
   IRQ7 = AC Fail, Battery Backup Input. Once triggered, Unrecoverable. Requires System Reset. (NMI equivalent?)

   Graphics board 68010 IRQs:
   IRQ5 = Vertical Blanking Interrupt. Autovectored. Used for timing graphics acceleration instructions.
   IRQ6 = Incoming request from the CPU Card. Used to signal us to go do something. Needs vector?
   IRQ7 = AC Fail, Battery Backup Input. (NMI equivalent?)

   Graphics board 8051 GPIO pins:
   P1.0(T2) = FIFO Empty Flag
   P1.1(T2EX) = Switch to Sat Video. (Active Low)(Drive high to shutdown genlock)
   P1.2 = Switch To Local Video. (Active High)
   P1.3 = Flag to 68K CPU (Control CPU Ready for Command, Active High)
   P1.4 = Sat Video Present/Odd-Even Frame Indicator
   P1.5 = Frame/Sync Control Register (Or Timer prescaler)
   P1.6 = Frame/Sync Control Register (Or Timer prescaler)
   P1.7 = Frame/Sync Control Register (Or Timer prescaler), Watchdog timer.
   P3.0 = RX from FPGA (?) Maybe FIFO Write?
   P3.1 = TX to FPGA (?) Maybe FIFO Read?
   P3.2(INT0) = Vertical Drive Interrupt (VSYNC?)
   P3.3(INT1) = Odd/Even Frame Sync Interrupt (HSYNC?)
   P3.4(T0) = Input from Frame/Sync Control Register
   P3.5(T1) = Input from Frame/Sync Control Register

   TODO:
   - Implement FIFOs between CPUs (Main <-> GFX, Main <-> Data)
   - Implement detailed I/O registers for all boards
   - Implement shared memory access control/arbitration if needed (VME bus?)
   - Implement inter-CPU interrupt signaling accurately
   - Implement Graphics 8051 control of RAMDAC, VRAM access, and sync generation
   - Implement Data 8344 SDLC communication (satellite feed) and PIO control (LEDs, etc.)
   - Implement I/O 8031 UART (modem) and Keyboard communication
   - Implement audio tone generation
   - Verify clock speeds and timings
   - Map unknown address areas based on code execution
   - Verify screen parameters and VRAM mapping
   - Verify Bt471 register mapping (pixel mask, overlay addr vs data)

***************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68010.h"
#include "cpu/mcs51/mcs51.h"
#include "machine/gen_latch.h" // For inter-CPU communication latches/FIFOs potentially
#include "machine/icm7170.h"
#include "machine/nvram.h"
#include "machine/timer.h" // For VBLANK timer if needed
#include "video/bt47x.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"

// Potentially needed devices
#include "machine/i8251.h"
#include "machine/keyboard.h" // For AT keyboard
//#include "machine/i8255.h" // For PIOs on Data board if they are 8255s

#define LOG_CPU_IO (1U << 1)
#define LOG_GFX_IO (1U << 2)
#define LOG_GFX_SUB_IO (1U << 3)
#define LOG_DATA_IO (1U << 4)
#define LOG_IO_IO (1U << 5)
#define LOG_IRQ (1U << 6)
#define LOG_WARN (1U << 7)

//#define VERBOSE (LOG_CPU_IO | LOG_GFX_IO | LOG_GFX_SUB_IO | LOG_DATA_IO | LOG_IO_IO | LOG_IRQ | LOG_WARN)
#define VERBOSE (LOG_WARN | LOG_GFX_SUB_IO) // Enable GFX Sub IO logging for Bt471 access check
#include "logmacro.h"

#define LOGCPU(x, ...)    LOGMASKED(LOG_CPU_IO, x, ##__VA_ARGS__)
#define LOGGFX(x, ...)    LOGMASKED(LOG_GFX_IO, x, ##__VA_ARGS__)
#define LOGGFXSUB(x, ...) LOGMASKED(LOG_GFX_SUB_IO, x, ##__VA_ARGS__)
#define LOGDATA(x, ...)   LOGMASKED(LOG_DATA_IO, x, ##__VA_ARGS__)
#define LOGIO(x, ...)     LOGMASKED(LOG_IO_IO, x, ##__VA_ARGS__)
#define LOGIRQ(x, ...)    LOGMASKED(LOG_IRQ, x, ##__VA_ARGS__)
#define LOGWARN(x, ...)   LOGMASKED(LOG_WARN, x, ##__VA_ARGS__)


namespace {

class wxstar4k_state : public driver_device
{
public:
	wxstar4k_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
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
		m_cpu_irq_vector(0),
		m_gfx_irq_vector(0),
		m_gfx_sub_fifo_in(0), // Placeholder for FIFO
		m_gfx_sub_fifo_out(0), // Placeholder for FIFO
		m_gfx_sub_p1(0xff),
		m_main_watchdog(0)
	{ }

	void wxstar4k(machine_config &config);

private:
	// Screen update
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

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
	virtual void video_start() override ATTR_COLD;

	// Devices
	required_device<m68010_device> m_maincpu;
	required_device<m68010_device> m_gfxcpu;
	required_device<mcs51_cpu_device> m_gfxsubcpu;
	required_device<mcs51_cpu_device> m_datacpu; // P8344 is i8051 based
	required_device<mcs51_cpu_device> m_iocpu; // i8031
	required_shared_ptr<uint16_t> m_mainram;
	required_shared_ptr<uint16_t> m_extram;
	required_shared_ptr<uint8_t> m_vram; // VRAM is usually 8-bit, check access width
	required_device<icm7170_device> m_rtc;
	required_device<bt471_device> m_ramdac;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<nvram_device> m_nvram;
	// optional_device<i8251_device> m_uart;
	// optional_device<generic_keyboard_device> m_keyboard;
	// optional_device<speaker_sound_device> m_speaker;

	// Internal state
	uint8_t m_cpu_irq_vector; // Vector for IRQ 2, 4, 5 on main CPU
	uint8_t m_gfx_irq_vector; // Vector for IRQ 6 on gfx CPU

	// Placeholder FIFO/Communication mechanism (replace with gen_latch or proper FIFO emulation later)
	uint8_t m_gfx_sub_fifo_in; // Data from GFX 68K to GFX 8051
	uint8_t m_gfx_sub_fifo_out; // Data from GFX 8051 to GFX 68K
	uint8_t m_gfx_sub_p1; // GFX 8051 Port 1 state
	uint16_t m_vram_addr_low = 0;
	uint16_t m_vram_addr_high = 0;

	uint8_t m_main_watchdog; // Simple watchdog counter/flag

	// CPU Board I/O Handlers
	uint16_t buserr_r();
	uint16_t cpubd_io_control_r();
	void cpubd_io_control_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t cpubd_data_fifo_r();
	void cpubd_data_fifo_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void cpubd_gfx_irq6_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void cpubd_watchdog_reset_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	// Graphics Board 68010 I/O Handlers
	uint16_t vidbd_status_r();
	void vidbd_vme_addr_hi_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_irq_vector_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_fifo_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void vidbd_main_cpu_irq4_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t vidbd_gfx_control_r(offs_t offset);
	void vidbd_gfx_control_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	// Graphics Board 8051 I/O Handlers
	uint8_t vidbd_sub_vram_counter_low_r(offs_t offset);
	void vidbd_sub_vram_counter_low_w(offs_t offset, uint8_t data);
	uint8_t vidbd_sub_vram_counter_high_r(offs_t offset);
	void vidbd_sub_vram_counter_high_w(offs_t offset, uint8_t data);
	uint8_t vidbd_sub_fifo_r();
	uint8_t vidbd_sub_p1_r();
	void vidbd_sub_p1_w(uint8_t data);
	uint8_t vidbd_sub_p3_r();
	void vidbd_sub_p3_w(uint8_t data);

	// Interrupt handling
	TIMER_DEVICE_CALLBACK_MEMBER(vblank_irq);
	void rtc_irq_w(int state);

	// Helper Functions
	inline uint32_t get_vram_address();
	inline uint8_t get_pixel(uint32_t addr);
};

// --- Video ---

uint32_t wxstar4k_state::get_vram_address()
{
	return (uint32_t(m_vram_addr_high) << 16) | m_vram_addr_low;
}

uint8_t wxstar4k_state::get_pixel(uint32_t addr)
{
	// VRAM is 2MB = 0x200000 bytes. Check bounds.
	if (addr < m_vram.bytes())
	{
		// Direct VRAM access from RAMDAC seems unlikely. Usually via shift registers or palette lookups.
		// Let's assume direct pixel lookup for now. The 68k maps it at 0x400000
		// Need to figure out how pixels map to screen coords. Assume linear for now.
		// Check byte order? VRAM shared ptr is uint8_t.
		return m_vram[addr];
	}
	return 0; // Out of bounds
}

uint32_t wxstar4k_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// Basic framebuffer draw. Assumes 8bpp indexed color.
	// Need to determine the actual resolution and pixel mapping.
	// The default is 512x256, visarea 40..399, 16..239 -> 360x224 visible?
	// Framebuffer size is 2MB (0x200000 bytes). If 8bpp, this is huge (e.g., 2048x1024).
	// Maybe it's used for multiple frames, layers, or higher color depth internally?
	// Or the mapping isn't linear 1 byte = 1 pixel.
	// Start with a plausible guess, e.g., 640x480? 512x256?
	// Using the configured screen size for now (512x256)

	// Fix: Use const pen_t*
	const pen_t *palette = m_palette->pens();
	uint32_t vram_addr = 0; // Needs to be based on scroll registers / display start address if any

	// Let's assume the visible area maps directly from the start of VRAM for now
	// This is almost certainly wrong, but gives *something* on screen.
	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		uint32_t *scanline = &bitmap.pix(y, cliprect.left());
		// Calculate VRAM start for this scanline based on assumed screen width
		vram_addr = y * 640; // Adjust 640 based on actual logical screen width stored in VRAM (using set_raw width for now)
		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			// Check bounds before reading VRAM
			if (vram_addr < m_vram.bytes()) {
				uint8_t pixel_data = m_vram[vram_addr++];
				*scanline++ = palette[pixel_data];
			} else {
				*scanline++ = rgb_t::black(); // Draw black if out of VRAM bounds
			}
		}
	}

	return 0;
}

void wxstar4k_state::video_start()
{
	// Allocate timers or save states if needed
	save_item(NAME(m_vram_addr_low));
	save_item(NAME(m_vram_addr_high));
	save_item(NAME(m_gfx_sub_p1));
	save_item(NAME(m_gfx_sub_fifo_in));
	save_item(NAME(m_gfx_sub_fifo_out));
}

// --- CPU Board ---

uint16_t wxstar4k_state::buserr_r()
{
	// Reading unmapped memory causes a bus error
	LOGWARN("%s: Bus error read access!\n", machine().describe_context());
	if (!machine().side_effects_disabled())
	{
		// Fix: Use set_input_line to assert BERR
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
		m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	}
	return 0xffff; // Return value is often ignored by CPU during exception
}

void wxstar4k_state::cpubd_watchdog_reset_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// Resets some kind of watchdog timer
	LOGCPU("Watchdog reset write: %04X & %04X\n", data, mem_mask);
	m_main_watchdog = 0; // Placeholder reset
}

void wxstar4k_state::cpubd_gfx_irq6_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// FDF000: Cause IRQ 6 on graphics card
	// FDF004: Cause IRQ 6 on graphics card 2 (not used)
	if (ACCESSING_BITS_0_15 && offset == 0) // Only trigger for FDF000
	{
		LOGIRQ("%s: Main CPU triggering GFX CPU IRQ 6\n", machine().describe_context());
		// TODO: Determine if it needs a vector provided or is auto-vectored. Assuming needs vector for now.
		m_gfxcpu->set_input_line_and_vector(M68K_IRQ_6, ASSERT_LINE, m_gfx_irq_vector); // Use stored vector
	}
	else
	{
		LOGCPU("Write to GFX IRQ trigger area offset %d, data %04X & %04X\n", offset, data, mem_mask);
	}
}

uint16_t wxstar4k_state::cpubd_io_control_r()
{
	// C00000 - I/O card control register read?
	LOGCPU("%s: Read from I/O card control register @ C00000\n", machine().describe_context());
	// Return value depends on what status the main CPU expects from the IO card
	return 0x0000; // Placeholder
}

void wxstar4k_state::cpubd_io_control_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// C00000 - I/O card control register write?
	LOGCPU("Write to I/O card control register @ C00000: data %04X & %04X\n", data, mem_mask);
	// This likely sends commands or configures the I/O board 8031
}

uint16_t wxstar4k_state::cpubd_data_fifo_r()
{
	// C0A400 - read byte from data card CPU (FIFO?)
	LOGCPU("%s: Read byte from Data card CPU @ C0A400\n", machine().describe_context());
	// TODO: Implement FIFO read from Data card 8344
	return 0x0000; // Placeholder
}

void wxstar4k_state::cpubd_data_fifo_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// C0A000 - data card FIFO write?
	// C0A200 - write byte to data card CPU?
	if (offset == 0) // C0A000
	{
		LOGCPU("Write byte to Data card FIFO @ C0A000: data %04X & %04X\n", data, mem_mask);
		// TODO: Implement FIFO write to Data card 8344
	}
	else if (offset == 0x100) // C0A200 (byte offset 0x200 -> word offset 0x100)
	{
		LOGCPU("Write byte to Data card CPU @ C0A200: data %04X & %04X\n", data, mem_mask);
		// TODO: Implement direct byte write/command to Data card 8344?
	}
	else if (offset == 0x300) // C0A600
	{
		LOGCPU("Write byte to audio control latch 1 @ C0A600: data %04X & %04X\n", data, mem_mask);
		// TODO: Control audio tone generator
	}
	else if (offset == 0x400) // C0A800
	{
		LOGCPU("Write byte to audio control latch 2 @ C0A800: data %04X & %04X\n", data, mem_mask);
		// TODO: Control audio tone generator
	}
	else {
		LOGCPU("Write to Data card area offset %X: data %04X & %04X\n", offset*2, data, mem_mask);
	}
}


void wxstar4k_state::cpubd_main_map(address_map &map)
{
	map(0x000000, 0x1fffff).ram().share("mainram"); // 2MB private RAM
	map(0x200000, 0x3fffff).ram().share("extram");  // 2MB RAM accessible by other cards (shared VME space?)
	// Gap where Bus Error occurs? Some systems map peripherals here.
	map(0x400000, 0xbfffff).r(FUNC(wxstar4k_state::buserr_r)); // Unmapped?

	// --- I/O Board Access ---
	// C00000 - I/O card control register
	map(0xc00000, 0xc00001).rw(FUNC(wxstar4k_state::cpubd_io_control_r), FUNC(wxstar4k_state::cpubd_io_control_w));
	// C00001-C001FF - I/O card UART buffer (511 bytes). Need read/write handlers. Shared RAM or FIFO? Assume FIFO for now.
	map(0xc00002, 0xc001ff).noprw(); // Placeholder
	// C04000-C041FF - I/O card modem buffer (512 bytes). Need read/write handlers. Shared RAM or FIFO? Assume FIFO for now.
	map(0xc04000, 0xc041ff).noprw(); // Placeholder

	// --- Data Card Access ---
	// C0A000 - data card FIFO write?
	// C0A200 - write byte to data card CPU?
	// C0A400 - read byte from data card CPU?
	// C0A600 - write byte to audio control latch 1
	// C0A800 - write byte to audio control latch 2
	map(0xc0a000, 0xc0a801).rw(FUNC(wxstar4k_state::cpubd_data_fifo_r), FUNC(wxstar4k_state::cpubd_data_fifo_w)); // Combine handlers for now

	map(0xc0a802, 0xfcffff).r(FUNC(wxstar4k_state::buserr_r)); // Unmapped?

	// --- On-board Peripherals ---
	// FD0000 - EEPROM (mapped via NVRAM device)
	map(0xfd0000, 0xfd1fff).ram().share("eeprom"); // Map the NVRAM share here

	// FDF000 - cause IRQ 6 on graphics card
	// FDF004 - cause IRQ 6 on graphics card 2 (not used)
	map(0xfdf000, 0xfdf007).w(FUNC(wxstar4k_state::cpubd_gfx_irq6_w));
	// FDF008 - reset watchdog
	map(0xfdf008, 0xfdf009).w(FUNC(wxstar4k_state::cpubd_watchdog_reset_w));

	// FDF00A - FDFFBF - Unused?
	map(0xfdf00a, 0xfdffbf).r(FUNC(wxstar4k_state::buserr_r));

	// FDFFC0 - FDFFDF - RTC ICM7170 (20 registers, 1 byte each, appears on D0-D7)
	map(0xfdffc0, 0xfdffdf).rw(m_rtc, FUNC(icm7170_device::read), FUNC(icm7170_device::write)).umask16(0x00ff);

	// FDFFE0 - FDFFFF - Unused?
	map(0xfdffe0, 0xfdffff).r(FUNC(wxstar4k_state::buserr_r));

	// FE0000 - FFFFFF - Boot ROM
	map(0xfe0000, 0xffffff).rom().region("maincpu", 0);
}

// --- Graphics Board 68010 ---

uint16_t wxstar4k_state::vidbd_status_r()
{
	// 200000 read: bit 0=i8051 FIFO full?, bit1=i8051 ready for command? (P1.3)
	uint16_t status = 0;
	// TODO: Implement actual FIFO status flags
	bool fifo_full = false; // Placeholder
	bool cpu_ready = (m_gfx_sub_p1 & 0x08) ? true : false; // Check P1.3

	if (fifo_full) status |= 0x0001;
	if (cpu_ready) status |= 0x0002;

	// Also reads watchdog reset + bit0=Sat video present(P1.4?), bit1=local video present(P1.2?) from 200002? Seems mixed.
	// Let's separate them based on address.
	LOGGFX("%s: Read GFX Status @ 200000 = %04X\n", machine().describe_context(), status);
	return status;
}

void wxstar4k_state::vidbd_vme_addr_hi_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 200000 write: top 7 bits of VME address? For accessing main CPU shared RAM?
	LOGGFX("Write GFX VME Addr Hi @ 200000: data %04X & %04X\n", data, mem_mask);
	// TODO: Store these bits for VME accesses via E00000 range
}

void wxstar4k_state::vidbd_irq_vector_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 200002 write: interrupt vector when causing a main CPU interrupt (IRQ4)
	if (ACCESSING_BITS_0_7)
	{
		m_cpu_irq_vector = data & 0xff;
		LOGIRQ("GFX CPU set Main CPU IRQ4 vector to %02X\n", m_cpu_irq_vector);
	}
}

void wxstar4k_state::vidbd_fifo_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 200004 - write i8051 FIFO
	if (ACCESSING_BITS_0_7) // Assuming byte write
	{
		LOGGFX("Write GFX 8051 FIFO @ 200004: data %02X\n", data & 0xff);
		// TODO: Implement FIFO write to GFX 8051
		m_gfx_sub_fifo_in = data & 0xff; // Very basic placeholder
	}
}

void wxstar4k_state::vidbd_main_cpu_irq4_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 200006 - cause IRQ4 on main CPU
	LOGIRQ("%s: GFX CPU triggering Main CPU IRQ 4\n", machine().describe_context());
	m_maincpu->set_input_line_and_vector(M68K_IRQ_4, ASSERT_LINE, m_cpu_irq_vector); // Use stored vector
}

uint16_t wxstar4k_state::vidbd_gfx_control_r(offs_t offset)
{
	// 300000-300003 - graphics control registers read?
	LOGGFX("%s: Read GFX Control Reg @ %08X\n", machine().describe_context(), 0x300000 + offset*2);
	return 0; // Placeholder
}

void wxstar4k_state::vidbd_gfx_control_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 300000-300003 - graphics control registers write?
	LOGGFX("Write GFX Control Reg @ %08X: data %04X & %04X\n", 0x300000 + offset*2, data, mem_mask);
	// These likely control drawing operations, VRAM addressing modes, etc.
}

void wxstar4k_state::vidbd_main_map(address_map &map)
{
	map(0x000000, 0x00ffff).rom().region("gfxcpu", 0); // 64KB ROM
	map(0x100000, 0x10ffff).ram(); // 64KB RAM

	// --- GFX Board Control / Status ---
	// 200000 - read status, write VME high address bits
	map(0x200000, 0x200001).rw(FUNC(wxstar4k_state::vidbd_status_r), FUNC(wxstar4k_state::vidbd_vme_addr_hi_w));
	// 200002 - read: watchdog reset + video status bits. write: main CPU IRQ vector
	map(0x200002, 0x200003).nopr().w(FUNC(wxstar4k_state::vidbd_irq_vector_w)); // TODO: Implement read side
	// 200004 - write i8051 FIFO
	map(0x200004, 0x200005).w(FUNC(wxstar4k_state::vidbd_fifo_w));
	// 200006 - cause IRQ4 on main CPU
	map(0x200006, 0x200007).w(FUNC(wxstar4k_state::vidbd_main_cpu_irq4_w));

	// --- Graphics Control ---
	// 300000-300003 - graphics control registers
	map(0x300000, 0x300003).rw(FUNC(wxstar4k_state::vidbd_gfx_control_r), FUNC(wxstar4k_state::vidbd_gfx_control_w));

	// --- Framebuffer VRAM ---
	// Note: Should be 8-bit wide access for standard VRAM chips? But 68k is 16-bit bus.
	// MAME handles this via umask or mapping as bytes/words appropriately.
	// Mapping as bytes directly is simpler if access is always byte-wide.
	map(0x400000, 0x5fffff).ram().share("vram"); // 2MB VRAM (mapped as bytes)

	// --- VME Access Window ---
	// E00000-E1FFFF - lower 16 address bits of VME access (to Main CPU shared RAM 0x200000-0x3fffff?)
	map(0xe00000, 0xe1ffff).noprw(); // Placeholder for VME access handler
}

// --- Graphics Board 8051 ---

uint8_t wxstar4k_state::vidbd_sub_vram_counter_low_r(offs_t offset)
{
	// 1000-17FF read VRAM counter low byte
	uint8_t data = m_vram_addr_low & 0xff;
	LOGGFXSUB("%s: Read VRAM Counter Low = %02X\n", machine().describe_context(), data);
	return data;
}

void wxstar4k_state::vidbd_sub_vram_counter_low_w(offs_t offset, uint8_t data)
{
	// 1000-17FF write VRAM counter low byte
	LOGGFXSUB("Write VRAM Counter Low = %02X\n", data);
	m_vram_addr_low = (m_vram_addr_low & 0xff00) | data;
}

uint8_t wxstar4k_state::vidbd_sub_vram_counter_high_r(offs_t offset)
{
	// 1800-1FFF read VRAM counter high byte
	// Assuming this is the *middle* byte of a 24-bit address? VRAM is 2MB (21 bits needed)
	// Or maybe low is 16 bits, high is 8 bits? Let's assume 16+16 for now.
	uint8_t data = (m_vram_addr_low >> 8) & 0xff; // Take high byte of the low word
	LOGGFXSUB("%s: Read VRAM Counter 'High' (Low Word High Byte) = %02X\n", machine().describe_context(), data);
	return data;
}

void wxstar4k_state::vidbd_sub_vram_counter_high_w(offs_t offset, uint8_t data)
{
	// 1800-1FFF write VRAM counter high byte
	LOGGFXSUB("Write VRAM Counter 'High' (Low Word High Byte) = %02X\n", data);
	m_vram_addr_low = (m_vram_addr_low & 0x00ff) | (uint16_t(data) << 8);
	// TODO: Where is the *actual* high part of the address written? Need another register.
	// Maybe accesses > 0xffff automatically use some other register? Or VRAM is banked?
}

uint8_t wxstar4k_state::vidbd_sub_fifo_r()
{
	// 2000-27FF - read FIFO from 68010
	uint8_t data = m_gfx_sub_fifo_in; // Read from placeholder
	LOGGFXSUB("%s: Read FIFO from 68010 = %02X\n", machine().describe_context(), data);
	// TODO: Implement FIFO empty flag (P1.0) logic
	return data;
}

// Note: The RAMDAC Bt471 access needs to be routed through the device interface
// e.g., m_ramdac->write_addr(data), m_ramdac->write_pal(data)

void wxstar4k_state::vidbd_sub_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("gfxsubcpu", 0); // 8KB ROM
}

void wxstar4k_state::vidbd_sub_io_map(address_map &map)
{
	map(0x0000, 0x07ff).ram(); // 2KB RAM (Internal?)
	map(0x0800, 0x0fff).noprw(); // Gap?

	// VRAM Address Counter (Assuming separate low/high word access, needs clarification)
	map(0x1000, 0x17ff).rw(FUNC(wxstar4k_state::vidbd_sub_vram_counter_low_r), FUNC(wxstar4k_state::vidbd_sub_vram_counter_low_w));
	map(0x1800, 0x1fff).rw(FUNC(wxstar4k_state::vidbd_sub_vram_counter_high_r), FUNC(wxstar4k_state::vidbd_sub_vram_counter_high_w)); // TODO: Revisit this mapping

	// FIFO from 68K
	map(0x2000, 0x27ff).r(FUNC(wxstar4k_state::vidbd_sub_fifo_r));
	// 2800-7FFF - undecoded? Mirrored?

	// Bt471 RAMDAC access (Addresses are likely mirrored)
	// Fix: Use public accessors: write_addr, read_pal, write_pal, read_addr, read_mask, write_mask, read_overlay, write_overlay
	map(0x8000, 0x8000).mirror(0x4000).w(m_ramdac, FUNC(bt471_device::write_addr)); // Palette Address Write
	map(0x8800, 0x8800).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read_pal), FUNC(bt471_device::write_pal)); // Palette Data R/W
	map(0x9000, 0x9000).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read_mask), FUNC(bt471_device::write_mask)); // Pixel Mask R/W
	map(0x9800, 0x9800).mirror(0x4000).r(m_ramdac, FUNC(bt471_device::read_addr)); // Palette Address Read
	map(0xa000, 0xa000).mirror(0x4000).nopw(); // Overlay Address Write? (No direct function, maybe implicitly sets address for A800 access?) - NOP for now
	map(0xa800, 0xa800).mirror(0x4000).rw(m_ramdac, FUNC(bt471_device::read_overlay), FUNC(bt471_device::write_overlay)); // Overlay Data R/W
	map(0xb800, 0xb800).mirror(0x4000).nopr(); // Overlay Address Read? (No direct function) - NOP for now

	// Any remaining space up to FFFF?
	map(0xc000, 0xffff).noprw(); // Assume unused for now
}

uint8_t wxstar4k_state::vidbd_sub_p1_r()
{
	// Read internal P1 latch
	// P1.0(T2) = FIFO Empty Flag
	// P1.1(T2EX) = Switch to Sat Video. (Active Low)
	// P1.2 = Switch To Local Video. (Active High)
	// P1.3 = Flag to 68K CPU (Control CPU Ready for Command, Active High)
	// P1.4 = Sat Video Present/Odd-Even Frame Indicator
	// P1.5 = Frame/Sync Control Register
	// P1.6 = Frame/Sync Control Register
	// P1.7 = Frame/Sync Control Register, Watchdog timer.
	// TODO: Update bits based on actual state (FIFO, Video status)
	LOGGFXSUB("Read P1 = %02X\n", m_gfx_sub_p1);
	return m_gfx_sub_p1;
}

void wxstar4k_state::vidbd_sub_p1_w(uint8_t data)
{
	LOGGFXSUB("Write P1 = %02X\n", data);
	m_gfx_sub_p1 = data;
	// Handle outputs:
	// P1.1: Control Genlock/Sat Video switch
	// P1.2: Control Local Video switch
	// P1.3: Update status read by GFX 68k @ 200000
	// P1.7: Possibly pets watchdog?
}

uint8_t wxstar4k_state::vidbd_sub_p3_r()
{
	// Read Port 3 pins
	// P3.0 = RX from FPGA (?) Maybe FIFO Write ack?
	// P3.1 = TX to FPGA (?) Maybe FIFO Read request?
	// P3.2(INT0) = Vertical Drive Interrupt (VSYNC?)
	// P3.3(INT1) = Odd/Even Frame Sync Interrupt (HSYNC?)
	// P3.4(T0) = Input from Frame/Sync Control Register
	// P3.5(T1) = Input from Frame/Sync Control Register
	LOGGFXSUB("Read P3\n");
	// TODO: Return actual pin states, connect INT0/INT1 to VSYNC/HSYNC sources
	return 0xff; // Return all high for now
}

void wxstar4k_state::vidbd_sub_p3_w(uint8_t data)
{
	LOGGFXSUB("Write P3 = %02X\n", data);
	// P3.0/P3.1 might control FIFO handshake if they are TX/RX related
}

// --- Data Board 8344 ---

void wxstar4k_state::databd_main_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("datacpu", 0); // 8KB ROM
}

void wxstar4k_state::databd_main_io_map(address_map &map)
{
	map(0x0000, 0x01ff).ram(); // 512 bytes internal RAM

	// 0200 - UART data (Internal UART of 8344 for SDLC?)
	// 0201 - UART command/status
	map(0x0200, 0x0201).noprw(); // Placeholder

	// Gap?
	map(0x0202, 0x7fff).noprw();

	// 8000-8005 PIO 1 (Looks like 8255 or similar interface logic?)
	// 8000 - PIO1 Command/Status register
	// 8001 - PIO1 Port A - VME address/data AD1-AD8 (Interface to main CPU?)
	// 8002 - PIO1 Port B - rear external switches
	// 8003 - PIO1 Port C
	// 8004 - PIO1 transfer count low
	// 8005 - PIO1 transfer count high
	map(0x8000, 0x8005).noprw(); // Placeholder for PIO1

	// Gap?
	map(0x8006, 0x80ff).noprw();

	// 8100-8105 PIO 2
	// 8100 - PIO2 Command/Status register
	// 8101 - PIO2 Port A - indicator LEDs
	// 8102 - PIO2 Port B - DTMF dialer codes + 1 LED
	// 8103 - PIO2 Port C - bus clear, charging indicator
	// 8104 - PIO2 transfer count low
	// 8105 - PIO2 transfer count high
	map(0x8100, 0x8105).noprw(); // Placeholder for PIO2

	// Gap to FFFF?
	map(0x8106, 0xffff).noprw();
}

// --- I/O Board 8031 ---

void wxstar4k_state::iobd_main_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("iocpu", 0); // 32KB ROM
}

void wxstar4k_state::iobd_main_io_map(address_map &map)
{
	// Needs internal RAM (128 bytes for 8031/8051) - mapped by core

	// Need to map external peripherals: i8251A UART, Keyboard controller registers
	// Addresses unknown, need code analysis. Example:
	// map(0x8000, 0x8000).rw(m_uart, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	// map(0x8001, 0x8001).rw(m_uart, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	// map(0x9000, 0x9000).rw(kbdc...); // Keyboard data
	// map(0x9001, 0x9001).rw(kbdc...); // Keyboard status/command
}

// --- Machine Lifecycle ---

void wxstar4k_state::machine_start()
{
	// Register state for saving
	save_item(NAME(m_cpu_irq_vector));
	save_item(NAME(m_gfx_irq_vector));
	save_item(NAME(m_main_watchdog));

	// Configure NVRAM device
	m_nvram->set_base(memshare("eeprom")->ptr(), 0x2000);
}

void wxstar4k_state::machine_reset()
{
	// Copy reset vector and initial SP from ROM to RAM for main CPU
	// Main RAM is 16-bit wide
	uint16_t *ram = m_mainram.target();
	uint16_t *rom = (uint16_t *)memregion("maincpu")->base();
	ram[0] = rom[0]; // Initial SP word 1
	ram[1] = rom[1]; // Initial SP word 2
	ram[2] = rom[2]; // Initial PC word 1
	ram[3] = rom[3]; // Initial PC word 2

	// Zero out interrupt vectors / flags
	m_cpu_irq_vector = 0; // Default vectors might be set by ROM
	m_gfx_irq_vector = 0;
	m_main_watchdog = 0;

	// Reset other state vars if necessary
	m_vram_addr_low = 0;
	m_vram_addr_high = 0;
	m_gfx_sub_p1 = 0xff; // Default high for inputs
	m_gfx_sub_fifo_in = 0;
	m_gfx_sub_fifo_out = 0;

	// Reset sub-CPUs (they have their own reset lines tied to system reset)
	m_gfxcpu->reset();
	m_gfxsubcpu->reset();
	m_datacpu->reset();
	m_iocpu->reset();
}

// --- Interrupt Handling ---

TIMER_DEVICE_CALLBACK_MEMBER(wxstar4k_state::vblank_irq)
{
	// GFX CPU IRQ 5 = Vertical Blanking Interrupt (Autovectored)
	LOGIRQ("VBLANK: Triggering GFX CPU IRQ 5\n");
	m_gfxcpu->set_input_line(M68K_IRQ_5, ASSERT_LINE);
	// Auto-vector, so CPU fetches vector from 0x74 (Level 5 Auto-vector)
	// Line should be cleared by the interrupt handler? Or automatically? Assume handler clears.
	// For now, clear it after a short delay or let handler do it. Let's clear immediately for now.
	m_gfxcpu->set_input_line(M68K_IRQ_5, CLEAR_LINE);

	// GFX Sub CPU P3.2 (INT0) = Vertical Drive Interrupt?
	LOGIRQ("VBLANK: Triggering GFX Sub CPU INT0 (P3.2)\n");
	m_gfxsubcpu->set_input_line(MCS51_INT0_LINE, ASSERT_LINE); // Needs to be cleared by handler
}

void wxstar4k_state::rtc_irq_w(int state)
{
	// Main CPU IRQ 1 = RTC Interrupt (Autovectored)
	LOGIRQ("RTC IRQ state %d: Triggering Main CPU IRQ 1\n", state);
	// ICM7170 output is level-triggered? Assuming active-high for now.
	m_maincpu->set_input_line(M68K_IRQ_1, state ? ASSERT_LINE : CLEAR_LINE);
	// Auto-vector, so CPU fetches vector from 0x64 (Level 1 Auto-vector)
}

static INPUT_PORTS_START( wxstar4k )
	// Add Dip Switches, Keyboard inputs etc. here if known
INPUT_PORTS_END


// --- Machine Configuration ---

void wxstar4k_state::wxstar4k(machine_config &config)
{
	/* basic machine hardware */
	M68010(config, m_maincpu, XTAL(20'000'000)/2);  // 10 MHz
	m_maincpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::cpubd_main_map);
	// TODO: Setup interrupt acknowledge handlers if needed (for non-autovectored IRQs 2, 4, 5)
	// m_maincpu->set_irq_acknowledge_callback(FUNC(wxstar4k_state::main_irq_ack_callback));

	NVRAM(config, "eeprom", nvram_device::DEFAULT_ALL_0); // 8KB EEPROM on CPU board (U72: X2864A)

	ICM7170(config, m_rtc, XTAL(32'768));
	// Connect RTC IRQ output to Main CPU IRQ1
	// Fix: Use correct callback name
	m_rtc->out_int_handler().set(FUNC(wxstar4k_state::rtc_irq_w));
	// Other ICM7170 pins if needed (e.g., backup battery status)

	/* Graphics board hardware */
	M68010(config, m_gfxcpu, XTAL(20'000'000)/2);   // 10 MHz (from main board clock)
	m_gfxcpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::vidbd_main_map);
	// TODO: Setup interrupt acknowledge handler if needed (for IRQ 6)
	// m_gfxcpu->set_irq_acknowledge_callback(FUNC(wxstar4k_state::gfx_irq_ack_callback));

	I8051(config, m_gfxsubcpu, XTAL(12'000'000));   // 12 MHz
	m_gfxsubcpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::vidbd_sub_map);
	m_gfxsubcpu->set_addrmap(AS_IO, &wxstar4k_state::vidbd_sub_io_map);
	m_gfxsubcpu->port_in_cb<1>().set(FUNC(wxstar4k_state::vidbd_sub_p1_r)); // P1 read
	m_gfxsubcpu->port_out_cb<1>().set(FUNC(wxstar4k_state::vidbd_sub_p1_w)); // P1 write
	m_gfxsubcpu->port_in_cb<3>().set(FUNC(wxstar4k_state::vidbd_sub_p3_r)); // P3 read (for INT0/INT1/T0/T1 inputs)
	m_gfxsubcpu->port_out_cb<3>().set(FUNC(wxstar4k_state::vidbd_sub_p3_w)); // P3 write

	BT471(config, m_ramdac, 0); // Address is placeholder, clock unknown (often 25-33 MHz range)
	// Connect RAMDAC output to palette
	// Fix: Remove set_palette_tag, connection is implicit
	// Need to determine RAMDAC clock - often derived from a pixel clock crystal near it or the main GFX clock

	/* Data/Audio board hardware */
	I8344(config, m_datacpu, XTAL(7'372'800));  // 7.3728 MHz
	m_datacpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::databd_main_map);
	m_datacpu->set_addrmap(AS_IO, &wxstar4k_state::databd_main_io_map);
	// TODO: Add PIO devices if they are standard parts (e.g., 8255)
	// TODO: Connect 8344 interrupt output to Main CPU IRQ5

	// TODO: Add audio speaker device and connect handlers for C0A600/C0A800
	// SPEAKER(config, "mono").front_center();
	// // Add sound device interface (e.g., discrete sound)
	// DISCRETE(config, m_discrete, ...); // Or simpler sound device
	// m_discrete->add_route(ALL_OUTPUTS, "mono", 1.0);

	/* I/O board hardware */
	I8031(config, m_iocpu, XTAL(11'059'200));   // 11.0592 MHz (common for UART baud rates)
	m_iocpu->set_addrmap(AS_PROGRAM, &wxstar4k_state::iobd_main_map);
	m_iocpu->set_addrmap(AS_IO, &wxstar4k_state::iobd_main_io_map);
	// TODO: Connect 8031 interrupt output to Main CPU IRQ2

	// TODO: Add I8251 UART device
	// I8251(config, m_uart, 0); // Clock? Likely derived from 11.0592 / N
	// m_uart->txd_handler().set(...); // Connect to RS232 device
	// m_uart->rts_handler().set(...);
	// ... other UART connections

	// TODO: Add AT Keyboard device
	// AT_KEYBOARD(config, m_keyboard, at_keyboard_device::KEYBOARD_TYPE::AT, 0); // Clock?
	// m_keyboard->keypress_callback().set(...) // Connect to 8031 input pins/IRQ

	/* video hardware */
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	// These parameters need verification from schematics or measurement
	// Using NTSC-like timings as a starting point, 640 width might match comments/code better
	// Fix: Chain screen configuration calls using m_screen finder
	m_screen->set_raw(XTAL(20'000'000) * 2 / 3, 858, 0, 640, 262, 0, 240); // Example pixel clock (~13.33MHz), total/visible scanlines/pixels
	//screen.set_refresh_hz(59.62); // Refresh from notes (set_raw calculates this)
	//screen.set_vblank_time(ATTOSECONDS_IN_USEC(2500)); // Placeholder vblank time (set_raw calculates this)
	m_screen->set_screen_update(FUNC(wxstar4k_state::screen_update));
	m_screen->set_palette(m_palette);

	PALETTE(config, m_palette).set_entries(256);

	// VBLANK Timer triggering GFX CPU IRQ 5 and GFX SUB CPU INT0
	TIMER(config, "vblank").configure_periodic(FUNC(wxstar4k_state::vblank_irq), m_screen->frame_period());

	/* sound hardware */
	// Add speaker and sound generation devices here later
}

ROM_START( wxstar4k )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* CPU board 68010 program (128KB) */
	ROM_LOAD16_BYTE( "u79 rom.bin",  0x000001, 0x010000, CRC(11df2d70) SHA1(ac6cdb5290c90b043562464dc001fc5e3d26f7c6) )
	ROM_LOAD16_BYTE( "u80 rom.bin",  0x000000, 0x010000, CRC(23e15f22) SHA1(a630bda39c0beec7e7fc3834178ec8a6fece70c8) )

	ROM_REGION( 0x2000, "eeprom", ROMREGION_ERASEFF ) /* CPU board EEPROM U72 (8KB), needs NVRAM handler */
	ROM_LOAD( "u72 eeprom.bin", 0x000000, 0x002000, CRC(f775b4d6) SHA1(a0895177c381919f9bfd99ee35edde0dd5fa379c) ) // Will be loaded by NVRAM

	ROM_REGION(0x2000, "datacpu", 0) /* Data board P8344 ROM U12 (8KB) */
	ROM_LOAD( "u12 rom.bin",  0x000000, 0x002000, CRC(f7d8432d) SHA1(0ff1dad65ecb4c3d8cb21feef56bbc6f06a2f712) )

	ROM_REGION(0x8000, "iocpu", 0) /* I/O board 8031 ROM U11 (32KB) */
	ROM_LOAD( "u11 rom.bin",  0x000000, 0x008000, CRC(f12cb28b) SHA1(3368f55717d8e9e7a06a4f241de02b7b2577b32b) )

	ROM_REGION(0x2000, "gfxsubcpu", 0) /* Graphics board 8051 ROM U13 (8KB) */
	ROM_LOAD( "u13 rom.bin",  0x000000, 0x002000, CRC(667b0a2b) SHA1(d60bcc271a73633544b0cf2f80589b2e5670b705) )

	ROM_REGION(0x10000, "gfxcpu", 0) /* Graphics board 68010 program (64KB) */
	ROM_LOAD16_BYTE( "u42 rom low.bin", 0x000001, 0x008000, CRC(84038ca3) SHA1(b28a0d357d489fb06ff0d5d36ea11ebd1f9612a5) ) // D0-D7?
	ROM_LOAD16_BYTE( "u43 rom high.bin", 0x000000, 0x008000, CRC(6f2a7592) SHA1(1aa2394db42b6f28277e35a48a7cef348c213e05) ) // D8-D15?
ROM_END

} // anonymous namespace


//                                                                    YEAR  NAME      PARENT COMPAT MACHINE   INPUT     CLASS           INIT        COMPANY                                            FULLNAME            FLAGS
COMP( 1990, wxstar4k, 0,     0,     wxstar4k, wxstar4k, wxstar4k_state, empty_init, "Applied Microelectronics Institute/The Weather Channel", "WeatherSTAR 4000", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
