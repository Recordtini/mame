// license:BSD-3-Clause
// copyright-holders:Ryan Holtz

#ifndef MAME_PHILIPS_CDI_H
#define MAME_PHILIPS_CDI_H

#include "machine/scc68070.h"
#include "cdislavehle.h"
#include "cdicdic.h"
#include "sound/dmadac.h"
#include "mcd212.h"
#include "cpu/mcs51/i8051.h"
#include "cpu/m6805/m68hc05.h"
#include "diserial.h"
#include "screen.h"

#include <array>
#include <deque>
#include <vector>

struct plm_buffer_t;
struct plm_video_t;
struct plm_audio_t;

/*----------- driver state -----------*/

class cdi_state : public driver_device
{
public:
	cdi_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_main_rom(*this, "maincpu")
		, m_dvc_rom(*this, "mpegs")
		, m_lcd(*this, "lcd")
		, m_slave_hle(*this, "slave_hle")
		, m_plane_ram(*this, "plane%u", 0U)
		, m_servo(*this, "servo")
		, m_slave(*this, "slave")
		, m_cdic(*this, "cdic")
		, m_cdrom(*this, "cdrom")
		, m_mcd212(*this, "mcd212")
		, m_dmadac(*this, "dvc_dac%u", 1U)
	{ }

	void cdimono1_base(machine_config &config);
	void cdimono1(machine_config &config);
	void cdimono2(machine_config &config);
	void cdi910(machine_config &config);

protected:
	struct dvc_video_frame
	{
		uint16_t width = 0;
		uint16_t height = 0;
		std::vector<uint32_t> pixels;
	};

	struct dvc_demux_state
	{
		enum phase_t : uint8_t
		{
			IDLE,
			MAGIC0,
			MAGIC2,
			MAGIC_MATCH,
			PACK0,
			PACK1,
			PACK2,
			PACK3,
			PACK4,
			PACK5,
			PES0,
			PES1,
			PES2,
			PES3,
			PES4,
			PES5,
			PES6,
			PES7,
			PES8,
			PES_DTS0,
			PES_DTS1,
			PES_DTS2,
			PES_DTS3,
			PES_DTS4
		};

		phase_t phase = IDLE;
		bool packet_length_decreasing = false;
		bool packet_body = false;
		bool dts_present = false;
		bool decoding_timestamp_updated = false;
		bool system_clock_reference_start_time_valid = false;
		bool event_program_end = false;
		uint16_t packet_length = 0;
		uint64_t system_clock_reference = 0;
		uint64_t presentation_timestamp = 0;
		uint64_t decoding_timestamp_temp = 0;
		int64_t decoding_timestamp = 0;
		int64_t system_clock_reference_start_time = 0;
	};

	enum servo_portc_bit_t
	{
		INV_JUC_OUT = (1 << 2),
		INV_DIV4_IN = (1 << 5),
		INV_CADDYSWITCH_IN = (1 << 7)
	};

	required_device<scc68070_device> m_maincpu;
	required_region_ptr<uint16_t> m_main_rom;
	optional_region_ptr<uint8_t> m_dvc_rom;
	optional_device<screen_device> m_lcd;
	optional_device<cdislave_hle_device> m_slave_hle;
	required_shared_ptr_array<uint16_t, 2> m_plane_ram;
	optional_device<m68hc05c8_device> m_servo;
	optional_device<m68hc05c8_device> m_slave;
	optional_device<cdicdic_device> m_cdic;
	required_device<cdrom_image_device> m_cdrom;
	required_device<mcd212_device> m_mcd212;

	required_device_array<dmadac_sound_device, 2> m_dmadac;

	uint32_t screen_update_cdimono1_lcd(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void cdimono1_mem(address_map &map) ATTR_COLD;

	void cdi910_mem(address_map &map) ATTR_COLD;
	void cdimono2_mem(address_map &map) ATTR_COLD;
	void cdi070_cpuspace(address_map &map) ATTR_COLD;

	template<int Channel> uint16_t plane_r(offs_t offset, uint16_t mem_mask = ~0);
	template<int Channel> void plane_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	uint16_t main_rom_r(offs_t offset);

	uint16_t dvc_r(offs_t offset, uint16_t mem_mask = ~0);
	void dvc_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t dvc_ram2_r(offs_t offset, uint16_t mem_mask = ~0);
	void dvc_ram2_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	uint16_t bus_error_r(offs_t offset);
	void bus_error_w(offs_t offset, uint16_t data);
	void cdic_dvc_irq_w(int state);

	TIMER_CALLBACK_MEMBER(dvc_timer_tick);
	TIMER_CALLBACK_MEMBER(dvc_video_tick);
	TIMER_CALLBACK_MEMBER(dvc_audio_tick);

	uint8_t cdimono1_iack4_r();
	uint8_t dvc_iack_r();
	void dvc_reset();
	void dvc_reset_video_decoder();
	void dvc_reset_audio_decoder();
	void dvc_reset_demux(dvc_demux_state &state);
	void dvc_restore_state();
	void dvc_update_irq();
	void dvc_update_irq_timer();
	void dvc_update_video_timer();
	void dvc_update_audio_timer();
	void dvc_raise_fmv_irq(uint16_t bits);
	void dvc_raise_fma_irq(uint16_t bits);
	void dvc_handle_fmv_command(uint16_t data);
	void dvc_handle_fmv_video_command(uint16_t data);
	void dvc_handle_fma_command(uint16_t data);
	void dvc_handle_dma_transfer(bool video);
	void dvc_feed_word(bool video, uint16_t data);
	void dvc_feed_bytes(bool video, const uint8_t *data, size_t bytes);
	void dvc_process_demux_byte(bool video, uint8_t data);
	void dvc_decode_video();
	void dvc_decode_audio();
	void dvc_present_next_frame();
	void dvc_rebuild_external_video();

	dvc_demux_state m_dvc_video_demux_state;
	plm_video_t *m_dvc_video_plm = nullptr;
	plm_buffer_t *m_dvc_video_buffer = nullptr;
	dvc_demux_state m_dvc_audio_demux_state;
	plm_audio_t *m_dvc_audio_plm = nullptr;
	plm_buffer_t *m_dvc_audio_buffer = nullptr;

	std::deque<dvc_video_frame> m_dvc_video_queue;
	dvc_video_frame m_dvc_display_frame;
	std::deque<int16_t> m_dvc_audio_pcm[2];

	emu_timer *m_dvc_timer = nullptr;
	emu_timer *m_dvc_video_timer = nullptr;
	emu_timer *m_dvc_audio_timer = nullptr;

	uint16_t m_dvc_fma_command = 0;
	uint8_t m_dvc_fma_status = 0;
	uint16_t m_dvc_fma_interrupt_status = 0;
	uint16_t m_dvc_fma_interrupt_enable = 0;
	uint16_t m_dvc_fma_interrupt_vector = 0;
	uint16_t m_dvc_fma_stream = 0;
	uint16_t m_dvc_fma_dsp_addr = 0;
	uint16_t m_dvc_fma_dsp_enable = 0;
	uint32_t m_dvc_fma_dclk = 0;

	uint16_t m_dvc_fmv_interrupt_status = 0;
	uint16_t m_dvc_fmv_interrupt_enable = 0;
	uint16_t m_dvc_fmv_interrupt_vector = 0;
	uint16_t m_dvc_fmv_system_command = 0;
	uint16_t m_dvc_fmv_video_command = 0;
	uint16_t m_dvc_fmv_system_control = 0;
	uint16_t m_dvc_fmv_timer_compare = 56 - 1;
	uint16_t m_dvc_fmv_frame_rate = 0;
	uint16_t m_dvc_fmv_decoder_command = 0;
	uint16_t m_dvc_fmv_video_data_input_command = 0;
	uint16_t m_dvc_fmv_stream = 0;
	uint16_t m_dvc_fmv_y_offset = 0;
	uint16_t m_dvc_fmv_x_offset = 0;
	uint16_t m_dvc_fmv_y_active = 0;
	uint16_t m_dvc_fmv_x_active = 0;
	uint16_t m_dvc_fmv_y_display = 0;
	uint16_t m_dvc_fmv_x_display = 0;
	uint16_t m_dvc_fmv_window_height = 0;
	uint16_t m_dvc_fmv_window_width = 0;
	uint16_t m_dvc_fmv_decoder_offset_y = 0;
	uint16_t m_dvc_fmv_decoder_offset_x = 0;
	uint16_t m_dvc_fmv_program = 0;
	uint16_t m_dvc_fmv_demux_timestamp = 0;
	uint16_t m_dvc_fmv_last_decoded_timestamp = 0;
	uint16_t m_dvc_image_width = 0;
	uint16_t m_dvc_image_height = 0;
	uint16_t m_dvc_image_rt = 0;
	std::array<uint16_t, 0x1c00> m_dvc_fmv_program_ram{};
	std::array<uint16_t, 0x40000> m_dvc_ram2{};

	bool m_dvc_decoder_enabled = false;
	bool m_dvc_playback_active = false;
	bool m_dvc_video_visible = false;
	bool m_dvc_video_show_pending = false;
	bool m_dvc_fma_started = false;
	bool m_dvc_fma_pending_stream_change = false;
	bool m_dvc_audio_output_active = false;
	bool m_dvc_audio_output_started_once = false;
	bool m_dvc_fmv_register_update_latch = false;
	bool m_dvc_fmv_register_update_scroll = false;
	bool m_dvc_mpeg_ram_enabled = false;
	bool m_cdic_irq_pending = false;
	int16_t m_dvc_audio_output_level[2] = { 0, 0 };
	uint8_t m_dvc_mpeg_ram_enable_count = 0;
	uint8_t m_dvc_dma_preview_count = 0;
	uint8_t m_dvc_audio_empty_ticks = 0;
	uint32_t m_dvc_video_present_accum = 0;
	uint32_t m_dvc_audio_sample_rate = 0;
	uint32_t m_dvc_audio_dma_last_mac = 0;
	uint32_t m_dvc_audio_dma_span_hint = 0;
	double m_dvc_frame_rate_hz = 25.0;
	u64 m_dvc_dclk_base = 0;
};

class quizard_state : public cdi_state, public device_serial_interface
{
public:
	quizard_state(const machine_config &mconfig, device_type type, const char *tag)
		: cdi_state(mconfig, type, tag)
		, device_serial_interface(mconfig, *this)
		, m_mcu(*this, "mcu")
		, m_inputs(*this, "P%u", 0U)
	{ }

	void quizard(machine_config &config);

private:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void rcv_complete() override;

	TIMER_CALLBACK_MEMBER(boot_press_tick);

	uint8_t mcu_p0_r();
	uint8_t mcu_p1_r();
	uint8_t mcu_p2_r();
	uint8_t mcu_p3_r();
	void mcu_p0_w(uint8_t data);
	void mcu_p1_w(uint8_t data);
	void mcu_p2_w(uint8_t data);
	void mcu_p3_w(uint8_t data);

	void mcu_rx_from_cpu(uint8_t data);
	void mcu_rtsn_from_cpu(int state);

	uint8_t mcu_button_press();

	required_device<i8751_device> m_mcu;
	required_ioport_array<3> m_inputs;

	bool m_boot_press = false;
	emu_timer *m_boot_timer = nullptr;
	uint8_t m_mcu_p3;
};

// Quizard 2 language values:
// 0x2b1: Italian
// 0x001: French
// 0x188: German

#endif // MAME_PHILIPS_CDI_H
