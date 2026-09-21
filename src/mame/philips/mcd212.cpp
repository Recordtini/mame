// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/******************************************************************************


    CD-i MCD212 Video Decoder and System Controller emulation
    -------------------

    written by Ryan Holtz, Vincent.Halver


*******************************************************************************

STATUS:

- Just enough for the Mono-I CD-i board to work somewhat properly.

TODO:

- QHY DYUV Image Decoder

*******************************************************************************/

#include "emu.h"
#include "mcd212.h"
#include "screen.h"

#define LOG_UNKNOWNS        (1U << 1)
#define LOG_REGISTERS       (1U << 2)
#define LOG_ICA             (1U << 3)
#define LOG_DCA             (1U << 4)
#define LOG_VSR             (1U << 5)
#define LOG_STATUS          (1U << 6)
#define LOG_MAIN_REG_READS  (1U << 7)
#define LOG_MAIN_REG_WRITES (1U << 8)
#define LOG_CLUT            (1U << 9)
#define LOG_ALL             (LOG_UNKNOWNS | LOG_REGISTERS | LOG_ICA | LOG_DCA | LOG_VSR | LOG_STATUS | LOG_MAIN_REG_READS | LOG_MAIN_REG_WRITES | LOG_CLUT)

#define VERBOSE             (0)
#include "logmacro.h"

// device type definition
DEFINE_DEVICE_TYPE(MCD212, mcd212_device, "mcd212", "MCD212 VDSC")

namespace
{
	constexpr int32_t MCD212_VIDEO_BLACK_LEVEL = 0x10;

	inline int32_t mcd212_weight_calc(const int32_t rgb, const uint8_t weight, const bool mister_weight_math)
	{
		if (mister_weight_math)
		{
			// MiSTer weights from black at zero with a +1 weight bias.  Keep it
			// as a debug option so we can compare against the Green Book formula
			// without changing the default hardware-documentation path.
			return weight ? std::clamp((rgb * (int32_t(weight) + 1)) >> 6, 0, 255) : 0;
		}

		// MCD212 section 8.5 specifies black-preserving contribution as
		// (component - 16) * weight / 64 + 16.
		return std::clamp(MCD212_VIDEO_BLACK_LEVEL
			+ (((rgb - MCD212_VIDEO_BLACK_LEVEL) * int32_t(weight)) >> 6), 0, 255);
	}

	inline uint8_t mcd212_mix_weighted(const int32_t weighted_a, const int32_t weighted_b)
	{
		return std::clamp((weighted_a - MCD212_VIDEO_BLACK_LEVEL) + (weighted_b - MCD212_VIDEO_BLACK_LEVEL) + MCD212_VIDEO_BLACK_LEVEL, 0, 255);
	}

	inline bool mcd212_fetch_ev_pixel(const bitmap_rgb32 &bitmap, const bool enabled, const bool ignore_alpha, const int x, const int y, uint32_t &pixel)
	{
		if (enabled && y >= 0 && y < bitmap.height() && x >= 0 && x < bitmap.width())
		{
			const uint32_t ev_pix = bitmap.pix(y, x);
			if ((ev_pix >> 24) || (ignore_alpha && (ev_pix & 0x00ffffff)))
			{
				pixel = 0xff000000 | (ev_pix & 0x00ffffff);
				return true;
			}
		}
		return false;
	}

	inline bool mcd212_eval_transparency(const uint8_t tp_ctrl, const bool color_match, const bool color_key_available, const bool transparency_bit, const bool matte_flag0, const bool matte_flag1)
	{
		// MCD212 section 5.4.4.2 defines these as complete four-bit
		// functions.  Evaluate that table directly; deriving the result from
		// the low bits adds undocumented DYUV/color-key exceptions and breaks
		// the region-flag transparency used by CD-RTOS full-motion video.
		switch (tp_ctrl)
		{
		case 0x0: return true;
		case 0x1: return color_key_available && color_match;
		case 0x2: return transparency_bit;
		case 0x3: return matte_flag0;
		case 0x4: return matte_flag1;
		case 0x5: return matte_flag0 || (color_key_available && color_match);
		case 0x6: return matte_flag1 || (color_key_available && color_match);
		case 0x8: return false;
		case 0x9: return color_key_available && !color_match;
		case 0xa: return !transparency_bit;
		case 0xb: return !matte_flag0;
		case 0xc: return !matte_flag1;
		case 0xd: return !matte_flag0 || (color_key_available && !color_match);
		case 0xe: return !matte_flag1 || (color_key_available && !color_match);
		default: return false;
		}
	}

}

inline ATTR_FORCE_INLINE uint8_t mcd212_device::get_weight_factor(const uint32_t matte_idx)
{
	return (uint8_t)((m_matte_control[matte_idx] & MC_WF) >> MC_WF_SHIFT);
}

void mcd212_device::set_external_video_enable(bool enable)
{
	if (m_external_video_pending_enabled != enable)
	{
		m_external_video_pending_enabled = enable;
		m_external_video_dirty = true;
		log_external_video_state(enable ? "pending enable on" : "pending enable off", true);
	}
}

void mcd212_device::set_external_video_select(bool select)
{
	if (m_external_video_select_pending != select)
	{
		m_external_video_select_pending = select;
		m_external_video_dirty = true;
		log_external_video_state(select ? "pending UCM EV select" : "pending UCM EV deselect", true);
	}
}

void mcd212_device::set_external_video_mode(uint8_t mode)
{
	if (mode != EXTERNAL_VIDEO_FORCE_TOP && mode != EXTERNAL_VIDEO_BETWEEN_PLANES)
		mode = EXTERNAL_VIDEO_BACKDROP;
	if (m_external_video_mode != mode)
	{
		m_external_video_mode = mode;
		if (mode == EXTERNAL_VIDEO_BETWEEN_PLANES)
			reset_external_video_overlay_baseline();
		else
		{
			m_external_video_overlay_capture = false;
			m_external_video_overlay_baseline_valid = false;
			m_external_video_overlay_blockers = 0;
		}
		m_last_rendered_scanline = m_ica_height - 1;
		const char *const reason = (mode == EXTERNAL_VIDEO_FORCE_TOP) ? "ev mode force top"
			: (mode == EXTERNAL_VIDEO_BETWEEN_PLANES) ? "ev mode overlay preserve"
			: "ev mode backdrop";
		log_external_video_state(reason, true);
	}
}

void mcd212_device::reset_external_video_overlay_baseline()
{
	m_external_video_overlay_capture = true;
	m_external_video_overlay_baseline_valid = false;
	m_external_video_overlay_blockers = 0;
	std::fill_n(&m_external_video_overlay_baseline_dyuv[0][0][0], 2 * 312 * 768, false);
	std::fill_n(&m_external_video_overlay_dense_history[0][0], 2 * 312, 0);
	std::fill_n(&m_external_video_overlay_dense_current[0][0], 2 * 312, 0);
	m_last_rendered_scanline = m_ica_height - 1;
}

void mcd212_device::set_external_video_page(uint8_t page)
{
	if (m_external_video_page != page)
	{
		m_external_video_page = page;
		log_external_video_state("ev show page", true);
	}
}

void mcd212_device::set_debug_layer_mask(uint8_t mask)
{
	if (m_debug_layer_mask != mask)
	{
		m_debug_layer_mask = mask;
		m_last_rendered_scanline = m_ica_height - 1;
		log_external_video_state("debug layer mask", true);
	}
}

void mcd212_device::set_debug_video_mask(uint32_t mask)
{
	if (m_debug_video_mask != mask)
	{
		m_debug_video_mask = mask;
		m_last_rendered_scanline = m_ica_height - 1;
		log_external_video_state("debug video mask", true);
	}
}

void mcd212_device::log_external_video_state(const char *reason, bool force)
{
	uint32_t sample_nonblack = 0;
	if (m_external_video_active.width() > 0 && m_external_video_active.height() > 0)
	{
		const size_t total_pixels = size_t(m_external_video_active.width()) * size_t(m_external_video_active.height());
		const size_t step = std::max<size_t>(size_t(1), total_pixels / 256);
		size_t index = 0;
		for (int y = 0; y < m_external_video_active.height(); y++)
		{
			for (int x = 0; x < m_external_video_active.width(); x++, index++)
			{
				if ((index % step) != 0)
					continue;
				const uint32_t pixel = m_external_video_active.pix(y, x);
				if (pixel & 0x00ffffff)
					sample_nonblack++;
			}
		}
	}

	const uint32_t signature =
		(m_external_video_pending_enabled ? 0x00000001 : 0x00000000) |
		(m_external_video_active_enabled  ? 0x00000002 : 0x00000000) |
		(m_external_video_dirty           ? 0x00000004 : 0x00000000) |
		(external_video_icm_enabled() ? 0x00000008 : 0x00000000) |
		(uint32_t(m_external_video_mode) << 4) |
		(m_external_video_select_pending ? 0x00000040 : 0x00000000) |
		(m_external_video_select_active  ? 0x00000080 : 0x00000000) |
		(uint32_t(m_external_video_page) << 24) |
		(uint32_t(m_debug_layer_mask) << 8) |
		((uint32_t(m_debug_video_mask & 0xffff) << 16) ^ uint32_t(m_debug_video_mask >> 16));

	if (!force && (signature == m_last_ev_log_signature) && (sample_nonblack == m_last_ev_log_nonblack))
		return;

	m_last_ev_log_signature = signature;
	m_last_ev_log_nonblack = sample_nonblack;

	logerror("MCD212 EV state [%s] pending=%d active=%d dirty=%d icm_ev=%d ucm_select=%d/%d mode=%u page=%u dbg_layers=%02x dbg_video=%05x sample_nonblack=%u dcr=%04x/%04x vsr=%04x/%04x dca=%06x/%06x\n",
		reason,
		m_external_video_pending_enabled ? 1 : 0,
		m_external_video_active_enabled ? 1 : 0,
		m_external_video_dirty ? 1 : 0,
		external_video_icm_enabled() ? 1 : 0,
		m_external_video_select_pending ? 1 : 0,
		m_external_video_select_active ? 1 : 0,
		m_external_video_mode,
		m_external_video_page,
		m_debug_layer_mask,
		m_debug_video_mask,
		sample_nonblack,
		m_dcr[0],
		m_dcr[1],
		m_vsr[0],
		m_vsr[1],
		m_dca[0],
		m_dca[1]);
}

inline ATTR_FORCE_INLINE uint8_t mcd212_device::get_matte_op(const uint32_t matte_idx)
{
	return (m_matte_control[matte_idx] & MC_OP) >> MC_OP_SHIFT;
}

void mcd212_device::update_matte_arrays()
{
	const int width = get_screen_width();
	// The "matte" controls are really the MCD212 region-control entries.
	// MiSTer models them as live region-flag/weight transitions across the
	// line, with either implicit (0123->RF0, 4567->RF1) or explicit control.
	// Burger/Repeat rely on that behavior; the older "two matte streams"
	// shortcut leaves Plane A opaque where TA=RF0 should make it yield.
	const bool implicit_regions = BIT(m_image_coding_method, ICM_NM_BIT);
	const bool region_next_pixel = (m_debug_video_mask & DEBUG_VIDEO_REGION_NEXT_PIXEL) != 0;
	bool region_flags[2]{ false, false };
	// Region commands modify the live contribution-factor registers.  They are
	// not reset at the next line; only the region flags and sequencer indices
	// are.  This matters for effects whose first weight transition is not at X=0.
	uint8_t latched_wf[2] = { m_current_weight_factor[0], m_current_weight_factor[1] };
	int rf0_index = 0;
	int rf1_index = 0;

	auto apply_region_entry = [&](int entry_index, int default_flag)
	{
		const uint32_t ctrl = m_matte_control[entry_index];
		const uint8_t op = get_matte_op(entry_index);

		if (op == 0)
			return false;

		const int flag = implicit_regions ? default_flag : BIT(ctrl, MC_MF_BIT);
		switch (op)
		{
		case 0x4: // Change Weight Plane A
			latched_wf[0] = get_weight_factor(entry_index);
			break;
		case 0x6: // Change Weight Plane B
			latched_wf[1] = get_weight_factor(entry_index);
			break;
		case 0x8: // Reset Region Flag
			region_flags[flag] = false;
			break;
		case 0x9: // Set Region Flag
			region_flags[flag] = true;
			break;
		case 0xc: // Reset Region Flag and Change Weight Plane A
			region_flags[flag] = false;
			latched_wf[0] = get_weight_factor(entry_index);
			break;
		case 0xd: // Set Region Flag and Change Weight Plane A
			region_flags[flag] = true;
			latched_wf[0] = get_weight_factor(entry_index);
			break;
		case 0xe: // Reset Region Flag and Change Weight Plane B
			region_flags[flag] = false;
			latched_wf[1] = get_weight_factor(entry_index);
			break;
		case 0xf: // Set Region Flag and Change Weight Plane B
			region_flags[flag] = true;
			latched_wf[1] = get_weight_factor(entry_index);
			break;
		default:
			// Reserved operations have no effect, but the sequencer still
			// advances to the following region-control register.
			break;
		}

		return true;
	};

	for (int x = 0; x < width; x++)
	{
		// MiSTer updates region state in the same sequential path that advances
		// active_pixel; this debug mode lets us test whether MAME is applying
		// matte/weight changes one rendered pixel too early.
		if (region_next_pixel)
		{
			m_weight_factor[0][x] = latched_wf[0];
			m_weight_factor[1][x] = latched_wf[1];
			m_matte_flag[0][x] = region_flags[0];
			m_matte_flag[1][x] = region_flags[1];
		}

		if (implicit_regions)
		{
			if (rf0_index < 4)
			{
				const int entry = rf0_index;
				if (x == int(m_matte_control[entry] & MC_X))
				{
					if (apply_region_entry(entry, 0))
						rf0_index++;
				}
			}
			if (rf1_index < 4)
			{
				const int entry = 4 + rf1_index;
				if (x == int(m_matte_control[entry] & MC_X))
				{
					if (apply_region_entry(entry, 1))
						rf1_index++;
				}
			}
		}
		else
		{
			if (rf0_index < 8)
			{
				const int entry = rf0_index;
				if (x == int(m_matte_control[entry] & MC_X))
				{
					if (apply_region_entry(entry, 0))
						rf0_index++;
				}
			}
		}

		if (!region_next_pixel)
		{
			m_weight_factor[0][x] = latched_wf[0];
			m_weight_factor[1][x] = latched_wf[1];
			m_matte_flag[0][x] = region_flags[0];
			m_matte_flag[1][x] = region_flags[1];
		}
	}

	m_current_weight_factor[0] = latched_wf[0];
	m_current_weight_factor[1] = latched_wf[1];
}

template <int Path>
void mcd212_device::set_register(uint8_t reg, uint32_t value)
{
	const int32_t scanline = (m_current_render_scanline >= 0)
		? m_current_render_scanline
		: physical_to_logical_scanline(screen().vpos());

	switch (reg)
	{
		case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87: // CLUT 0 - 63
		case 0x88: case 0x89: case 0x8a: case 0x8b: case 0x8c: case 0x8d: case 0x8e: case 0x8f:
		case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97:
		case 0x98: case 0x99: case 0x9a: case 0x9b: case 0x9c: case 0x9d: case 0x9e: case 0x9f:
		case 0xa0: case 0xa1: case 0xa2: case 0xa3: case 0xa4: case 0xa5: case 0xa6: case 0xa7:
		case 0xa8: case 0xa9: case 0xaa: case 0xab: case 0xac: case 0xad: case 0xae: case 0xaf:
		case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6: case 0xb7:
		case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe: case 0xbf:
			{
				const uint8_t clut_index = m_clut_bank[Path] * 0x40 + (reg - 0x80);
				LOGMASKED(LOG_CLUT, "%s: Path %d: CLUT[%d] = %08x\n", machine().describe_context(), Path, clut_index, value);
				m_clut[clut_index] = value & 0x00fcfcfc;
			}
			break;
		case 0xc0: // Image Coding Method
			// MiSTer and the MCD212 register tables both model the shared display
			// control registers as CH1-owned. Letting Path 1 overwrite them makes
			// Repeat/Burger style popup scripts clobber the global EV state.
			if (Path != 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path %d attempted shared Image Coding Method write %08x (ignored)\n",
					machine().describe_context(), scanline, Path, value);
				break;
			}
			m_image_coding_method_programmed = value;
			if (m_image_coding_method != value)
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Image Coding Method = %08x (programmed=%08x ucm_ev=%d)\n",
					machine().describe_context(), scanline, value, m_image_coding_method_programmed,
					m_external_video_select_active ? 1 : 0);
			m_image_coding_method = value;
			break;
		case 0xc1: // Transparency Control
			if (Path != 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path %d attempted shared Transparency Control write %08x (ignored)\n",
					machine().describe_context(), scanline, Path, value);
				break;
			}
			if (m_transparency_control != value)
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Transparency Control = %08x\n",
					machine().describe_context(), scanline, value);
			m_transparency_control = value;
			break;
		case 0xc2: // Plane Order
			if (Path != 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path %d attempted shared Plane Order write %08x (ignored)\n",
					machine().describe_context(), scanline, Path, value & 7);
				break;
			}
			if (m_plane_order != (value & 0x00000007))
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Plane Order = %08x\n",
					machine().describe_context(), scanline, value & 7);
			m_plane_order = value & 0x00000007;
			break;
		case 0xc3: // CLUT Bank Register
			LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path %d: CLUT Bank Register = %08x\n", machine().describe_context(), screen().vpos(), Path, value & 3);
			m_clut_bank[Path] = Path ? (2 | (value & 0x00000001)) : (value & 0x00000003);
			break;
		case 0xc4: // Transparent Color A
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Transparent Color A = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_transparent_color[0] = value & 0x00fcfcfc;
			}
			break;
		case 0xc6: // Transparent Color B
			if (Path == 1)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1: Transparent Color B = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_transparent_color[1] = value & 0x00fcfcfc;
			}
			break;
		case 0xc7: // Mask Color A
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Mask Color A = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_mask_color[0] = value & 0x00fcfcfc;
			}
			break;
		case 0xc9: // Mask Color B
			if (Path == 1)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1: Mask Color B = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_mask_color[1] = value & 0x00fcfcfc;
			}
			break;
		case 0xca: // Delta YUV Absolute Start Value A
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Delta YUV Absolute Start Value A = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_dyuv_abs_start[0] = value;
			}
			break;
		case 0xcb: // Delta YUV Absolute Start Value B
			if (Path == 1)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1: Delta YUV Absolute Start Value B = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_dyuv_abs_start[1] = value;
			}
			break;
		case 0xcd: // Cursor Position
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Cursor Position = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_cursor_position = value;
			}
			break;
		case 0xce: // Cursor Control
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Cursor Control = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_cursor_control = value;
			}
			break;
		case 0xcf: // Cursor Pattern
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Cursor Pattern[%d] = %04x\n", machine().describe_context(), screen().vpos(), (value >> 16) & 0x000f, value & 0x0000ffff);
				m_cursor_pattern[(value >> 16) & 0x000f] = value & 0x0000ffff;
			}
			break;
		case 0xd0: // matte Control 0-7
		case 0xd1:
		case 0xd2:
		case 0xd3:
		case 0xd4:
		case 0xd5:
		case 0xd6:
		case 0xd7:
			{
				const uint8_t index = reg & 7;
				// MCD212 5.4.4.12 specifies CH#1 priority for simultaneous
				// region-control loads; CH#2's write is ignored in that case.
				if (Path == 1 && m_matte_control_scanline[index] == scanline && m_matte_control_path[index] == 0)
				{
					LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1 region Control %d write %08x lost to Path 0 priority\n",
						machine().describe_context(), scanline, index, value);
					break;
				}

				if ((m_debug_video_mask & DEBUG_VIDEO_LOG_PLANE_STATS) != 0)
				{
					logerror("MCD212 region write y=%d path=%d idx=%d value=%06x op=%x mf=%d wf=%02x x=%03x icm=%06x tcr=%06x order=%u prev=%06x prev_path=%d prev_y=%d\n",
						scanline, Path, index, value & 0x00ffffff, (value >> 20) & 0x0f, BIT(value, 16),
						(value >> 10) & 0x3f, value & 0x03ff, m_image_coding_method & 0x00ffffff,
						m_transparency_control & 0x00ffffff, m_plane_order & 0x07,
						m_matte_control[index] & 0x00ffffff,
						m_matte_control_path[index], m_matte_control_scanline[index]);
				}

				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path %d: matte Control %d = %08x\n",
					machine().describe_context(), scanline, Path, index, value);
				m_matte_control[index] = value;
				m_matte_control_scanline[index] = scanline;
				m_matte_control_path[index] = Path;
			}
			break;
		case 0xd8: // Backdrop Color
			if (Path != 0)
			{
				logerror("%s: Scanline %d, Path %d attempted shared Backdrop Color write %08x (ignored)\n",
					machine().describe_context(), scanline, Path, value);
				break;
			}
			logerror("%s: Scanline %d, Path 0: Backdrop Color = %08x\n",
				machine().describe_context(), scanline, value);
			m_backdrop_color = value;
			break;
		case 0xd9: // Mosaic Pixel Hold Factor A
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Mosaic Pixel Hold Factor A = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_mosaic_hold[0] = value;
			}
			break;
		case 0xda: // Mosaic Pixel Hold Factor B
			if (Path == 1)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1: Mosaic Pixel Hold Factor B = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_mosaic_hold[1] = value;
			}
			break;
		case 0xdb: // Weight Factor A
			if (Path == 0)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 0: Weight Factor A = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_base_weight_factor[0] = uint8_t(value & 0x3f);
				m_current_weight_factor[0] = m_base_weight_factor[0];
			}
			break;
		case 0xdc: // Weight Factor B
			if (Path == 1)
			{
				LOGMASKED(LOG_REGISTERS, "%s: Scanline %d, Path 1: Weight Factor B = %08x\n", machine().describe_context(), screen().vpos(), value);
				m_base_weight_factor[1] = uint8_t(value & 0x3f);
				m_current_weight_factor[1] = m_base_weight_factor[1];
			}
			break;
	}
}

template <int Path>
inline ATTR_FORCE_INLINE uint32_t mcd212_device::get_vsr()
{
	return ((m_dcr[Path] & 0x3f) << 16) | m_vsr[Path];
}

template <int Path>
inline ATTR_FORCE_INLINE void mcd212_device::set_vsr(uint32_t value)
{
	m_vsr[Path] = value & 0x0000ffff;
	m_dcr[Path] &= 0xffc0;
	m_dcr[Path] |= (value >> 16) & 0x003f;
}

template <int Path>
inline ATTR_FORCE_INLINE void mcd212_device::set_dcp(uint32_t value)
{
	m_dcp[Path] = value & 0x0000ffff;
	m_ddr[Path] &= 0xffc0;
	m_ddr[Path] |= (value >> 16) & 0x003f;
}

template <int Path>
inline ATTR_FORCE_INLINE uint32_t mcd212_device::get_dcp()
{
	return ((m_ddr[Path] & 0x3f) << 16) | m_dcp[Path];
}

uint16_t mcd212_device::read_dram_word(uint32_t address) const
{
	address &= 0x003fffff;
	const unsigned bank = BIT(address, 18);
	const uint32_t bank_offset = ((address & 0x0003ffff) | (BIT(address, 21) ? 0x00040000 : 0)) >> 1;
	return bank ? m_planeb[bank_offset] : m_planea[bank_offset];
}

uint8_t mcd212_device::read_dram_byte(uint32_t address) const
{
	const uint16_t word = read_dram_word(address & ~1U);
	return BIT(address, 0) ? uint8_t(word) : uint8_t(word >> 8);
}

template <int Path>
inline ATTR_FORCE_INLINE void mcd212_device::set_display_parameters(uint8_t value)
{
	m_ddr[Path] &= 0xf0ff;
	m_ddr[Path] |= (value & 0x0f) << 8;
	m_dcr[Path] &= 0xf7ff;
	m_dcr[Path] |= (value & 0x10) << 7;
}

int mcd212_device::get_screen_width()
{
	int width = 768;
	if (!BIT(m_dcr[0], DCR_CF_BIT) || BIT(m_csrw[0], CSR1W_ST_BIT))
		width = 720;
	return width;
}

int mcd212_device::get_border_width()
{
	int width = 0;
	if (!BIT(m_dcr[0], DCR_CF_BIT) || BIT(m_csrw[0], CSR1W_ST_BIT))
		width = 24;
	return width;
}

int mcd212_device::get_dca_trigger_x()
{
	// The MCD212 docs describe DCA operating in the horizontal blanking area.
	// For the 360/720-pixel mode, the right-side masked border is already part
	// of blanking, so start DCA at the end of the active picture rather than
	// after the border. Repeat Offender relies on these per-line display
	// updates landing before the next visible line is mixed.
	return get_screen_width();
}

bool mcd212_device::active_lines_are_doubled() const
{
	// Mono-I keeps the 32 ICA lines at native timing and doubles only the
	// 280 display lines into the 560-line visible bitmap.
	return screen().height() == (m_ica_height + ((m_total_height - m_ica_height) * 2));
}

int mcd212_device::physical_to_logical_scanline(int physical_scanline) const
{
	if (!active_lines_are_doubled() || physical_scanline < m_ica_height)
		return physical_scanline;

	return m_ica_height + ((physical_scanline - m_ica_height) >> 1);
}

int mcd212_device::logical_to_physical_scanline(int logical_scanline) const
{
	if (!active_lines_are_doubled() || logical_scanline < m_ica_height)
		return logical_scanline;

	return m_ica_height + ((logical_scanline - m_ica_height) << 1);
}

int mcd212_device::dca_trigger_scanline(int logical_scanline) const
{
	// DCA is fetched in horizontal blank immediately before the display line
	// it controls. A doubled line pair must consume only one DCA row.
	return logical_to_physical_scanline(logical_scanline) - 1;
}

uint32_t mcd212_device::get_backdrop_plane(int x, int y)
{
	if (!(m_debug_layer_mask & DEBUG_LAYER_BACKDROP))
		return 0xff000000;

	const uint32_t backdrop_color = s_4bpp_color[m_backdrop_color];
	const bool ignore_ev_alpha = (m_debug_video_mask & DEBUG_VIDEO_IGNORE_ALPHA) != 0;
	// The full-motion plane replaces the backdrop while either the title's DCP
	// selects ICM.EV or CD-RTOS has latched an active MV_Show map.
	const bool ev_selected = external_video_icm_enabled();

	if (ev_selected)
	{
		uint32_t ev_pix = 0;
		if (mcd212_fetch_ev_pixel(m_external_video_active, m_external_video_active_enabled, ignore_ev_alpha, x, y, ev_pix))
			return ev_pix;
		// External video replaces the normal backdrop. If the current EV pixel
		// hasn't been written, fall back to the programmed backdrop color rather
		// than forcing black, which can otherwise mask the intended overlay state.
		return backdrop_color;
	}
	else
		return backdrop_color;
}

template <int Path>
void mcd212_device::process_ica()
{
	const int max_to_process = m_ica_height * 120;
	// Table 5-8 assigns h400 to non-interlace and odd interlace fields, and
	// h404 only to even interlace fields. Channel 2 starts at h200400. Every
	// linked pointer remains an absolute 22-bit DRAM address; the address
	// decoder, rather than the ICA channel, selects the physical bank.
	const bool interlaced = BIT(m_dcr[0], DCR_SM_BIT);
	const uint32_t ica_base = Path ? 0x00200000 : 0;
	uint32_t addr = ica_base | ((!interlaced || BIT(m_csrr[0], CSR1R_PA_BIT)) ? 0x400 : 0x404);

	if (m_external_video_select_pending || m_external_video_select_active)
	{
		const uint32_t single_odd_head = (uint32_t(read_dram_word(ica_base | 0x400)) << 16) | read_dram_word(ica_base | 0x402);
		const uint32_t even_head = (uint32_t(read_dram_word(ica_base | 0x404)) << 16) | read_dram_word(ica_base | 0x406);
		const uint64_t signature = (uint64_t(single_odd_head) << 32) | even_head;
		if (m_last_ica_head_signature[Path] != signature)
		{
			m_last_ica_head_signature[Path] = signature;
			logerror("MCD212 ICA map path=%d sm=%d pa=%d selected=%04x head400=%08x head404=%08x dcp=%06x vsr=%06x\n",
				Path, interlaced ? 1 : 0, BIT(m_csrr[0], CSR1R_PA_BIT) ? 1 : 0,
				unsigned(addr), single_odd_head, even_head, get_dcp<Path>(), get_vsr<Path>());
		}
	}

	for (int i = 0; i < max_to_process; i++)
	{
		const uint32_t command_addr = addr;
		uint32_t cmd = uint32_t(read_dram_word(addr)) << 16;
		cmd |= read_dram_word(addr + 2);
		addr = (addr + 4) & 0x003fffff;
		switch ((cmd & 0xff000000) >> 24)
		{
			case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: // STOP
			case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: STOP\n", command_addr, cmd, Path);
				return;
			case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: // NOP
			case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: NOP\n", command_addr, cmd, Path);
				break;
			case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: // RELOAD DCP
			case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: RELOAD DCP: %06x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				set_dcp<Path>(cmd & 0x003ffffc);
				break;
			case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: // RELOAD DCP and STOP
			case 0x38: case 0x39: case 0x3a: case 0x3b: case 0x3c: case 0x3d: case 0x3e: case 0x3f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: RELOAD DCP and STOP: %06x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				set_dcp<Path>(cmd & 0x003ffffc);
				return;
			case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47: // RELOAD VSR (ICA)
			case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: RELOAD VSR: %06x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				addr = cmd & 0x003ffffc;
				break;
			case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57: // RELOAD VSR and STOP
			case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c: case 0x5d: case 0x5e: case 0x5f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: RELOAD VSR and STOP: VSR = %05x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				set_vsr<Path>(cmd & 0x003fffff);
				return;
			case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67: // INTERRUPT
			case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c: case 0x6d: case 0x6e: case 0x6f:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: INTERRUPT\n", command_addr, cmd, Path);
				m_csrr[1] |= 1 << (2 - Path);
				if (m_csrr[1] & (CSR2R_IT1 | CSR2R_IT2))
					m_int_callback(ASSERT_LINE);
				break;
			case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7e: case 0x7f: // RELOAD DISPLAY PARAMETERS
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: RELOAD DISPLAY PARAMETERS\n", command_addr, cmd, Path);
				set_display_parameters<Path>(cmd & 0x1f);
				break;
			default:
				LOGMASKED(LOG_ICA, "%08x: %08x: ICA %d: SET REGISTER %02x = %06x\n", command_addr, cmd, Path, cmd >> 24, cmd & 0x00ffffff);
				set_register<Path>(cmd >> 24, cmd & 0x00ffffff);
				break;
		}
	}
}

template <int Path>
void mcd212_device::process_dca()
{
	const uint32_t line_addr = m_dca[Path] & 0x003ffffc;
	uint32_t next_line_addr = (line_addr + 64) & 0x003fffff;
	uint32_t addr = line_addr;
	uint32_t cmd = 0;
	uint32_t count = 0;
	// MCD212 table 5-10 permits only 32 DCA bytes at the 15 MHz crystal
	// frequency and 64 at 30 MHz.  The reserved RAM stride remains 64 bytes
	// in both modes; bytes beyond the fetch window are an automatic STOP.
	const uint32_t fetch_limit = BIT(m_dcr[0], DCR_CF_BIT) ? 64 : 32;
	bool processing = true;
	const int trace_line = std::clamp(m_current_render_scanline, 0, m_total_height - 1);
	uint32_t trace_signature = 2166136261U;
	uint32_t trace_icm = 0xffffffff;
	uint32_t trace_tcr = 0xffffffff;
	uint32_t trace_order = 0xffffffff;
	uint32_t trace_link = 0xffffffff;
	auto trace_command = [&](uint32_t value)
	{
		trace_signature = (trace_signature ^ value) * 16777619U;
		const uint8_t code = value >> 24;
		if (code == 0xc0)
			trace_icm = value & 0x00ffffff;
		else if (code == 0xc1)
			trace_tcr = value & 0x00ffffff;
		else if (code == 0xc2)
			trace_order = value & 0x00ffffff;
		else if ((code & 0xe0) == 0x20)
			trace_link = value;
	};
	auto log_control_row = [&]()
	{
		if (!(m_external_video_select_pending || m_external_video_select_active)
			|| (trace_icm == 0xffffffff && trace_tcr == 0xffffffff
				&& trace_order == 0xffffffff && trace_link == 0xffffffff))
			return;

		uint32_t &last = m_last_dca_control_signature[Path][trace_line];
		if (last == trace_signature)
			return;
		last = trace_signature;
		logerror("MCD212 DCA control y=%d path=%d addr=%06x hash=%08x icm=%06x tcr=%06x order=%06x link=%08x effective=%06x/%06x/%u\n",
			trace_line, Path, line_addr, trace_signature,
			trace_icm & 0x00ffffff, trace_tcr & 0x00ffffff,
			trace_order & 0x00ffffff, trace_link,
			m_image_coding_method & 0x00ffffff,
			m_transparency_control & 0x00ffffff,
			m_plane_order & 7);
	};

	LOGMASKED(LOG_DCA, "Scanline %d: Processing DCA %d\n", screen().vpos(), Path);

	while (processing && count < fetch_limit)
	{
		const uint32_t command_addr = addr;
		cmd = uint32_t(read_dram_word(addr)) << 16;
		cmd |= read_dram_word(addr + 2);
		addr = (addr + 4) & 0x003fffff;
		count += 4;
		trace_command(cmd);
		switch ((cmd & 0xff000000) >> 24)
		{
			case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: // STOP
			case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: STOP\n", command_addr, cmd, Path);
				processing = false;
				break;
			case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: // NOP
			case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: NOP\n", command_addr, cmd, Path);
				break;
			case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: // RELOAD DCP
			case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
				// Unlike ICA, RELOAD DCP without STOP is explicitly a no-op in
				// DCA (MCD212 table 5-12).  Only the $30 form changes the row
				// pointer.  Treating $20 as a link can jump into unrelated screen
				// data and prevents CD-RTOS display-list handoffs from completing.
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: RELOAD DCP (NOP)\n", command_addr, cmd, Path);
				break;
			case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: // RELOAD DCP and STOP
			case 0x38: case 0x39: case 0x3a: case 0x3b: case 0x3c: case 0x3d: case 0x3e: case 0x3f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: RELOAD DCP and STOP\n", command_addr, cmd, Path);
				set_dcp<Path>(cmd & 0x003ffffc);
				m_dca[Path] = cmd & 0x003ffffc;
				log_control_row();
				return;
			case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47: // RELOAD VSR
			case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: RELOAD VSR: %06x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				set_vsr<Path>(cmd & 0x003fffff);
				break;
			case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57: // RELOAD VSR and STOP
			case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c: case 0x5d: case 0x5e: case 0x5f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: RELOAD VSR and STOP: %06x\n", command_addr, cmd, Path, cmd & 0x003fffff);
				set_vsr<Path>(cmd & 0x003fffff);
				processing = false;
				break;
			case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67: // INTERRUPT
			case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c: case 0x6d: case 0x6e: case 0x6f:
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: INTERRUPT\n", command_addr, cmd, Path);
				m_csrr[1] |= 1 << (2 - Path);
				if (m_csrr[1] & (CSR2R_IT1 | CSR2R_IT2))
					m_int_callback(ASSERT_LINE);
				break;
			case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7e: case 0x7f: // RELOAD DISPLAY PARAMETERS
				LOGMASKED(LOG_DCA, "%08x: %08x: DCA %d: RELOAD DISPLAY PARAMETERS\n", command_addr, cmd, Path);
				set_display_parameters<Path>(cmd & 0x1f);
				break;
			default:
				set_register<Path>(cmd >> 24, cmd & 0x00ffffff);
				break;
		}
	}

	// DCA rows normally occupy 64 bytes. DC_LLnk places a $20 command in the
	// final column to select the row that the following display line executes.
	m_dca[Path] = next_line_addr;
	log_control_row();
}

template <int Path>
static inline uint8_t BYTE_TO_CLUT(int icm, uint8_t byte, bool clut_select)
{
	switch (icm)
	{
	case 1:
		return byte;
	case 3:
		return (Path ? 0x80 : 0) | (byte & 0x7f);
	case 4:
		if (Path == 0)
		{
			return (clut_select ? 0x80 : 0) | (byte & 0x7f);
		}
		break;
	case 11:
		return (Path ? 0x80 : 0) | (byte & 0x0f);
	default:
		break;
	}
	return 0;
}

template <int Path>
inline ATTR_FORCE_INLINE uint8_t mcd212_device::get_transparency_control()
{
	const bool swap_ab = (m_debug_video_mask & DEBUG_VIDEO_SWAP_TP_AB) != 0;
	const int shift = (swap_ab ? !Path : Path) ? 8 : 0;
	return (m_transparency_control >> shift) & 0x0f;
}

template <int Path>
inline ATTR_FORCE_INLINE uint8_t mcd212_device::get_icm()
{
	const uint32_t mask = Path ? ICM_MODE2 : ICM_MODE1;
	const uint32_t shift = Path ? ICM_MODE2_SHIFT : ICM_MODE1_SHIFT;
	return (m_image_coding_method & mask) >> shift;
}

template <int Path>
inline ATTR_FORCE_INLINE bool mcd212_device::get_mosaic_enable()
{
	return (m_ddr[Path] & DDR_FT) == DDR_FT_MOSAIC;
}

template <int Path>
inline ATTR_FORCE_INLINE uint8_t mcd212_device::get_mosaic_factor()
{
	return 1 << (((m_ddr[Path] & DDR_MT) >> DDR_MT_SHIFT) + 1);
}

template <int Path>
void mcd212_device::process_vsr(uint32_t *pixels, bool *transparent)
{
	const uint8_t icm = get_icm<Path>();
	const uint8_t tp_ctrl = get_transparency_control<Path>();
	const int width = get_screen_width();
	// DCR1.DE is the global display/ICA-DCA gate; DCR2 does not have its own
	// DE bit. A path cannot contribute picture data unless global display is
	// enabled and that path's ICA is enabled.
	const bool path_enabled = BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[Path], DCR_ICA_BIT);

	uint32_t vsr = get_vsr<Path>();
	uint32_t vsr2 = get_vsr<!Path>();
	// Identify the authored display-map line independently of its palette.
	// Popup maps may reuse background colours; RGB differencing alone then
	// cuts holes in their opaque panels.
	m_video_line_source[Path] = vsr | (uint32_t(icm) << 24)
		| (uint32_t((m_ddr[Path] & DDR_FT) >> 8) << 28);

	// A disabled channel or an explicit "always transparent" TCR makes the
	// plane transparent.  ICM=0 is deliberately not included here: the MCD212
	// still runs the enabled plane through its transparency control as a black
	// source.  Burger King uses that distinction during its scanline-controlled
	// screen transitions; treating ICM=0 as transparent punches video-shaped
	// holes through otherwise opaque menu/control artwork.
	if (!path_enabled || tp_ctrl == TCR_ALWAYS)
	{
		std::fill_n(pixels, width, s_4bpp_color[0]);
		std::fill_n(m_dyuv_pixel[Path], width, false);
		// A channel without ICA cannot contribute picture data or obscure the
		// backdrop/external-video path.
		const bool layer_enabled = BIT(m_debug_layer_mask, Path ? 2 : 1);
		std::fill_n(transparent, width, !layer_enabled || !path_enabled || (tp_ctrl == TCR_ALWAYS));
		return;
	}

	const uint32_t decodingMode = m_ddr[Path] & DDR_FT;

	const uint8_t mosaic_enable = get_mosaic_enable<Path>();
	const uint8_t mosaic_factor = get_mosaic_factor<Path>();

	const uint32_t dyuv_abs_start = m_dyuv_abs_start[Path];
	uint8_t y = (dyuv_abs_start >> 16) & 0x000000ff;
	uint8_t u = (dyuv_abs_start >>  8) & 0x000000ff;
	uint8_t v = (dyuv_abs_start >>  0) & 0x000000ff;

	const uint32_t mask_bits = (~m_mask_color[Path]) & 0x00fcfcfc;
	const uint32_t tp_color_match = m_transparent_color[Path] & mask_bits;
	const bool *const matte_flags0 = m_matte_flag[0];
	const bool *const matte_flags1 = m_matte_flag[1];
	const bool is_dyuv_rgb = (icm == ICM_DYUV) || ((icm == ICM_RGB555) && (Path == 1)); // DYUV and RGB do not have access to color key.
	const bool disable_color_key = (m_debug_video_mask & (Path ? DEBUG_VIDEO_DISABLE_B_COLOR_KEY : DEBUG_VIDEO_DISABLE_A_COLOR_KEY)) != 0;
	const bool use_color_key = !disable_color_key && !is_dyuv_rgb
		&& ((tp_ctrl == TCR_KEY) || (tp_ctrl == TCR_NOT_KEY)
			|| (tp_ctrl == TCR_MF0_KEY1) || (tp_ctrl == TCR_MF1_KEY1)
			|| (tp_ctrl == TCR_NOT_MF0_KEY) || (tp_ctrl == TCR_NOT_MF1_KEY));

	LOGMASKED(LOG_VSR, "Scanline %d: VSR Path %d, ICM (%02x), VSR (%08x)\n", screen().vpos(), Path, icm, vsr);

	if (!icm)
	{
		// MCD212 section 8.4 lists disabling a plane as a transparency source.
		// Coding method zero is OFF, so it must expose the lower plane/backdrop
		// regardless of the pixel transparency condition selected in TCR.
		std::fill_n(pixels, width, s_4bpp_color[0]);
		std::fill_n(m_dyuv_pixel[Path], width, false);
		std::fill_n(transparent, width, true);
		return;
	}

	for (uint32_t x = 0; x < width; )
	{
		const uint8_t byte = read_dram_byte(vsr++);
		uint32_t color0 = 0;
		uint32_t color1 = 0;
		if (icm == ICM_DYUV)
		{
			const uint8_t byte1 = read_dram_byte(vsr++);
			const uint8_t y2 = y + m_delta_y_lut[byte];
			y = y2 + m_delta_y_lut[byte1];
			u += m_delta_uv_lut[byte];
			v += m_delta_uv_lut[byte1];

			const uint32_t *limit_rgb = m_dyuv_limit_lut + y2 + 0x100;
			const uint32_t *limit_rgb2 = m_dyuv_limit_lut + y + 0x100;

			color0 = (limit_rgb[m_dyuv_v_to_r[v]] << 16) | (limit_rgb[m_dyuv_u_to_g[u] + m_dyuv_v_to_g[v]] << 8) | limit_rgb[m_dyuv_u_to_b[u]];

			const uint8_t byte2 = read_dram_byte(vsr); // Peek ahead, for calculating the half-step.
			const uint8_t byte3 = read_dram_byte(vsr + 1);
			const uint8_t u8 = u + m_delta_uv_lut[byte2];
			const uint8_t v8 = v + m_delta_uv_lut[byte3];
			const uint8_t u6 = (u >> 1) + (u8 >> 1) + (u & u8 & 1);
			const uint8_t v6 = (v >> 1) + (v8 >> 1) + (v & v8 & 1);

			color1 = (limit_rgb2[m_dyuv_v_to_r[v6]] << 16) | (limit_rgb2[m_dyuv_u_to_g[u6] + m_dyuv_v_to_g[v6]] << 8) | limit_rgb2[m_dyuv_u_to_b[u6]];

			// TODO: Does not support QHY
			pixels[x] = color0;
			pixels[x + 1] = color0;
			pixels[x + 2] = color1;
			pixels[x + 3] = color1;
			for (int i = 0; i < 4; i++)
			{
				m_dyuv_pixel[Path][x + i] = true;
				transparent[x + i] = mcd212_eval_transparency(tp_ctrl, false, false, false, matte_flags0[x + i], matte_flags1[x + i]);
			}
			x += 4;
		}
		else
		{
			bool clut_select = BIT(m_image_coding_method, ICM_CS_BIT);
			bool rgb555_transparency_bit = false;
			if (icm == ICM_RGB555 && Path == 1)
			{
				const uint8_t byte1 = read_dram_byte(vsr2++);
				rgb555_transparency_bit = BIT(byte1, 7);
				const uint8_t blue = (byte & 0b11111) << 3;
				const uint8_t green = ((byte & 0b11100000) >> 2) + ((byte1 & 0b11) << 6);
				const uint8_t red = (byte1 & 0b01111100) << 1;
				color1 = color0 = (uint32_t(red) << 16) | (uint32_t(green) << 8) | blue;
			}
			else if (icm == ICM_CLUT4)
			{
				const uint8_t mask = (decodingMode == DDR_FT_RLE) ? 0x7 : 0xf;
				color0 = m_clut[BYTE_TO_CLUT<Path>(icm, mask & (byte >> 4), clut_select)];
				color1 = m_clut[BYTE_TO_CLUT<Path>(icm, mask & byte, clut_select)];
			}
			else
			{
				color1 = color0 = m_clut[BYTE_TO_CLUT<Path>(icm, byte, clut_select)];
			}

			int length_m = mosaic_enable ? (mosaic_factor * 2) : 2;
			if (decodingMode == DDR_FT_RLE)
			{
				const uint16_t length = (byte & 0x80) ? read_dram_byte(vsr++) : 1;
				length_m = length ? (length * 2) : width;
			}

			const bool color_match0 = ((mask_bits & color0) == tp_color_match);
			const bool color_match1 = ((mask_bits & color1) == tp_color_match);
			const int end = std::min<int>(width, x + length_m);
			for (int rl_index = x; rl_index < end; rl_index += 2)
			{
				pixels[rl_index    ] = color0;
				pixels[rl_index + 1] = color1;
				m_dyuv_pixel[Path][rl_index] = false;
				m_dyuv_pixel[Path][rl_index + 1] = false;
				auto eval_transparency = [rgb555_transparency_bit, tp_ctrl, use_color_key, matte_flags0, matte_flags1](int idx, bool color_match)
				{
					return mcd212_eval_transparency(tp_ctrl, use_color_key && color_match, use_color_key, rgb555_transparency_bit, matte_flags0[idx], matte_flags1[idx]);
				};
				transparent[rl_index    ] = eval_transparency(rl_index, color_match0);
				transparent[rl_index + 1] = eval_transparency(rl_index + 1, color_match1);
			}
			x = end;
		}
	}
	set_vsr<Path>(vsr);
	set_vsr<!Path>(vsr2);
}

const uint32_t mcd212_device::s_4bpp_color[16] =
{
	0xff101010, 0xff10107a, 0xff107a10, 0xff107a7a, 0xff7a1010, 0xff7a107a, 0xff7a7a10, 0xff7a7a7a,
	0xff101010, 0xff1010e6, 0xff10e610, 0xff10e6e6, 0xffe61010, 0xffe610e6, 0xffe6e610, 0xffe6e6e6
};

template <bool MosaicA, bool MosaicB, bool OrderAB>
void mcd212_device::mix_lines(uint32_t *plane_a, bool *transparent_a, uint32_t *plane_b, bool *transparent_b, uint32_t *out, int y)
{
	const uint8_t icmA = get_icm<0>();
	const uint8_t icmB = get_icm<1>();
	const uint8_t tcrA = get_transparency_control<0>();
	const uint8_t tcrB = get_transparency_control<1>();
	uint16_t mosaic_count_a = (m_mosaic_hold[0] & 0x0000ff) << 1;
	uint16_t mosaic_count_b = (m_mosaic_hold[1] & 0x0000ff) << 1;
	const int width = get_screen_width();
	const int border_width = get_border_width();

	uint8_t *weight_a = &m_weight_factor[0][0];
	uint8_t *weight_b = &m_weight_factor[1][0];

	// Console Verified. CLUT4 pixels are drawn in pairs during VSR. So the mosaic here is halved.
	if (icmA == ICM_CLUT4)
		mosaic_count_a >>= 1;
	if (icmB == ICM_CLUT4)
		mosaic_count_b >>= 1;

	// If PAL and 'Standard' bit set, insert a 24px border on the left/right
	uint32_t offset = (!BIT(m_dcr[0], DCR_CF_BIT) || BIT(m_csrw[0], CSR1W_ST_BIT)) ? 24 : 0;
	std::fill_n(out, offset, s_4bpp_color[0]);
	out += offset;
	uint32_t backdrop_hits = 0;
	uint32_t plane_a_hits = 0;
	uint32_t plane_b_hits = 0;
	uint32_t mixed_hits = 0;
	const bool mister_weight_math = false;
	const bool preserve_changed_overlays = m_external_video_mode == EXTERNAL_VIDEO_BETWEEN_PLANES
		&& m_external_video_active_enabled;
	auto overlay_changed_from_baseline = [](uint32_t current, bool current_visible, uint32_t baseline)
	{
		const bool baseline_visible = BIT(baseline, 31);
		const int dr = std::abs(int((current >> 16) & 0xff) - int((baseline >> 16) & 0xff));
		const int dg = std::abs(int((current >> 8) & 0xff) - int((baseline >> 8) & 0xff));
		const int db = std::abs(int(current & 0xff) - int(baseline & 0xff));
		return current_visible != baseline_visible || std::max({ dr, dg, db }) > 12;
	};
	uint32_t visible_count[2] = { 0, 0 };
	bool suppress_dense_change[2] = { false, false };
	bool replaced_line[2] = { false, false };
	if (preserve_changed_overlays)
	{
		for (int plane = 0; plane < 2; plane++)
		{
			if (m_external_video_overlay_capture)
				m_external_video_overlay_source[plane][y] = m_video_line_source[plane];
			else if (m_external_video_overlay_baseline_valid)
				replaced_line[plane] = m_video_line_source[plane] != m_external_video_overlay_source[plane][y];
		}
		for (int x = 0; x < width; x++)
		{
			visible_count[0] += BIT(m_debug_layer_mask, 1) && !transparent_a[x];
			visible_count[1] += BIT(m_debug_layer_mask, 2) && !transparent_b[x];
		}

		// SHOW_NT should replace the old display map, but some CD-RTOS titles
		// leave that map selected in the current MAME model. Identify any dense
		// plane covering the top of the movie as the stale map. Sparse planes are
		// retained for subtitles, popup controls, and hover feedback.
		if (m_external_video_overlay_capture && y < 64)
		{
			if (visible_count[0] > (uint32_t(width) * 3U) / 4U)
				m_external_video_overlay_blockers |= 0x01;
			if (visible_count[1] > (uint32_t(width) * 3U) / 4U)
				m_external_video_overlay_blockers |= 0x02;
			if (y == 63 && !m_external_video_overlay_blockers)
				m_external_video_overlay_blockers = OrderAB
					? ((visible_count[0] >= visible_count[1]) ? 0x01 : 0x02)
					: ((visible_count[1] >= visible_count[0]) ? 0x02 : 0x01);
		}

		if (!m_external_video_overlay_capture && m_external_video_overlay_baseline_valid)
		{
			uint32_t changed_count[2] = { 0, 0 };
			for (int x = 0; x < width; x++)
			{
				const uint32_t current_a = MosaicA ? plane_a[x - (x % mosaic_count_a)] : plane_a[x];
				const uint32_t current_b = MosaicB ? plane_b[x - (x % mosaic_count_b)] : plane_b[x];
				const bool visible_a = BIT(m_debug_layer_mask, 1) && !transparent_a[x];
				const bool visible_b = BIT(m_debug_layer_mask, 2) && !transparent_b[x];
				if ((BIT(m_external_video_overlay_blockers, 0) || m_external_video_overlay_baseline_dyuv[0][y][x])
					&& overlay_changed_from_baseline(current_a, visible_a, m_external_video_overlay_baseline[0][y][x]))
					changed_count[0]++;
				if ((BIT(m_external_video_overlay_blockers, 1) || m_external_video_overlay_baseline_dyuv[1][y][x])
					&& overlay_changed_from_baseline(current_b, visible_b, m_external_video_overlay_baseline[1][y][x]))
					changed_count[1]++;
			}

			for (int plane = 0; plane < 2; plane++)
			{
				const bool blocker = BIT(m_external_video_overlay_blockers, plane);
				const uint32_t candidate_count = blocker ? changed_count[plane] : visible_count[plane];
				const bool dense = candidate_count > (uint32_t(width) * 2U) / 3U;
				m_external_video_overlay_dense_current[plane][y] = dense ? 1 : 0;
				if (dense)
				{
					int dense_neighbours = 0;
					for (int dy = -8; dy <= 8; dy++)
					{
						const int neighbour_y = y + dy;
						if (dy && neighbour_y >= 0 && neighbour_y < 312)
							dense_neighbours += m_external_video_overlay_dense_history[plane][neighbour_y] != 0;
					}
					// Controls occupy a real vertical region. Burger's stale transition
					// enables Plane B for only four full-width lines (137-140); rejecting
					// short dense runs removes that residue without hiding real panels.
					suppress_dense_change[plane] = !replaced_line[plane] && dense_neighbours < 8;
				}
			}
		}
	}

	for (int x = 0; x < width; x++)
	{
		uint32_t plane_a_cur = MosaicA ? plane_a[x - (x % mosaic_count_a)] : plane_a[x];
		uint32_t plane_b_cur = MosaicB ? plane_b[x - (x % mosaic_count_b)] : plane_b[x];

		bool plane_a_visible = BIT(m_debug_layer_mask, 1) && !transparent_a[x];
		bool plane_b_visible = BIT(m_debug_layer_mask, 2) && !transparent_b[x];
		uint32_t ev_pix = 0;
		const bool ignore_ev_alpha = (m_debug_video_mask & DEBUG_VIDEO_IGNORE_ALPHA) != 0;
		const bool raw_ev_present = mcd212_fetch_ev_pixel(m_external_video_active, m_external_video_active_enabled, ignore_ev_alpha, x, y, ev_pix);
		const bool force_ev_base = (m_external_video_mode == EXTERNAL_VIDEO_FORCE_TOP) && raw_ev_present;

		if (preserve_changed_overlays && raw_ev_present)
		{
			const uint32_t baseline_a = m_external_video_overlay_baseline[0][y][x];
			const uint32_t baseline_b = m_external_video_overlay_baseline[1][y][x];
			if (m_external_video_overlay_capture)
			{
				m_external_video_overlay_baseline[0][y][x] = (plane_a_cur & 0x00ffffff) | (plane_a_visible ? 0x80000000 : 0);
				m_external_video_overlay_baseline[1][y][x] = (plane_b_cur & 0x00ffffff) | (plane_b_visible ? 0x80000000 : 0);
				m_external_video_overlay_baseline_dyuv[0][y][x] = m_dyuv_pixel[0][x];
				m_external_video_overlay_baseline_dyuv[1][y][x] = m_dyuv_pixel[1][x];
			}

			// SHOW_NT replaces the captured DYUV movie/still placeholder, not an
			// indiscriminate whole plane. CLUT controls on the same plane remain
			// available, while an unchanged DYUV strip yields to external video.
			const bool replace_a = BIT(m_external_video_overlay_blockers, 0)
				|| m_external_video_overlay_baseline_dyuv[0][y][x];
			const bool replace_b = BIT(m_external_video_overlay_blockers, 1)
				|| m_external_video_overlay_baseline_dyuv[1][y][x];
			if (replace_a)
				plane_a_visible = plane_a_visible
					&& !m_external_video_overlay_capture
					&& !suppress_dense_change[0]
					&& (replaced_line[0] || overlay_changed_from_baseline(plane_a_cur, plane_a_visible, baseline_a));
			else if (suppress_dense_change[0])
				plane_a_visible = false;
			if (replace_b)
				plane_b_visible = plane_b_visible
					&& !m_external_video_overlay_capture
					&& !suppress_dense_change[1]
					&& (replaced_line[1] || overlay_changed_from_baseline(plane_b_cur, plane_b_visible, baseline_b));
			else if (suppress_dense_change[1])
				plane_b_visible = false;
		}

		if ((m_debug_video_mask & DEBUG_VIDEO_RAW_EV_ONLY) != 0)
		{
			out[x] = raw_ev_present ? ev_pix : 0xff000000;
			backdrop_hits++;
			continue;
		}

		if (((m_debug_video_mask & DEBUG_VIDEO_FORCE_EV_TOP) != 0 || force_ev_base) && raw_ev_present)
		{
			out[x] = ev_pix;
			backdrop_hits++;
			continue;
		}

		if (!plane_a_visible)
			plane_a_cur = 0;
		if (!plane_b_visible)
			plane_b_cur = 0;

		const int32_t plane_a_r = 0xff & (plane_a_cur >> 16);
		const int32_t plane_a_g = 0xff & (plane_a_cur >> 8);
		const int32_t plane_a_b = 0xff & plane_a_cur;
		const int32_t plane_b_r = 0xff & (plane_b_cur >> 16);
		const int32_t plane_b_g = 0xff & (plane_b_cur >> 8);
		const int32_t plane_b_b = 0xff & plane_b_cur;

		const int32_t weighted_a_r = mcd212_weight_calc(plane_a_r, weight_a[x], mister_weight_math);
		const int32_t weighted_a_g = mcd212_weight_calc(plane_a_g, weight_a[x], mister_weight_math);
		const int32_t weighted_a_b = mcd212_weight_calc(plane_a_b, weight_a[x], mister_weight_math);

		const int32_t weighted_b_r = mcd212_weight_calc(plane_b_r, weight_b[x], mister_weight_math);
		const int32_t weighted_b_g = mcd212_weight_calc(plane_b_g, weight_b[x], mister_weight_math);
		const int32_t weighted_b_b = mcd212_weight_calc(plane_b_b, weight_b[x], mister_weight_math);

		if (!plane_a_visible && !plane_b_visible)
		{
			backdrop_hits++;
			out[x] = get_backdrop_plane(x, y);
			continue;
		}

		if (m_transparency_control & TCR_DISABLE_MX)
		{
			if (OrderAB)
			{
				if (plane_a_visible)
				{
					plane_a_hits++;
					out[x] = 0xff000000
						| (weighted_a_r << 16)
						| (weighted_a_g << 8)
						| weighted_a_b;
				}
				else
				{
					plane_b_hits++;
					out[x] = 0xff000000
						| (weighted_b_r << 16)
						| (weighted_b_g << 8)
						| weighted_b_b;
				}
			}
			else
			{
				if (plane_b_visible)
				{
					plane_b_hits++;
					out[x] = 0xff000000
						| (weighted_b_r << 16)
						| (weighted_b_g << 8)
						| weighted_b_b;
				}
				else
				{
					plane_a_hits++;
					out[x] = 0xff000000
						| (weighted_a_r << 16)
						| (weighted_a_g << 8)
						| weighted_a_b;
				}
			}
			continue;
		}

		if (plane_a_visible && plane_b_visible)
		{
			mixed_hits++;
			const uint8_t out_r = mcd212_mix_weighted(weighted_a_r, weighted_b_r);
			const uint8_t out_g = mcd212_mix_weighted(weighted_a_g, weighted_b_g);
			const uint8_t out_b = mcd212_mix_weighted(weighted_a_b, weighted_b_b);
			out[x] = 0xff000000 | (out_r << 16) | (out_g << 8) | out_b;
		}
		else if (plane_a_visible)
		{
			plane_a_hits++;
			out[x] = 0xff000000
				| (weighted_a_r << 16)
				| (weighted_a_g << 8)
				| weighted_a_b;
		}
		else
		{
			plane_b_hits++;
			out[x] = 0xff000000
				| (weighted_b_r << 16)
				| (weighted_b_g << 8)
				| weighted_b_b;
		}
	}

	if (m_external_video_active_enabled)
	{
		m_ev_backdrop_hits += backdrop_hits;
		m_ev_plane_a_hits += plane_a_hits;
		m_ev_plane_b_hits += plane_b_hits;
		m_ev_mixed_hits += mixed_hits;
	}

	if ((m_debug_video_mask & DEBUG_VIDEO_LOG_PLANE_STATS) != 0
		&& m_external_video_active_enabled
		&& ((y % 16) == 0 || (y >= (screen().height() - 48))))
	{
		logerror("MCD212 EV line y=%d hits(bg/a/b/mix)=%u/%u/%u/%u order=%c icm=%x/%x tcr=%x/%x dcr=%04x/%04x ev=%d\n",
			y,
			backdrop_hits,
			plane_a_hits,
			plane_b_hits,
			mixed_hits,
			OrderAB ? 'A' : 'B',
			icmA,
			icmB,
			tcrA,
			tcrB,
			m_dcr[0],
			m_dcr[1],
			external_video_icm_enabled() ? 1 : 0);
	}

	if (border_width)
	{
		std::fill_n(&out[width], border_width, s_4bpp_color[0]);
	}

	if (preserve_changed_overlays && m_external_video_overlay_capture
		&& y == (m_total_height - m_ica_height - 1))
	{
		m_external_video_overlay_capture = false;
		m_external_video_overlay_baseline_valid = true;
		logerror("MCD212 EV overlay baseline captured blockers=%u\n", m_external_video_overlay_blockers);
	}
	else if (preserve_changed_overlays && m_external_video_overlay_baseline_valid
		&& y == (m_total_height - m_ica_height - 1))
	{
		std::copy_n(&m_external_video_overlay_dense_current[0][0], 2 * 312,
			&m_external_video_overlay_dense_history[0][0]);
		std::fill_n(&m_external_video_overlay_dense_current[0][0], 2 * 312, 0);
	}
}

void mcd212_device::draw_cursor(uint32_t *scanline)
{
	if (!(m_debug_layer_mask & DEBUG_LAYER_CURSOR))
		return;

	if (!(m_cursor_control & CURCNT_EN))
		return; // Cursor is Disabled

	uint8_t color_index = m_cursor_control & CURCNT_COLOR;
	if (m_blink_active)
	{
		const bool invert = BIT(m_cursor_control, CURCNT_BLKC_SHIFT);
		if (!invert)
			return; // Normal Blink
		else
			color_index = color_index ^ 0x7; // Inverted Color Blink. MCD212 Section 7.5
	}

	const uint16_t cursor_x = m_cursor_position & 0x3ff;
	const uint16_t cursor_y = ((m_cursor_position >> 12) & 0x3ff) + m_ica_height;
	const int32_t y = m_current_render_scanline - cursor_y;
	const int width = get_screen_width();

	if ((0 <= y) && (y < 16))
	{
		const uint32_t color = s_4bpp_color[color_index];
		const uint8_t resolution = (m_cursor_control & CURCNT_CUW) ? 1 : 2;
		for (int x = 0; x < 16; x++)
		{
			if (BIT(m_cursor_pattern[y], 15 - x))
			{
				for (uint32_t j = 0; j < resolution; j++)
				{
					const uint32_t index = cursor_x + x * resolution + j;
					if (index < width)
						scanline[index] = color;
				}
			}
		}
	}
}

void mcd212_device::map(address_map &map)
{
	map(0x00, 0x01).w(FUNC(mcd212_device::csr2_w));
	map(0x01, 0x01).r(FUNC(mcd212_device::csr2_r));
	map(0x02, 0x03).rw(FUNC(mcd212_device::dcr2_r), FUNC(mcd212_device::dcr2_w));
	map(0x04, 0x05).rw(FUNC(mcd212_device::vsr2_r), FUNC(mcd212_device::vsr2_w));
	map(0x08, 0x09).rw(FUNC(mcd212_device::ddr2_r), FUNC(mcd212_device::ddr2_w));
	map(0x0a, 0x0b).rw(FUNC(mcd212_device::dca2_r), FUNC(mcd212_device::dca2_w));

	map(0x10, 0x11).w(FUNC(mcd212_device::csr1_w));
	map(0x11, 0x11).r(FUNC(mcd212_device::csr1_r));
	map(0x12, 0x13).rw(FUNC(mcd212_device::dcr1_r), FUNC(mcd212_device::dcr1_w));
	map(0x14, 0x15).rw(FUNC(mcd212_device::vsr1_r), FUNC(mcd212_device::vsr1_w));
	map(0x18, 0x19).rw(FUNC(mcd212_device::ddr1_r), FUNC(mcd212_device::ddr1_w));
	map(0x1a, 0x1b).rw(FUNC(mcd212_device::dca1_r), FUNC(mcd212_device::dca1_w));
}

uint8_t mcd212_device::csr1_r()
{
	LOGMASKED(LOG_STATUS, "%s: Control/Status Register 1 Read: %02x\n", machine().describe_context(), m_csrr[0]);
	return m_csrr[0];
}

void mcd212_device::csr1_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Control/Status Register 1 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_csrw[0]);
}

uint16_t mcd212_device::dcr1_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Display Command Register 1 Read: %04x & %08x\n", machine().describe_context(), m_dcr[0], mem_mask);
	return m_dcr[0];
}

void mcd212_device::dcr1_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Display Command Register 1 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_dcr[0]);
}

uint16_t mcd212_device::vsr1_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Video Start Register 1 Read: %04x & %08x\n", machine().describe_context(), m_vsr[0], mem_mask);
	return m_vsr[0];
}

void mcd212_device::vsr1_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Video Start Register 1 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_vsr[0]);
}

uint16_t mcd212_device::ddr1_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Display Decoder Register 1 Read: %04x & %08x\n", machine().describe_context(), m_ddr[0], mem_mask);
	return m_ddr[0];
}

void mcd212_device::ddr1_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Display Decoder Register 1 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_ddr[0]);
}

uint16_t mcd212_device::dca1_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: DCA Pointer 1 Read: %04x & %08x\n", machine().describe_context(), m_dca[0], mem_mask);
	return m_dca[0];
}

void mcd212_device::dca1_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: DCA Pointer 1 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_dca[0]);
}

uint8_t mcd212_device::csr2_r()
{
	if (machine().side_effects_disabled())
	{
		return m_csrr[1];
	}

	const uint8_t data = m_csrr[1];
	LOGMASKED(LOG_STATUS, "%s: Status Register 2: %02x\n", machine().describe_context(), data);

	m_csrr[1] &= ~(CSR2R_IT1 | CSR2R_IT2);
	if (data & (CSR2R_IT1 | CSR2R_IT2))
		m_int_callback(CLEAR_LINE);

	return data;
}

void mcd212_device::csr2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Control/Status Register 2 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_csrw[1]);
}

uint16_t mcd212_device::dcr2_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Display Command Register 2 Read: %04x & %08x\n", machine().describe_context(), m_dcr[1], mem_mask);
	return m_dcr[1];
}

void mcd212_device::dcr2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Display Command Register 2 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_dcr[1]);
}

uint16_t mcd212_device::vsr2_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Video Start Register 2 Read: %04x & %08x\n", machine().describe_context(), m_vsr[1], mem_mask);
	return m_vsr[1];
}

void mcd212_device::vsr2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Video Start Register 2 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_vsr[1]);
}

uint16_t mcd212_device::ddr2_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: Display Decoder Register 2 Read: %04x & %08x\n", machine().describe_context(), m_ddr[1], mem_mask);
	return m_ddr[1];
}

void mcd212_device::ddr2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: Display Decoder Register 2 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_ddr[1]);
}

uint16_t mcd212_device::dca2_r(offs_t offset, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_READS, "%s: DCA Pointer 2 Read: %04x & %08x\n", machine().describe_context(), m_dca[1], mem_mask);
	return m_dca[1];
}

void mcd212_device::dca2_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	LOGMASKED(LOG_MAIN_REG_WRITES, "%s: DCA Pointer 2 Write: %04x & %08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_dca[1]);
}

TIMER_CALLBACK_MEMBER(mcd212_device::ica_tick)
{
	m_csrr[0] &= ~CSR1R_DA;
	// PA only describes odd/even fields in interlace modes. Non-interlace
	// always executes the single-field h400 FCT, so report the matching odd
	// state while CD-RTOS updates its display-list bookkeeping.
	if (!BIT(m_dcr[0], DCR_SM_BIT))
		m_csrr[0] |= CSR1R_PA;
	m_last_rendered_scanline = m_ica_height - 1;
	m_pending_vsr_valid[0] = false;
	m_pending_vsr_valid[1] = false;

	// Process ICA
	m_current_render_scanline = 0;
	if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[0], DCR_ICA_BIT))
		process_ica<0>();
	if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[1], DCR_ICA_BIT))
		process_ica<1>();
	m_current_render_scanline = -1;

	if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[0], DCR_DCA_BIT))
		m_dca[0] = get_dcp<0>();
	if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[1], DCR_DCA_BIT))
		m_dca[1] = get_dcp<1>();

	m_ica_timer->adjust(screen().time_until_pos(0, 0));

	// Cursor Blink
	m_blink_time += 5 + BIT(m_dcr[0], DCR_FD_BIT); // FD bit * 8... Page 4-3 MCD
	// Adjust the blink time once per frame
	if (!m_blink_active && (m_blink_time >= ((m_cursor_control & CURCNT_CON) >> CURCNT_CON_SHIFT) * 60))
	{
		m_blink_active = true;
		m_blink_time = 0;
	}
	// If blink off time is 0, immediately turn back on.
	if (m_blink_active && (m_blink_time >= ((m_cursor_control & CURCNT_COF) >> CURCNT_COF_SHIFT) * 60))
	{
		m_blink_active = false;
		m_blink_time = 0;
	}
}

TIMER_CALLBACK_MEMBER(mcd212_device::dca_tick)
{
	const int physical_scanline = screen().vpos();
	const int logical_scanline = physical_to_logical_scanline(physical_scanline);
	const int next_logical_scanline = logical_scanline + 1;
	const int dca_x = get_dca_trigger_x();

	// DCA instructions are fetched during horizontal retrace. Finish the
	// current display line first, then prepare the following one.
	if (physical_scanline >= m_ica_height)
		screen().update_partial(physical_scanline);

	if (next_logical_scanline >= m_ica_height && next_logical_scanline < m_total_height)
	{
		m_current_render_scanline = next_logical_scanline;
		if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[0], DCR_DCA_BIT))
			process_dca<0>();
		if (BIT(m_dcr[0], DCR_DE_BIT) && BIT(m_dcr[1], DCR_DCA_BIT))
			process_dca<1>();
		m_current_render_scanline = -1;
	}

	const int next_trigger = (next_logical_scanline >= (m_total_height - 1))
		? dca_trigger_scanline(m_ica_height)
		: dca_trigger_scanline(next_logical_scanline + 1);
	m_dca_timer->adjust(screen().time_until_pos(next_trigger, dca_x));
}

uint32_t mcd212_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint32_t plane_a[768];
	uint32_t plane_b[768];
	bool transparent_a[768];
	bool transparent_b[768];
	m_current_render_scanline = -1;

	const int physical_scanline = screen.vpos();
	const int logical_scanline = physical_to_logical_scanline(physical_scanline);
	if (logical_scanline >= m_total_height)
		return 0;

	if (logical_scanline < m_last_rendered_scanline)
		m_last_rendered_scanline = m_ica_height - 1;

	const int start_scanline = std::max(m_ica_height, m_last_rendered_scanline + 1);
	const int end_scanline = std::min(logical_scanline, m_total_height - 1);

	for (int scanline = start_scanline; scanline <= end_scanline; scanline++)
	{
		m_current_render_scanline = scanline;

		if (scanline == m_ica_height)
		{
			m_ev_backdrop_hits = 0;
			m_ev_plane_a_hits = 0;
			m_ev_plane_b_hits = 0;
			m_ev_mixed_hits = 0;
			m_ev_replace_a_hits = 0;
			m_ev_replace_b_hits = 0;
			m_ev_yield_a_hits = 0;
			m_ev_yield_b_hits = 0;
			m_ev_src_a_hits = 0;
			m_ev_src_b_hits = 0;
			if (m_external_video_dirty)
			{
				const bool active_enable_changed = (m_external_video_active_enabled != m_external_video_pending_enabled);
				const bool active_select_changed = (m_external_video_select_active != m_external_video_select_pending);
				copybitmap(m_external_video_active, m_external_video_pending, 0, 0, 0, 0, rectangle(0, m_external_video_pending.width() - 1, 0, m_external_video_pending.height() - 1));
				m_external_video_active_enabled = m_external_video_pending_enabled;
				m_external_video_select_active = m_external_video_select_pending;
				m_external_video_dirty = false;
				if (active_enable_changed || active_select_changed)
					m_last_rendered_scanline = m_ica_height - 1;
				log_external_video_state("frame apply", true);
			}

		}

		uint32_t const bitmap_line = logical_to_physical_scanline(scanline);
		uint32_t *const out = &bitmap.pix(bitmap_line + BIT(~m_csrr[0], CSR1R_PA_BIT));
		uint32_t *const out2 = &bitmap.pix(bitmap_line + BIT(m_csrr[0], CSR1R_PA_BIT));

		bool draw_line = true;
		if (!BIT(m_dcr[0], DCR_FD_BIT) && BIT(m_csrw[0], CSR1W_ST_BIT))
		{
			// If PAL and 'Standard' bit set, insert a 20-line border on the top/bottom
			if ((scanline - m_ica_height < 20) || (scanline >= (m_total_height - 20)))
			{
				std::fill_n(out, 768, s_4bpp_color[0]);
				draw_line = false;
			}
		}

		m_csrr[0] |= CSR1R_DA;

		if (m_external_video_active_enabled)
		{
			const uint64_t line_signature =
				(uint64_t(m_image_coding_method & 0x00ffffff) << 32)
				| (uint64_t(m_transparency_control & 0x00ffffff) << 8)
				| uint64_t(m_plane_order & 7);
			if (line_signature != m_last_ev_line_signature)
			{
				m_last_ev_line_signature = line_signature;
				logerror("MCD212 EV line-state y=%d icm=%06x tcr=%06x order=%u dca=%06x/%06x vsr=%06x/%06x\n",
					scanline - m_ica_height,
					m_image_coding_method | (m_external_video_select_active ? ICM_EV : 0),
					m_transparency_control,
					m_plane_order,
					m_dca[0],
					m_dca[1],
					m_vsr[0],
					m_vsr[1]);
			}
		}

		// Region flags restart low each line, while contribution factors begin
		// with the values left by the previous line.
		update_matte_arrays();

		if (draw_line)
		{
			process_vsr<0>(plane_a, transparent_a);
			process_vsr<1>(plane_b, transparent_b);

			const uint8_t mosaic_enable_a = (m_mosaic_hold[0] & 0x800000) >> 23;
			const uint8_t mosaic_enable_b = (m_mosaic_hold[1] & 0x800000) >> 22;
			const uint8_t mixing_mode = (mosaic_enable_a | mosaic_enable_b) | (BIT(m_plane_order, 0) << 2);
			switch (mixing_mode & 7)
			{
				case 0: // No Mosaic A/B, A->B->Backdrop plane ordering
					mix_lines<false, false, true>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 1: // Mosaic A, No Mosaic B, A->B->Backdrop plane ordering
					mix_lines<true, false, true>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 2: // No Mosaic A, Mosaic B, A->B->Backdrop plane ordering
					mix_lines<false, true, true>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 3: // Mosaic A/B, A->B->Backdrop plane ordering
					mix_lines<true, true, true>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 4: // No Mosaic A/B, B->A->Backdrop plane ordering
					mix_lines<false, false, false>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 5: // Mosaic A, No Mosaic B, B->A->Backdrop plane ordering
					mix_lines<true, false, false>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 6: // No Mosaic A, Mosaic B, B->A->Backdrop plane ordering
					mix_lines<false, true, false>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
				case 7: // Mosaic A/B, B->A->Backdrop plane ordering
					mix_lines<true, true, false>(plane_a, transparent_a, plane_b, transparent_b, out, scanline - m_ica_height);
					break;
			}

			draw_cursor(out);
		}

		if (BIT(m_dcr[0], DCR_SM_BIT))
		{
			// Interlace Output
			std::copy_n(m_interlace_field[scanline], 768, out2);
			std::copy_n(out, 768, m_interlace_field[scanline]);
		}
		else
		{
			// Single Field Output (duplicate lines)
			std::copy_n(out, 768, out2);
		}

		if ((scanline == (m_total_height - 1)) && m_external_video_active_enabled)
		{
			uint64_t summary_signature = 1469598103934665603ULL;
			auto hash_summary = [&summary_signature](uint32_t value)
			{
				summary_signature = (summary_signature ^ value) * 1099511628211ULL;
			};
			hash_summary(m_ev_backdrop_hits);
			hash_summary(m_ev_plane_a_hits);
			hash_summary(m_ev_plane_b_hits);
			hash_summary(m_ev_mixed_hits);
			hash_summary(m_image_coding_method);
			hash_summary(m_transparency_control);
			hash_summary((uint32_t(m_dcr[0]) << 16) | m_dcr[1]);
			hash_summary(m_plane_order);
			if (summary_signature != m_last_ev_summary_signature)
			{
				m_last_ev_summary_signature = summary_signature;
				logerror("MCD212 EV summary backdrop=%u plane_a=%u plane_b=%u mixed=%u repl_a=%u repl_b=%u yield_a=%u yield_b=%u src_a=%u src_b=%u icm=%06x tcr=%06x dcr=%04x/%04x order=%u evmode=%u dbg=%05x\n",
				m_ev_backdrop_hits,
				m_ev_plane_a_hits,
				m_ev_plane_b_hits,
				m_ev_mixed_hits,
				m_ev_replace_a_hits,
				m_ev_replace_b_hits,
				m_ev_yield_a_hits,
				m_ev_yield_b_hits,
				m_ev_src_a_hits,
				m_ev_src_b_hits,
				m_image_coding_method | (m_external_video_select_active ? ICM_EV : 0),
				m_transparency_control,
				m_dcr[0],
				m_dcr[1],
				m_plane_order,
				m_external_video_mode,
				m_debug_video_mask);
			}
		}
	}

	if (end_scanline >= start_scanline)
	{
		m_last_rendered_scanline = end_scanline;
		m_current_render_scanline = -1;
	}

	// PA alternates only for interlace. Non-interlace always uses the h400 FCT.
	if (end_scanline == (m_total_height - 1))
	{
		if (BIT(m_dcr[0], DCR_SM_BIT))
			m_csrr[0] ^= CSR1R_PA;
		else
			m_csrr[0] |= CSR1R_PA;
	}

	return 0;
}

template int mcd212_device::ram_dtack_cycle_count<0>();
template int mcd212_device::ram_dtack_cycle_count<1>();

template <int Path>
int mcd212_device::ram_dtack_cycle_count()
{
	// Per MCD-212 documentation, it takes 4 CLKs (2 SCC68070 clocks) for a VRAM access during the System timing slot.

	// No contending for Ch.1/Ch.2 timing slots if display is disabled
	if (!BIT(m_dcr[0], DCR_DE_BIT))
		return 2;

	// No contending for Ch.1/Ch.2 timing slots if a relevant Path is disabled
	if (!BIT(m_dcr[Path], DCR_ICA_BIT))
		return 2;

	const int x = screen().hpos();
	const int y = physical_to_logical_scanline(screen().vpos());
	const bool x_outside_active_display = (x >= 408);

	// No contending for Ch.1/Ch.2 timing slots during the final 8-pixel area on all lines
	if (x >= 472)
		return 2;

	// No contending for Ch.1/Ch.2 timing slots during the free-run area of ICA lines
	if (y < m_ica_height && x_outside_active_display)
		return 2;

	// No contending for Ch.1/Ch.2 timing slots during the free-run area of DCA lines if DCA is disabled
	if (!BIT(m_dcr[Path], DCR_DCA_BIT) && x_outside_active_display)
		return 2;

	// System access is restricted to the last 5 out of every 16 CLKs.
	const int slot_cycle = int(machine().time().as_ticks(clock()) & 0xf);
	if (slot_cycle >= 11)
		return 2;

	return 2 + std::max((11 - slot_cycle) >> 1, 1);
}

int mcd212_device::rom_dtack_cycle_count()
{
	static const int s_dd_values[4] = { 2, 3, 4, 5 };
	if (!BIT(m_csrw[0], CSR1W_DD_BIT))
		return 7;
	return s_dd_values[(m_csrw[0] & CSR1W_DD2) >> CSR1W_DD2_SHIFT];
}

void mcd212_device::device_reset()
{
	std::fill_n(m_csrr, 2, 0);
	std::fill_n(m_csrw, 2, 0);
	std::fill_n(m_dcr, 2, 0);
	std::fill_n(m_vsr, 2, 0);
	std::fill_n(m_ddr, 2, 0);
	std::fill_n(m_dcp, 2, 0);
	std::fill_n(m_dca, 2, 0);
	std::fill_n(m_clut, 256, 0);
	m_image_coding_method_programmed = 0;
	m_image_coding_method = 0;
	m_transparency_control = 0;
	m_plane_order = 0;
	std::fill_n(m_clut_bank, 2, 0);
	std::fill_n(m_transparent_color, 2, 0);
	std::fill_n(m_mask_color, 2, 0);
	std::fill_n(m_dyuv_abs_start, 2, 0);
	m_cursor_position = 0;
	m_cursor_control = 0;
	std::fill_n(m_cursor_pattern, std::size(m_cursor_pattern), 0);
	std::fill_n(m_matte_control, 8, 0);
	std::fill_n(m_matte_control_scanline, 8, -1);
	std::fill_n(m_matte_control_path, 8, 0);
	m_backdrop_color = 0;
	std::fill_n(m_mosaic_hold, 2, 0);
	std::fill_n(m_base_weight_factor, 2, 0);
	std::fill_n(m_current_weight_factor, 2, 0);
	std::fill_n(m_weight_factor[0], std::size(m_weight_factor[0]), 0);
	std::fill_n(m_weight_factor[1], std::size(m_weight_factor[1]), 0);
	std::fill_n(m_matte_flag[0], std::size(m_matte_flag[0]), false);
	std::fill_n(m_matte_flag[1], std::size(m_matte_flag[1]), false);
	std::fill_n(m_dyuv_pixel[0], std::size(m_dyuv_pixel[0]), false);
	std::fill_n(m_dyuv_pixel[1], std::size(m_dyuv_pixel[1]), false);
	std::fill_n(m_video_line_source, 2, 0);
	std::fill_n(&m_external_video_overlay_source[0][0], 2 * 312, 0);
	std::fill_n(m_pending_vsr, 2, 0);
	std::fill_n(m_pending_vsr_valid, 2, false);
	m_external_video_pending_enabled = false;
	m_external_video_active_enabled = false;
	m_external_video_select_pending = false;
	m_external_video_select_active = false;
	m_external_video_dirty = false;
	m_external_video_mode = EXTERNAL_VIDEO_BACKDROP;
	m_external_video_page = 0;
	m_external_video_overlay_capture = false;
	m_external_video_overlay_baseline_valid = false;
	m_external_video_overlay_blockers = 0;
	std::fill_n(&m_external_video_overlay_baseline[0][0][0], 2 * 312 * 768, 0);
	std::fill_n(&m_external_video_overlay_baseline_dyuv[0][0][0], 2 * 312 * 768, false);
	std::fill_n(&m_external_video_overlay_dense_history[0][0], 2 * 312, 0);
	std::fill_n(&m_external_video_overlay_dense_current[0][0], 2 * 312, 0);
	m_ev_replace_a_hits = 0;
	m_ev_replace_b_hits = 0;
	m_ev_yield_a_hits = 0;
	m_ev_yield_b_hits = 0;
	m_ev_src_a_hits = 0;
	m_ev_src_b_hits = 0;
	m_last_ev_summary_signature = ~uint64_t(0);
	m_last_ev_line_signature = ~uint64_t(0);
	for (auto &path : m_last_dca_control_signature)
		std::fill(std::begin(path), std::end(path), 0xffffffff);
	std::fill_n(m_last_ica_head_signature, 2, ~uint64_t(0));
	m_external_video_pending.fill(0x00000000);
	m_external_video_active.fill(0x00000000);

	m_ica_height = 32;
	m_total_height = 312;
	m_last_rendered_scanline = m_ica_height - 1;
	m_current_render_scanline = -1;
	m_blink_time = 0;
	for (int i = 0; i < m_total_height; i++)
	{
		std::fill_n(m_interlace_field[i], 768, 0);
	}

	m_int_callback(CLEAR_LINE);

	// Prime the first visible line from the horizontal retrace immediately
	// preceding it. ICA runs at the beginning of vertical blank.
	m_dca_timer->adjust(screen().time_until_pos(dca_trigger_scanline(m_ica_height), get_dca_trigger_x()));
	m_ica_timer->adjust(screen().time_until_pos(0, 0));
}

//-------------------------------------------------
//  mcd212_device - constructor
//-------------------------------------------------

mcd212_device::mcd212_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MCD212, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, m_int_callback(*this)
	, m_planea(*this, finder_base::DUMMY_TAG)
	, m_planeb(*this, finder_base::DUMMY_TAG)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mcd212_device::device_start()
{
	static const uint8_t s_dyuv_deltas[16] = { 0, 1, 4, 9, 16, 27, 44, 79, 128, 177, 212, 229, 240, 247, 252, 255 };

	m_external_video_pending.allocate(768, 312);
	m_external_video_active.allocate(768, 312);
	clear_external_video();
	m_external_video_active.fill(0x00000000);

	for (uint16_t d = 0; d < 0x100; d++)
	{
		m_delta_y_lut[d] = s_dyuv_deltas[d & 15];
		m_delta_uv_lut[d] = s_dyuv_deltas[d >> 4];
	}

	for (uint16_t w = 0; w < 0x300; w++)
	{
		const uint8_t limit = (w < 0x100) ? 0 : (w < 0x200) ? (w - 0x100) : 0xff;
		m_dyuv_limit_lut[w] = limit;
	}

	for (int16_t sw = 0; sw < 0x100; sw++)
	{
		m_dyuv_u_to_b[sw] = (444 * (sw - 128)) / 256;
		m_dyuv_u_to_g[sw] = - (86 * (sw - 128)) / 256;
		m_dyuv_v_to_g[sw] = - (179 * (sw - 128)) / 256;
		m_dyuv_v_to_r[sw] = (351 * (sw - 128)) / 256;
	}

	save_item(NAME(m_csrr));
	save_item(NAME(m_csrw));
	save_item(NAME(m_dcr));
	save_item(NAME(m_vsr));
	save_item(NAME(m_ddr));
	save_item(NAME(m_dcp));
	save_item(NAME(m_dca));
	save_item(NAME(m_clut));
	save_item(NAME(m_image_coding_method_programmed));
	save_item(NAME(m_image_coding_method));
	save_item(NAME(m_transparency_control));
	save_item(NAME(m_plane_order));
	save_item(NAME(m_clut_bank));
	save_item(NAME(m_transparent_color));
	save_item(NAME(m_mask_color));
	save_item(NAME(m_dyuv_abs_start));
	save_item(NAME(m_cursor_position));
	save_item(NAME(m_cursor_control));
	save_item(NAME(m_cursor_pattern));
	save_item(NAME(m_matte_control));
	save_item(NAME(m_matte_control_scanline));
	save_item(NAME(m_matte_control_path));
	save_item(NAME(m_backdrop_color));
	save_item(NAME(m_mosaic_hold));
	save_item(NAME(m_base_weight_factor));
	save_item(NAME(m_current_weight_factor));
	save_item(NAME(m_weight_factor[0]));
	save_item(NAME(m_weight_factor[1]));

	save_item(NAME(m_matte_flag));
	save_item(NAME(m_dyuv_pixel));
	save_item(NAME(m_ica_height));
	save_item(NAME(m_total_height));
	save_item(NAME(m_last_rendered_scanline));
	save_item(NAME(m_current_render_scanline));

	save_item(NAME(m_blink_time));
	save_item(NAME(m_blink_active));
	save_item(NAME(m_external_video_pending_enabled));
	save_item(NAME(m_external_video_active_enabled));
	save_item(NAME(m_external_video_select_pending));
	save_item(NAME(m_external_video_select_active));
	save_item(NAME(m_external_video_dirty));
	save_item(NAME(m_external_video_mode));
	save_item(NAME(m_external_video_page));
	save_item(NAME(m_external_video_overlay_baseline));
	save_item(NAME(m_external_video_overlay_baseline_dyuv));
	save_item(NAME(m_video_line_source));
	save_item(NAME(m_external_video_overlay_source));
	save_item(NAME(m_external_video_overlay_dense_history));
	save_item(NAME(m_external_video_overlay_dense_current));
	save_item(NAME(m_external_video_overlay_capture));
	save_item(NAME(m_external_video_overlay_baseline_valid));
	save_item(NAME(m_external_video_overlay_blockers));
	save_item(NAME(m_debug_layer_mask));
	save_item(NAME(m_debug_video_mask));
	save_item(NAME(m_ev_backdrop_hits));
	save_item(NAME(m_ev_plane_a_hits));
	save_item(NAME(m_ev_plane_b_hits));
	save_item(NAME(m_ev_mixed_hits));
	save_item(NAME(m_ev_replace_a_hits));
	save_item(NAME(m_ev_replace_b_hits));
	save_item(NAME(m_ev_yield_a_hits));
	save_item(NAME(m_ev_yield_b_hits));
	save_item(NAME(m_ev_src_a_hits));
	save_item(NAME(m_ev_src_b_hits));
	save_item(NAME(m_last_ev_summary_signature));
	save_item(NAME(m_last_ev_line_signature));
	save_item(NAME(m_last_dca_control_signature));
	save_item(NAME(m_last_ica_head_signature));
	save_item(NAME(m_pending_vsr));
	save_item(NAME(m_pending_vsr_valid));

	save_item(NAME(m_interlace_field));

	m_dca_timer = timer_alloc(FUNC(mcd212_device::dca_tick), this);
	m_dca_timer->adjust(attotime::never);

	m_ica_timer = timer_alloc(FUNC(mcd212_device::ica_tick), this);
	m_ica_timer->adjust(attotime::never);
}

void mcd212_device::clear_external_video()
{
	// Keep unwritten EV pixels distinguishable from drawn DVC video so the
	// backdrop path can ignore untouched areas cleanly.
	m_external_video_pending.fill(0x00000000);
	m_external_video_dirty = true;
}
