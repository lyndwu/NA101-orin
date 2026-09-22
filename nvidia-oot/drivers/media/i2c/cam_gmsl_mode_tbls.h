/*
 * cam_gmsl_mode_tbls.h - cam_gmsl sensor mode tables
 *
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __CAM_GMSL_I2C_TABLES__
#define __CAM_GMSL_I2C_TABLES__

#include <media/camera_common.h>

#define CAM_GMSL_TABLE_WAIT_MS	0xff00
#define CAM_GMSL_TABLE_END	0xff01
#define CAM_GMSL_MAX_RETRIES	3
#define CAM_GMSL_WAIT_MS_STOP	1
#define CAM_GMSL_WAIT_MS_START	30
#define CAM_GMSL_WAIT_MS_STREAM	210
#define CAM_GMSL_GAIN_TABLE_SIZE 255

#define cam_gmsl_reg struct reg_16

struct index_reg_8 {
	u16 source;
	u16 addr;
	u16 val;
};

static struct index_reg_8 cam_gmsl_start[] = {
	// Enable MIPI Output
	{0x52,0x040B,0x02},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 50}, // Delay 50ms

	{0x00, CAM_GMSL_TABLE_END, 0x00}
};

static struct index_reg_8 cam_gmsl_stop[] = {
	{0x52,0x040B,0x00}, //Disable MIPI CSI-2
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 50}, // Delay 50ms

	{0x00, CAM_GMSL_TABLE_END, 0x00}
};

static struct index_reg_8 cam_gmsl_Double_Dser_Ser[] = {
	{0x52,0x0013,0x40}, //Device Reset
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms

	// Setup MAX9295A -> MAX9296A --MIPI PORT A
	{0x52,0x040B,0x00}, //Disable MIPI CSI-2

	// Link Initiaion
	{0x52,0x0006,0xFF}, // Enable all 4 Links in GMSL2 mode
	// Video Pipe Selection
	{0x52,0x00F0,0x62}, //40/62	// pipe X in link B to video pipe 1, pipe X in link A to video pipe 0
	{0x52,0x00F1,0xEA}, //ea/c8	// pipe X in link D to video pipe 3, pipe X in link C to video pipe 2
	{0x52,0x00F2,0x51},			// pipe Y in link B to video pipe 4, pipe Y in link A to video pipe 5
	{0x52,0x00F3,0xD9},			// pipe Y in link D to video pipe 6, pipe Y in link C to video pipe 7
	{0x52,0x00F4,0x0F},			// Turn on 4 pipes

	// (optional) enable for max. bandwidth efficiency
	// ---- Enable bpp12 double mode for all controllers ---------
	{0x52,0x0933,0x01}, // ALT_MEM_MAP12 = 1 on Ctrl 0
	{0x52,0x0973,0x01}, // ALT_MEM_MAP12 = 1 on Ctrl 1
	{0x52,0x09B3,0x01}, // ALT_MEM_MAP12 = 1 on Ctrl 2
	{0x52,0x09F3,0x01}, // ALT_MEM_MAP12 = 1 on Ctrl 3

	// Efficiency updates (disable HEARTBEAT Mode) for image data pipes 0-3
	{0x52,0x0106,0x0A},
	{0x52,0x0118,0x0A},
	{0x52,0x012A,0x0A},
	{0x52,0x013C,0x0A},

	// YUV422 8bit, video pipe 0, map FS/FE
	{0x52, 0x090B, 0x07},
	{0x52, 0x092D, 0x15}, // map to MIPI Controller 1
	{0x52, 0x090D, 0x1E},
	{0x52, 0x090E, 0x1E}, // map to VC0
	{0x52, 0x090F, 0x00},
	{0x52, 0x0910, 0x00},
	{0x52, 0x0911, 0x01},
	{0x52, 0x0912, 0x01},
	// YUV422 8bit, video pipe 1, map FS/FE
	{0x52, 0x094B, 0x07},
	{0x52, 0x096D, 0x15}, // map to MIPI Controller 1
	{0x52, 0x094D, 0x1E},
	{0x52, 0x094E, 0x5E}, // map to VC1
	{0x52, 0x094F, 0x00}, // frame start
	{0x52, 0x0950, 0x40},
	{0x52, 0x0951, 0x01},
	{0x52, 0x0952, 0x41},
	// YUV422 8bit, video pipe 2, map FS/FE
	{0x52, 0x098B, 0x07},
	{0x52, 0x09AD, 0x15}, // map to MIPI Controller 1
	{0x52, 0x098D, 0x1E},
	{0x52, 0x098E, 0x9E}, // map to VC2
	{0x52, 0x098F, 0x00},
	{0x52, 0x0990, 0x80},
	{0x52, 0x0991, 0x01},
	{0x52, 0x0992, 0x81},
	// YUV422 8bit, video pipe 3, map FS/FE
	{0x52, 0x09CB, 0x07},
	{0x52, 0x09ED, 0x15}, // map to MIPI Controller 1
	{0x52, 0x09CD, 0x1E},
	{0x52, 0x09CE, 0xDE}, // map to VC3
	{0x52, 0x09CF, 0x00},
	{0x52, 0x09D0, 0xC0},
	{0x52, 0x09D1, 0x01},
	{0x52, 0x09D2, 0xC1},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 15}, // Delay 15ms

	// MIPI PHY Setting
	// Set Des in 2x4 mode
	{0x52, 0x08A0, 0x04}, // CSI output is 2x4
	// Set Lane Mapping for 4-lane port A
	{0x52, 0x08A3, 0xE4}, // Default 4x2 lane mapping
	{0x52, 0x08A4, 0xE4}, // Default 4x2 lane mapping
	// Set 4 lane D-PHY
	{0x52, 0x090A, 0xC0},
	{0x52, 0x094A, 0xC0},
	{0x52, 0x098A, 0xC0},
	{0x52, 0x09CA, 0xC0},
	// Turn on MIPI PHYs
	{0x52, 0x08A2, 0xF0},

	// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	{0x52, 0x1C00, 0xF4},
	{0x52, 0x1D00, 0xF4},
	{0x52, 0x1E00, 0xF4},
	{0x52, 0x1F00, 0xF4},
	// Set Data rate to be 1500Mbps/lane for port A and enable software override
	{0x52, 0x0415, 0x2F},
	{0x52, 0x0418, 0x2F},
	{0x52, 0x041B, 0x2F},
	{0x52, 0x041E, 0x2F},

	// Release reset to DPLL (config_soft_rst_n = 1)
	{0x52, 0x1C00, 0xF5},
	{0x52, 0x1D00, 0xF5},
	{0x52, 0x1E00, 0xF5},
	{0x52, 0x1F00, 0xF5},

	// ------------- I2C translation table update -------------
	// Enable Link A only
	{0x52,0x0006,0xF1}, // Turn on GMSL2 mode for link A
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	//set MAX9295
	{0x80,0x0053,0x10},
	{0x80,0x0057,0x11},
	{0x80,0x005B,0x12}, // change stream ID 2
	{0x80,0x005F,0x13},
	{0x80,0x02BE,0x10}, // camera reset
	{0x80,0x0318,0x5E}, //set DT
	{0x80,0x02D3,0x00},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	{0x80,0x02D3,0x04},
	{0x80,0x02D5,0x07},
	{0x80,0x0000,0x84},  // Change I2C address for this Link A serializer

	// Enable Link B only
	{0x52,0x0006,0xF2}, // Turn on GMSL2 mode for link B
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	//set MAX9295
	{0x80,0x0053,0x10},
	{0x80,0x0057,0x11},
	{0x80,0x005B,0x12}, // change stream ID 2
	{0x80,0x005F,0x13},
	{0x80,0x02BE,0x10}, // camera reset
	{0x80,0x0318,0x5E}, // set DT
	{0x80,0x02D3,0x00},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	{0x80,0x02D3,0x04},
	{0x80,0x02D5,0x07},
	{0x80,0x0000,0x88}, // Change I2C address for this Link A serializer

	// Enable Link C only
	{0x52,0x0006,0xF4}, // Turn on GMSL2 mode for link C
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	//set MAX9295
	{0x80,0x0053,0x10}, 
	{0x80,0x0057,0x11},
	{0x80,0x005B,0x12}, // change stream ID 2
	{0x80,0x005F,0x13},
	{0x80,0x02BE,0x10}, // camera reset
	{0x80,0x0318,0x5E}, // set DT
	{0x80,0x02D3,0x00},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	{0x80,0x02D3,0x04},
	{0x80,0x02D5,0x07},
	{0x80,0x0000,0xC4},  // Change I2C address for this Link A serializer

	// Enable Link D only
	{0x52,0x0006,0xF8},  // Turn on GMSL2 mode for link D
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	//set MAX9295
	{0x80,0x0053,0x10},
	{0x80,0x0057,0x11},
	{0x80,0x005B,0x12}, // change stream ID 2
	{0x80,0x005F,0x13},
	{0x80,0x02BE,0x10}, // camera reset
	{0x80,0x0318,0x5E}, // set DT
	{0x80,0x02D3,0x00},
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms
	{0x80,0x02D3,0x04}, // senyun FrameSync MFP7
	{0x80,0x02D5,0x07}, // senyun FrameSync MFP7
	{0x80,0x0000,0xC8},  // Change I2C address for this Link A serializer

	{0x06, CAM_GMSL_TABLE_WAIT_MS, 50}, // Delay 50ms
	{0x52,0x0006,0xFF}, // Enable all links back
	{0x52,0x0018,0x0F}, // One-shot link reset for all links
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 100}, // Delay 100ms

	// ------------- GMSL2 Sensor Settings -------------
	// PHY 2 copy PHY 0 output
	{0x52,0x08A9,0xC8},//C0
	// PHY 3 copy PHY 1 output
	{0x52,0x08AA,0xC8},// C8/EA

	// FrameSync MFP2
	{0x52,0x0306,0x83},
	{0x52,0x0307,0xA7},

	{0x52,0x033D,0x27},//port B
	{0x52,0x0374,0x27},//port C
	{0x52,0x03AA,0x27},//port D

	// End of Script
	{0x06, CAM_GMSL_TABLE_WAIT_MS, 50}, // Delay 50ms
	// Enable MIPI Output
	{0x52,0x040B,0x02},
	{0x52,0x08A0,0x84},

	{0x00, CAM_GMSL_TABLE_END, 0x00}
};



static struct index_reg_8 cam_gmsl_Single_Dser_Ser[] = {
	{0x80, 0x02be, 0x83}, // hawk1(0x84) max9295D MFP0-- ACCEL interrupt
	{0x80, 0x02bf, 0x11},
	{0x52, 0x030c, 0x04}, // max96712 MTF4---ACCEL1 interrupt
	{0x52, 0x030e, 0x11}, // MTF4

	{0x80, 0x02c7, 0x83}, // hawk1(0x84)max9295D MFP3-- gyro interrupt
	{0x80, 0x02c8, 0x12},
	{0x52, 0x0320, 0x04}, // max96712 MTF10---gyro1 interrupt
	{0x52, 0x0322, 0x12}, // MTF10

	{0x00, CAM_GMSL_TABLE_END, 0x00 }
};

enum {
	CAM_GMSL_MODE_Dser_Ser,
	CAM_GMSL_MODE_Single_Dser_Ser,
	CAM_GMSL_MODE_START_STREAM,
	CAM_GMSL_MODE_STOP_STREAM,
};

static struct index_reg_8 *mode_table[] = {
	[CAM_GMSL_MODE_Dser_Ser] = cam_gmsl_Double_Dser_Ser,
	[CAM_GMSL_MODE_Single_Dser_Ser] = cam_gmsl_Single_Dser_Ser,
	[CAM_GMSL_MODE_START_STREAM] = cam_gmsl_start,
	[CAM_GMSL_MODE_STOP_STREAM] = cam_gmsl_stop,
};

static const int cam_gmsl_30fps[] = {
	30,
};

static const int cam_gmsl_60fps[] = {
	60,
};

static const int cam_gmsl_50fps[] = {
  50,
};

static const struct camera_common_frmfmt cam_gmsl_frmfmt[] = {
	{{1920, 1536}, cam_gmsl_60fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};
static const struct camera_common_frmfmt cam_gmsl_30fps_1920x1536_frmfmt[] = {
	{{1920, 1536}, cam_gmsl_30fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};

static const struct camera_common_frmfmt cam_gmsl_30fps_1920x1280_frmfmt[] = {
	{{1920, 1280}, cam_gmsl_30fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};

static const struct camera_common_frmfmt cam_gmsl_30fps_1920x1200_frmfmt[] = {
	{{1920, 1200}, cam_gmsl_60fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};

static const struct camera_common_frmfmt cam_gmsl_60fps_1920x1536_frmfmt[] = {
	{{1920, 1536}, cam_gmsl_60fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};

static const struct camera_common_frmfmt cam_gmsl_50fps_2560x1440_frmfmt[] = {
	{{2560, 1440}, cam_gmsl_50fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};

static const struct camera_common_frmfmt cam_gmsl_60fps_2592x1944_frmfmt[] = {
  {{2592, 1944}, cam_gmsl_60fps, 1, 0, CAM_GMSL_MODE_Dser_Ser},
};
#endif /* __CAM_GMSL_I2C_TABLES__ */
