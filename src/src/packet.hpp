/**
 * @file packet.hpp
 * @brief header only lilbrary for ilidar packet
 * @author JSon (json@hybo.co)
 * @data 2023-12-28
 * @version 1.11.10
 */

//////////////////////////////////////////////////////////////////////////////////////
//	MIT License(MIT)																//
//																					//
//	Copyright(c) 2022 - Present HYBO Inc.											//
//																					//
//	Permission is hereby granted, free of charge, to any person obtaining a copy	//
//	of this software and associated documentation files(the "Software"), to deal	//
//	in the Software without restriction, including without limitation the rights	//
//	to use, copy, modify, merge, publish, distribute, sublicense, and /or sell		//
//	copies of the Software, and to permit persons to whom the Software is			//
//	furnished to do so, subject to the following conditions :						//
//																					//
//	The above copyright notice and this permission notice shall be included in all	//
//	copies or substantial portions of the Software.									//
//																					//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR		//
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,		//
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE		//
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER			//
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,	//
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE	//
//	SOFTWARE.																		//
//////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdio.h>

namespace iTFS {
	namespace packet {
		constexpr uint8_t	ilidar_pack_ver[3] = { 10, 11, 1 };

		/*
		iTFS PACKET STRUCTURE DESCRIPTIONS

		idx	field	SIZE	VALUE
		0	STX0	1		0xA5
		1	STX1	1		0x5A
		2	ID		2		0x0000 for img_data, ...
		4	LEN		2		N
		6	PAYLOAD	N 	
		N+6	ETX0	1		0xA5
		N+7	ETX1	1		0x5A		 	
		TOTAL SIZE	N+8

		See ilidar.cpp to use this packet with encode and decode functions
		*/

		constexpr uint8_t	stx0				= 0xA5;
		constexpr uint8_t	stx1				= 0x5A;
		constexpr int		stx_len				= 2;
		constexpr int		id_len				= 2;
		constexpr int		len_len				= 2;
		constexpr int		header_len			= 6;
		constexpr uint8_t	etx0				= 0xA5;
		constexpr uint8_t	etx1				= 0x5A;
		constexpr int		overheader_len		= 8;

		constexpr uint16_t	img_data_id			= 0x0000;
		constexpr uint16_t	img_data_len		= 1282;

		constexpr uint16_t	status_id			= 0x0010;
		constexpr uint16_t	status_len			= 28;

		constexpr uint16_t	status_full_id		= 0x0011;
		constexpr uint16_t	status_full_len		= 312;

		constexpr uint16_t	info_id				= 0x0020;
		constexpr uint16_t	info_len			= 110;

		constexpr uint16_t	cmd_id				= 0x0030;
		constexpr uint16_t	cmd_len				= 4;

		constexpr uint16_t	cmd_sync			= 0x0000;	// BROADCAST only
		constexpr uint16_t	cmd_select			= 0x0001;	// BROADCAST only

		constexpr uint16_t	cmd_measure			= 0x0100;
		constexpr uint16_t	cmd_pause			= 0x0101;
		constexpr uint16_t	cmd_reboot			= 0x0102;
		constexpr uint16_t	cmd_store			= 0x0103;

		constexpr uint16_t	cmd_reset_factory	= 0x0200;	// UNICAST only, cmd_msg = sensor_sn
		constexpr uint16_t	cmd_reset_log		= 0x0201;	// UNICAST only, cmd_msg = sensor_sn

		constexpr uint16_t	cmd_read_info		= 0x0300;
		constexpr uint16_t	cmd_read_log		= 0x0301;	// UNICAST only, cmd_msg = sensor_sn
		constexpr uint16_t	cmd_read_depth		= 0x0302;	// UNICAST only, cmd_msg = sensor_sn
		constexpr uint16_t	cmd_read_tf			= 0x0303;	// UNICAST only, cmd_msg = sensor_sn

		constexpr uint16_t	cmd_redirect		= 0x0400;

		constexpr uint16_t	cmd_lock			= 0x0500;	// UNICAST only, cmd_msg = sensor_sn
		constexpr uint16_t	cmd_unlock			= 0x0501;	// UNICAST only, cmd_msg = sensor_sn

		typedef struct {
			uint8_t		row;
			uint8_t		mframe;
			uint16_t	data[640];
		}img_data_packet_t;

		typedef struct {
			uint8_t		capture_mode;				// Mode, 0 = GRAY, 1 = MODE1(NB), 2 = MODE2(VB), 3 = MODE3(HV)
			uint8_t		capture_frame;				// Frame number, repeats 0 ~ 63
			uint16_t	sensor_sn;					// Serial number
			uint64_t	sensor_time_th;				// Sensor time in [ms]
			uint16_t	sensor_time_tl;				// Sensor time in [us]
			uint16_t	sensor_frame_status;		// Sensor status flag for depth image
			int16_t		sensor_temp_rx;				// Sensor RX temperature in [C��]
			int16_t		sensor_temp_core;			// Sensor core temperature in [C��]
			int16_t		sensor_vcsel_level;			// Sensor VCSEL voltage
			int16_t		sensor_power_level;			// Sensor internal power voltage
			uint32_t	sensor_warning;				// Sensor warning flag
		}status_t;

		typedef struct {
			uint8_t		capture_mode;				// Mode, 0 = GRAY, 1 = MODE1(NB), 2 = MODE2(VB), 3 = MODE3(HV)
			uint8_t		capture_frame;				// Frame number, repeats 0 ~ 63
			uint16_t	sensor_sn;					// Serial number
			uint64_t	sensor_time_th;				// Sensor time in [ms]
			uint16_t	sensor_time_tl;				// Sensor time in [us]
			uint16_t	sensor_frame_status;		// Sensor status flag for depth image
			int16_t		sensor_temp_rx;				// Sensor RX temperature in [C��]
			int16_t		sensor_temp_core;			// Sensor core temperature in [C��]
			int16_t		sensor_temp[4];				// Sensor housing temperature in [C��]
			int16_t		sensor_vcsel_level;			// Sensor VCSEL voltage
			int16_t		sensor_vcsel_on[4][16];		// Sensor VCSEL dynamic voltage for debugging
			int16_t		sensor_power_level;			// Sensor internal power voltage
			int16_t		sensor_power_on[4][16];		// Sensor internal power dynamic voltage for debugging
			int16_t		sensor_level[10];			// Sensor internal voltage for debugging
			uint32_t	sensor_warning;				// Sensor warning flag
		}status_full_t;

		typedef struct {							// [RW]
			uint16_t	sensor_sn;					// [R-] Sensor serial number
			uint8_t		sensor_hw_id[30];			// [R-] Sensor HW ID
			uint8_t		sensor_fw_ver[3];			// [R-] Sensor firmware version
			char		sensor_fw_date[12];			// [R-] Sensor firmware date
			char		sensor_fw_time[9];			// [R-] Sensor firmware time
			uint32_t	sensor_calib_id;			// [R-] Sensor calibration ID
			uint8_t		capture_mode;				// [RW] Capture mode, 0 = GRAY, 1 = MODE1(NB), 2 = MODE2(VB), 3 = MODE3(HV)
			uint8_t		capture_row;				// [RW] Capture row number, 4 <= ROW <= 160, default = 160, only the value in multiples of 4
			uint16_t	capture_period;				// [RW] Capture period, 50 <= period <= 1000, default = 100
			uint16_t	capture_shutter[5];			// [RW] Capture shutter integration time, 2 <= shutter <= 600, 2 <= gray <= 10000, default = [400 40 4 4 8000]
			uint16_t	capture_limit[2];			// [RW] Capture intensity limit, 0 <= limit <= 500, default = 200
			uint8_t		data_output;				// [RW] Data output flag
			uint32_t	data_baud;					// [RW] UART baudrate, 9600 <= baud <= 6000000, default = 115200
			uint8_t		data_sensor_ip[4];			// [RW] Sensor IP
			uint8_t		data_dest_ip[4];			// [RW] Destination IP
			uint8_t		data_subnet[4];				// [RW] Subnet Mask
			uint8_t		data_gateway[4];			// [RW] Gateway
			uint16_t	data_port;					// [RW] Data output port number
			uint8_t		sync;						// [RW] flag, default = 0
			uint16_t	sync_delay;					// [RW] delay <= period, default = 0
			uint8_t		arb;						// [RW] flag, default = 0
			uint32_t	arb_timeout;				// [RW] timeout <= 60 * 60 * 1000, default = 5 * 60 * 1000
			uint8_t		lock;						// [RW] flag, configuration locker, default = 0
		}info_t;

		typedef struct {
			uint16_t	cmd_id;						// Command ID
			uint16_t	cmd_msg;					// Extended message for some commands
		}cmd_t;

		constexpr int		capture_mode_bin_pos		= 0;
		constexpr uint8_t	capture_mode_bin_mask		= (0x03 << capture_mode_bin_pos);
		constexpr uint8_t	capture_mode_bin_mode0		= (0 << capture_mode_bin_pos);
		constexpr uint8_t	capture_mode_bin_mode1		= (1 << capture_mode_bin_pos);
		constexpr uint8_t	capture_mode_bin_mode2		= (2 << capture_mode_bin_pos);
		constexpr uint8_t	capture_mode_bin_mode3		= (3 << capture_mode_bin_pos);
		//constexpr int		capture_mode_filter_pos		= 2;
		//constexpr uint8_t	capture_mode_filter_mask	= (0x03 << capture_mode_filter_pos);


		constexpr int		data_output_depth_pos		= 0;
		constexpr uint8_t	data_output_depth_mask		= (0x01 << data_output_depth_pos);
		constexpr uint8_t	data_output_depth_off		= (0 << data_output_depth_pos);
		constexpr uint8_t	data_output_depth_on		= (1 << data_output_depth_pos);

		constexpr int		data_output_intensity_pos	= 1;
		constexpr uint8_t	data_output_intensity_mask	= (0x01 << data_output_intensity_pos);
		constexpr uint8_t	data_output_intensity_off	= (0 << data_output_intensity_pos);
		constexpr uint8_t	data_output_intensity_on	= (1 << data_output_intensity_pos);

		constexpr int		data_output_status_pos		= 2;
		constexpr uint8_t	data_output_status_mask		= (0x01 << data_output_status_pos);
		constexpr uint8_t	data_output_status_normal	= (0 << data_output_status_pos);
		constexpr uint8_t	data_output_status_full		= (1 << data_output_status_pos);
		

		constexpr int		sync_mode_pos				= 0;
		constexpr uint8_t	sync_mode_mask				= (0x03 << sync_mode_pos);
		constexpr uint8_t	sync_mode_off				= (0 << sync_mode_pos);
		constexpr uint8_t	sync_mode_udp				= (1 << sync_mode_pos);
		constexpr uint8_t	sync_mode_trigger			= (2 << sync_mode_pos);
		//constexpr uint8_t	sync_mode_uart				= (3 << sync_mode_pos);

		constexpr int		sync_strobe_pos				= 2;
		constexpr uint8_t	sync_strobe_mask			= (0x03 << sync_strobe_pos);
		constexpr uint8_t	sync_strobe_off				= (0 << sync_strobe_pos);
		constexpr uint8_t	sync_strobe_on				= (1 << sync_strobe_pos);
		//constexpr uint8_t	sync_strobe_uart			= (3 << sync_strobe_pos);


		constexpr int		arb_mode_pos				= 0;
		constexpr uint8_t	arb_mode_mask				= (0x03 << arb_mode_pos);
		constexpr uint8_t	arb_mode_off				= (0 << arb_mode_pos);
		constexpr uint8_t	arb_mode_udp				= (1 << arb_mode_pos);
		constexpr uint8_t	arb_mode_trigger			= (2 << arb_mode_pos);
		//constexpr uint8_t	arb_mode_uart				= (3 << arb_mode_pos);


		constexpr uint8_t	lock_mode_unlocked			= 0x00;
		constexpr uint8_t	lock_mode_locked			= 0xA5;
		
		static void decode_status(uint8_t* src, status_t* dst) {
			dst->capture_mode = src[0];
			dst->capture_frame = src[1];

			dst->sensor_sn = (src[3] << 8) | src[2];
			uint64_t th = src[11];
			for (int _i = 0; _i < 7; _i++) {
				th <<= 8;
				th |= src[10 - _i];
			}
			dst->sensor_time_th = th;
			dst->sensor_time_tl = (src[13] << 8) | src[12];
			dst->sensor_frame_status = (src[15] << 8) | src[14];
			dst->sensor_temp_rx = (src[17] << 8) | src[16];
			dst->sensor_temp_core = (src[19] << 8) | src[18];
			dst->sensor_vcsel_level = (src[21] << 8) | src[20];
			dst->sensor_power_level = (src[23] << 8) | src[22];
			dst->sensor_warning = (src[27] << 24) | (src[26] << 16) | (src[25] << 8) | src[24];
		}

		static void decode_status_full(uint8_t* src, status_full_t* dst) {
			dst->capture_mode = src[0];
			dst->capture_frame = src[1];

			dst->sensor_sn = (src[3] << 8) | src[2];
			uint64_t th = src[11];
			for (int _i = 0; _i < 7; _i++) {
				th <<= 8;
				th |= src[10 - _i];
			}
			dst->sensor_time_th = th;
			dst->sensor_time_tl = (src[13] << 8) | src[12];
			dst->sensor_frame_status = (src[15] << 8) | src[14];
			dst->sensor_temp_rx = (src[17] << 8) | src[16];
			dst->sensor_temp_core = (src[19] << 8) | src[18];
			for (int _i = 0; _i < 4; _i++) { dst->sensor_temp[_i] = (src[21 + 2 * _i] << 8) | src[20 + 2 * _i]; }
			dst->sensor_vcsel_level = (src[29] << 8) | src[28];
			for (int _i = 0; _i < 16; _i++) { dst->sensor_vcsel_on[0][_i] = (src[31 + 2 * _i] << 8) | src[30 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_vcsel_on[1][_i] = (src[63 + 2 * _i] << 8) | src[62 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_vcsel_on[2][_i] = (src[95 + 2 * _i] << 8) | src[94 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_vcsel_on[3][_i] = (src[127 + 2 * _i] << 8) | src[126 + 2 * _i]; }
			dst->sensor_power_level = (src[159] << 8) | src[158];
			for (int _i = 0; _i < 16; _i++) { dst->sensor_power_on[0][_i] = (src[161 + 2 * _i] << 8) | src[160 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_power_on[1][_i] = (src[193 + 2 * _i] << 8) | src[192 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_power_on[2][_i] = (src[225 + 2 * _i] << 8) | src[224 + 2 * _i]; }
			for (int _i = 0; _i < 16; _i++) { dst->sensor_power_on[3][_i] = (src[257 + 2 * _i] << 8) | src[256 + 2 * _i]; }
			for (int _i = 0; _i < 10; _i++) { dst->sensor_level[_i] = (src[289 + 2 * _i] << 8) | src[288 + 2 * _i]; }
			dst->sensor_warning = (src[311] << 24) | (src[310] << 16) | (src[309] << 8) | src[308];
		}

		static void encode_info(info_t* src, uint8_t* dst) {
			dst[0] = src->sensor_sn % 256;
			dst[1] = src->sensor_sn / 256;

			for (int _i = 0; _i < (30 + 3 + 12 + 9); _i++) { dst[2 + _i] = 0; }	// Read only

			dst[60] = src->capture_mode;
			dst[61] = src->capture_row;
			dst[62] = (src->capture_period >> 0) & 0xFF;
			dst[63] = (src->capture_period >> 8) & 0xFF;
			dst[64] = (src->capture_shutter[0] >> 0) & 0xFF;
			dst[65] = (src->capture_shutter[0] >> 8) & 0xFF;
			dst[66] = (src->capture_shutter[1] >> 0) & 0xFF;
			dst[67] = (src->capture_shutter[1] >> 8) & 0xFF;
			dst[68] = (src->capture_shutter[2] >> 0) & 0xFF;
			dst[69] = (src->capture_shutter[2] >> 8) & 0xFF;
			dst[70] = (src->capture_shutter[3] >> 0) & 0xFF;
			dst[71] = (src->capture_shutter[3] >> 8) & 0xFF;
			dst[72] = (src->capture_shutter[4] >> 0) & 0xFF;
			dst[73] = (src->capture_shutter[4] >> 8) & 0xFF;
			dst[74] = (src->capture_limit[0] >> 0) & 0xFF;
			dst[75] = (src->capture_limit[0] >> 8) & 0xFF;
			dst[76] = (src->capture_limit[1] >> 0) & 0xFF;
			dst[77] = (src->capture_limit[1] >> 8) & 0xFF;

			dst[78] = src->data_output;

			dst[79] = src->arb;

			dst[80] = (src->data_baud >> 0) & 0xFF;
			dst[81] = (src->data_baud >> 8) & 0xFF;
			dst[82] = (src->data_baud >> 16) & 0xFF;
			dst[83] = (src->data_baud >> 24) & 0xFF;
			dst[84] = src->data_sensor_ip[0];
			dst[85] = src->data_sensor_ip[1];
			dst[86] = src->data_sensor_ip[2];
			dst[87] = src->data_sensor_ip[3];
			dst[88] = src->data_dest_ip[0];
			dst[89] = src->data_dest_ip[1];
			dst[90] = src->data_dest_ip[2];
			dst[91] = src->data_dest_ip[3];
			dst[92] = src->data_subnet[0];
			dst[93] = src->data_subnet[1];
			dst[94] = src->data_subnet[2];
			dst[95] = src->data_subnet[3];
			dst[96] = src->data_gateway[0];
			dst[97] = src->data_gateway[1];
			dst[98] = src->data_gateway[2];
			dst[99] = src->data_gateway[3];
			dst[100] = (src->data_port >> 0) & 0xFF;
			dst[101] = (src->data_port >> 8) & 0xFF;

			dst[102] = src->sync;
			dst[103] = 0;	// This flag will not be written with info packet

			dst[104] = (src->sync_delay >> 0) & 0xFF;
			dst[105] = (src->sync_delay >> 8) & 0xFF;

			dst[106] = (src->arb_timeout >> 0) & 0xFF;
			dst[107] = (src->arb_timeout >> 8) & 0xFF;
			dst[108] = (src->arb_timeout >> 16) & 0xFF;
			dst[109] = (src->arb_timeout >> 24) & 0xFF;
		}

		static void decode_info(uint8_t* src, info_t* dst) {
			uint16_t	sensor_sn = (src[1] << 8) | src[0];
			uint8_t		sensor_hw_id[30] = { src[2], src[3], src[4], src[5], src[6], src[7], src[8], src[9],
											src[10], src[11], src[12], src[13], src[14], src[15], src[16], src[17],
											src[18], src[19], src[20], src[21], src[22], src[23], src[24], src[25],
											src[26], src[27], src[28], src[29], src[30], src[31] };
			uint8_t		sensor_fw_ver[3] = { src[32], src[33], src[34] };
			uint8_t		sensor_fw_date[12] = { src[35], src[36], src[37], src[38],
											src[39], src[40], src[41], src[42],
											src[43], src[44], src[45], src[46] };
			uint8_t		sensor_fw_time[9] = { src[47], src[48], src[49], src[50],
											src[51], src[52], src[53], src[54], src[55] };
			uint32_t	sensor_calib_id = (src[59] << 24) | (src[58] << 16) | (src[57] << 8) | src[56];

			uint8_t		capture_mode = src[60];
			uint8_t		capture_row = src[61];
			uint16_t	capture_period = (src[63] << 8) | src[62];
			uint16_t	capture_shutter[5] = { (uint16_t)((src[65] << 8) | src[64]),
											(uint16_t)((src[67] << 8) | src[66]),
											(uint16_t)((src[69] << 8) | src[68]),
											(uint16_t)((src[71] << 8) | src[70]),
											(uint16_t)((src[73] << 8) | src[72]) };
			uint16_t	capture_limit[2] = { (uint16_t)((src[75] << 8) | src[74]),
											(uint16_t)((src[77] << 8) | src[76]) };

			uint8_t		data_output = src[78];

			uint8_t		arb = src[79];

			uint32_t	data_baud = (src[83] << 24) | (src[82] << 16) | (src[81] << 8) | src[80];
			uint8_t		data_sensor_ip[4] = { src[84], src[85], src[86], src[87] };
			uint8_t		data_dest_ip[4] = { src[88], src[89], src[90], src[91] };
			uint8_t		data_subnet[4] = { src[92], src[93], src[94], src[95] };
			uint8_t		data_gateway[4] = { src[96], src[97], src[98], src[99] };
			uint16_t	data_port = (src[101] << 8) | src[100];

			uint8_t		sync = src[102];

			uint8_t		lock = src[103];

			uint16_t	sync_delay = (src[105] << 8) | src[104];

			uint32_t	arb_timeout = (src[109] << 24) | (src[108] << 16) | (src[107] << 8) | src[106];


			dst->sensor_sn = sensor_sn;
			for (int _i = 0; _i < 30; _i++) { dst->sensor_hw_id[_i] = sensor_hw_id[_i]; }
			for (int _i = 0; _i < 3; _i++) { dst->sensor_fw_ver[_i] = sensor_fw_ver[_i]; }
			for (int _i = 0; _i < 12; _i++) { dst->sensor_fw_date[_i] = sensor_fw_date[_i]; }
			for (int _i = 0; _i < 9; _i++) { dst->sensor_fw_time[_i] = sensor_fw_time[_i]; }
			dst->sensor_calib_id = sensor_calib_id;

			dst->capture_mode = capture_mode;
			dst->capture_row = capture_row;
			dst->capture_period = capture_period;
			dst->capture_shutter[0] = capture_shutter[0];
			dst->capture_shutter[1] = capture_shutter[1];
			dst->capture_shutter[2] = capture_shutter[2];
			dst->capture_shutter[3] = capture_shutter[3];
			dst->capture_shutter[4] = capture_shutter[4];
			dst->capture_limit[0] = capture_limit[0];
			dst->capture_limit[1] = capture_limit[1];

			dst->data_output = data_output;
			dst->data_baud = data_baud;
			dst->data_sensor_ip[0] = data_sensor_ip[0];
			dst->data_sensor_ip[1] = data_sensor_ip[1];
			dst->data_sensor_ip[2] = data_sensor_ip[2];
			dst->data_sensor_ip[3] = data_sensor_ip[3];
			dst->data_dest_ip[0] = data_dest_ip[0];
			dst->data_dest_ip[1] = data_dest_ip[1];
			dst->data_dest_ip[2] = data_dest_ip[2];
			dst->data_dest_ip[3] = data_dest_ip[3];
			dst->data_subnet[0] = data_subnet[0];
			dst->data_subnet[1] = data_subnet[1];
			dst->data_subnet[2] = data_subnet[2];
			dst->data_subnet[3] = data_subnet[3];
			dst->data_gateway[0] = data_gateway[0];
			dst->data_gateway[1] = data_gateway[1];
			dst->data_gateway[2] = data_gateway[2];
			dst->data_gateway[3] = data_gateway[3];
			dst->data_port = data_port;

			dst->sync = sync;
			dst->sync_delay = sync_delay;

			dst->arb = arb;
			dst->arb_timeout = arb_timeout;

			dst->lock = lock;
		}

		static void encode_cmd(cmd_t* src, uint8_t* dst) {
			dst[0] = (src->cmd_id >> 0) & 0xFF;
			dst[1] = (src->cmd_id >> 8) & 0xFF;
			dst[2] = (src->cmd_msg >> 0) & 0xFF;
			dst[3] = (src->cmd_msg >> 8) & 0xFF;
		}

		/*
			// About iLidar sensor time
			uint64_t	sensor_time_th;	// [ms] Sensor time in ms
			uint16_t	sensor_time_tl;	// [us] Sensor time for sub-ms

			// So, we can get the sensor time in s
			double		sensor_time = ((double)sensor_time_th + (((double)sensor_time_tl) * 0.001)) * 0.001;

			// When we need to use the precise time, we can use time in us with integer
			uint64_t	sensor_time_us = (uint64_t)(1000 * sensor_time_th) + (uint64_t)sensor_time_tl;
		*/

		static double get_sensor_time(uint64_t sensor_time_th, uint16_t sensor_time_tl) {
			return (((double)sensor_time_th + ((double)sensor_time_tl) * 0.001) * 0.001);
		}

		static double get_sensor_time(status_full_t * status) {
			return (((double)status->sensor_time_th + ((double)status->sensor_time_tl) * 0.001) * 0.001);
		}

		static double get_sensor_time(status_t* status) {
			return (((double)status->sensor_time_th + ((double)status->sensor_time_tl) * 0.001) * 0.001);
		}

		static uint64_t get_sensor_time_in_us(uint64_t sensor_time_th, uint16_t sensor_time_tl) {
			return ((uint64_t)(1000 * sensor_time_th) + (uint64_t)sensor_time_tl);
		}

		static uint64_t get_sensor_time_in_us(status_full_t* status) {
			return ((uint64_t)(1000 * status->sensor_time_th) + (uint64_t)status->sensor_time_tl);
		}

		static uint64_t get_sensor_time_in_us(status_t* status) {
			return ((uint64_t)(1000 * status->sensor_time_th) + (uint64_t)status->sensor_time_tl);
		}
	}
}
