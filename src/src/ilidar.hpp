/**
 * @file ilidar.cpp
 * @brief ilidar basic class header
 * @see ilidar.hpp
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

#include <iostream>
#include <fstream>
#include <thread>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <string.h>

#if defined(_WIN32) || defined( _WIN64)
 // Windows headers
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib")
#include <windows.h>
#define SOCKET_TYPE	SOCKET		
#define SOCKET_LEN	int
#else
// Non-windows headers
/* Assume that any non-Windows platform uses POSIX-style sockets instead */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET_TYPE	int	
#define SOCKET_LEN	socklen_t
#define closesocket	close
#endif

#include "packet.hpp"

namespace iTFS {
	constexpr uint8_t	ilidar_lib_ver[3] = { 10, 11, 1 };

	constexpr int	max_col			= 320;
	constexpr int	max_row			= 160;
	constexpr int	gray_row		= 240;
	constexpr int	max_device		= 1;

	/*
	UDP PORT DESCRIPTIONS
	DATA	LiDAR [4905] -> [7256] USER
	CONFIG	USER  [7257] -> [4906] LiDAR
	*/
	constexpr uint16_t	lidar_data_port		= 4905;
	constexpr uint16_t	lidar_config_port	= 4906;
	constexpr uint16_t	user_data_port		= 7256;
	constexpr uint16_t	user_config_port	= 7257;

	constexpr float temp_comp	= 500.0f;
	constexpr float temp_scale	= 0.25f * temp_comp;
	constexpr float temp_bias	= 30.0f * temp_comp;

	typedef struct {
		uint8_t		mode;
		uint8_t		frame;

		int			capture_row;

		uint16_t	img[2 * max_row][max_col];
	}img_t;

	typedef struct {
		int			idx;

		uint8_t		ip[4];
		uint16_t	port;
		
		int			recvCount;
		img_t		data;

		packet::status_t		status;
		packet::status_full_t	status_full;
		packet::info_t			info;
	}device_t;

	typedef void(*callback_handler)(device_t*);

	class LiDAR {
	public:
		LiDAR(
			callback_handler	img_data_handler_func,
			callback_handler	status_packet_handler_func,
			callback_handler	info_packet_handler_func,
			uint8_t				*brodcast_ip = NULL,
			uint16_t			listening_port = user_data_port);

		~LiDAR();

		bool		Ready()		{ return is_ready; }
		void		Try_exit()	{ this->exit = true; }
		void		Join()		{ this->read_thread.join(); this->send_thread.join(); }

		int			Send_cmd(int device_idx, packet::cmd_t* cmd);
		int			Send_cmd_to_all(packet::cmd_t* cmd);
		int			Send_config(int device_idx, packet::info_t* config);
		void		Set_broadcast_ip(uint8_t *ip);

		int			device_cnt;
		device_t	device[max_device];

	private:
		bool		is_ready;
		bool		exit;

		std::thread	read_thread;
		std::thread	send_thread;

		bool		is_read_ready;
		bool		is_send_ready;

		void		Read_run();
		void		Send_run();

		uint16_t	listening_port;

		std::mutex	send_mutex;
		SOCKET_TYPE	send_sockfd;
		uint8_t		broadcast_ip[4];

		int			Get_device_num(uint8_t ip[4], uint16_t port);
		int			Set_device_num(uint8_t ip[4], uint16_t port);

		callback_handler	img_data_handler_func;
		callback_handler	status_packet_handler_func;
		callback_handler	info_packet_handler_func;

		void		Copy_status(device_t *device);
	};
}
