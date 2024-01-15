/**
 * @file ilidar-ros.cpp
 * @brief ilidar ros example source file
 * @author JSon (json@hybo.co)
 * @data 2023-12-28
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

// Include ROS SYSTEM
#include <ros/ros.h>

// Include file IO
#include <iostream>

// Include PCL for ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Include ROS TOPICS
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

// Include General libraries
#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

// Include ilidar library
#include "src/ilidar.hpp"

// Data place holder
static cv::Mat					lidar_img_data[iTFS::max_device];

// Synchronization variables
static std::condition_variable	lidar_cv;
static std::mutex				lidar_cv_mutex;
static std::queue<int>			lidar_q;

// Basic lidar data handler function
static void lidar_data_handler(iTFS::device_t *device) {
	// Print message example
	// printf("[MESSAGE] iTFS::LiDAR image  | D# %d  M %d  F# %2d  %d.%d.%d.%d:%d\n",
	// 	device->idx, device->data.mode, device->data.frame,
	// 	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);

	// Deep-copy depth image data
	memcpy((void *)lidar_img_data[device->idx].data, (const void *)device->data.img, sizeof(device->data.img));

	// Notify the reception to the main thread
	int idx = device->idx;
	std::lock_guard<std::mutex> lk(lidar_cv_mutex);
	lidar_q.push(idx);
	lidar_cv.notify_one();
}

// Basic lidar status packet handler function
static void status_packet_handler(iTFS::device_t* device) {
	// Print message example
	// printf("[MESSAGE] iTFS::LiDAR status | D# %d  M %d  F# %2d  T %.3f  %d.%d.%d.%d:%d\n",
	// 	device->idx, device->status.capture_mode, device->status.capture_frame, iTFS::packet::get_sensor_time(&device->status),
	// 	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);
}

// Basic lidar info packet handler function
static void info_packet_handler(iTFS::device_t* device) {
	// Print message example
	printf("[MESSAGE] iTFS::LiDAR info   | D# %d  lock %d\n",
		device->idx, device->info.lock);

	printf("\tSN #%d mode %d, rows %d, period %d\n",
		device->info.sensor_sn,
		device->info.capture_mode,
		device->info.capture_row,
		device->info.capture_period);

	printf("\tshutter [ %d, %d, %d, %d, %d ]\n",
		device->info.capture_shutter[0],
		device->info.capture_shutter[1],
		device->info.capture_shutter[2],
		device->info.capture_shutter[3],
		device->info.capture_shutter[4]);

	printf("\tlimit [ %d, %d ]\n",
		device->info.capture_limit[0],
		device->info.capture_limit[1]);

	printf("\tip   %d.%d.%d.%d\n",
		device->info.data_sensor_ip[0],
		device->info.data_sensor_ip[1],
		device->info.data_sensor_ip[2],
		device->info.data_sensor_ip[3]);

	printf("\tdest %d.%d.%d.%d:%d\n",
		device->info.data_dest_ip[0],
		device->info.data_dest_ip[1],
		device->info.data_dest_ip[2],
		device->info.data_dest_ip[3],
		device->info.data_port);

	printf("\tsync %d, syncBase %d autoReboot %d, autoRebootTick %d\n",
		device->info.sync,
		device->info.sync_delay,
		device->info.arb,
		device->info.arb_timeout);

	printf("\tFW version: V%d.%d.%d - ",
		device->info.sensor_fw_ver[2],
		device->info.sensor_fw_ver[1],
		device->info.sensor_fw_ver[0]);
	printf((const char*)device->info.sensor_fw_time);
	printf(" ");
	printf((const char*)device->info.sensor_fw_date);
	printf("\n");
}

// Read transformation vectors
static float direction[240][320][3];

// Read transformation vectors from intrinsic calibration results
static void read_mapping_file(std::string file) {
	FILE *fp = fopen((const char *)file.c_str(), "rb");
    fread(direction, 1, sizeof(direction), fp);
    fclose(fp);
}

// Get the depth vector of the pixel
#define OFFSET_X ((int)0)
#define OFFSET_Y ((int)0)
static inline pcl::PointXYZ get_direction_vector(int u, int v) {
    int u_idx = u + OFFSET_X;
    int v_idx = v + OFFSET_Y + (240 - iTFS::max_row) / 2;
    return pcl::PointXYZ(direction[v_idx][u_idx][0], direction[v_idx][u_idx][1], direction[v_idx][u_idx][2]);
}

// Helloworld example starts here
int main(int argc, char* argv[]) {
    // Initialize this ros node as ilidar-roscore
    ros::init(argc, argv, "ilidar-roscore");

    // Initialize the node handle for the node
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Initialize the image transporter for the depth and the intensity images
    image_transport::ImageTransport it(nh);

    /************************************************** ROS VARIABLES **************************************************/
    // LiDAR post-processing
    bool	edge_rejection;
    double	edge_threshold;
    nh_private.param("edge_rejection", edge_rejection, true);
    nh_private.param("edge_threshold", edge_threshold, 0.20);

    // Colormap
	bool	colormap;
    int		colormap_type;
    nh_private.param("colormap", colormap, true);
	nh_private.param("colormap_type", colormap_type, 2);

    int		depth_cmax;
    nh_private.param("depth_cmax", depth_cmax, 12000);

    int		intensity_cmax;
    nh_private.param("intensity_cmax", intensity_cmax, 4 * 4096);

    // Depth cut
    int		depth_min, depth_max;
    nh_private.param("depth_min", depth_min, 500);
    nh_private.param("depth_max", depth_max, 12000);

    // Calibration file
    std::string mapping_file;
    nh_private.param("mapping_file", mapping_file, std::string("/home/intrinsic.dat"));

    // Frames and topics
    std::string frame_id;
    std::string gray_name, depth_name, intensity_name, points_name;
    nh_private.param("frame_id", frame_id, std::string("local"));
    nh_private.param("depth_name", depth_name, std::string("/ilidar/depth"));
    nh_private.param("intensity_name", intensity_name, std::string("/ilidar/intensity"));
    nh_private.param("points_name", points_name, std::string("/ilidar/points"));
    /************************************************** ROS VARIABLES **************************************************/

    // Create the publishers
    image_transport::Publisher pub_2D_depth = it.advertise(depth_name, 1);
    image_transport::Publisher pub_2D_intensity = it.advertise(intensity_name, 1);
    ros::Publisher pub_3D_points = nh.advertise<sensor_msgs::PointCloud2>(points_name, 1);

    // Create the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_scan = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 lidar_scan_message;

	// Initialize cv::Mat
	for (int _i = 0; _i < iTFS::max_device; _i++) {
		lidar_img_data[_i] = cv::Mat::zeros(2 * iTFS::max_row, iTFS::max_col, CV_16SC1);
	}

    // Create the depth and intensity images
	cv::Mat					cv_depth_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_16SC1);
	cv::Mat					cv_depth_comp_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_8UC1);
	cv::Mat					cv_depth_color_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_8UC3);
    sensor_msgs::ImagePtr	ptr_depth_image;

	cv::Mat					cv_intensity_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_16SC1);
	cv::Mat					cv_intensity_comp_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_8UC1);
	cv::Mat					cv_intensity_color_image = cv::Mat::zeros(iTFS::max_row, iTFS::max_col, CV_8UC3);
    sensor_msgs::ImagePtr	ptr_intensity_image;

    // Set point cloud
    lidar_scan.get()->header.frame_id = frame_id;
    lidar_scan.get()->is_dense   	= false;
    lidar_scan.get()->width      	= iTFS::max_col;
    lidar_scan.get()->height     	= iTFS::max_row;
    lidar_scan.get()->points.resize(lidar_scan.get()->width * lidar_scan.get()->height);

	// Read mapping file
	read_mapping_file(mapping_file);

	// Create iTFS LiDAR class
	iTFS::LiDAR* lidar;
	lidar = new iTFS::LiDAR(lidar_data_handler,
							status_packet_handler,
							info_packet_handler,
							iTFS::user_data_port);

	// Check the sensor driver is ready
	while (lidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
	printf("[MESSAGE] iTFS::LiDAR is ready.\n");

	/* Main loop starts here */
	int recv_device_idx = 0;
	while (ros::ok()) {
		// Wait for new data
		std::unique_lock<std::mutex> lk(lidar_cv_mutex);
		lidar_cv.wait(lk, []{ return !lidar_q.empty(); });
		recv_device_idx = lidar_q.front();
		lidar_q.pop();

		// Check the main loop underrun
		if (!lidar_q.empty()) {
			/* The main loop is slower than data reception handler */
			printf("[WARNING] iTFS::LiDAR The main loop seems to be slower than the LiDAR data reception handler.\n");

			// Flush the queue
			while (!lidar_q.empty()) { recv_device_idx = lidar_q.front(); lidar_q.pop(); }
		}

		/* Main processing starts here */
		// Check the mode
		if ((lidar->device[recv_device_idx].info.capture_mode & iTFS::packet::capture_mode_bin_mask) != iTFS::packet::capture_mode_bin_mode0) {
			// Copy depth and intensity 
			cv_depth_image = lidar_img_data[recv_device_idx](cv::Rect(0, 0, iTFS::max_col, iTFS::max_row));
			cv_intensity_image = lidar_img_data[recv_device_idx](cv::Rect(0, iTFS::max_row, iTFS::max_col, iTFS::max_row));

			// Edge rejection
			if (edge_rejection == true) {
				cv::Mat	mask, dx, dy;
				cv::Sobel(cv_depth_image, dx, -1, 1, 0);
				cv::Sobel(cv_depth_image, dy, -1, 0, 1);
				mask = cv::abs(dx) + cv::abs(dy);
				cv::threshold(mask, mask, 6 * edge_threshold, 1, cv::THRESH_BINARY_INV);
				cv_depth_image = cv_depth_image.mul(mask);
			}

			// Reconstruction
			float distance_min = static_cast<float>(depth_min) * 0.001f;
			float distance_max = static_cast<float>(depth_max) * 0.001f;
			for (int _v = 0; _v < iTFS::max_row; _v++) {
				uint16_t* distancePtr = cv_depth_image.ptr<uint16_t>(_v);
				uint16_t* intensityPtr = cv_intensity_image.ptr<uint16_t>(_v);
				for (int _u = 0; _u < iTFS::max_col; _u++) {
					float distance = static_cast<float>(distancePtr[_u]) * 0.001f;	// Convert mm to m
					float intensity = static_cast<float>(intensityPtr[_u]);	

					// Get point
					if (distance < distance_min || distance > distance_max) {
						lidar_scan.get()->points[_v * iTFS::max_col + _u].x = 0;
						lidar_scan.get()->points[_v * iTFS::max_col + _u].y = 0;
						lidar_scan.get()->points[_v * iTFS::max_col + _u].z = 0;
						lidar_scan.get()->points[_v * iTFS::max_col + _u].intensity = intensity;
					}
					else {
						pcl::PointXYZ direction_vector = get_direction_vector(_u, _v);

						// Coordinate converted to local Cartesian
						lidar_scan.get()->points[_v * iTFS::max_col + _u].x = distance * direction_vector.x;	// DO NOT CHANGE ROTATION
						lidar_scan.get()->points[_v * iTFS::max_col + _u].y = distance * direction_vector.z;	// DO NOT CHANGE ROTATION
						lidar_scan.get()->points[_v * iTFS::max_col + _u].z = -distance * direction_vector.y;	// DO NOT CHANGE ROTATION
						lidar_scan.get()->points[_v * iTFS::max_col + _u].intensity = intensity;
					}
				}
			}

			// Convert pcl to pointcloud2
			pcl_conversions::toPCL(ros::Time::now(), lidar_scan->header.stamp);
			pcl::toROSMsg(*lidar_scan.get(), lidar_scan_message);

			// Publish the point cloud topic
			pub_3D_points.publish(lidar_scan_message);

			// Publish the image topics
			if (colormap != true) {
				ptr_depth_image = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_depth_image).toImageMsg();
				ptr_intensity_image = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_intensity_image).toImageMsg();
				pub_2D_depth.publish(ptr_depth_image);
				pub_2D_intensity.publish(ptr_intensity_image);
			}
			else {
				cv_depth_image.convertTo(cv_depth_comp_image, CV_8UC1, 255.0/((double)depth_cmax));
				cv::applyColorMap(cv_depth_comp_image, cv_depth_color_image, colormap_type);

				cv_intensity_image.convertTo(cv_intensity_comp_image, CV_8UC1, 255.0/((double)intensity_cmax));
				cv::applyColorMap(cv_intensity_comp_image, cv_intensity_color_image, colormap_type);

				ptr_depth_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_depth_color_image).toImageMsg();
				ptr_intensity_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_intensity_color_image).toImageMsg();
				pub_2D_depth.publish(ptr_depth_image);
				pub_2D_intensity.publish(ptr_intensity_image);
			}
		}
	}

	// Stop and delete iTFS LiDAR class
	delete lidar;
	printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");
}
