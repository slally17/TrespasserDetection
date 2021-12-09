#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include "stb_image_write.h"

#include <iostream>
#include <string>

#include "windows.h"
#include "assert.h"
#include <fstream>

std::string TrespasserDetection(int MAXRUNTIME) {
	std::string errorMessage = "";
	uint32_t kinectCount = k4a_device_get_installed_count();

	if (kinectCount == 1 && errorMessage == "") { //Run program if Kinect is found
		//Connect to the Kinect
		k4a_device_t device = NULL;
		if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
			errorMessage += "Kinect was found by program, but can't connect. Please try reconnecting.\n";
		}

		//Initialize the Kinect
		k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_5;
		device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
		device_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
		device_config.synchronized_images_only = true;
		k4a_calibration_t sensor_calibration;
		if (errorMessage == "" && K4A_FAILED(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration))) {
			errorMessage += "Get depth camera calibration failed. \n";
		}

		//Create body tracker
		k4abt_tracker_t tracker = NULL;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		if (errorMessage == "" && K4A_FAILED(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker))) {
			errorMessage += "Body tracker initialization failed. \n";
		}

		//Start recording
		if (errorMessage == "" && K4A_FAILED(k4a_device_start_cameras(device, &device_config))) {
			errorMessage += "Kinect camera failed to start, please try reconnecting.\n";
			k4a_device_close(device);
		}

		//Process Kinect recording data
		int runTime = 0;
		int xMin, xMax, yMin, yMax;
		bool running = true;
		while (running && errorMessage == "" && runTime < MAXRUNTIME) {
			// Reset mins and maxs
			xMin = 1280;
			xMax = 0;
			yMin = 720;
			yMax = 0;
			//Increment frame counter, MAXRUNTIME/fps = run time in seconds
			runTime++;

			//Press spacebar to stop recording
			if (GetAsyncKeyState(VK_SPACE)) {
				running = false;
			}

			//Get current frame
			k4a_capture_t sensor_capture;
			k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

			//Process current frame
			if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
				const k4a_image_t color_image = k4a_capture_get_color_image(sensor_capture);

				k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
				k4a_capture_release(sensor_capture);
				if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
					errorMessage += ("Add capture to tracker process queue failed.\n");
				}

				//Get skeleton from current frame
				k4abt_frame_t body_frame = NULL;
				k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
				if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED && errorMessage == "") {
					uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
					uint8_t* originalBuffer = k4a_image_get_buffer(color_image);
					int stride = k4a_image_get_stride_bytes(color_image);
					int width = k4a_image_get_width_pixels(color_image);
					int height = k4a_image_get_height_pixels(color_image);
					for (int i = 0; i<num_bodies; i++) {
						k4a_float2_t joints2D[32] = {};
						k4abt_skeleton_t skeleton;
						k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
						int jointCount = 0;
						int valid = 0;

						for (k4abt_joint_t joint : skeleton.joints) {
							if (K4A_FAILED(k4a_calibration_3d_to_2d(&sensor_calibration, &joint.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &joints2D[jointCount], &valid))) {
								errorMessage += ("Joint failed to convert from 3D to 2D.\n");
							}
							jointCount++;
						}

						if (errorMessage == "") {
							for (k4a_float2_t point : joints2D) {
								if (point.xy.x < xMin) {
									xMin = point.xy.x;
								}
								if (point.xy.x > xMax) {
									xMax = point.xy.x;
								}
								if (point.xy.y < yMin) {
									yMin = point.xy.y;
								}
								if (point.xy.y > yMax) {
									yMax = point.xy.y;
								}
							}

							int personWidth = xMax - xMin;
							int personHeight = yMax - yMin;
							xMin = xMin - int(personWidth / 4);
							xMax = xMax + int(personWidth / 4);
							yMin = yMin - int(personHeight / 4);
							yMax = yMax + int(personHeight / 4);
							personWidth = xMax - xMin;
							personHeight = yMax - yMin;
							if (xMin < 0) {
								xMin = 0;
							}
							if (xMax > 1280) {
								xMax = 1280;
							}
							if (yMin < 0) {
								yMin = 0;
							}
							if (yMax > 720) {
								yMax = 720;
							}
							// 250 added to compensate for skeleton measuring ankle to nose
							int calcHeight = skeleton.joints[24].position.xyz.y - skeleton.joints[30].position.xyz.y + 250;
							int bufferSize = personWidth * personHeight * 3;
							int bufferCount = 0;
							
							uint8_t* imageBuffer = new uint8_t[bufferSize];
							for (int y = yMin; y < yMax; y++) {
								for (int x = xMin; x < xMax; x++) {
									imageBuffer[bufferCount] = originalBuffer[y*stride + x*4];
									imageBuffer[bufferCount+1] = originalBuffer[y*stride + x*4 + 1];
									imageBuffer[bufferCount+2] = originalBuffer[y*stride + x*4 + 2];
									bufferCount+=3;
								}
							}

							std::string personTextFileName = "person" + std::to_string(i) + "Image" + std::to_string(runTime) + ".txt";
							std::ofstream fw(personTextFileName, std::ofstream::out);
							fw << calcHeight << "\n";
							fw.close();

							std::string personFileName = "person" + std::to_string(i) + "Image" + std::to_string(runTime) + ".jpg";
							int result = stbi_write_jpg(personFileName.c_str(), xMax-xMin, yMax-yMin, 3, imageBuffer, 90);

							delete[] imageBuffer;
						}
					}
					std::string colorFileName = "colorImage" + std::to_string(runTime) + ".jpg";
					int result = stbi_write_jpg(colorFileName.c_str(), width, height, 4, originalBuffer, 90);

					k4abt_frame_release(body_frame);			
				}
				else {
					errorMessage += "Pop body frame result failed.\n";
				}
				k4a_image_release(color_image);
			}
			else {
				errorMessage += "Get depth capture returned error.\n";
			}
		}

		//Stop Kinect and release tracker
		k4abt_tracker_shutdown(tracker);
		k4abt_tracker_destroy(tracker);
		k4a_device_stop_cameras(device);
		k4a_device_close(device);
	}
	else if (kinectCount == 0) { //End program if Kinect isn't found
		errorMessage += "Kinect can't be found by program, please try reconnecting.\n";
	}
	else if (errorMessage == "") { //End program if multiple Kinects are found
		errorMessage += "Multiple Kinects, detected. Please unplug additional ones.\n";
	}

	return errorMessage;
}