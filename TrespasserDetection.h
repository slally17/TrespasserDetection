#pragma once

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include "windows.h"

#include <iostream>
#include <string>

#include "assert.h"
#include <fstream>

void writeToFile(const char* fileName, void* buffer, size_t bufferSize) {
	assert(buffer != NULL);

	std::ofstream hFile;
	hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);
	if (hFile.is_open())
	{
		hFile.write((char*)buffer, static_cast<std::streamsize>(bufferSize));
		hFile.close();
	}
	std::cout << "[Streaming Service] frame is stored in " << fileName << std::endl;
}

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
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
		device_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
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
					for (int i = 0; i < num_bodies; i++) {
						k4a_float2_t joints2D[32] = {};
						k4abt_skeleton_t skeleton;
						k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);
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
							int width = k4a_image_get_width_pixels(color_image);
							int bufferSize = (xMax - xMin + 1) * (yMax - yMin + 1);
							int bufferCount = 0;
							uint8_t* originalBuffer = k4a_image_get_buffer(color_image);
							uint8_t* imageBuffer = new uint8_t[bufferSize];
							for (int y = yMin; y <= yMax; y++) {
								for (int x = xMin; x <= xMax; x++) {
									imageBuffer[bufferCount] = originalBuffer[y*width + x];
									bufferCount++;
								}
							}

							std::string personFileName = "person" + std::to_string(i) + "Image" + std::to_string(runTime) + ".jpg";
							writeToFile(personFileName.c_str(), imageBuffer, bufferSize);

							delete[] imageBuffer;
						}
					}
					std::string colorFileName = "colorImage" + std::to_string(runTime) + ".jpg";
					writeToFile(colorFileName.c_str(), k4a_image_get_buffer(color_image), k4a_image_get_size(color_image));

					k4abt_frame_release(body_frame);
					k4a_image_release(color_image);
					k4a_capture_release(sensor_capture);
				}
				else {
					errorMessage += "Pop body frame result failed.\n";
				}
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