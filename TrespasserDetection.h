#pragma once

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include <iostream>
#include <string>

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
		k4a_transformation_t transformation_handle = NULL;
		transformation_handle = k4a_transformation_create(&sensor_calibration);

		//Start recording
		if (errorMessage == "" && K4A_FAILED(k4a_device_start_cameras(device, &device_config))) {
			errorMessage += "Kinect camera failed to start, please try reconnecting.\n";
			k4a_device_close(device);
		}

		//Process Kinect recording data
		int runTime = 0;
		bool running = true;
		while (running && errorMessage == "" && runTime < MAXRUNTIME) {
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
				k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
				k4a_capture_release(sensor_capture);
				if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
					errorMessage += ("Add capture to tracker process queue failed.\n");
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