/***************************************************************************//**
 * \file smc_driver.hpp
 *
 * \brief ROS Implementation of the C Driver (header)
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * API for the ROS driver
 *
 * \section license License (BSD-3)
 * Copyright (c) 2013, Scott K Logan\n
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Willow Garage, Inc. nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef _smc_driver_hpp
#define _smc_driver_hpp

#include "pololu_smc_driver/smc.h"
#include "pololu_smc_driver/SMCDriverConfig.h"
#include "pololu_smc_driver/SMCChannelConfig.h"
#include "pololu_smc_driver/SMCLimitsConfig.h"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

/*!
 * \brief Used for the ROS SMC Driver.
 *
 * All elements of the ROS implementation including the wrapped driver and the
 * dynamic reconfigure elements fall within this namespace.
 */
namespace pololu_smc_driver
{
	/*!
	 * \brief ROS-wrapped implementation of the SMC driver.
	 *
	 * \author Scott K Logan
	 *
	 * This class interfaces with a single SMC device. It automatically subscribes
	 * to the control feed and creates instances of the dynamic reconfigure server
	 * and the diagnostic updater.
	 */
	class SMCDriver
	{
	public:
		/*!
		 * \brief Constructor.
		 *
		 * \author Scott K Logan
		 *
		 * This constructor initializes the standalone C driver and the various
		 * components of the ROS interface from that driver.
		 *
		 * \param _nh Node handle to use for all ROS topic communication
		 *   within for this device.
		 * \param _nh_priv Private node handle to use for all ROS parameters
		 *   within for this device. There should never be two drivers sharing the
		 *   same private node handle. If one is not provided, it will be
		 *   automatically created.
		 * \param _serial Serial number of the target device. If no serial is given
		 *   or an empty string is given, the first available device will be used.
		 *   Please note that if there are multiple devices connected, this may not
		 *   be the same device every time!
		 */
		SMCDriver( const ros::NodeHandle &_nh = ros::NodeHandle( ),
			const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ),
			const std::string _serial = "" );
		/*!
		 * \brief Destructor.
		 *
		 * \author Scott K Logan
		 *
		 * Closes the interface to the driver, unsubscribes, unadvertises, and frees
		 * the standalone driver private data.
		 */
		~SMCDriver( );

		/*!
		 * \brief Opens the interface with the device.
		 *
		 * \author Scott K Logan
		 *
		 * Calls the standalone driver routines for opening the USB communication
		 * pipeline with the device. Each time this is called, the settings displayed
		 * by dynamic reconfigure are updated according to the device settings. ROS
		 * subscriptions and advertisements are made at this time.
		 *
		 * \returns True if device was successfully opened.
		 */
		bool SMCOpen( );
		/*!
		 * \brief Closes the interface with the device.
		 *
		 * \author Scott K Logan
		 *
		 * Closes the non-diagnostic interfaces with ROS and closes the interface
		 * with the driver and device.
		 */
		void SMCClose( );
		/*!
		 * \brief Communication status of the device.
		 *
		 * \author Scott K Logan
		 *
		 * Tests for proper communication with the device driver.
		 *
		 * \returns False if there is a communications problem.
		 */
		bool SMCStat( );
		/*!
		 * \brief Sets the speed of the motor.
		 *
		 * \author Scott K Logan
		 *
		 * Converts the generic float-style message to the driver-friendly values.
		 * If the device is currently in a disconnected state, this function will
		 * first attempt to connect to the device.
		 *
		 * \param spd Target speed for the device, between -3200 and 3200.
		 *
		 * \returns True on successful speed change.
		 */
		bool set_speed( short int spd );
	private:
		/*!
		 * \brief ROS message callback for setting the speed of the motor.
		 *
		 * \author Scott K Logan
		 *
		 * Calls the native SMCDriver::set_speed function
		 *
		 * \param msg JointTrajectory message indicating the desired speed of the
		 *   motor. Should be between -3200 and 3200.
		 */
		void JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg );
		/*!
		 * \brief Dynamic Reconfigure Change Callback.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed this callback is called to
		 * interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void DynReCB( pololu_smc_driver::SMCDriverConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Channel RC1.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for channel RC1, this callback
		 * is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void RC1DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Channel RC2.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for channel RC2, this callback
		 * is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void RC2DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Channel Analog 1.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for channel Analog 1, this
		 * callback is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void Analog1DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Channel Analog 2.
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for channel Analog 2, this
		 * callback is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void Analog2DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Forward Limits
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for forward limits, this
		 * callback is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void FWLimitsDynReCB( pololu_smc_driver::SMCLimitsConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Dynamic Reconfigure Change Callback for Reverse Limits
		 *
		 * \author Scott K Logan
		 *
		 * When dynamic reconfigure values are changed for reverse limits, this
		 * callback is called to interface the new values with the device.
		 *
		 * \param cfg New dynamic reconfigure values to be sent to the device
		 * \param lvl Level of change in the values (indicates if reset is necessary)
		 */
		void REVLimitsDynReCB( pololu_smc_driver::SMCLimitsConfig &cfg, const uint32_t lvl );
		/*!
		 * \brief Callback for the Diagnostic Updater Timer
		 *
		 * \author Scott K Logan
		 *
		 * This timer is used to call the diagnostic update function at the configured
		 * interval. This function simply calls that function.
		 *
		 * \param e Timer event (not used)
		 */
		void DiagTimerCB( const ros::WallTimerEvent &e );
		/*!
		 * \brief Callback for the Channel and VIN Information Timer
		 *
		 * \author Scott K Logan
		 *
		 * This timer is used to publish information from the inputs on the SMC.
		 *
		 * \param e Timer event (not used)
		 */
		void InputTimerCB( const ros::WallTimerEvent &e );
		/*!
		 * \brief Callback for the Channel and VIN Publishers
		 *
		 * \author Scott K Logan
		 *
		 * This function should be called whenever there is a change in the number
		 * of subscribers to any of the channel or VIN publishers.
		 */
		void InputPubCB( );
		/*!
		 * \brief Diagnostic update callback
		 *
		 * \author Scott K Logan
		 *
		 * Whenever the diagnostic_updater deems it necessary to update the values
		 * therein, this callback is called to fetch the values from the device.
		 *
		 * \param[out] stat Structure in which to store the values for
		 * diagnostic_updater to report
		 */
		void DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat );
		/*!
		 * \brief Service callback for bringing the motor out of safe start mode.
		 *
		 * \author Scott K Logan
		 *
		 * This service callback causes the device to resume normal operation if
		 * there has been a safe start violation.
		 *
		 * \param req Service request (not used)
		 * \param[out] res Service response (not used)
		 *
		 * \returns True if request was successful
		 */
		bool SafeStartCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
		/*!
		 * \brief Service callback for causing an Emergency Stop.
		 *
		 * \author Scott K Logan
		 *
		 * This will call the driver to trigger the USB killswitch and cause a safe
		 * start violation.
		 *
		 * \param req Service request (not used)
		 * \param[out] res Service response (not used)
		 *
		 * \returns True if request was successful
		 */
		bool EStopCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
		/*!
		 * \brief Update the values in Dynamic Reconfigure from the device.
		 *
		 * \author Scott K Logan
		 *
		 * Queries the device for settings and merges them with any settings
		 * explicitly set in the parameter server. Also updates the dynamic
		 * reconfigure server values appropriately. This is called
		 * automatically every time the device is connected.
		 *
		 * \returns True if values were successfully updated.
		 */
		bool merge_settings( );
		/*!
		 * \brief Converts an error code to a comma separated string.
		 *
		 * \author Scott K Logan
		 *
		 * \param error Error value to convert
		 *
		 * \returns C++ string containing the error descriptions
		 */
		std::string ErrorToStr( const uint16_t error ) const;
		/*!
		 * \brief Converts a speed limit code to a comma separated string.
		 *
		 * \author Scott K Logan
		 *
		 * \param limit Speed limit value to convert
		 *
		 * \returns C++ string containing the limiting cause descriptions
		 */
		std::string LimitToStr( const uint16_t limit ) const;

		/*!
		 * \brief Mutex used for dynamic reconfigure
		 */
		boost::recursive_mutex dyn_re_mutex;

		/*!
		 * \brief NodeHanlde used to interface with ROS
		 */
		ros::NodeHandle nh;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS
		 */
		ros::NodeHandle nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Channel RC1 Config
		 */
		ros::NodeHandle rc1_nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Channel RC2 Config
		 */
		ros::NodeHandle rc2_nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Channel Analog1 Config
		 */
		ros::NodeHandle analog1_nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Channel Analog2 Config
		 */
		ros::NodeHandle analog2_nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Forward Limits Config
		 */
		ros::NodeHandle fwlimits_nh_priv;
		/*!
		 * \brief Private NodeHanlde used to interface with ROS for Reverse Limits Config
		 */
		ros::NodeHandle revlimits_nh_priv;
		/*!
		 * \brief Raw input reading from Channel RC1
		 */
		ros::Publisher rc1_raw_pub;
		/*!
		 * \brief Raw input reading after limit configuration from Channel RC1
		 */
		ros::Publisher rc1_raw_limited_pub;
		/*!
		 * \brief Scaled input reading (see SMC
		 *   pololu_smc_driver::SMCChannelConfig::scalingDegree) from Channel RC1
		 */
		ros::Publisher rc1_scaled_pub;
		/*!
		 * \brief Raw input reading from Channel RC2
		 */
		ros::Publisher rc2_raw_pub;
		/*!
		 * \brief Raw input reading after limit configuration from Channel RC2
		 */
		ros::Publisher rc2_raw_limited_pub;
		/*!
		 * \brief Scaled input reading (see SMC
		 *   pololu_smc_driver::SMCChannelConfig::scalingDegree) from Channel RC2
		 */
		ros::Publisher rc2_scaled_pub;
		/*!
		 * \brief Raw input reading from Channel Analog1
		 */
		ros::Publisher analog1_raw_pub;
		/*!
		 * \brief Raw input reading after limit configuration from Channel Analog1
		 */
		ros::Publisher analog1_raw_limited_pub;
		/*!
		 * \brief Scaled input reading (see SMC
		 *   pololu_smc_driver::SMCChannelConfig::scalingDegree) from Channel Analog1
		 */
		ros::Publisher analog1_scaled_pub;
		/*!
		 * \brief Raw input reading from Channel Analog2
		 */
		ros::Publisher analog2_raw_pub;
		/*!
		 * \brief Raw input reading after limit configuration from Channel Analog2
		 */
		ros::Publisher analog2_raw_limited_pub;
		/*!
		 * \brief Scaled input reading (see SMC
		 *   pololu_smc_driver::SMCChannelConfig::scalingDegree) from Channel Analog2
		 */
		ros::Publisher analog2_scaled_pub;
		/*!
		 * \brief Raw input voltage publisher
		 */
		ros::Publisher vin_pub;
		/*!
		 * \brief Subscription to speed control data
		 */
		ros::Subscriber joint_traj_sub;
		/*!
		 * \brief Safe start service provider
		 */
		ros::ServiceServer safe_start_srv;
		/*!
		 * \brief Emergency stop service provider
		 */
		ros::ServiceServer estop_srv;
		/*!
		 * \brief Dynamic reconfigure server
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCDriverConfig> *dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Channel RC1
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig> *rc1_dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Channel RC2
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig> *rc2_dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Channel Analog1
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig> *analog1_dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Channel Analog2
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig> *analog2_dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Forward Limits
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig> *fwlimits_dyn_re;
		/*!
		 * \brief Dynamic reconfigure server for Reverse Limits
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig> *revlimits_dyn_re;
		/*!
		 * \brief Dynamic reconfigure callback handle
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCDriverConfig>::CallbackType dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Channel RC1
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>::CallbackType rc1_dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Channel RC2
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>::CallbackType rc2_dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Channel Analog1
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>::CallbackType analog1_dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Channel Analog2
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>::CallbackType analog2_dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Forward Limits
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig>::CallbackType fwlimits_dyn_re_cb_type;
		/*!
		 * \brief Dynamic reconfigure callback handle for Reverse Limits
		 */
		dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig>::CallbackType revlimits_dyn_re_cb_type;
		/*!
		 * \brief Timer for calling the diagnostic update function
		 */
		ros::WallTimer diag_timer;
		/*!
		 * \brief Timer for publishing channel and VIN information
		 */
		ros::WallTimer input_timer;
		/*!
		 * \brief Diagnostic updater
		 */
		diagnostic_updater::Updater diag;
		/*!
		 * \brief Normal acceptable update rate minimum
		 */
		double min_update_rate;
		/*!
		 * \brief Normal acceptable update rate maximum
		 */
		double max_update_rate;
		/*!
		 * \brief Diagnostic rate for speed update
		 */
		diagnostic_updater::FrequencyStatus diag_up_freq;
		/*!
		 * \brief Callback for changes to the channel and VIN publishers' subscribers
		 */
		const ros::SubscriberStatusCallback input_pub_cb;

		/*!
		 * \brief SMC device handle to use when interfacing with the standalone
		 *   driver.
		 */
		int smcd;
		/*!
		 * \brief Serial number of the device with which we should be interfacing, or
		 *   a blank string for the first available device.
		 */
		std::string serial;
		/*!
		 * \brief C string which indicates the model of the currently connected
		 *   device
		 */
		const char *model;
		/*!
		 * \brief Joint name to set speed based on (within received JointTrajectory
		 *   message
		 */
		std::string joint_name;
		/*!
		 * \brief Rate at which channel and VIN information is queried and published
		 */
		double inputQueryRate;
	};
}

#endif /* _smc_driver_hpp */
