/***************************************************************************//**
 * \file smc_driver.cpp
 *
 * \brief ROS Implementation of the C Driver
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * Defined here is a class which wraps the basic C driver and sets up data
 * pipelines with ROS. The features include basic speed control, dynamic
 * reconfigure, diagnostics, and services for estop/safe-start.
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

#include "pololu_smc_driver/smc_driver.hpp"

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>

namespace pololu_smc_driver
{
	SMCDriver::SMCDriver( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv,
		const std::string _serial ) :
		nh( _nh ),
		nh_priv( _nh_priv ),
		rc1_nh_priv( nh_priv, "rc1" ),
		rc2_nh_priv( nh_priv, "rc2" ),
		analog1_nh_priv( nh_priv, "analog1" ),
		analog2_nh_priv( nh_priv, "analog2" ),
		fwlimits_nh_priv( nh_priv, "fwlimits" ),
		revlimits_nh_priv( nh_priv, "revlimits" ),
		dyn_re( NULL ),
		rc1_dyn_re( NULL ),
		rc2_dyn_re( NULL ),
		analog1_dyn_re( NULL ),
		analog2_dyn_re( NULL ),
		fwlimits_dyn_re( NULL ),
		revlimits_dyn_re( NULL ),
		dyn_re_cb_type( boost::bind( &SMCDriver::DynReCB, this, _1, _2) ),
		rc1_dyn_re_cb_type( boost::bind( &SMCDriver::RC1DynReCB, this, _1, _2) ),
		rc2_dyn_re_cb_type( boost::bind( &SMCDriver::RC2DynReCB, this, _1, _2) ),
		analog1_dyn_re_cb_type( boost::bind( &SMCDriver::Analog1DynReCB, this, _1, _2) ),
		analog2_dyn_re_cb_type( boost::bind( &SMCDriver::Analog2DynReCB, this, _1, _2) ),
		fwlimits_dyn_re_cb_type( boost::bind( &SMCDriver::FWLimitsDynReCB, this, _1, _2) ),
		revlimits_dyn_re_cb_type( boost::bind( &SMCDriver::REVLimitsDynReCB, this, _1, _2) ),
		min_update_rate( 10.0 ),
		max_update_rate( 100.0 ),
		diag_up_freq( diagnostic_updater::FrequencyStatusParam( &min_update_rate, &max_update_rate, 0.1, 5 ) ),
		input_pub_cb( boost::bind( &SMCDriver::InputPubCB, this ) ),
		smcd( -1 ),
		serial( _serial ),
		model( "" ),
		joint_name( "motor" ),
		inputQueryRate( 1.0 )
	{
		smc_init( );
		nh_priv.param( "joint_name", joint_name, joint_name );
		nh_priv.param( "inputQueryRate", inputQueryRate, inputQueryRate );
		diag.setHardwareIDf( "Pololu SMC %s", serial.length( ) ? serial.c_str( ) : "(unknown device)" );
		diag.add( "Pololu SMC Status", this, &SMCDriver::DiagCB );
		diag.add( diag_up_freq );
		diag_timer = nh_priv.createWallTimer( ros::WallDuration( 1 ), &SMCDriver::DiagTimerCB, this );
		input_timer = nh_priv.createWallTimer( ros::WallDuration( 1.0 / inputQueryRate ), &SMCDriver::InputTimerCB, this, false, false );
	}

	SMCDriver::~SMCDriver( )
	{
		SMCClose( );
		smc_exit( );
		delete dyn_re;
	}

	bool SMCDriver::set_speed( short int spd )
	{
		if( !SMCStat( ) )
			return false;

		short int dir = 1;
		if( spd < 0.0 )
		{
			dir = -1;
			spd *= -1;
		}

		// Clip
		if( spd > 3200 )
			spd = 3200;
		
		int ret = smc_set_speed( smcd, spd, dir, 2000 );
		if( ret >= 0 )
			diag_up_freq.tick( );
		return ret;
	}

	void SMCDriver::JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg )
	{
		int idx = -1;
		for( int i = 0; i < msg->joint_names.size( ); i++ )
			if( msg->joint_names[i] == joint_name )
				idx = i;
		if( !msg->points.size( ) || idx < 0 || idx >= msg->points[0].velocities.size( ) )
			return;
		set_speed( msg->points[0].velocities[idx] + 0.5 );
	}

	void SMCDriver::DynReCB( pololu_smc_driver::SMCDriverConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		if( cfg.inputQueryRate != inputQueryRate )
		{
			inputQueryRate = cfg.inputQueryRate;
			input_timer.setPeriod( ros::WallDuration( 1.0 / inputQueryRate ) );
		}
		set.neverSuspend = cfg.neverSuspend;
		set.uartResponseDelay = cfg.uartResponseDelay;
		set.useFixedBaudRate = cfg.useFixedBaudRate;
		set.disableSafeStart = cfg.disableSafeStart;
		set.fixedBaudRateRegister = cfg.fixedBaudRateRegister;
		set.speedUpdatePeriod = cfg.speedUpdatePeriod;
		set.commandTimeout = cfg.commandTimeout;
		set.serialDeviceNumber = cfg.serialDeviceNumber;
		set.crcMode = cfg.crcMode;
		set.overTempMin = cfg.overTempMin;
		set.overTempMax = cfg.overTempMax;
		set.inputMode = cfg.inputMode;
		set.pwmMode = cfg.pwmMode;
		set.pwmPeriodFactor = cfg.pwmPeriodFactor;
		set.mixingMode = cfg.mixingMode;
		set.minPulsePeriod = cfg.minPulsePeriod;
		set.maxPulsePeriod = cfg.maxPulsePeriod;
		set.rcTimeout = cfg.rcTimeout;
		set.ignorePotDisconnect = cfg.ignorePotDisconnect;
		set.tempLimitGradual = cfg.tempLimitGradual;
		set.consecGoodPulses = cfg.consecGoodPulses;
		set.motorInvert = cfg.motorInvert;
		set.speedZeroBrakeAmount = cfg.speedZeroBrakeAmount;
		set.ignoreErrLineHigh = cfg.ignoreErrLineHigh;
		set.vinMultiplierOffset = cfg.vinMultiplierOffset;
		set.lowVinShutoffTimeout = cfg.lowVinShutoffTimeout;
		set.lowVinShutoffMv = cfg.lowVinShutoffMv;
		set.serialMode = cfg.serialMode;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::RC1DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.rc1.invert = cfg.invert;
		set.rc1.scalingDegree = cfg.scalingDegree;
		set.rc1.alternateUse = cfg.alternateUse;
		set.rc1.pinMode = cfg.pinMode;
		set.rc1.errorMin = cfg.errorMin;
		set.rc1.errorMax = cfg.errorMax;
		set.rc1.inputMin = cfg.inputMin;
		set.rc1.inputMax = cfg.inputMax;
		set.rc1.inputNeutralMin = cfg.inputNeutralMin;
		set.rc1.inputNeutralMax = cfg.inputNeutralMax;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::RC2DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.rc2.invert = cfg.invert;
		set.rc2.scalingDegree = cfg.scalingDegree;
		set.rc2.alternateUse = cfg.alternateUse;
		set.rc2.pinMode = cfg.pinMode;
		set.rc2.errorMin = cfg.errorMin;
		set.rc2.errorMax = cfg.errorMax;
		set.rc2.inputMin = cfg.inputMin;
		set.rc2.inputMax = cfg.inputMax;
		set.rc2.inputNeutralMin = cfg.inputNeutralMin;
		set.rc2.inputNeutralMax = cfg.inputNeutralMax;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::Analog1DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.analog1.invert = cfg.invert;
		set.analog1.scalingDegree = cfg.scalingDegree;
		set.analog1.alternateUse = cfg.alternateUse;
		set.analog1.pinMode = cfg.pinMode;
		set.analog1.errorMin = cfg.errorMin;
		set.analog1.errorMax = cfg.errorMax;
		set.analog1.inputMin = cfg.inputMin;
		set.analog1.inputMax = cfg.inputMax;
		set.analog1.inputNeutralMin = cfg.inputNeutralMin;
		set.analog1.inputNeutralMax = cfg.inputNeutralMax;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::Analog2DynReCB( pololu_smc_driver::SMCChannelConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.analog2.invert = cfg.invert;
		set.analog2.scalingDegree = cfg.scalingDegree;
		set.analog2.alternateUse = cfg.alternateUse;
		set.analog2.pinMode = cfg.pinMode;
		set.analog2.errorMin = cfg.errorMin;
		set.analog2.errorMax = cfg.errorMax;
		set.analog2.inputMin = cfg.inputMin;
		set.analog2.inputMax = cfg.inputMax;
		set.analog2.inputNeutralMin = cfg.inputNeutralMin;
		set.analog2.inputNeutralMax = cfg.inputNeutralMax;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::FWLimitsDynReCB( pololu_smc_driver::SMCLimitsConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.forwardLimits.maxSpeed = cfg.maxSpeed;
		set.forwardLimits.maxAcceleration = cfg.maxAcceleration;
		set.forwardLimits.maxDeceleration = cfg.maxDeceleration;
		set.forwardLimits.brakeDuration = cfg.brakeDuration;
		set.forwardLimits.startingSpeed = cfg.startingSpeed;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	void SMCDriver::REVLimitsDynReCB( pololu_smc_driver::SMCLimitsConfig &cfg, const uint32_t lvl )
	{
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

		set.reverseLimits.maxSpeed = cfg.maxSpeed;
		set.reverseLimits.maxAcceleration = cfg.maxAcceleration;
		set.reverseLimits.maxDeceleration = cfg.maxDeceleration;
		set.reverseLimits.brakeDuration = cfg.brakeDuration;
		set.reverseLimits.startingSpeed = cfg.startingSpeed;

		if( smc_set_settings( smcd, &set, 5000 ) < 0 )
			return;
	}

	bool SMCDriver::SMCOpen( )
	{
		const char *ser = NULL;

		if( !smc_stat( smcd ) )
			return true;

		if( serial.length( ) )
			ser = serial.c_str( );

		if( ( smcd = smc_open( ser ) ) < 0 )
			return false;

		char mySerial[256];
		smc_get_serial( smcd, mySerial );
		serial = mySerial;
		model = smc_get_model( smcd );
		diag.setHardwareIDf( "Pololu SMC %s %s", model, mySerial );

		if( !merge_settings( ) )
		{
			smc_close( smcd );
			smcd = -1;
			return false;
		}

		// If this is initial startup, make sure that the DR server is running. We
		// needed to wait for the initial device settings load before doing this.
		if( !dyn_re )
			dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCDriverConfig>( dyn_re_mutex, nh_priv );
		if( !rc1_dyn_re )
			rc1_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>( dyn_re_mutex, rc1_nh_priv );
		if( !rc2_dyn_re )
			rc2_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>( dyn_re_mutex, rc2_nh_priv );
		if( !analog1_dyn_re )
			analog1_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>( dyn_re_mutex, analog1_nh_priv );
		if( !analog2_dyn_re )
			analog2_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCChannelConfig>( dyn_re_mutex, analog2_nh_priv );
		if( !fwlimits_dyn_re )
			fwlimits_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig>( dyn_re_mutex, fwlimits_nh_priv );
		if( !revlimits_dyn_re )
			revlimits_dyn_re = new dynamic_reconfigure::Server<pololu_smc_driver::SMCLimitsConfig>( dyn_re_mutex, revlimits_nh_priv );

		dyn_re->setCallback( dyn_re_cb_type );
		rc1_dyn_re->setCallback( rc1_dyn_re_cb_type );
		rc2_dyn_re->setCallback( rc2_dyn_re_cb_type );
		analog1_dyn_re->setCallback( analog1_dyn_re_cb_type );
		analog2_dyn_re->setCallback( analog2_dyn_re_cb_type );
		fwlimits_dyn_re->setCallback( fwlimits_dyn_re_cb_type );
		revlimits_dyn_re->setCallback( revlimits_dyn_re_cb_type );

		if( !joint_traj_sub )
			joint_traj_sub = nh.subscribe( "joint_trajectory", 1, &SMCDriver::JointTrajCB, this );
		if( !safe_start_srv )
			safe_start_srv = nh_priv.advertiseService( "safe_start", &SMCDriver::SafeStartCB, this );
		if( !estop_srv )
			estop_srv = nh_priv.advertiseService( "estop", &SMCDriver::EStopCB, this );

		if( !rc1_raw_pub )
			rc1_raw_pub = rc1_nh_priv.advertise<std_msgs::Float64>( "raw", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !rc1_raw_limited_pub )
			rc1_raw_limited_pub = rc1_nh_priv.advertise<std_msgs::Float64>( "raw_limited", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !rc1_scaled_pub )
			rc1_scaled_pub = rc1_nh_priv.advertise<std_msgs::Int16>( "scaled", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !rc2_raw_pub )
			rc2_raw_pub = rc2_nh_priv.advertise<std_msgs::Float64>( "raw", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !rc2_raw_limited_pub )
			rc2_raw_limited_pub = rc2_nh_priv.advertise<std_msgs::Float64>( "raw_limited", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !rc2_scaled_pub )
			rc2_scaled_pub = rc2_nh_priv.advertise<std_msgs::Int16>( "scaled", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog1_raw_pub )
			analog1_raw_pub = analog1_nh_priv.advertise<std_msgs::Float32>( "raw", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog1_raw_limited_pub )
			analog1_raw_limited_pub = analog1_nh_priv.advertise<std_msgs::Float32>( "raw_limited", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog1_scaled_pub )
			analog1_scaled_pub = analog1_nh_priv.advertise<std_msgs::Int16>( "scaled", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog2_raw_pub )
			analog2_raw_pub = analog2_nh_priv.advertise<std_msgs::Float32>( "raw", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog2_raw_limited_pub )
			analog2_raw_limited_pub = analog2_nh_priv.advertise<std_msgs::Float32>( "raw_limited", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !analog2_scaled_pub )
			analog2_scaled_pub = analog2_nh_priv.advertise<std_msgs::Int16>( "scaled", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );
		if( !vin_pub )
			vin_pub = nh_priv.advertise<std_msgs::Float32>( "vin", 1, input_pub_cb, input_pub_cb, ros::VoidConstPtr( ), true );

		// Check for initial subscribers
		InputPubCB( );

		return true;
	}

	void SMCDriver::SMCClose( )
	{
		input_timer.stop( );

		if( estop_srv )
			estop_srv.shutdown( );
		if( safe_start_srv )
			safe_start_srv.shutdown( );
		if( joint_traj_sub )
			joint_traj_sub.shutdown( );

		if( rc1_raw_pub )
			rc1_raw_pub.shutdown( );
		if( rc1_raw_limited_pub )
			rc1_raw_limited_pub.shutdown( );
		if( rc1_scaled_pub )
			rc1_scaled_pub.shutdown( );
		if( rc2_raw_pub )
			rc2_raw_pub.shutdown( );
		if( rc2_raw_limited_pub )
			rc2_raw_limited_pub.shutdown( );
		if( rc2_scaled_pub )
			rc2_scaled_pub.shutdown( );
		if( analog1_raw_pub )
			analog1_raw_pub.shutdown( );
		if( analog1_raw_limited_pub )
			analog1_raw_limited_pub.shutdown( );
		if( analog1_scaled_pub )
			analog1_scaled_pub.shutdown( );
		if( vin_pub )
			vin_pub.shutdown( );

		if( dyn_re )
			dyn_re->clearCallback( );
		if( rc1_dyn_re )
			rc1_dyn_re->clearCallback( );
		if( rc2_dyn_re )
			rc2_dyn_re->clearCallback( );
		if( analog1_dyn_re )
			analog1_dyn_re->clearCallback( );
		if( analog2_dyn_re )
			analog2_dyn_re->clearCallback( );
		if( fwlimits_dyn_re )
			fwlimits_dyn_re->clearCallback( );
		if( revlimits_dyn_re )
			revlimits_dyn_re->clearCallback( );

		smc_close( smcd );
		smcd = -1;
	}

	bool SMCDriver::SMCStat( )
	{
		if( smc_stat( smcd ) )
		{
			if( !SMCOpen( ) )
				return false;
		}
		return true;
	}

	void SMCDriver::DiagTimerCB( const ros::WallTimerEvent &e )
	{
		diag.update( );

		// This seems to do two things:
		// - restart the timer (otherwise the diagnostic_updater limits us and we hit 1/2 of the time)
		// - update the timer if the diagnostic period changed
		diag_timer.setPeriod( ros::WallDuration( diag.getPeriod( ) ) );
	}

	void SMCDriver::InputTimerCB( const ros::WallTimerEvent &e )
	{
		struct SmcVariables vars;

		if( !SMCStat( ) )
			return;

		if( smc_get_variables( smcd, &vars, 5000 ) < 0 )
			return;

		// RC1
		if( rc1_raw_pub && vars.rc1.unlimitedRawValue != 0xFFFF )
		{
			std_msgs::Float64Ptr msg( new std_msgs::Float64 );
			msg->data = vars.rc1.unlimitedRawValue / 4000000.0;
			rc1_raw_pub.publish( msg );
		}
		if( rc1_raw_limited_pub && vars.rc1.rawValue != 0xFFFF )
		{
			std_msgs::Float64Ptr msg( new std_msgs::Float64 );
			msg->data = vars.rc1.rawValue / 4000000.0;
			rc1_raw_limited_pub.publish( msg );
		}
		if( rc1_scaled_pub )
		{
			std_msgs::Int16Ptr msg( new std_msgs::Int16 );
			msg->data = vars.rc1.scaledValue;
			rc1_scaled_pub.publish( msg );
		}
		// RC2
		if( rc2_raw_pub && vars.rc2.unlimitedRawValue != 0xFFFF )
		{
			std_msgs::Float64Ptr msg( new std_msgs::Float64 );
			msg->data = vars.rc2.unlimitedRawValue / 4000000.0;
			rc2_raw_pub.publish( msg );
		}
		if( rc2_raw_limited_pub && vars.rc2.rawValue != 0xFFFF )
		{
			std_msgs::Float64Ptr msg( new std_msgs::Float64 );
			msg->data = vars.rc2.rawValue / 4000000.0;
			rc2_raw_limited_pub.publish( msg );
		}
		if( rc2_scaled_pub )
		{
			std_msgs::Int16Ptr msg( new std_msgs::Int16 );
			msg->data = vars.rc2.scaledValue;
			rc2_scaled_pub.publish( msg );
		}
		// Analog1
		if( analog1_raw_pub && vars.analog1.unlimitedRawValue != 0xFFFF )
		{
			std_msgs::Float32Ptr msg( new std_msgs::Float32 );
			msg->data = vars.analog1.unlimitedRawValue * 0.000805861;
			analog1_raw_pub.publish( msg );
		}
		if( analog1_raw_limited_pub && vars.analog1.rawValue != 0xFFFF )
		{
			std_msgs::Float32Ptr msg( new std_msgs::Float32 );
			msg->data = vars.analog1.rawValue * 0.000805861;
			analog1_raw_limited_pub.publish( msg );
		}
		if( analog1_scaled_pub )
		{
			std_msgs::Int16Ptr msg( new std_msgs::Int16 );
			msg->data = vars.analog1.scaledValue;
			analog1_scaled_pub.publish( msg );
		}
		// Analog2
		if( analog2_raw_pub && vars.analog2.unlimitedRawValue != 0xFFFF )
		{
			std_msgs::Float32Ptr msg( new std_msgs::Float32 );
			msg->data = vars.analog2.unlimitedRawValue / ( .33 * 4095 );
			analog2_raw_pub.publish( msg );
		}
		if( analog2_raw_limited_pub && vars.analog2.rawValue != 0xFFFF )
		{
			std_msgs::Float32Ptr msg( new std_msgs::Float32 );
			msg->data = vars.analog2.rawValue / ( .33 * 4095 );
			analog2_raw_limited_pub.publish( msg );
		}
		if( analog2_scaled_pub )
		{
			std_msgs::Int16Ptr msg( new std_msgs::Int16 );
			msg->data = vars.analog2.scaledValue;
			analog2_scaled_pub.publish( msg );
		}
		// VIN
		if( vin_pub )
		{
			std_msgs::Float32Ptr msg( new std_msgs::Float32 );
			msg->data = vars.vinMv / 1000.0;
			vin_pub.publish( msg );
		}
	}

	void SMCDriver::InputPubCB( )
	{
		if( rc1_raw_pub.getNumSubscribers( ) > 0 ||
			rc1_raw_limited_pub.getNumSubscribers( ) > 0 ||
			rc1_scaled_pub.getNumSubscribers( ) > 0 ||
			rc2_raw_pub.getNumSubscribers( ) > 0 ||
			rc2_raw_limited_pub.getNumSubscribers( ) > 0 ||
			rc2_scaled_pub.getNumSubscribers( ) > 0 ||
			analog1_raw_pub.getNumSubscribers( ) > 0 ||
			analog1_raw_limited_pub.getNumSubscribers( ) > 0 ||
			analog1_scaled_pub.getNumSubscribers( ) > 0 ||
			analog2_raw_pub.getNumSubscribers( ) > 0 ||
			analog2_raw_limited_pub.getNumSubscribers( ) > 0 ||
			analog2_scaled_pub.getNumSubscribers( ) > 0 ||
			vin_pub.getNumSubscribers( ) > 0 )
			input_timer.start( );
		else
			input_timer.stop( );
	}

	void SMCDriver::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		if( !SMCStat( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "SMC status OK" );

		int r;

		// Firmware Version
		unsigned short int maj;
		unsigned short int min;
		if( ( r = smc_get_fw_version( smcd, &maj, &min, 1000 ) ) < 0 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Failed to fetch fw_version" );
			if( r == SMC_ERROR_NO_DEVICE )
				SMCClose( );
		}
		else
		{
			float fw_ver = min;
			while( fw_ver > 1.0 )
				fw_ver /= 10;
			fw_ver += maj;
			stat.add( "fw_version", fw_ver );
		}

		// Other Variables
		struct SmcVariables vars;
		if( ( r = smc_get_variables( smcd, &vars, 5000 ) ) < 0 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Failed to fetch SMC variables" );
			if( r == SMC_ERROR_NO_DEVICE )
				SMCClose( );
		}
		else
		{
			if( vars.errorStatus )
				stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "SMC is reporting errors" );
			else if( vars.errorOccurred )
				stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "SMC has recorded errors" );
			else if( vars.limitStatus )
				stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "SMC is limiting speed" );
			stat.add( "errorStatus", ErrorToStr( vars.errorStatus ) );
			stat.add( "errorOccurred", ErrorToStr( vars.errorOccurred ) );
			stat.add( "limitStatus", LimitToStr( vars.limitStatus ) );
			stat.add( "targetSpeed", vars.targetSpeed / 32.0 );
			stat.add( "speed", vars.speed / 32.0 );
			stat.add( "brakeAmount", ( vars.brakeAmount == 255 ) ? 0.0 : vars.brakeAmount / .320 );
			stat.add( "vin", vars.vinMv / 1000.0 );
			stat.add( "temperature", vars.temperature / 10.0 );
			stat.add( "rcPeriod", vars.rcPeriod / 1000.0 );
			stat.add( "baudRate", vars.baudRateRegister );
			stat.add( "time", vars.timeMs / 1000.0 );
		}
	}

	std::string SMCDriver::ErrorToStr( const uint16_t error ) const
	{
		std::string str;
		bool found = false;

		if( error & SmcErrorSafeStart )
		{
			str += "Safe-Start";
			found = true;
		}
		if( error & SmcErrorChannelInvalid )
		{
			if( found )
				str += ", ";
			str += "Invalid Channel";
			found = true;
		}
		if( error & SmcErrorSerial )
		{
			if( found )
				str += ", ";
			str += "Serial Comm";
			found = true;
		}
		if( error & SmcErrorCommandTimeout )
		{
			if( found )
				str += ", ";
			str += "Command Timeout";
			found = true;
		}
		if( error & SmcErrorLimitSwitch )
		{
			if( found )
				str += ", ";
			str += "Kill/Limit Switch";
			found = true;
		}
		if( error & SmcErrorVinLow )
		{
			if( found )
				str += ", ";
			str += "Low Voltage";
			found = true;
		}
		if( error & SmcErrorVinHigh )
		{
			if( found )
				str += ", ";
			str += "High Voltage";
			found = true;
		}
		if( error & SmcErrorTemperatureHigh )
		{
			if( found )
				str += ", ";
			str += "High Temperature";
			found = true;
		}
		if( error & SmcErrorMotorDriverError )
		{
			if( found )
				str += ", ";
			str += "Motor Driver Fault";
			found = true;
		}
		if( error & SmcErrorErrLineHigh )
		{
			if( found )
				str += ", ";
			str += "Error Line High";
			found = true;
		}

		if( !str.length( ) )
			str = "None";
		return str;
	}

	std::string SMCDriver::LimitToStr( const uint16_t limit ) const
	{
		std::string str;
		bool found = false;

		if( limit & SmcLimitStatusStartedState )
		{
			str += "Safe-Start";
			found = true;
		}
		if( limit & SmcLimitStatusTemperature )
		{
			if( found )
				str += ", ";
			str += "Temperature";
			found = true;
		}
		if( limit & SmcLimitStatusMaxSpeed )
		{
			if( found )
				str += ", ";
			str += "Max Speed";
			found = true;
		}
		if( limit & SmcLimitStatusStartingSpeed )
		{
			if( found )
				str += ", ";
			str += "Min Speed";
			found = true;
		}
		if( limit & SmcLimitStatusAcceleration )
		{
			if( found )
				str += ", ";
			str += "Max Acceleration";
			found = true;
		}
		if( limit & SmcLimitStatusRc1 )
		{
			if( found )
				str += ", ";
			str += "RC Ch 1 Limit";
			found = true;
		}
		if( limit & SmcLimitStatusRc2 )
		{
			if( found )
				str += ", ";
			str += "RC Ch 2 Limit";
			found = true;
		}
		if( limit & SmcLimitStatusAnalog1 )
		{
			if( found )
				str += ", ";
			str += "Analog Ch 1 Limit";
			found = true;
		}
		if( limit & SmcLimitStatusAnalog2 )
		{
			if( found )
				str += ", ";
			str += "Analog Ch 2 Limit";
			found = true;
		}
		if( limit & SmcLimitStatusUsbKill )
		{
			if( found )
				str += ", ";
			str += "USB Killswitch";
			found = true;
		}

		if( !str.length( ) )
			str = "None";
		return str;
	}

	bool SMCDriver::SafeStartCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		if( !SMCStat( ) )
			return false;

		return ( smc_resume( smcd, 2000 ) >= 0 );
	}

	bool SMCDriver::EStopCB( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
	{
		if( !SMCStat( ) )
			return false;

		return ( smc_stop( smcd, 2000 ) >= 0 );
	}

	bool SMCDriver::merge_settings( )
	{
		if( !SMCStat( ) )
			return false;

		struct SmcSettings set;

		boost::recursive_mutex::scoped_lock lock( dyn_re_mutex );

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return false;

		// SMCDriver
		{
			struct pololu_smc_driver::SMCDriverConfig cfg;

			// Start with the device config
			cfg.neverSuspend = set.neverSuspend;
			cfg.uartResponseDelay = set.uartResponseDelay;
			cfg.useFixedBaudRate = set.useFixedBaudRate;
			cfg.disableSafeStart = set.disableSafeStart;
			cfg.fixedBaudRateRegister = set.fixedBaudRateRegister;
			cfg.speedUpdatePeriod = set.speedUpdatePeriod;
			cfg.commandTimeout = set.commandTimeout;
			cfg.serialDeviceNumber = set.serialDeviceNumber;
			cfg.crcMode = set.crcMode;
			cfg.overTempMin = set.overTempMin;
			cfg.overTempMax = set.overTempMax;
			cfg.inputMode = set.inputMode;
			cfg.pwmMode = set.pwmMode;
			cfg.pwmPeriodFactor = set.pwmPeriodFactor;
			cfg.mixingMode = set.mixingMode;
			cfg.minPulsePeriod = set.minPulsePeriod;
			cfg.maxPulsePeriod = set.maxPulsePeriod;
			cfg.rcTimeout = set.rcTimeout;
			cfg.ignorePotDisconnect = set.ignorePotDisconnect;
			cfg.tempLimitGradual = set.tempLimitGradual;
			cfg.consecGoodPulses = set.consecGoodPulses;
			cfg.motorInvert = set.motorInvert;
			cfg.speedZeroBrakeAmount = set.speedZeroBrakeAmount;
			cfg.ignoreErrLineHigh = set.ignoreErrLineHigh;
			cfg.vinMultiplierOffset = set.vinMultiplierOffset;
			cfg.lowVinShutoffTimeout = set.lowVinShutoffTimeout;
			cfg.lowVinShutoffMv = set.lowVinShutoffMv;
			cfg.serialMode = set.serialMode;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( dyn_re )
				dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( nh_priv );
		}

		// SMCChannel RC1
		{
			struct pololu_smc_driver::SMCChannelConfig cfg;

			// Start with the device config
			cfg.invert = set.rc1.invert;
			cfg.scalingDegree = set.rc1.scalingDegree;
			cfg.alternateUse = set.rc1.alternateUse;
			cfg.pinMode = set.rc1.pinMode;
			cfg.errorMin = set.rc1.errorMin;
			cfg.errorMax = set.rc1.errorMax;
			cfg.inputMin = set.rc1.inputMin;
			cfg.inputMax = set.rc1.inputMax;
			cfg.inputNeutralMin = set.rc1.inputNeutralMin;
			cfg.inputNeutralMax = set.rc1.inputNeutralMax;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( rc1_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( rc1_dyn_re )
				rc1_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( rc1_nh_priv );
		}

		// SMCChannel RC2
		{
			struct pololu_smc_driver::SMCChannelConfig cfg;

			// Start with the device config
			cfg.invert = set.rc2.invert;
			cfg.scalingDegree = set.rc2.scalingDegree;
			cfg.alternateUse = set.rc2.alternateUse;
			cfg.pinMode = set.rc2.pinMode;
			cfg.errorMin = set.rc2.errorMin;
			cfg.errorMax = set.rc2.errorMax;
			cfg.inputMin = set.rc2.inputMin;
			cfg.inputMax = set.rc2.inputMax;
			cfg.inputNeutralMin = set.rc2.inputNeutralMin;
			cfg.inputNeutralMax = set.rc2.inputNeutralMax;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( rc2_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( rc2_dyn_re )
				rc2_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( rc2_nh_priv );
		}

		// SMCChannel Analog1
		{
			struct pololu_smc_driver::SMCChannelConfig cfg;

			// Start with the device config
			cfg.invert = set.analog1.invert;
			cfg.scalingDegree = set.analog1.scalingDegree;
			cfg.alternateUse = set.analog1.alternateUse;
			cfg.pinMode = set.analog1.pinMode;
			cfg.errorMin = set.analog1.errorMin;
			cfg.errorMax = set.analog1.errorMax;
			cfg.inputMin = set.analog1.inputMin;
			cfg.inputMax = set.analog1.inputMax;
			cfg.inputNeutralMin = set.analog1.inputNeutralMin;
			cfg.inputNeutralMax = set.analog1.inputNeutralMax;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( analog1_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( analog1_dyn_re )
				analog1_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( analog1_nh_priv );
		}

		// SMCChannel Analog2
		{
			struct pololu_smc_driver::SMCChannelConfig cfg;

			// Start with the device config
			cfg.invert = set.analog2.invert;
			cfg.scalingDegree = set.analog2.scalingDegree;
			cfg.alternateUse = set.analog2.alternateUse;
			cfg.pinMode = set.analog2.pinMode;
			cfg.errorMin = set.analog2.errorMin;
			cfg.errorMax = set.analog2.errorMax;
			cfg.inputMin = set.analog2.inputMin;
			cfg.inputMax = set.analog2.inputMax;
			cfg.inputNeutralMin = set.analog2.inputNeutralMin;
			cfg.inputNeutralMax = set.analog2.inputNeutralMax;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( analog2_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( analog2_dyn_re )
				analog2_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( analog2_nh_priv );
		}

		// SMCLimits Forward
		{
			struct pololu_smc_driver::SMCLimitsConfig cfg;

			// Start with the device config
			cfg.maxSpeed = set.forwardLimits.maxSpeed;
			cfg.maxAcceleration = set.forwardLimits.maxAcceleration;
			cfg.maxDeceleration = set.forwardLimits.maxDeceleration;
			cfg.brakeDuration = set.forwardLimits.brakeDuration;
			cfg.startingSpeed = set.forwardLimits.startingSpeed;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( fwlimits_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( fwlimits_dyn_re )
				fwlimits_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( fwlimits_nh_priv );
		}

		// SMCLimits Reverse
		{
			struct pololu_smc_driver::SMCLimitsConfig cfg;

			// Start with the device config
			cfg.maxSpeed = set.reverseLimits.maxSpeed;
			cfg.maxAcceleration = set.reverseLimits.maxAcceleration;
			cfg.maxDeceleration = set.reverseLimits.maxDeceleration;
			cfg.brakeDuration = set.reverseLimits.brakeDuration;
			cfg.startingSpeed = set.reverseLimits.startingSpeed;

			// Merge in the parameter server's values. These changes will be pushed to the
			// device when the callback is set later
			cfg.__fromServer__( revlimits_nh_priv );
			cfg.__clamp__( );

			// If DR server is running, update it internally. Otherwise, make sure
			// parameter server is right when it starts up
			if( revlimits_dyn_re )
				revlimits_dyn_re->updateConfig( cfg );
			else
				cfg.__toServer__( revlimits_nh_priv );
		}

		return true;
	}
}

