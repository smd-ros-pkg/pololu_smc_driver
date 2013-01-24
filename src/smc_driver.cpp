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
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>

namespace pololu_smc_driver
{
	SMCDriver::SMCDriver( const ros::NodeHandle &_nh_priv, const std::string _serial ) :
		nh_priv( _nh_priv ),
		dyn_re( dyn_re_mutex, nh_priv ),
		dyn_re_cb_type( boost::bind( &SMCDriver::DynReCB, this, _1, _2) ),
		min_update_rate( 10.0 ),
		max_update_rate( 100.0 ),
		diag_up_freq( diagnostic_updater::FrequencyStatusParam( &min_update_rate, &max_update_rate, 0.1, 5 ) ),
		smcd( -1 ),
		serial( _serial )
	{
		smc_init( );
		diag.setHardwareID( "Pololu SMC (unknown serial)" );
		diag.add( "Pololu SMC Status", this, &SMCDriver::DiagCB );
		diag.add( diag_up_freq );
		diag_timer = nh_priv.createWallTimer( ros::WallDuration( 1 ), &SMCDriver::TimerCB, this );
	}

	SMCDriver::~SMCDriver( )
	{
		SMCClose( );
		smc_exit( );
	}

	bool SMCDriver::set_speed( float spd )
	{
		if( !SMCStat( ) )
			return false;

		short int dir = 1;
		if( spd < 0.0 )
		{
			dir = -1;
			spd *= -1;
		}
		spd *= 3200;
		
		int ret = smc_set_speed( smcd, (int)(spd + .5), dir, 2000 );
		if( ret >= 0 )
			diag_up_freq.tick( );
		return ret;
	}

	void SMCDriver::SpeedCB( const std_msgs::Float32Ptr &msg )
	{
		set_speed( msg->data );
	}

	void SMCDriver::DynReCB( pololu_smc_driver::SMCDriverConfig &cfg, const uint32_t lvl )
	{
		/// \todo Implement settings changes
		if( !SMCStat( ) )
			return;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return;

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
		diag.setHardwareIDf( "Pololu SMC %s", mySerial );

		if( !refresh_settings( ) )
		{
			smc_close( smcd );
			smcd = -1;
			return false;
		}

		dyn_re.setCallback( dyn_re_cb_type );
		speed_sub = nh_priv.subscribe( "speed", 1, &SMCDriver::SpeedCB, this );
		safe_start_srv = nh_priv.advertiseService( "safe_start", &SMCDriver::SafeStartCB, this );
		estop_srv = nh_priv.advertiseService( "estop", &SMCDriver::EStopCB, this );

		return true;
	}

	void SMCDriver::SMCClose( )
	{
		estop_srv.shutdown( );
		safe_start_srv.shutdown( );
		speed_sub.shutdown( );
		dyn_re.clearCallback( );
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

	void SMCDriver::TimerCB( const ros::WallTimerEvent &e )
	{
		diag.update( );

		// This seems to do two things:
		// - restart the timer (otherwise the diagnostic_updater limits us and we hit 1/2 of the time)
		// - update the timer if the diagnostic period changed
		diag_timer.setPeriod( ros::WallDuration( diag.getPeriod( ) ) );
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

	bool SMCDriver::refresh_settings( )
	{
		if( !SMCStat( ) )
			return false;

		struct SmcSettings set;

		if( smc_get_settings( smcd, &set, 5000 ) < 0 )
			return false;

		struct pololu_smc_driver::SMCDriverConfig cfg;

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

		boost::recursive_mutex::scoped_lock lock( dyn_re_mutex );
		dyn_re.updateConfig( cfg );

		return true;
	}
}

