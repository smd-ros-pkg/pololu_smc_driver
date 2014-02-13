#!/usr/bin/env python

##
# \file SMCDriver.py
#
# \brief Dynamic Reconfigure definitions
# \author Scott K Logan
# \date January 07, 2013
#
# The dynamic reconfigure implementation for the SMC driver includes parameters
# for control of the device as well as parameters for control of the driver.
#
# \section license License (BSD-3)
# Copyright (c) 2013, Scott K Logan\n
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Willow Garage, Inc. nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## \brief Name of the SMC driver ROS package
PACKAGE='pololu_smc_driver'

from driver_base.msg import SensorLevels
from dynamic_reconfigure.parameter_generator_catkin import *

## \brief Parameter generator for SMC dynamic reconfigure
gen = ParameterGenerator( )

gen.add( "inputQueryRate", double_t, SensorLevels.RECONFIGURE_RUNNING, "How fast to poll for channel and VIN information.", 1.0, 0.5, 1000.0 )
gen.add( "neverSuspend", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Never enter USB suspend.", False )
gen.add( "uartResponseDelay", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Insert delay before serial responses.", False )
gen.add( "useFixedBaudRate", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Don't auto-detect baud rate.", False )
gen.add( "disableSafeStart", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Don't require safe-start to resume motor operation.", False )
gen.add( "fixedBaudRateRegister", int_t, SensorLevels.RECONFIGURE_RUNNING, "Used when useFixedBaudRate is true.", 7500, 0, 65535 )
gen.add( "speedUpdatePeriod", int_t, SensorLevels.RECONFIGURE_RUNNING, "Time between internal updates, units of 1ms.", 1, 1, 65535 )
gen.add( "commandTimeout", int_t, SensorLevels.RECONFIGURE_RUNNING, "Time before a command timeout occurs, units of 10ms, 0 to disable.", 0, 0, 65535 )
gen.add( "serialDeviceNumber", int_t, SensorLevels.RECONFIGURE_RUNNING, "Serial address for use with the Pololu protocol.", 0, 0, 254 )
## \brief Enumerator for possible values in crcMode
crcModeEnum = gen.enum([gen.const( "Disabled", int_t, 0, "Disable CRC Checks." ),
			gen.const( "RX", int_t, 1, "Required for RX." ),
			gen.const( "All", int_t, 3, "Required for RX and used for TX." )],
			"Enum to set crcMode")
gen.add( "crcMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "CRC mode for serial I/O.", 0, 0, 3, edit_method=crcModeEnum )
gen.add( "overTempMin", int_t, SensorLevels.RECONFIGURE_RUNNING, "Minimum level for over-temp, units of .1C.", 700, 0, 65535 )
gen.add( "overTempMax", int_t, SensorLevels.RECONFIGURE_RUNNING, "Maximum level for over-temp, units of .1C.", 800, 0, 65535 )
## \brief Enumerator for possible values in inputMode
inputModeEnum = gen.enum([gen.const( "SerialUSB", int_t, 0, "Serial and/or USB." ),
			gen.const( "Analog", int_t, 1, "A1/A2 Voltage." ),
			gen.const( "RC", int_t, 2, "RC1/RC2 PWM." )],
			"Enum to set inputMode")
gen.add( "inputMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "Input mode for motor control.", 0, 0, 2, edit_method=inputModeEnum )
## \brief Enumerator for possible values in pwmMode
pwmModeEnum = gen.enum([gen.const( "Brake", int_t, 0, "PWM between driving and braking." ),
			gen.const( "Coast", int_t, 1, "PWM between driving and coasting." )],
			"Enum to set pwmMode")
gen.add( "pwmMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "Controls braking and coasting.", 0, 0, 1, edit_method=pwmModeEnum )
gen.add( "pwmPeriodFactor", int_t, SensorLevels.RECONFIGURE_RUNNING, "PWM frequency (0 is highest)", 0, 0, 19 )
## \brief Enumerator for possible values in mixingMode
mixingModeEnum = gen.enum([gen.const( "None", int_t, 0, "Motor speed determined by Ch. 1 only." ),
			gen.const( "Left", int_t, 1, "Motor speed determined by Ch. 1 + Ch. 2." ),
			gen.const( "Right", int_t, 2, "Motor speed determined by Ch. 1 - Ch. 2." )],
			"Enum to set mixingMode")
gen.add( "mixingMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "Analog/RC channel handling.", 0, 0, 2, edit_method=mixingModeEnum )
gen.add( "minPulsePeriod", int_t, SensorLevels.RECONFIGURE_RUNNING, "Minimum time between consecutive RC rising edges, units of 1ms.", 9, 0, 65535 )
gen.add( "maxPulsePeriod", int_t, SensorLevels.RECONFIGURE_RUNNING, "Maximum time between consecutive RC rising edges, units of 1ms.", 100, 0, 65535 )
gen.add( "rcTimeout", int_t, SensorLevels.RECONFIGURE_RUNNING, "Time before an RC timeout occurs, units of 1ms.", 500, 0, 65535 )
gen.add( "ignorePotDisconnect", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Don't check pot by toggling power pins.", False )
gen.add( "tempLimitGradual", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Gradual speed limit starting at overTempMin.", False )
gen.add( "consecGoodPulses", int_t, SensorLevels.RECONFIGURE_RUNNING, "Number of consecutive pulses needed for RC control", 2, 0, 255 )
gen.add( "motorInvert", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Invert motor direction.", False )
gen.add( "speedZeroBrakeAmount", int_t, SensorLevels.RECONFIGURE_RUNNING, "Amount to brake while motors are stopped.", 32, 0, 32 )
gen.add( "ignoreErrLineHigh", bool_t, SensorLevels.RECONFIGURE_RUNNING, "Don't kill on high err line.", False )
gen.add( "vinMultiplierOffset", int_t, SensorLevels.RECONFIGURE_RUNNING, "Offsets values read from VIN", 0, -5000, 5000 )
gen.add( "lowVinShutoffTimeout", int_t, SensorLevels.RECONFIGURE_RUNNING, "VIN must stay below lowVinShutoffMv for this duration for an error to occur, units of 1ms.", 250, 0, 65535 )
gen.add( "lowVinShutoffMv", int_t, SensorLevels.RECONFIGURE_RUNNING, "Dropping below this voltage triggers a low-voltage error, units of 1mV", 5500, 0, 65535 )
## \brief Enumerator for possible values in serialMode
serialModeEnum = gen.enum([gen.const( "Binary", int_t, 0, "Pololu, compact or mini SSC protocols are used." ),
			gen.const( "Ascii", int_t, 1, "ASCII command protocol is used." )],
			"Enum to set serialMode")
gen.add( "serialMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "Which types of commands are accepted.", 0, 0, 1, edit_method=serialModeEnum )

exit( gen.generate( PACKAGE, "smc_driver", "SMCDriver" ) )
