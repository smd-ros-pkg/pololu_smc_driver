#!/usr/bin/env python

##
# \file SMCChannel.py
#
# \brief Dynamic Reconfigure definitions for Channels
# \author Scott K Logan
# \date January 07, 2013
#
# This dynamic reconfigure implementation for the SMC driver includes parameters
# for control of the device's various channels.
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

## \brief Parameter generator for SMC Channel Settings
gen = ParameterGenerator( )

gen.add( "invert", bool_t, SensorLevels.RECONFIGURE_RUNNING, "If true, then a higher rawValue corresponds to lower scaledValue.", False )
## \brief Enumerator for possible values in scalingDegree
scalingDegreeEnum = gen.enum([gen.const( "Linear", int_t, 0, "Linear scaling." ),
			gen.const( "Quadratic", int_t, 1, "Quadratic scaling." ),
			gen.const( "Cubic", int_t, 2, "Cubic scaling." )],
			"Enum to set crcMode")
gen.add( "scalingDegree", int_t, SensorLevels.RECONFIGURE_RUNNING, "Channel scaling value", 0, 0, 2, edit_method=scalingDegreeEnum )
## \brief Enumerator for possible values in alternateUse
alternateUseEnum = gen.enum([gen.const( "None", int_t, 0, "This channel is not used for anything special but its value can be read using Serial or USB commands." ),
			gen.const( "LimitForward", int_t, 1, "If this channel is active (Scaled Value >= 1600), then the motor is not allowed to move forward." ),
			gen.const( "LimitReverse", int_t, 2, "If this channel is active (Scaled Value >= 1600), then the motor is not allowed to move in reverse." ),
			gen.const( "KillSwitch", int_t, 3, "If this channel is active (Scale Value >= 1600), then the motor is not allowed to move." )],
			"Enum to set alternateUse")
gen.add( "alternateUse", int_t, SensorLevels.RECONFIGURE_RUNNING, "Used to allow a channel be a limit switch.", 0, 0, 3, edit_method=alternateUseEnum )
## \brief Enumerator for possible values in pinMode
pinModeEnum = gen.enum([gen.const( "Floating", int_t, 0, "No pull-up or pull-down resistors enabled." ),
			gen.const( "PullUp", int_t, 1, "Weak pull-up resistor (to 3.3 V) enabled." ),
			gen.const( "PullDown", int_t, 2, "Weak pull-down resistor (to 0 V) enabled." )],
			"Enum to set pinMode")
gen.add( "pinMode", int_t, SensorLevels.RECONFIGURE_RUNNING, "Determines if the analog input is floating, pulled up, or pulled down.", 0, 0, 2, edit_method=pinModeEnum )
gen.add( "errorMin", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues greater than this generate an error.", 0, 0, 65535 )
gen.add( "errorMax", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues less than this generate an error.", 4095, 0, 65535 )
gen.add( "inputMin", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues greater than or equal to this are inverted.", 40, 0, 65535 )
gen.add( "inputMax", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues less than or equal to this are inverted.", 4055, 0, 65535 )
gen.add( "inputNeutralMin", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues between inputNeutralMin and inputNeutralMax get mapped to a speed of zero.", 2080, 0, 65535 )
gen.add( "inputNeutralMax", int_t, SensorLevels.RECONFIGURE_RUNNING, "rawValues between inputNeutralMin and inputNeutralMax get mapped to a speed of zero.", 2015, 0, 65535 )

exit( gen.generate( PACKAGE, "smc_driver", "SMCChannel" ) )
