#!/usr/bin/env python

##
# \file SMCLimits.py
#
# \brief Dynamic Reconfigure definitions for Channels
# \author Scott K Logan
# \date January 07, 2013
#
# This dynamic reconfigure implementation for the SMC driver includes parameters
# for control of the device's forward and reverse limits.
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

## \brief Parameter generator for SMC Limits Settings
gen = ParameterGenerator( )

gen.add( "maxSpeed", int_t, SensorLevels.RECONFIGURE_RUNNING, "Maximum speed at which the motor controller will ever drive the motor.", 3200, 0, 3200 )
gen.add( "maxAcceleration", int_t, SensorLevels.RECONFIGURE_RUNNING, "How much the magnitude of the motor speed is allowed to increase every speed update period.", 0, 0, 3200 )
gen.add( "maxDeceleration", int_t, SensorLevels.RECONFIGURE_RUNNING, "How much the magnitude of the motor speed is allowed to decrease every speed update period.", 0, 0, 3200 )
gen.add( "brakeDuration", int_t, SensorLevels.RECONFIGURE_RUNNING, "Time that the controller will spend braking before allowing the speed to change signs, units of 1ms.", 0, 0, 65535 )
gen.add( "startingSpeed", int_t, SensorLevels.RECONFIGURE_RUNNING, "Minimum non-zero speed. In RC or Analog mode, a scaled value of 1 maps to this speed. 0 means no effect.", 0, 0, 3200 )

exit( gen.generate( PACKAGE, "smc_driver", "SMCLimits" ) )
