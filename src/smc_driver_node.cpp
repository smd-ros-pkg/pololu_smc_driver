/***************************************************************************//**
 * \file smc_driver_node.cpp
 *
 * \brief Single controller node
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * This binary creates a simple node for communication and control of a single
 * controller. It is useful primarily for diagnostics, testing and
 * preliminary configuration.
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
#include <cstdlib>

/*!
 * \brief Main Function
 *
 * \author Scott K Logan
 *
 * Initializes ROS, instantiates the node handle for the driver to use and
 * instantiates the SMCDriver class.
 *
 * \param argc Number of command line arguments
 * \param argv 2D character array of command line arguments
 *
 * \returns EXIT_SUCCESS, or an error state
 */
int main( int argc, char *argv[] )
{
	ros::init( argc, argv, "smc_driver_node" );

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv( "~" );

	std::string sn;
	nh_priv.param( "smc_serial", sn, (const std::string)"" );

	pololu_smc_driver::SMCDriver smc( nh, nh_priv, sn );

	if( !smc.SMCOpen( ) )
		ROS_ERROR( "Failed to open SMC device (will keep trying)" );

	ros::spin( );

	std::exit( EXIT_SUCCESS );
}
