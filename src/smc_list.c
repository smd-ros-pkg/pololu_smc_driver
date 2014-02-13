/***************************************************************************//**
 * \file smc_list.c
 *
 * \brief Utility to list SMC devices on a system
 * \author Scott K Logan
 * \date February 12, 2014
 *
 * To make it easier to look up what the serial number of an SMC device is,
 * this utility was created. It also verifies that the permissions are correct
 * for each of the SMC devices on the system.
 *
 * \section license License (BSD-3)
 * Copyright (c) 2014, Scott K Logan\n
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

#include <stdio.h>

#include <pololu_smc_driver/smc.h>

/*!
 * \brief Main Function
 *
 * \author Scott K Logan
 *
 * Calls the SMC routines which enumerate the devices on the system, and
 * displays each vendor and product ID, model name and serial number.
 *
 * \param argc Number of command line arguments
 * \param argv 2D character array of command line arguments
 *
 * \returns EXIT_SUCCESS, or an error state
 */
int main( int argc, char *argv[] )
{
	struct SmcList *smc = NULL;
	struct SmcList *smc_curr = NULL;
	unsigned int count = 0;

	smc_init( );

	printf( "idVendor | idProduct | Name             | iSerialNumber\n" );
	printf( "-----------------------------------------------------------------------\n" );

	smc = smc_list_devices( );
	smc_curr = smc;

	while( smc_curr )
	{
		const char *name = smc_lookup( smc_curr->idProduct );
		printf( "0x%04X   | 0x%04X    | ", smc_curr->idVendor, smc_curr->idProduct );
		if( name )
			printf( "Pololu SMC %s | ", name );
		else
			printf( "(unknown device) | " );
		if( smc_curr->iSerialNumber )
			printf( "%s\n", smc_curr->iSerialNumber );
		else
			printf( "???? (permission denied?)\n" );
		smc_curr = smc_curr->next;
		count++;
	}

	smc_free_list( smc );

	printf( "-----------------------------------------------------------------------\n" );
	printf( "Total: %u devices\n", count );

	smc_exit( );
	return 0;
}
