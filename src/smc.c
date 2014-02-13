/***************************************************************************//**
 * \file smc.c
 *
 * \brief Standalone C Driver for Pololu SMC Motor Controllers
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * This is a standolone C driver for the Pololu SMC family of motor
 * controllers. It uses LibUSB to interface with the system's USB drivers, and
 * is interfaced with similarly to files, in which a device is opened and is
 * referenced with an integer handle.
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

#include "pololu_smc_driver/smc.h"

#include <libusb.h>
#include <stdlib.h>
#include <string.h>

/*!
 * \brief Maximum number of simultaneously open device handles.
 */
#define MAX_HANDLES 256

/*!
 * \brief Number of product IDs this driver can handle.
 */
#define NUM_PRODUCTS 5

/*!
 * \brief Vendor ID of Pololu.
 */
static const uint16_t idVendorTarget = 0x1FFB;

/*!
 * \brief Product IDs of the 5 SMC models.
 */
static const uint16_t idProductTargetArr[NUM_PRODUCTS] = { 0x98, 0x9A, 0x9C, 0x9E, 0xA1 };

/*!
 * \brief Internal codes for USB communication with the SMCs.
 *
 * Used to USB setup control transfers with the SMC.
 */
enum SmcRequest
{
	/*!
	 * \brief Used to read the current settings from the device.
	 */
	GetSettings = 0x81,
	/*!
	 * \brief Used to send a new set of settings to the device.
	 *
	 * These settings will be written to flash. This operation usually takes about
	 * 26ms.
	 */
	SetSettings = 0x82,
	/*!
	 * \brief Used Gets the current state of the device.
	 */
	GetVariables = 0x83,
	/*!
	 * \brief Used to reset the device to its default, factory settings.
	 *
	 * This operation usually takes about 26ms.
	 */
	ResetSettings = 0x84,
	/*!
	 * \brief Used to fetch the cause of the device's last reset.
	 */
	GetResetFlags = 0x85,
	/*!
	 * \brief Used to set the speed of the motor.
	 *
	 * This only works if you are in Serial/USB input mode.
	 */
	SetSpeed = 0x90,
	/*!
	 * \brief Used to clear the Safe Start Violation bit in
	 *   SmcVariables::errorStatus.
	 *
	 * This has no effect unless you are in Serial/USB input mode.
	 */
	ExitSafeStart = 0x91,
	/*!
	 * \brief Used to temporarily set a motor limit.
	 *
	 * This change will last until the next time the device resets, or until another
	 * setMotorLimit command is issued which changes the limit. This function will
	 * not violate the hard motor limits that are stored in flash.
	 */
	SetMotorLimit = 0x92,
	/*!
	 * \brief Used to set the state of the USB kill switch.
	 */
	UsbKill = 0x93,
	/*!
	 * \brief Used to cause the device to disconnect and enter bootloader mode.
	 *
	 * In bootloader mode, you can upgrade the firmware on the device. After using
	 * this, you should disconnect from the device and delete any references to it
	 * because it is no longer usable.
	 */
	StartBootloader = 0xFF
};

/*!
 * \brief Private structure for handling SMC communication.
 *
 * Un-changing variables are stored here, as well as the LibUSB device handle.
 */
struct smc_priv
{
	/*!
	 * \brief LibUSB device handle.
	 */
	libusb_device_handle *dev;
	/*!
	 * \brief Kernel driver status.
	 *
	 * Set to 1 if the kernel driver was active and needed to be unloaded before
	 * the driver could assume exclusive control of the device. If this is 1,
	 * kernel driver control is resumed when this driver is unloaded.
	 */
	short unsigned int kdriver_active;
	/*!
	 * \brief Reset flags from the SMC.
	 */
	char *reset_flags;
	/*!
	 * \brief Major version of the firmware on the device.
	 */
	unsigned short int *ver_maj;
	/*!
	 * \brief Minor version of the firmware on the device.
	 */
	unsigned short int *ver_min;
	/*!
	 * \brief Serial number of the USB device.
	 */
	char *serial;
	/*!
	 * \brief USB product ID of this device.
	 */
	uint16_t idProduct;
};

/*!
 * \brief List of communication handles.
 */
static struct smc_priv * smc_list[MAX_HANDLES] = { NULL };

/*!
 * \brief LibUSB context for USB communication.
 */
static libusb_context *ctx = NULL;

/*!
 * \brief Grabs the next available device handle slot.
 *
 * Iterates through the ::MAX_HANDLES slots for the lowest available index.
 *
 * \returns Open slot index between 0 and ::MAX_HANDLES
 * \retval -1 No available slots
 */
static int next_available_handle( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( !smc_list[i] )
			return i;
	}
	return -1;
}

int smc_init( )
{
	int r = LIBUSB_SUCCESS;

	if( ( r = libusb_init( &ctx ) ) )
		return r;

	#if DEBUG
	libusb_set_debug( ctx, 3 );
	#endif

	return r;
}

int smc_open( const char *serial )
{
	libusb_device **devs;
	ssize_t c;
	ssize_t i;
	int r;

	c = libusb_get_device_list( ctx, &devs );

	if( c < 0 )
		return c;

	for( i = 0; i < c; i++ )
	{
		struct libusb_device_descriptor desc;

		r = libusb_get_device_descriptor( devs[i], &desc );
		if( r || desc.idVendor != idVendorTarget )
			continue;

		unsigned short int j;
		for( j = 0; j < NUM_PRODUCTS; j++ )
		{
			if( desc.idProduct == idProductTargetArr[j] )
			{
				libusb_device_handle *dev_handle;
				unsigned short int kernel_driver_active = 0;

				r = libusb_open( devs[i], &dev_handle );
				if( r )
					break;

				char mySerial[256];

				r = libusb_get_string_descriptor_ascii( dev_handle, desc.iSerialNumber, (unsigned char *)mySerial, 256 );
				if( serial && ( r < 0 || strcmp( mySerial, serial ) ) )
				{
					libusb_close( dev_handle );
					break;
				}

				r = libusb_kernel_driver_active( dev_handle, 0 );
				if( r < 0 )
				{
					libusb_close( dev_handle );
					break;
				}
				else if( r == 1 )
				{
					r = libusb_detach_kernel_driver( dev_handle, 0 );
					if( r )
					{
						libusb_close( dev_handle );
						break;
					}
					kernel_driver_active = 1;
				}

				r = libusb_claim_interface( dev_handle, 0 );
				if( r )
				{
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					break;
				}

				int mydev = next_available_handle( );

				if( mydev < 0 )
				{
					libusb_release_interface( dev_handle, 0 );
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					return LIBUSB_ERROR_NO_MEM;
				}

				smc_list[mydev] = malloc( sizeof( struct smc_priv ) );
				if( !smc_list[mydev] )
				{
					libusb_release_interface( dev_handle, 0 );
					if( kernel_driver_active )
						libusb_attach_kernel_driver( dev_handle, 0 );
					libusb_close( dev_handle );
					return LIBUSB_ERROR_NO_MEM;
				}
				memset( smc_list[mydev], 0, sizeof( struct smc_priv ) );
				smc_list[mydev]->dev = dev_handle;
				smc_list[mydev]->kdriver_active = kernel_driver_active;
				smc_list[mydev]->serial = malloc( sizeof( char ) * ( strlen( mySerial ) + 1 ) );
				strcpy( smc_list[mydev]->serial, mySerial );
				smc_list[mydev]->idProduct = desc.idProduct;

				libusb_free_device_list( devs, 1 );
				return mydev;
			}
		}
	}

	libusb_free_device_list( devs, 1 );
	return LIBUSB_ERROR_NOT_FOUND;
}

/*!
 * \brief Sets the state of the USB killswitch.
 *
 * \author Scott K Logan
 *
 * Explicitly sets the USB killswitch to an active or inactive state. If
 * transitioning from inactive to active, may cause the safe-start bit
 * to engage.
 *
 * \param smcd SMC device handle
 * \param active State to set, should be 0 or 1
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
static int set_usb_kill( const int smcd, const unsigned short int active, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0x40, UsbKill, (active) ? 1 : 0, 0, NULL, 0, to );
}

/*!
 * \brief Brings the device out of a safe-start state.
 *
 * \author Scott K Logan
 *
 * Clears the safe-start bit and brings the device to a running state. Usually
 * only useful if safe-start is enabled in the ::SmcSettings.
 *
 * \param smcd SMC device handle
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
static int exit_safe_start( const int smcd, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0x40, ExitSafeStart, 0, 0, NULL, 0, to );
}

int smc_resume( const int smcd, const unsigned int to )
{
	int r;

	if( ( r = set_usb_kill( smcd, 0, to ) ) < 0 )
		return r;

	if( ( r = exit_safe_start( smcd, to ) ) < 0 )
		return r;

	return r;
}

int smc_stop( const int smcd, const unsigned int to )
{
	int r;

	if( ( r = set_usb_kill( smcd, 1, to ) ) < 0 )
		return r;

	return r;
}

int smc_get_fw_version( const int smcd, unsigned short int *major, unsigned short int *minor, const unsigned int to )
{
	if( smc_list[smcd]->ver_maj && smc_list[smcd]->ver_min )
	{
		*major = *smc_list[smcd]->ver_maj;
		*minor = *smc_list[smcd]->ver_min;
		return LIBUSB_SUCCESS;
	}

	int r;
	char buff[14];

	if( ( r = libusb_control_transfer( smc_list[smcd]->dev, 0x80, 6, 0x0100, 0, (unsigned char *)buff, sizeof( buff ), to ) ) != sizeof( buff ) )
		return r;

	*minor = ( buff[12] & 0xF ) + ( ( buff[12] >> 4 & 0xF ) * 10 );
	*major = ( buff[13] & 0xF ) + ( ( buff[13] >> 4 & 0xF ) * 10 );

	smc_list[smcd]->ver_maj = malloc( sizeof( unsigned short int ) );
	smc_list[smcd]->ver_min = malloc( sizeof( unsigned short int ) );
	if( !smc_list[smcd]->ver_maj || !smc_list[smcd]->ver_min )
		return LIBUSB_ERROR_NO_MEM;

	*smc_list[smcd]->ver_maj = *major;
	*smc_list[smcd]->ver_min = *minor;

	return r;
}

int smc_get_serial( const int smcd, char *sn )
{
	strcpy( sn, smc_list[smcd]->serial );
	return strlen( smc_list[smcd]->serial );
}

int smc_reset_settings( const int smcd, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0x40, ResetSettings, 0, 0, NULL, 0, to );
}

int smc_set_speed( const int smcd, const unsigned int val, const short int dir, const unsigned int to )
{
	if( dir == 0 && val < 32 )
		return libusb_control_transfer( smc_list[smcd]->dev, 0x40, SetSpeed, val, 2, NULL, 0, to );
	else if( dir == 1 && val <= 3200 )
		return libusb_control_transfer( smc_list[smcd]->dev, 0x40, SetSpeed, val, 0, NULL, 0, to );
	else if( dir == -1 && val <= 3200 )
		return libusb_control_transfer( smc_list[smcd]->dev, 0x40, SetSpeed, val, 1, NULL, 0, to );
	return LIBUSB_ERROR_INVALID_PARAM;
}

int smc_get_reset_flags( const int smcd, char *reset_flags, const unsigned int to )
{
	if( smc_list[smcd]->reset_flags )
	{
		*reset_flags = *smc_list[smcd]->reset_flags;
		return LIBUSB_SUCCESS;
	}

	int r;
	char buff[1];

	if( ( r = libusb_control_transfer( smc_list[smcd]->dev, 0xC0, GetResetFlags, 0, 0, (unsigned char *)buff, sizeof( buff ), to ) ) != sizeof( buff ) )
		return r;

	*reset_flags = buff[0];

	smc_list[smcd]->reset_flags = malloc( sizeof( char ) );
	if( !smc_list[smcd]->reset_flags )
		return LIBUSB_ERROR_NO_MEM;

	*smc_list[smcd]->reset_flags = *reset_flags;

	return r;
}

int smc_get_variables( const int smcd, struct SmcVariables *vars, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0xC0, GetVariables, 0, 0, (unsigned char *)vars, sizeof( struct SmcVariables ), to );
}

int smc_get_settings( const int smcd, struct SmcSettings *set, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0xC0, GetSettings, 0, 0, (unsigned char *)set, sizeof( struct SmcSettings ), to );
}

int smc_set_settings( const int smcd, struct SmcSettings *set, const unsigned int to )
{
	return libusb_control_transfer( smc_list[smcd]->dev, 0x40, SetSettings, 0, 0, (unsigned char *)set, sizeof( struct SmcSettings ), to );
}

int smc_stat( const int smcd )
{
	if( smcd < 0 || !smc_list[smcd] )
		return LIBUSB_ERROR_NOT_FOUND;

	/// \todo It would be nice to do some libusb checks here...

	return LIBUSB_SUCCESS;
}

void smc_close( const int smcd )
{
		if( smcd < 0 || !smc_list[smcd] )
			return;

		libusb_release_interface( smc_list[smcd]->dev, 0 );
		if( smc_list[smcd]->kdriver_active )
			libusb_attach_kernel_driver( smc_list[smcd]->dev, 0 );
		libusb_close( smc_list[smcd]->dev );

		free( smc_list[smcd]->reset_flags );
		free( smc_list[smcd]->ver_maj );
		free( smc_list[smcd]->ver_min );
		free( smc_list[smcd]->serial );

		free( smc_list[smcd] );
		smc_list[smcd] = NULL;
}

void smc_exit( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( smc_list[i] )
		{
			smc_close( i );
		}
	}

	if( ctx )
		libusb_exit( ctx );
	ctx = NULL;
}

struct SmcList * smc_list_devices( )
{
	struct SmcList *lst = NULL;
	struct SmcList *lst_curr = NULL;
	libusb_device **devs;
	ssize_t c;
	ssize_t i;
	int r;

	c = libusb_get_device_list( ctx, &devs );

	if( c < 0 )
		return lst;

	for( i = 0; i < c; i++ )
	{
		struct libusb_device_descriptor desc;

		r = libusb_get_device_descriptor( devs[i], &desc );
		if( r || desc.idVendor != idVendorTarget )
			continue;

		unsigned short int j;
		for( j = 0; j < NUM_PRODUCTS; j++ )
		{
			if( desc.idProduct == idProductTargetArr[j] )
			{
				if( !lst )
				{
					lst = malloc( sizeof( struct SmcList ) );
					if( !lst )
						break;
					lst_curr = lst;
				}
				else
				{
					lst_curr->next = malloc( sizeof( struct SmcList ) );
					if( !lst_curr->next )
						break;
					lst_curr = lst_curr->next;
				}
	
				lst_curr->idVendor = desc.idVendor;
				lst_curr->idProduct = desc.idProduct;
				lst_curr->iSerialNumber = NULL;
				lst_curr->next = NULL;

				libusb_device_handle *dev_handle;
				unsigned short int kernel_driver_active = 0;

				r = libusb_open( devs[i], &dev_handle );
				if( r )
					break;

				char mySerial[256];

				r = libusb_get_string_descriptor_ascii( dev_handle, desc.iSerialNumber, (unsigned char *)mySerial, 256 );
				if( r < 0 )
				{
					libusb_close( dev_handle );
					break;
				}

				lst_curr->iSerialNumber = malloc( strlen( mySerial ) + 1 );
				strcpy( lst_curr->iSerialNumber, mySerial );

				libusb_close( dev_handle );

				break;
			}
		}
	}

	libusb_free_device_list( devs, 1 );
	return lst;
}

void smc_free_list( struct SmcList *lst )
{
	struct SmcList *lst_tmp;

	while( lst )
	{
		lst_tmp = lst;
		lst = lst->next;
		free( lst_tmp->iSerialNumber );
		free( lst_tmp );
	}
}

const char * smc_lookup( uint16_t idProduct )
{
	switch( idProduct )
	{
	case 0x0098:
		return "18v15";
	case 0x009A:
		return "24v12";
	case 0x009C:
		return "18v25";
	case 0x009E:
		return "24v23";
	case 0x00A1:
		return "18v7";
	default:
		return NULL;
	}
}

const char * smc_get_model( int smcd )
{
	return smc_lookup( smc_list[smcd]->idProduct );
}
