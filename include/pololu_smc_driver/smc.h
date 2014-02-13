/***************************************************************************//**
 * \file smc.h
 *
 * \brief Standalone C Driver for Pololu SMC Motor Controllers (header)
 * \author Scott K Logan
 * \date January 07, 2013
 *
 * API for the standalone C driver
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

#ifndef _smc_h
#define _smc_h

#include <stdint.h>
#include <libusb.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Represents limits on the motor's behavior.
 */
struct SmcMotorLimitsStruct
{
	/*!
	 * \brief Maximum speed at which the controller will ever drive a motor.
	 *
	 * Should be a number between 0 and 3200.
	 */
	uint16_t maxSpeed;
	/*!
	 * \brief How much the magnitude of the motor speed may increase in one update
	 *   period.
	 *
	 * Should be a number between 0 and 3200. A value of 0 indicates no limit.
	 */
	uint16_t maxAcceleration;
	/*!
	 * \brief How much the magnitude of the motor speed may decrease in one update
	 *   period.
	 *
	 * Should be a number between 0 and 3200. A value of 0 indicates no limit.
	 */
	uint16_t maxDeceleration;
	/*!
	 * \brief Amount of time that the controller will brake the motor before
	 *   allowing reversal.
	 */
	uint16_t brakeDuration;
	/*!
	 * \brief Minimum non-zero speed.
	 *
	 * Should be a number between 0 and 3200. A value of 0 indicates no minimum.
	 */
	uint16_t startingSpeed;
	/*!
	 * \brief Reserved for future use and/or padding.
	 */
	uint16_t RESERVED0;
} __attribute__((packed));

/*!
 * \brief Influences the behavior of the analog/PWM channels on the SMC.
 */
struct SmcChannelSettingsStruct
{
	/*!
	 * \brief Used to invert the direction of scaling.
	 *
	 * If true, then a higher rawValue corresponds to lower scaledValue.
	 */
	uint8_t invert;
	/*!
	 * \brief 0 = Linear, 1 = Quadratic, 2 = Cubic, etc.
	 */
	uint8_t scalingDegree;
	/*!
	 * \brief This is used to allow a channel be a limit switch if it is not
	 *   controlling the motor.
	 *
	 * See ::SmcChannelAlternateUse for details.
	 */
	uint8_t alternateUse;
	/*!
	 * \brief Determines if the analog input is floating, pulled up, or pulled
	 *   down.
	 *
	 * Does not apply to RC channels.
	 */
	uint8_t pinMode;
	/*!
	 * \brief rawValues greater than this generate an error.
	 */
	uint16_t errorMin;
	/*!
	 * \brief rawValues less than this generate an error.
	 */
	uint16_t errorMax;
	/*!
	 * \brief rawValues greater than or equal to this get mapped to a speed of
	 *   -reverseLimits.maxSpeed (or forwardLimits.maxSpeed if invert==true).
	 */
	uint16_t inputMin;
	/*!
	 * \brief rawValues less than or equal to this get mapped to a speed of
	 *   forwardLimits.maxSpeed (or -reverseLimits.maxSpeed if invert==true).
	 */
	uint16_t inputMax;
	/*!
	 * \brief rawValues between inputNeutralMin and inputNeutralMax get mapped
	 *   to a speed of zero.
	 */
	uint16_t inputNeutralMin;
	/*!
	 * \brief rawValues between inputNeutralMin and inputNeutralMax get mapped
	 *   to a speed of zero.
	 */
	uint16_t inputNeutralMax;
} __attribute__((packed));

/*!
 * \brief Settings which influence the behavior of the SMC.
 *
 * These values are stored on the flash ROM of the device.
 */
struct SmcSettings
{
	/*!
	 * \brief If true, then never enter USB suspend mode.
	 */
	uint8_t neverSuspend;
	/*!
	 * \brief If true, then insert a one-byte delay before sending serial
	 *   responses.
	 *
	 * This allows slower processors time to get ready to receive the response.
	 */
	uint8_t uartResponseDelay;
	/*!
	 * \brief If true, then don't auto-detect the baud rate and instead use a
	 *   fixed baud rate, specified by fixedBaudRateRegister.
	 */
	uint8_t useFixedBaudRate;
	/*!
	 * \brief When false, puts strct requirements on what is necessary to
	 *   start the motor.
	 *
	 * See the user's guide for for more information.
	 */
	uint8_t disableSafeStart;
	/*!
	 * \brief The value to put in to the device's baud rate register to generate
	 *   the fixed baud rate.
	 *
	 * Only relevant if SmcSettings::useFixedBaudRate is true.
	 */
	uint16_t fixedBaudRateRegister;
	/*!
	 * \brief Time between accel/decel updates to the speed. Units of 1 ms.
	 *
	 * Should never be 0!
	 */
	uint16_t speedUpdatePeriod;
	/*!
	 * \brief The time before a command timeout error occurs, in units of 10 ms.
	 *
	 * A value of 0 disables the command timeout feature.
	 * Max is 65535 * 10 ms == 655.35s
	 */
	uint16_t commandTimeout;
	/*!
	 * \brief The serial device number used to address this device in the Pololu
	 *   protocol.
	 *
	 * Cannot be 255.
	 */
	uint8_t serialDeviceNumber;
	/*!
	 * \brief Whether CRC is disabled, required for reception, or required for
	 *   reception and sent during transmission.
	 *
	 * See ::SmcCrcMode for details.
	 */
	uint8_t crcMode;
	/*!
	 * \brief See tempLimitGradual.
	 *
	 * Units: tenths of a degree Celsius.
	 */
	uint16_t overTempMin;
	/*!
	 * \brief The temperature where speed is limited to 0.
	 *
	 * Units: tenths of a degree Celsius.
	 */
	uint16_t overTempMax;
	/*!
	 * \brief Control input mode.
	 *
	 * See ::SmcInputMode for details.
	 */
	uint8_t inputMode;
	/*!
	 * \brief PWM control mode, given SmcSettings::inputMode is ::SmcInputModeRC.
	 */
	uint8_t pwmMode;
	/*!
	 * \brief Determines the PWM frequency (0-19, 0=highest freq).
	 *
	 * See ::SmcPwmMode for details.
	 */
	uint8_t pwmPeriodFactor;
	/*!
	 * \brief The Mixing Mode setting for the device.
	 *
	 * See ::SmcMixingMode for details.
	 */
	uint8_t mixingMode;
	/*!
	 * \brief Minimum allowed time between consecutive RC pulse rising edges.
	 *
	 * Units: 1 ms.
	 */
	uint16_t minPulsePeriod;
	/*!
	 * \brief Maximum allowed time between consecutive RC pulse rising edges.
	 *
	 * If this amount of time elapses and no pulses at all have been received,
	 * then the motor is shut down and an error is generated.
	 * Units: 1 ms.
	 */
	uint16_t maxPulsePeriod;
	/*!
	 * \brief Generates error and shuts down the motor if we go this long without
	 *   heeding a pulse (units of 1 ms).
	 */
	uint16_t rcTimeout;
	/*!
	 * \brief If false, check for pot disconnect by toggling the pot power pins.
	 */
	uint8_t ignorePotDisconnect;
	/*!
	 * \brief False: abrupt limit past overTempMax asserted until temp falls below
	 *   SmcSettings::overTempMin.
	 *
	 * True: gradual speed limit starting at overTempMin. Should alwas be 0 or 1.
	 */
	uint8_t tempLimitGradual;
	/*!
	 * \brief Number of previously received consecutive good pulses needed to heed
	 *   the latest pulse received and update the channel's rawValue.
	 *
	 * 0 means no previous good pulses are required, so we update the channel's
	 * value on every good pulse. Full range is valid (0-255).
	 */
	uint8_t consecGoodPulses;
	/*!
	 * \brief Invert Motor Direction.
	 *
	 * Boolean that specifies the correspondence between speed (-3200 to 3200) and
	 * voltages on OUTA/OUTB. Normally, a speed of 3200 means OUTA~VIN, OUTB=0. If
	 * true, then a speed of 3200 means OUTA=0, OUTB~VIN.
	 */
	uint8_t motorInvert;
	/*!
	 * \brief Brake amount while input is in deadband (0-32), or there is an error,
	 *   or motor is driving at speed zero.
	 */
	uint8_t speedZeroBrakeAmount;
	/*!
	 * \brief If false, stop the motor if the ERR line is high (allows you to
	 *   connect the error lines of two devices and have them both stop when one has
	 *   an error).
	 */
	uint8_t ignoreErrLineHigh;
	/*!
	 * \brief Addative offset for voltage readings.
	 *
	 * Range is -5000 to 5000
	 */
	int16_t vinMultiplierOffset;
	/*!
	 * \brief VIN must stay below SmcSettings::lowVinShutoffMv for this duration
	 *   before a low-VIN error occurs (units of 1 ms).
	 */
	uint16_t lowVinShutoffTimeout;
	/*!
	 * \brief Dropping below this voltage threshold for a duration of
	 *   SmcSettings::lowVinShutoffTimeout triggers a low-voltage error (units of mV).
	 *
	 * Dropping below this voltage threshold triggers a low-voltage error (units of
	 * mV).
	 */
	uint16_t lowVinShutoffMv;
	/*!
	 * \brief Once asserting a low-voltage error, the voltage required to stop
	 *   asserting this error (units of mV).
	 */
	uint16_t lowVinStartupMv;
	/*!
	 * \brief Rising above this voltage threshold triggers a high-voltage error and
	 *   causes the motor to immediately brake at 100% (units of mV).
	 */
	uint16_t highVinShutoffMv;
	/*!
	 * \brief Determines what types of commands are accepted and whether to echo
	 *   incoming bytes.
	 */
	uint8_t serialMode;
	/*!
	 * \brief Reserved for future use and/or padding.
	 */
	uint8_t RESERVED0;
	/*!
	 * \brief Settings for RC channel 1.
	 */
	struct SmcChannelSettingsStruct rc1;
	/*!
	 * \brief Settings for RC channel 2.
	 */
	struct SmcChannelSettingsStruct rc2;
	/*!
	 * \brief Settings for analog channel 1.
	 */
	struct SmcChannelSettingsStruct analog1;
	/*!
	 * \brief Settings for analog channel 2.
	 */
	struct SmcChannelSettingsStruct analog2;
	/*!
	 * \brief Limits on the forward motion of the motor.
	 */
	struct SmcMotorLimitsStruct forwardLimits;
	/*!
	 * \brief Limits on the backward motion of the motor.
	 */
	struct SmcMotorLimitsStruct reverseLimits;
} __attribute__((packed));

/*!
 * \brief Represents the current state of an input channel.
 */
struct SmcChannelVariables
{
	/*!
	 * \brief The raw value of the channel as read from the pin.
	 *
	 * This is mainly useful for the control input setup wizard. 0xFFFF if
	 * disconnected but not affected by absolute max/min limits. Units of
	 * quarter-microseconds if an RC channel. 12-bit ADC reading (0-4095) if analog.
	 * 0xFFFF if input is disconnected.
	 */
	uint16_t unlimitedRawValue;
	/*!
	 * \brief This is just like unlimitedRawValue except that it will be 0xFFFF if
	 *   the absolute max/min limits are violated.
	 */
	uint16_t rawValue;
	/*!
	 * \brief The result of scaling the rawValue.
	 *
	 * This value depends on all the scaling settings in the channel's
	 * ::SmcChannelSettingsStruct.
	 */
	int16_t scaledValue;
	/*!
	 * \brief Reserved for future use and/or padding.
	 */
	uint16_t RESERVED0;
} __attribute__((packed));

/*!
 * \brief SMC Variables
 *
 * Represents the current state of the device, including all input channels
 * and motor limits.
 */
struct SmcVariables
{
	/*!
	 * \brief The errors that are currently happening.
	 */
	uint16_t errorStatus;
	/*!
	 * \brief The errors that occurred since the last time this register was
	 *   cleared.
	 */
	uint16_t errorOccurred;
	/*!
	 * \brief The serial errors that occurred since the last time this register
	 *   was cleared.
	 */
	uint16_t serialErrorOccurred;
	/*!
	 * \brief Status bits for anything that is limiting the motor speed that isn't
	 *   an error (but it could be caused by an error, such as
	 *   ::SmcLimitStatusStartedState).
	 */
	uint16_t limitStatus;
	/*!
	 * \brief The current state of RC Channel 1.
	 */
	struct SmcChannelVariables rc1;
	/*!
	 * \brief The current state of RC Channel 2.
	 */
	struct SmcChannelVariables rc2;
	/*!
	 * \brief The current state of Analog Channel 1.
	 */
	struct SmcChannelVariables analog1;
	/*!
	 * \brief The current state of Analog Channel 2.
	 */
	struct SmcChannelVariables analog2;
	/*!
	 * \brief Target speed of motor from -3200 to 3200.
	 */
	int16_t targetSpeed;
	/*!
	 * \brief Current speed of motor from -3200 to 3200.
	 *
	 * Can be non-zero even if power is off.
	 */
	int16_t speed;
	/*!
	 * \brief Current braking amount, from 0 to 32.
	 *
	 * This value is only relevant when speed==0,
	 * otherwise it will be 0xFF.
	 */
	uint16_t brakeAmount;
	/*!
	 * \brief The voltage on the VIN line in units of millivolts.
	 */
	uint16_t vinMv;
	/*!
	 * \brief The reading from the temperature sensor, in units of one tenth of a
	 *   degree Celsius.
	 *
	 * Temperatures below 0 degrees are reported as 0.
	 */
	uint16_t temperature;
	/*!
	 * \brief Reserved for future use and/or padding.
	 */
	uint16_t RESERVED0;
	/*!
	 * \brief The measured period of the RC pulses, in units of 0.1 ms.
	 */
	uint16_t rcPeriod;
	/*!
	 * \brief Value from the device's baud-rate register.
	 */
	uint16_t baudRateRegister;
	/*!
	 * \brief The time that the device has been running.
	 *
	 * Units: 1 ms. Overflows after 49.7 days back to 0.
	 */
	uint32_t timeMs;
	/*!
	 * \brief The currently-used motor limits for the forward direction.
	 *
	 * By default, these are equal to the hard limits in SmcSettingsStruct,
	 * but they can be temporarily changed by USB or Serial commands.
	 */
	struct SmcMotorLimitsStruct forwardLimits;
	/*!
	 * \brief The currently-used motor limits for the reverse direction.
	 *
	 * By default, these are equal to the hard limits in SmcSettingsStruct,
	 * but they can be temporarily changed by USB or Serial commands.
	 */
	struct SmcMotorLimitsStruct reverseLimits;
} __attribute__((packed));

/*!
 * \brief The Input Mode setting for the device.
 *
 * This specifies how the motor speed is determined.
 */
enum SmcInputMode
{
	/*!
	 * \brief Motor speed is set via serial and USB commands.
	 */
	SmcInputModeSerialUsb = 0,
	/*!
	 * \brief The voltage on A1/A2 determines the motor speed.
	 */
	SmcInputModeAnalog = 1,
	/*!
	 * \brief The width of pulses received on RC1 and RC2 determines the motor
	 *   speed.
	 */
	SmcInputModeRC = 2,
};

/*!
 * \brief The Mixing Mode setting for the device.
 *
 * This specifies which channels are used in calculating the motor speed and how
 * the calculation is to be done. The Mixing Mode only applies if the Input Mode
 * is Analog or RC.
 */
enum SmcMixingMode
{
	/*!
	 * \brief The motor speed is determined entirely by RC/Analog Channel 1.
	 */
	SmcMixingModeNone = 0,
	/*!
	 * \brief The motor speed is determined by adding RC/Analog Channels 1 and 2.
	 */
	SmcMixingModeLeft = 1,
	/*!
	 * \brief The motor speed is determined by subtracting RC/Analog Channel 2 from
	 *   Channel 1.
	 */
	SmcMixingModeRight = 2,
};

/*!
 * \brief Specifies what kinds of serial commands will be accepted.
 */
enum SmcSerialMode
{
	/*!
	 * \brief Commands and responses support the Pololu, Compact, or Mini SSC binary
	 *   protocols, specified in the user's guide.
	 */
	SmcSerialModeBinary = 0,
	/*!
	 * \brief Commands and responses obey the ASCII command protocol, specified in
	 *   the user's guide.
	 */
	SmcSerialModeAscii = 1,
};

/*!
 * \brief Specifies how Cyclic Redundancy Checks (CRC) will be used in serial
 *   communication.
 */
enum SmcCrcMode
{
	/*!
	 * \brief In this mode, CRC will not be used.
	 */
	SmcCrcModeDisabled = 0,
	/*!
	 * \brief In this mode, you must append a CRC byte to all commands sent to the
	 *   device.
	 */
	SmcCrcModeCommands = 1,
	/*!
	 * \brief In this mode, you must append a CRC byte to all commands sent to the
	 *   device, and responses from the device will contain a CRC byte.
	 */
	SmcCrcModeCommandsAndResponses = 3,
};

/*!
 * \brief Specifies the alternate use setting for a channel.
 *
 * This setting is only relevant if the channel is not configured to control the
 * motor speed (see ::SmcInputMode and ::SmcMixingMode).
 */
enum SmcChannelAlternateUse
{
	/*!
	 * \brief None: This channel is not used for anything special but its value can
	 *   be read using Serial or USB commands.
	 */
	SmcChannelAlternateUseNone = 0,
	/*!
	 * \brief Forward Limit Switch: if this channel is active (Scaled Value >=
	 *   1600), then the motor is not allowed to move forward.
	 */
	SmcChannelAlternateUseLimitForward = 1,
	/*!
	 * \brief Reverse Limit Switch: if this channel is active (Scaled Value >=
	 *   1600), then the motor is not allowed to move in reverse.
	 */
	SmcChannelAlternateUseLimitReverse = 2,
	/*!
	 * \brief Kill Switch: if this channel is active (Scale Value >= 1600), then the
	 *   motor is not allowed to move.
	 */
	SmcChannelAlternateUseKillSwitch = 3
};

/*!
 * \brief Specifies the pin mode for an Analog input channel.
 *
 * This setting is not relevant for RC input cahnnels.
 */
enum SmcPinMode
{
	/*!
	 * \brief Floating: no pull-up or pull-down resistors enabled.
	 */
	SmcPinModeFloating = 0,
	/*!
	 * \brief Weak pull-up resistor (to 3.3 V) enabled.
	 */
	SmcPinModePullUp = 1,
	/*!
	 * \brief Weak pull-down resistor (to 0 V) enabled.
	 */
	SmcPinModePullDown = 2
};

/*!
 * \brief SMC Speed Limiting Causes
 *
 * Defines the bits in the LimitStatus register, which tells us what things are
 * currently limiting the motor speed
 */
enum SmcLimitStatus
{
	/*!
	 * \brief Motor is not allowed to run due to an error or safe-start violation.
	 */
	SmcLimitStatusStartedState = ( 1 << 0 ),
	/*!
	 * \brief Temperature is actively reducing Target Speed.
	 */
	SmcLimitStatusTemperature = ( 1 << 1 ),
	/*!
	 * \brief Max speed limit is actively reducing Target Speed.
	 *
	 * Only happens when Input Mode is Serial/USB.
	 */
	SmcLimitStatusMaxSpeed = ( 1 << 2 ),
	/*!
	 * \brief Starting speed limit is actively reducing Target Speed to 0.
	 *
	 * Only happens when Input Mode is Serial/USB.
	 */
	SmcLimitStatusStartingSpeed = ( 1 << 3 ),
	/*!
	 * \brief Motor speed is not equal to target speed because of acceleration,
	 *   deceleration, or brake duration limits.
	 */
	SmcLimitStatusAcceleration = ( 1 << 4 ),
	/*!
	 * \brief RC Channel 1 is configured as a limit/kill switch and it is active
	 *   (Scaled Value >= 1600).
	 */
	SmcLimitStatusRc1 = ( 1 << 5 ),
	/*!
	 * \brief RC Channel 2 is configured as a limit/kill switch and it is active
	 *   (Scaled Value >= 1600).
	 */
	SmcLimitStatusRc2 = ( 1 << 6 ),
	/*!
	 * \brief Analog Channel 1 is configured as a limit/kill switch and it is active
	 *   (Scaled Value >= 1600).
	 */
	SmcLimitStatusAnalog1 = ( 1 << 7 ),
	/*!
	 * \brief Analog Channel 2 is configured as a limit/kill switch and it is active
	 *   (Scaled Value >= 1600).
	 */
	SmcLimitStatusAnalog2 = ( 1 << 8 ),
	/*!
	 * \brief USB kill switch is active.
	 */
	SmcLimitStatusUsbKill = ( 1 << 9 )
};

/*!
 * \brief PWM Mode
 *
 * The PWM mode to use.  This feature is not exposed to users because
 * we found the DriveBrake mode to be much better than DriveCode mode.
 */
enum SmcPwmMode
{
	/*!
	 * \brief PWM between driving and braking (both low-side MOSFETs on).
	 */
	SmcPwmModeDriveBrake = 0,
	/*!
	 * \brief PWM between driving and coasting.
	 */
	SmcPwmModeDriveCoast = 1,
};

/*!
 * \brief SMC Errors
 *
 * Defines the different errors that the device has.
 */
enum SmcError
{
	/*!
	 * \brief None.
	 */
	SmcErrorNone = 0,
	/*!
	 * \brief See http://www.pololu.com/docs/0J44
	 */
	SmcErrorSafeStart = ( 1 << 0 ),
	/*!
	 * \brief This error occurs whenever any required RC or Analog channel is
	 *   invalid.
	 */
	SmcErrorChannelInvalid = ( 1 << 1 ),
	/*!
	 * \brief Serial/USB communication error
	 *
	 * This error occurs whenever the Input Mode is Serial/USB and something goes
	 * wrong with the serial communication, either on the RX/TX lines or on the
	 * USB virtual COM port.
	 */
	SmcErrorSerial = ( 1 << 2 ),
	/*!
	 * \brief Command timeout
	 *
	 * This error occurs if Input Mode is Serial/USB and the (configurable) time
	 * period has elapsed with no valid serial or USB commands being received by
	 * the controller.  See commandTimeout in SmcSettings.
	 */
	SmcErrorCommandTimeout = ( 1 << 3 ),
	/*!
	 * \brief This error occurs when a limit or kill switch channel stops the motor.
	 */
	SmcErrorLimitSwitch = ( 1 << 4 ),
	/*!
	 * \brief This error occurs whenever your power supply's voltage is too low or
	 *   it is disconnected.
	 */
	SmcErrorVinLow = ( 1 << 5 ),
	/*!
	 * \brief This error occurs whenever your power supply's voltage is too high.
	 */
	SmcErrorVinHigh = ( 1 << 6 ),
	/*!
	 * \brief This error occurs whenever the reading from the temperature sensor is
	 *   too high.
	 */
	SmcErrorTemperatureHigh = ( 1 << 7 ),
	/*!
	 * \brief Motor driver error
	 *
	 * This error occurs whenever the motor driver chip reports an under-voltage or
	 * over-temperature error (by driving its fault line low).
	 */
	SmcErrorMotorDriverError = ( 1 << 8 ),
	/*!
	 * \brief Error line high
	 *
	 * This error occurs whenever there are no other errors but the voltage on the
	 * ERR line is high (2.3-5 V).
	 */
	SmcErrorErrLineHigh = ( 1 << 9 )
};

/*!
 * \brief Defines the different serial errors that are recorded.
 */
enum SmcSerialError
{
	/*!
	 * \brief None.
	 */
	SmcSerialErrorNone = 0,
	/*!
	 * \brief This is not used.
	 */
	SmcSerialErrorParity = ( 1 << 0 ),
	/*!
	 * \brief This is error occurs when a de-synchronization or excessive noise on
	 *   the RX line is detected.
	 */
	SmcSerialErrorFrame = ( 1 << 1 ),
	/*!
	 * \brief This error occurs when noise is detected on the RX line.
	 */
	SmcSerialErrorNoise = ( 1 << 2 ),
	/*!
	 * \brief This error occurs when the buffer for storing bytes received on the RX
	 *   line is full and data was lost as a result.
	 */
	SmcSerialErrorRxOverrun = ( 1 << 3 ),
	/*!
	 * \brief This error occurs if the serial bytes received on RX or the virtual
	 *   COM port do not obey the protocol specified in this guide.
	 */
	SmcSerialErrorFormat = ( 1 << 5 ),
	/*!
	 * \brief This error occurs if you have enabled cyclic redundancy check (CRC)
	 *   for serial commands, but the CRC byte received was invalid.
	 */
	SmcSerialErrorCrc = ( 1 << 6 ),
};

/*!
 * \brief Specifies what motor limits to set.
 *
 * You can use the OR (|) operator to apply the ForwardOnly or ReverseOnly
 * modifiers to MaxSpeed, MaxAcceleration, MaxDeceleration, or BrakeDuration.
 * Without any modifies, this value specifies woth the forward and reverse
 * limits.
 */
enum SmcMotorLimit
{
	/*!
	 * \brief Maximum speed at which the controller will ever drive a motor.
	 *
	 * Should be a number between 0 and 3200.
	 */
	SmcMotorLimitMaxSpeed = 0,
	/*!
	 * \brief How much the magnitude of the motor speed may increase in one update
	 *   period.
	 *
	 * Should be a number between 0 and 3200. A value of 0 indicates no limit.
	 */
	SmcMotorLimitMaxAcceleration = 1,
	/*!
	 * \brief How much the magnitude of the motor speed may decrease in one update
	 *   period.
	 *
	 * Should be a number between 0 and 3200. A value of 0 indicates no limit.
	 */
	SmcMotorLimitMaxDeceleration = 2,
	/*!
	 * \brief Amount of time that the controller will brake the motor before
	 *   allowing reversal.
	 */
	SmcMotorLimitBrakeDuration = 3,
	/*!
	 * \brief A modifier that specifies that only the forward limit should be set.
	 */
	SmcMotorLimitForwardOnly = 4,
	/*!
	 * \brief A modified that specifies that only the reverse limit should be set.
	 */
	SmcMotorLimitReverseOnly = 8,
};

/*!
 * \brief Specifies the return code from a Set Motor Limit command.
 */
enum SmcSetMotorLimitProblem
{
	/*!
	 * \brief There were no problems with the set motor limit command.
	 */
	SmcSetMotorLimitProblemNone = 0,
	/*!
	 * \brief The value you were trying to set was more dangerous than the hard
	 *   limit for the Forward direction, so the hard limit was used instead.
	 *
	 * This may be the desired behavior.
	 */
	SmcSetMotorLimitProblemForwardConflict = 1,
	/*!
	 * \brief The value you were trying to set was more dangerous than the hard
	 *   limit for the Reverse direction, so the hard limit was used instead.
	 *
	 * This may be the desired behavior.
	 */
	SmcSetMotorLimitProblemReverseConflict = 2,
};

/*!
 * \brief Specifies the causes of the device's last reset.
 */
enum SmcResetFlags
{
	/*!
	 * \brief The device was reset because the voltage on the Reset pin went low.
	 */
	SmcResetFlagsResetPin = 0x04,
	/*!
	 * \brief The device was reset because power was turned off/on.
	 */
	SmcResetFlagsPower = 0x0C,
	/*!
	 * \brief The device was reset by software running on the device.
	 *
	 * This happens at the end of a firmware upgrade when the bootloader starts the
	 * new firmware.
	 */
	SmcResetFlagsSoftware = 0x14,
	/*!
	 * \brief The device was reset by the watchdog timer.
	 *
	 * This indicates a problem with the firmware which should be reported to
	 * pololu.com.
	 */
	SmcResetFlagsWatchdog = 0x24,
};

/*!
 * \brief Node in an SMC list
 *
 * This structure is used when a list of attached SMC devices is requested.
 */
struct SmcList
{
	/*!
	 * \brief Vendor ID of this SMC
	 */
	uint16_t idVendor;
	/*!
	 * \brief Product ID of this SMC
	 */
	uint16_t idProduct;
	/*!
	 * \brief Serial number of this SMC (NULL if unable to open)
	 */
	char *iSerialNumber;
	/*!
	 * \brief Pointer to next SMC device (NULL if at end of list)
	 */
	struct SmcList *next;
};

/*!
 * \brief Error codes for the SMC driver.
 *
 * Based on the error codes for LibUSB. For the most part, a return value of 0
 * indicates success.
 */
enum smc_error
{
	/*!
	 * \brief Success (no error)
	 */
	SMC_SUCCESS = LIBUSB_SUCCESS,
	/*!
	 * \brief Input/output error
	 */
	SMC_ERROR_IO = LIBUSB_ERROR_IO,
	/*!
	 * \brief Invalid parameter
	 */
	SMC_ERROR_INVALID_PARAM = LIBUSB_ERROR_INVALID_PARAM,
	/*!
	 * \brief Access denied (insufficient permissions)
	 */
	SMC_ERROR_ACCESS = LIBUSB_ERROR_ACCESS,
	/*!
	 * \brief No such device (it may have been disconnected)
	 */
	SMC_ERROR_NO_DEVICE = LIBUSB_ERROR_NO_DEVICE,
	/*!
	 * \brief Entity not found
	 */
	SMC_ERROR_NOT_FOUND = LIBUSB_ERROR_NOT_FOUND,
	/*!
	 * \brief Resource busy
	 */
	SMC_ERROR_BUSY = LIBUSB_ERROR_BUSY,
	/*!
	 * \brief Operation timed out
	 */
	SMC_ERROR_TIMEOUT = LIBUSB_ERROR_TIMEOUT,
	/*!
	 * \brief Overflow
	 */
	SMC_ERROR_OVERFLOW = LIBUSB_ERROR_OVERFLOW,
	/*!
	 * \brief Pipe error
	 */
	SMC_ERROR_PIPE = LIBUSB_ERROR_PIPE,
	/*!
	 * \brief System call interrupted (perhaps due to signal)
	 */
	SMC_ERROR_INTERRUPTED = LIBUSB_ERROR_INTERRUPTED,
	/*!
	 * \brief Insufficient memory
	 */
	SMC_ERROR_NO_MEM = LIBUSB_ERROR_NO_MEM,
	/*!
	 * \brief Operation not supported or unimplemented on this platform
	 */
	SMC_ERROR_NOT_SUPPORTED = LIBUSB_ERROR_NOT_SUPPORTED,
	/*!
	 * \brief Other error
	 */
	SMC_ERROR_OTHER = LIBUSB_ERROR_OTHER,
};

/*!
 * \brief SMC Driver initialization routine.
 *
 * \author Scott K Logan
 *
 * Allocates memory and initializes LibUSB. This function should be called
 * before any of the other driver functions are used.
 *
 * \returns ::smc_error code
 */
int smc_init( );

/*!
 * \brief Open a new connection to an SMC device.
 *
 * \author Scott K Logan
 *
 * Searches through the connected USB devices for a device that matches the
 * Pololu vendor ID and one of the SMC product IDs. If a serial number is
 * specified, only a matching serial number will result in a proper device
 * open.
 *
 * \param serial C-string of the serial number of the device to connect to. If
 *   the string is empty, the first available device is chosen.
 *
 * \returns New SMC handle to be use for future references to that device
 * (which is greater than -1), otherwise an ::smc_error code.
 */
int smc_open( const char *serial );

/*!
 * \brief Causes a USB kill.
 *
 * \author Scott K Logan
 *
 * Triggers the USB killswitch and if safe start is enabled, causes a safe
 * start violation.
 *
 * \param smcd SMC device handle
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_stop( const int smcd, const unsigned int to );

/*!
 * \brief Clears a USB kill and/or safe start violation.
 *
 * \author Scott K Logan
 *
 * Clears a USB kill and causes the device to leave a safe start state.
 *
 * \param smcd SMC device handle
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_resume( const int smcd, const unsigned int to );

/*!
 * \brief Retrieves the firmware version from the device.
 *
 * \author Scott K Logan
 *
 * Simple routine for fetching the firmware version from the device. Note that
 * these values will never change during this power cycle.
 *
 * \param smcd SMC device handle
 * \param major Non-null pointer to a short int. If the function returns
 *   ::SMC_SUCCESS, this will be populated with the major version number.
 * \param minor Non-null pointer to a short int. If the function returns
 *   ::SMC_SUCCESS, this will be populated with the minor version number.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_get_fw_version( const int smcd, unsigned short int *major,
	unsigned short int *minor, const unsigned int to );

/*!
 * \brief Retrieves the serial number from USB.
 *
 * \author Scott K Logan
 *
 * This routine fetches the serial number from the system, which was set when
 * the device was connected. Note that this value will never change during this
 * power cycle.
 *
 * \param smcd SMC device handle
 * \param sn Non-null pointer to a character array of size >= 256
 *
 * \returns ::smc_error code
 */
int smc_get_serial( const int smcd, char *sn );

/*!
 * \brief Resets settings to factory defaults.
 *
 * \author Scott K Logan
 *
 * All settings in the flash memory will be reset to the default values.
 *
 * \param smcd SMC device handle
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_reset_settings( const int smcd, const unsigned int to );

/*!
 * \brief Sets the speed of the motor.
 *
 * \author Scott K Logan
 *
 * This is probably called more often than any other function. It changes the
 * motor controller's duty cycle to change the speed of the motor.
 *
 * \param smcd SMC device handle
 * \param val Value to set speed to. For forward and backward, range is from 0
 *   to 3200. For braking, range is from 0 to 32.
 * \param dir Direction of travel, 0 is for braking, 1 is foreward, -1 is
 *   backward.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_set_speed( const int smcd, const unsigned int val,
	const short int dir, const unsigned int to );

/*!
 * \brief Fetches the cause of the last reset.
 *
 * \author Scott K Logan
 *
 * Gets the cause of the device's last reset, which is one of the possibilities
 * for ::SmcResetFlags.
 *
 * \param smcd SMC device handle
 * \param reset_flags Non-null pointer to a character. If the function returns
 *   ::SMC_SUCCESS, this will be populated with ::SmcResetFlags.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_get_reset_flags( const int smcd, char *reset_flags,
	const unsigned int to );

/*!
 * \brief Gets the variables from the device.
 *
 * \author Scott K Logan
 *
 * Fetches the non-ROM variables from the device, including many diagnostic
 * values.
 *
 * \param smcd SMC device handle
 * \param vars Non-null pointer to an ::SmcVariables struct. If the function
 *   returns ::SMC_SUCCESS, this will be populated with the variables from the
 *   device.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_get_variables( const int smcd, struct SmcVariables *vars,
	const unsigned int to );

/*!
 * \brief Gets the settings from the device.
 *
 * \author Scott K Logan
 *
 * Fetches the ROM values from the device, including many configurable
 * parameters.
 *
 * \param smcd SMC device handle
 * \param set Non-null pointer to an ::SmcSettings struct. If the function
 *   returns ::SMC_SUCCESS, this will be populated with the current settings of
 *   the device.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_get_settings( const int smcd, struct SmcSettings *set,
	const unsigned int to );

/*!
 * \brief Sends new settings to the device.
 *
 * \author Scott K Logan
 *
 * Stores the given settings in the flash ROM of the device.
 *
 * \param smcd SMC device handle
 * \param set Non-null pointer to an ::SmcSettings struct. The data at this
 *   address will be sent to the device and written to the flash.
 * \param to Timeout associated with the control transfer
 *
 * \returns ::smc_error code
 */
int smc_set_settings( const int smcd, struct SmcSettings *set,
	const unsigned int to );

/*!
 * \brief Checks the internal status of an SMC device handle.
 *
 * \author Scott K Logan
 *
 * Queries the internal workings of the communications link in the driver.
 *
 * \param smcd SMC device handle
 *
 * \returns ::smc_error code
 */
int smc_stat( const int smcd );

/*!
 * \brief Closes communication with an SMC device, and restores kernel control
 *   if necessary.
 *
 * \author Scott K Logan
 *
 * After calling this, all refrences to the device should be cleared, as the
 * handle is no longer valid.
 *
 * \param smcd SMC device handle
 */
void smc_close( const int smcd );

/*!
 * \brief Closes all remaining handles and frees driver memory.
 *
 * \author Scott K Logan
 *
 * This should be called at program termination to free memory. After calling
 * this, all references to SMC device handles should be cleared, as they
 * are no longer valid. No other functions should be called until ::smc_init
 * is called again.
 */
void smc_exit( );

/*!
 * \brief Lists the SMC devices on the system.
 *
 * \author Scott K Logan
 *
 * This function creates a linked list of all of the SMC devices on the system.
 * The list is of type ::SmcList. After the list is no longer used, use
 * ::smc_free_list to free the memory.
 */
struct SmcList * smc_list_devices( );

/*!
 * \brief Frees a list of SMC devices.
 *
 * \author Scott K Logan
 *
 * This function frees a list of SMCs which was generated by ::smc_list_devices.
 *
 * \param lst Pointer to first element of list to be freed
 */
void smc_free_list( struct SmcList *lst );

/*!
 * \brief Returns a string representation of the given SMC idProduct.
 *
 * \author Scott K Logan
 *
 * A simple switch-lookup which maps USB product IDs to Pololu SMC model names.
 *
 * \param idProduct Product ID in question
 */
const char * smc_lookup( uint16_t idProduct );

/*!
 * \brief Returns a string representation of the given SMC device.
 *
 * \author Scott K Logan
 *
 * This function returns the model name of an open SMC device.
 *
 * \param smcd SMC device handle
 */
const char * smc_get_model( int smcd );

#ifdef __cplusplus
}
#endif

#endif /* _smc_h */
