// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Martí Morta Garriga  (mmorta@iri.upc.edu)
// All rights reserved.
//
// Copyright (C) 2013 Jochen Sprickerhof <jochen@sprickerhof.de>
//
// This file is part of IRI EPOS2 Driver
// IRI EPOS2 Driver is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef Epos2_H
#define Epos2_H

#include <string>
#include <stdexcept>
#include <ftdi.hpp>

/*! \class CEpos2
 \brief Implementation of a driver for EPOS2 Motor Controller
 \author Martí Morta (mmorta @ iri.upc.edu)
 \version 1.0
 \date august 2009

 This class is a low level driver to manage Maxon Motor EPOS2 Controller.
 It is tested in EPOS2 50/5 using firmware Epos_2110h_6322h_0000h_0000h.bin.
 All functions objects in controller are in the document EPOS2 Firmware
 Specification of December 2008 Edition/ document number 821384-02.

 It implements all functions to use one unique controller and adds other useful
 features to simplify some tasks.

 \warning Units are in default (position:[qc](=4*envoder counts/revolution),
 velocity: [rev/min], acceleration: [rev/min/s]) if you change dimension
 or notation index you will have to take them into account.

 \warning This class doesn't implements operation modes that need other
 controllers or * external signals. Neither CAN, SDO, PDO functions of the
 controller.

 \todo
   - Smarter ReadObject (16-32 bits)
   - Current mode functions
   - Sensor functions
   - Motor functions
   - Utilities functions: window/window times, notation/dimension index,
     analog Output
   .
*/

class CEpos2 {


	private:

    int8_t  node_id;

    /**
     * \brief a reference to the FTDI USB device
     *
     * This attribute points to the communication device used to send and receive data
     * to and from the EPOS2. The communication device is cretaed inside the
     * object at initialization time using the CFTDIServer and the description or serial
     * number of the desired platform via the connect() function.
     *
     * It is not possible to send or receive data to or from the epos2 until the
     * driver is not connected to the hardware platform. Any attempt to do so will
     * result in an exception being thrown.
     *
     */
    static Ftdi::Context ftdi;
    static bool ftdi_initialized;


   /// @name Communication low level
   /// @{


    /**
     * \brief open EPOS2 device using CFTDI
     *
     * It finds and configures the FTDI communication.
     */
    void openDevice();

    /**
     * \brief function to read an object from the EPOS2
     *
     *  This function sends a read object request using CFTDI.
     *
     *  \param index the hexadecimal index of the object you want to read
     *  \param subindex hexadecimal value of the object (usually 0x00)
     *  \return the value in the object as a long
     */
    int32_t readObject(int16_t index, int8_t subindex);

    /**
     * \brief function to write an object to the EPOS2
     *
     *  This function sends a write object request using CFTDI.
     *
     *  \param index the hexadecimal index of the object you want to read
     *  \param subindex hexadecimal value of the object (usually 0x00)
     *  \param data information to send
     *  \return returned value by write
     */
    int writeObject(int16_t index, int8_t subindex, int32_t data);

    /**
     * \brief function send a frame to EPOS2
     *
     *  It transforms the 16 bit frame to 8 bit and does data stuffing for
     *  character 0x90.
     *
     *  \param frame data frame which will be sent to EPOS2
     */
    void sendFrame(int16_t *frame);

    /**
     * \brief function receive a frame from EPOS2
     *
     *  It uses CFTDI events in order to get complete frames of data from EPOS2.
     *  While frame is being received it calculates its length using the fourth
     *  piece of data and checks if data pieces have been stuffed for character
     *  0x90, If there are, frame length is changed.
     *  Also It converts the 8 bit received frame to 16 bit frame.
     *
     *  \param frame data frame which will be saved from EPOS2
     */
    void receiveFrame(uint16_t* ans_frame);

    /**
     * \brief function to compute EPOS2 checksum
     *
     *  This function is implemented in the EPOS2 Communication Guide.
     *
     *  \param pDataArray
     *  \param numberOfWords
     *  \return checksum (16 bits)
     */
    int16_t computeChecksum(int16_t *pDataArray, int16_t numberOfWords);


///@}

    /*!
    \brief function to make a unsigned long signed

    This function converts an unsigned long to a signed value

    \param number unsigned long number needed to convert
    \return signed number
    */
    long getNegativeLong  (long number);

    bool verbose;

    /*!
    \brief function to make a unsigned long signed

    This function converts an unsigned long to a signed value

    \param number unsigned long number needed to convert
    \return signed number
     */
    void p(const char *text);


    void p(const std::stringstream& text);


	public:

		/*! \brief Constructor
		*/
		CEpos2(int8_t nodeId = 0x00);

		/*! \brief Destructor
		*/
		~CEpos2();

    /*! \brief Connects hardware
     */
    void init();

    /*! \brief Disconnects hardware
     */
    void close();

/// @name State Management
/// @{

		/**
		 * \brief function to get current controller's state
		 *
		 *  \ref state_machine See EPOS2 state machine
		 *  \return arbitrary state number of STATES
		 */
		long getState			();

		/**
		 * \brief function to reach ready_to_switch_on state
		 *
		 *  Transitions: 2,6 \ref state_machine See EPOS2 state machine
		 */
		void shutdown			();

		/**
		 * \brief function to reach switch_on state
		 *
		 *  Transitions: 3 \ref state_machine See EPOS2 state machine
		 */
		void switchOn			();

		/**
		 * \brief function to reach switch_on state
		 *
		 *  Transitions: 7,10,11 \ref state_machine See EPOS2 state machine
		 */
		void quickStop			();

		/**
		 * \brief function to reach operation_enable state and enables power on motor
		 *
		 *  Transition: 4,16 \ref state_machine See EPOS2 state machine
		 */
		void enableOperation		();

		/**
		 * \brief function to reach switch_on and disables power on motor
		 *
		 *  Transition: 5 \ref state_machine See EPOS2 state machine
		 */
		void disableOperation		();

		/**
		 * \brief function to reach switch_on_disabled state
		 *
		 *  Transition: 7,9,10,12 \ref state_machine See EPOS2 state machine
		 */
		void disableVoltage		();

		/**
		 * \brief function to reach switch_on_disabled state after a FAULT
		 *
		 *  This function does the transition to reach switch_on_disabled state after
		 *  a FAULT. It is necessary to clear Can passive mode error at the start
		 *  of EPOS controller
		 *
		 *  Transition: 15 \ref state_machine See EPOS2 state machine
		 */
		void faultReset			();

/// @name Operation Modes Management
/// @{

		/**
		 * \brief function to know which operation mode is set
		 *
		 *  This function gets the actual operation mode and returns its value as an
		 *  integrer
		 *  \return operation mode
		 */
		long getOperationMode		();

		/**
		 * \brief function to show the name of the operation mode
		 *
		 *  \param opmode integrer returned by getOperationMode
		 */
    std::string getOpModeDescription	(long opmode);

		/**
		 * \brief function to set the operation mode
		 *
		 *  \param opmode desired operation mode (one of epos_opmodes)
		 */
		void setOperationMode		(long opmode);

		/**
		 * \brief function to facititate transitions from the start of the controller to switch it on
		 *
		 *  This function does all transitions from start to switched on to be able
     *  to work with the controller.
		 */
		void enableController		();

		/**
		 * \brief function to facititate transitions from switched on to operation enabled
		 *
		 *  This function does all transitions from switched on to operation enabled
     *  work with the motor.
		 *
		 *  \param opmode desired operation mode (velocity, profile_position,
     *  profile_velocity,current,homing)
		 */
		void enableMotor		(long opmode);

		/**
		 * \brief function to know if the motor has reached the target position or velocity
		 *
		 *  \pre Operation Mode = profile_position or profile_velocity
		 *  \return A boolean, target reached
		 */
		bool isTargetReached	();

///@}


/// \anchor operation_modes
/// @name Operation Modes - Names
/// @{
    /*! \enum epos_states
        EPOS2 firmware states
    */
    enum epos_states{
      FAULT, START, NOT_READY_TO_SWITCH_ON, SWITCH_ON_DISABLED,
      READY_TO_SWITCH_ON, SWITCH_ON, REFRESH, MEASURE_INIT, OPERATION_ENABLE,
      QUICK_STOP, QUICK_STOP_ACTIVE_DISABLE, QUICK_STOP_ACTIVE_ENABLE };

    /*! \enum epos_opmodes
        EPOS2 firmware (9) operation modes.
        \note "no_operation" is an arbitrary number
     */
    enum epos_opmodes{
      NO_OPERATION = 99, VELOCITY = -2, POSITION = -1, PROFILE_VELOCITY = 3,
      PROFILE_POSITION = 1, INTERPOLATED_PROFILE_POSITION = 7, HOMING = 6 };

    /*! \enum epos_posmodes
        EPOS2 firmware profile position modes
     */
    enum epos_posmodes{
      HALT, ABSOLUTE, RELATIVE };

/// @}

/// @name Operation Mode - velocity
/// @{
		/**
		 * \brief function to GET the target velocity
		 *  This function gets the target velocity of velocity operation mode
		 *  \return Target Velocity
		 */
		long getTargetVelocity		();

		/**
		 * \brief function to SET the target velocity
		 *
		 *  This function sets the target velocity of velocity operation mode
		 *
		 *  \param velocity Target Velocity
		 */
		void setTargetVelocity		(long velocity);

		/**
		 * \brief function to move the motor in Velocity mode
		 *
		 *  This function makes move the motor if velocity mode is the actual operation mode
		 *
		 *  \pre Operation Mode = velocity
		 */
		void startVelocity		();

		/**
		 * \brief function to stop the motor in velocity mode
		 *
		 *  This function stops the motor if velocity mode is the actual operation mode
		 *
		 *  \pre Operation Mode = velocity
		 */
		void stopVelocity		();

///@}

/// @name Operation Mode - profile_position
/// @{
		/**
		 * \brief function to GET the target position
		 *
		 *  This function gets the target position of profile position operation mode
		 *
		 *  \return Target position
		 */
		long getTargetProfilePosition	();

		/**
		 * \brief  function to SET the target position
		 *
		 *  This function sets the target position of profile position operation mode
		 *
		 *  \param position Target position
		 */
		void setTargetProfilePosition		(long position);

		/**
		 * \brief function to move the motor to a position in profile position mode
		 *
		 *  This function makes move the motor if profile position mode is the actual operation mode
		 *  epos_posmodes: halt:0, absolut:1, relative:2
		 *
		 *  \pre Operation Mode = profile_position
		 *  \param mode epos_posmodes
		 *  \param blocking If it block the program or if it leave the program manage position reached
     *  \param wait If it has to wait current movement to finish or just start the new one
     *  \param new_point Assume Target position (seems that it doesn't do anything)
		 */
    void startProfilePosition (epos_posmodes mode, bool blocking=true, bool wait=true, bool new_point=true);

///@}

/// @name Operation Mode - profile_velocity
/// @{

		/**
		 * \brief [OPMODE=profile_velocity] function to get the velocity
		 *
		 *  \pre Operation Mode = profile_velocity
		 *  \return Target Velocity of the profile
		 */
		long getTargetProfileVelocity	();

		/**
		 * \brief [OPMODE=profile_velocity] function to set the velocity
		 *
		 *  \pre Operation Mode = profile_velocity
		 *  \param velocity desired velocity
 		*/
		void setTargetProfileVelocity	(long velocity);

		/**
		 * \brief [OPMODE=profile_velocity] function to move the motor in a velocity
		 *
		 *  \pre Operation Mode = profile_velocity
		 */
		void startProfileVelocity	();

		/**
		 * \brief [OPMODE=profile_velocity] function to stop the motor
		 *
		 *  \pre Operation Mode = profile_velocity
		 */
		void stopProfileVelocity		();

///@}

/// @name Operation Mode - current
/// @{

		/** \todo CURRENT MODE FUNCTIONS
		 * \brief [OPMODE=current] function to GET the target current
		 *
		 *  This function gets the target current of current operation mode
		 *
		 *  \return Target current
		 */
		long getTargetCurrent		();

		/**
		 * \brief [OPMODE=current] function to SET the target current
		 *
		 *  This function sets the target current of current operation mode
		 *
		 * \param current Target current
		 */
		void setTargetCurrent		(long current);

		/**
		 * \brief [OPMODE=current] function to move the motor in current mode
		 *
		 *  This function makes move the motor if current mode is the actual operation mode
		 *
		 * \pre Operation Mode = current
		 */
		void startCurrent		();

		/**
		 * \brief [OPMODE=current] function to stop the motor in current mode
		 *
		 *  This function stops the motor if current mode is the actual operation mode
		 *
		 *  \pre Operation Mode = current
		 */
		void stopCurrent		();
///@}

/// @name Operation Mode - home
/// @{

	/**
		 * \brief function to set a Home position with user interaction
		 *
		 *  This function sets a new home position with user interaction.
		 *  User has to follow instructions given when the function is called.
		 *  It disables power, then user has to rotate the load and then
		 *  the function reads the actual position and set there the new home
		 *
		 *  	disable -> user rotate -> readPosition -> setHomePosition
	 */
		void setHome			();

		/**
		 * \brief function to set a Home position
		 *
		 *  This function sets a home position.
		 *
		 *  \param home_position_qc position to set home
		 */
		void setHomePosition		(long home_position_qc);

		/**
		 * \brief function to get the Home position
		 *
		 *  This function gets a home position.
		 *
		 *  \return home position
		 */
		long getHomePosition		();
///@}


/** @name Control Tuning
 *  Motion Control Configuration
 */
/// @{

		/**
		 * \brief function to get P Current Gain
		 *
		 *  This function gets the actual P Current Gain
		 *
		 *  \return gain
		 */
		long getCurrentPGain		();

		/**
		 * \brief function to set P Current Gain
		 *
		 *  This function sets Current Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setCurrentPGain		(long gain);

		/**
		 * \brief function to get I Current Gain
		 *
		 *  This function gets the actual I Current Gain
		 *
		 *  \return gain (must be between 0 and 32767)
		 */
		long getCurrentIGain		();

		/**
		 * \brief function to set I Current Gain
		 *
		 *  This function sets Current Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setCurrentIGain		(long gain);

	// Velocity

		/**
		 * \brief function to get P Velocity Gain
		 *
		 *  This function gets the actual P Velocity Gain
		 *
		 *  \return gain
		 */
		long getVelocityPGain		();

		/**
		 * \brief function to set P Velocity Gain
		 *
		 *  This function sets P Velocity Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setVelocityPGain		(long gain);

		/**
		 * \brief function to get I Velocity Gain
		 *
		 *  This function gets the actual I Velocity Gain
		 *
		 *  \return gain
		 */
		long getVelocityIGain		();

		/**
		 * \brief function to set I Velocity Gain
		 *
		 *  This function sets Velocity Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setVelocityIGain		(long gain);

		/**
		 * \brief function to GET Speed Regulator Set Point Factor P Velocity Gain
		 *
		 *  This function gets the actual Speed Regulator Set Point Factor P Velocity Gain
		 *
		 *  \return gain
		 */
		long getVelocitySetPointFactorPGain	();

		/**
		 * \brief function to SET Speed Regulator Set Point Factor P Velocity Gain
		 *
		 *  This function sets Speed Regulator Set Point Factor P Velocity Gain
		 *
		 *  0..32767
		 *
		 *  \param gain
		 */
		void setVelocitySetPointFactorPGain	(long gain);

	// Position

		/**
		 * \brief function to get P Position Gain
		 *
		 *  This function gets the actual P Position Gain
		 *
		 *  \return gain
		 */
		long getPositionPGain		();

		/**
		 * \brief function to set P Position Gain
		 *
		 *  This function sets the actual P Position Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setPositionPGain		(long gain);

		/**
		 * \brief function to get I Position Gain
		 *
		 *  This function gets the actual I Position Gain
		 *
		 *  \return gain
		 */
		long getPositionIGain		();

		/**
		 * \brief function to set I Position Gain
		 *
		 *  This function sets the actual I Position Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setPositionIGain		(long gain);

		/**
		 * \brief function to get D Position Gain
		 *
		 *  This function gets the actual D Position Gain
		 *
		 *  \return gain
		 */
		long getPositionDGain		();

		/**
		 * \brief function to set D Position Gain
		 *
		 *  This function sets the actual D Position Gain
		 *
		 *  \param gain (must be between 0 and 32767)
		 */
		void setPositionDGain		(long gain);

		/**
		 * \brief function to get Velocity Feed Forward Position Gain
		 *
		 *  This function gets the actual Velocity Feed Forward Position Gain
		 *
		 *  \return gain
		 */
		long getPositionVFFGain		();

		/**
		 * \brief function to set Velocity Feed Forward Position Gain
		 *
		 *  This function sets the actual Velocity Feed Forward  Position Gain
		 *
		 *  \param gain (must be between 0 and 65535)
		 */
		void setPositionVFFGain		(long gain);

		/**
		 * \brief function to get Acceleration Feed Forward  Position Gain
		 *
		 *  This function gets the actual Acceleration Feed Forward  Position Gain
		 *
		 *  \return gain
		 */
		long getPositionAFFGain		();

		/**
		 * \brief function to set Acceleration Feed Forward Position Gain
		 *
		 *  This function sets the actual Acceleration Feed Forward  Position Gain
		 *
		 *  \param gain (must be between 0 and 65535)
		 */
		void setPositionAFFGain		(long gain);

	// General

		/**
		 * \brief function to get all control parameters
		 *
		 *  gets all control parameters and save them into user variables
		 *
		 *  \retval cp Current Proportional
		 *  \retval ci Current Integral
		 *  \retval vp Velocity Proportional
		 *  \retval vi Velocity Integral
		 *  \retval vspf Velocity Set Point Factor P Gain
		 *  \retval pp Postition Proportional
		 *  \retval pi Position Integral
		 *  \retval pd Position Derivative
		 *  \retval pv Position Velocity Feed Forward Factor
		 *  \retval pa Position Acceleration Feed Forward Factor
		 */
		void getControlParameters(long &cp,long &ci,long &vp,long &vi,long &vspf, long &pp,long &pi,long &pd,long &pv,long &pa);

		/**
		 * \brief function to set all control parameters
		 *
		 *  Sets all control parameters
		 *
		 *  \param cp Current Proportional
		 *  \param ci Current Integral
		 *  \param vp Velocity Proportional
		 *  \param vi Velocity Integral
		 *  \param vspf Velocity Set Point Factor P Gain
		 *  \param pp Postition Proportional
		 *  \param pi Position Integral
		 *  \param pd Position Derivative
		 *  \param pv Position Velocity Feed Forward Factor
		 *  \param pa Position Acceleration Feed Forward Factor
		 */
		void setControlParameters(long cp,long ci,long vp,long vi,long vspf,long pp,long pi,long pd,long pv,long pa);

		/**
		 * \brief function to show all control parameters
		 *
		 *  Prints all control parameters got before with getControlParameters in cout
		 *
		 *  \param cp Current Proportional
		 *  \param ci Current Integral
		 *  \param vp Velocity Proportional
		 *  \param vi Velocity Integral
		 *  \param vspf Velocity Set Point Factor P Gain
		 *  \param pp Postition Proportional
		 *  \param pi Position Integral
		 *  \param pd Position Derivative
		 *  \param pv Position Velocity Feed Forward Factor
		 *  \param pa Position Acceleration Feed Forward Factor
		 */
		void printControlParameters(long cp,long ci,long vp,long vi,long vspf,long pp,long pi,long pd,long pv,long pa);

///@}


/*! @name Profile
 * For Profile Position and Profile Velocity modes.
 */

/// @{

		/**
		 * \brief function to GET the velocity of the Velocity Profile
		 *
		 *  This function gets the velocity of the velocity profile used in
		 *  Profile Position and Profile Velocity
		 *  [rev/min]
		 *
		 *  \return Profile Velocity
		 */
		long getProfileVelocity	(void);


		/**
		 * \brief function to SET the velocity of the Velocity Profile
		 *
		 *  This function sets the velocity of the velocity profile used in
		 *  Profile Position and Profile Velocity
		 *  [rev/min]
		 *
		 *  \param velocity
		 */
		void setProfileVelocity		(long velocity);

		/**
		 * \brief function to GET the Max Velocity allowed in Profile
		 *
		 *  This function gets the Max Velocity allowed in Profile
		 *  [rev/min]
		 *
		 *  \return Max Velocity
		 */
		long getProfileMaxVelocity	(void);

		/**
		 * \brief function to SET the Max Velocity allowed in Profile
		 *
		 *  This function sets the Max Velocity allowed in Profile
		 *  [rev/min]
		 *
		 *  \param velocity
		 */
		void setProfileMaxVelocity	(long velocity);

		/**
		 * \brief function to GET the Acceleration in profile
		 *
		 *  This function gets the Acceleration in profile
		 *  [rev/min/s]
		 *
		 *  \return acceleration
		 */
		long getProfileAcceleration	(void);

		/**
		 * \brief function to SET the Acceleration in profile
		 *
		 *  This function sets the Acceleration in profile
		 *  [rev/min/s]
		 *
		 *  \param acceleration
		 */
		void setProfileAcceleration	(long acceleration);

		/**
		 * \brief function to GET the Deceleration in profile
		 *
		 *  This function gets the Deceleration in profile
		 *  [rev/min/s]
		 *
		 *  \return Deceleration
		 */
		long getProfileDeceleration	(void);

		/**
		 * \brief function to SET the Deceleration in profile
		 *
		 *  This function sets the Max Deceleration in profile
		 *  [rev/min/s]
		 *
		 *  \param deceleration
		 */
		void setProfileDeceleration	(long deceleration);

		/**
		 * \brief function to GET the Quick Stop Deceleration in profile
		 *
		 *  This function gets the Quick Stop Deceleration in profile
		 *  [rev/min/s]
		 *
		 *  \return deceleration
		 */
		long getProfileQuickStopDecel(void);

		/**
		 * \brief function to SET the Quick Stop Deceleration in profile
		 *
		 *  This function sets the Quick Stop Deceleration in profile
		 *  [rev/min/s]
		 *
		 *  \param deceleration
		 */
		void setProfileQuickStopDecel	(long deceleration);

		/**
		 * \brief function to GET the type in profile
		 *
		 *  This function gets the type in profile
		 *  trapezoidal if 0, sinusoidal otherwise
		 *
		 *  \return Max Position Limit
		 */
		long getProfileType		(void);

		/**
		 * \brief function to SET the type in profile
		 *
		 *  This function sets the type in profile (0=Trapezoidal or 1=Sinusoidal)
		 *
		 *  \param type
		 */
		void setProfileType		(long type);

		/**
		 * \brief function to GET Max Acceleration
		 *
		 *  This function gets Max Acceleration
		 *  [rev/min/s]
		 *
		 *  \return acceleration
		 */
		long getMaxAcceleration	(void);

		/**
		 * \brief function to SET Max Acceleration
		 *
		 *  This function sets Max Acceleration
		 *  [rev/min/s]
		 *
		 *  \param max_acceleration
		 */
		void setMaxAcceleration	(long max_acceleration);

	// General

		/**
		 * \brief function to GET all data of the velocity profile
		 *
		 *  This function gets all data and stores it in the output parameters defined.
		 *
		 *  \retval &vel Profile Velocity
		 *  \retval &maxvel Profile Max Velocity
		 *  \retval &acc Profile Acceleration
		 *  \retval &dec Profile Deceleration
		 *  \retval &qsdec Profile Quick Stop Deceleration
		 *  \retval &maxacc Profile Max Acceleration
		 *  \retval &type Profile Type
		 *
		 */
		void getProfileData		(long &vel,long &maxvel,long &acc,long &dec,long &qsdec, long &maxacc, long &type);

		/**
		 * \brief function to SET all data of the velocity profile
		 *
		 *  This function sets all data of the velocity profile.
		 *
		 *  \param vel Profile Velocity
		 *  \param maxvel Profile Max Velocity
		 *  \param acc Profile Acceleration
		 *  \param dec Profile Deceleration
		 *  \param qsdec Profile Quick Stop Deceleration
		 *  \param maxacc Profile Mac Acceleration
		 *  \param type Profile Type
		 *
		 */
		void setProfileData		(long vel,long maxvel,long acc,long dec,long qsdec,long maxacc,long type);

///@}


/// \name Info
/// @{
		/**
		 * \brief function to read motor average velocity
		 *
		 *  This function reads the actual velocity of the motor
		 *  [rev/min] -2147483648..2147483647
		 *
		 *  \return averaged velocity
		 */
		long readVelocity		();

		/**
		 * \brief function to read velocity sensor actual
		 *
		 *  \b Description
		 *  The velocity sensor actual value is given in quadcounts per second [inc/s].
		 *  \b Remarks
		 *  The resolution of the short time velocity measurement (Velocity actual value, Velocity sensor actual value) is dependent on the encoder pulse number (Sensor Configuration) and the velocity measurement method (Miscellaneous Configuration bit 3). To improve the short time velocity measurement resolution set the Miscellaneous Configuration bit 3 to 1 or use an encoder with higher resolution.
		 *  For example the short time velocity resolution with a 500-pulse encoder and Miscellaneous Configuration Bit 3 = 0 is: 1 quadcount / ms = 60’000 / (4 * 500) = 30 rpm.
		 *
		 *  [qc/s]
		 *
		 *  \return sensor velocity
		 */
		long readVelocitySensorActual	();

		/**
		 * \brief function to read velocity demand
		 *
		 *  Velocity demand value is generated by profile generator and is the set value for the velocity controller
		 *
		 *  \return demanded velocity
		*/
		long readVelocityDemand	();

		/**
		 * \brief function to read velocity sensor actual
		 *
		 * \b Description
		 * The velocity actual value is coupled to the velocity used as input to velocity controller [Velocity units].
		 * \b Remarks
		 * The resolution of the short time velocity measurement (Velocity actual value, Velocity sensor actual value) is dependent on the encoder pulse number (Sensor Configuration) and the velocity measurement method (Miscellaneous Configuration bit 3). To improve the short time velocity measurement resolution set the Miscellaneous Configuration bit 3 to 1 or use an encoder with higher resolution.
		 * For example the short time velocity resolution with a 500-pulse encoder and Miscellaneous Configuration Bit 3 = 0 is: 1 quadcount / ms = 60’000 / (4 * 500) = 30 rpm.
		 *  \return actual velocity
		*/
		long readVelocityActual	();

		/**
		 * \brief function to read motor current
		 *
		 *  This function reads the actual current of the motor
		 *  [mA]
		 *
		 *  \return actual current
		 */
		long readCurrent			();

		/**
		 * \brief function to read motor averaged current
		 *
		 *
		 *  The current actual value averaged [mA] represents the current actual value filtered by 1st order digital lowpass filter with a cut-off frequency of 50 Hz.This function reads the current of the motor averaged
		 *  [mA]
		 *
		 *  \return current averaged
		 */
		long readCurrentAveraged		();

		/**
		 * \brief function to read current demanded
		 *
		 * The «Current Demand Value» is the set value [mA] for the current controller
		 *
		 * [mA]
		 *
		 *  \return demanded current
		 */
		long readCurrentDemanded		();

		/**
		 * \brief function to read motor position
		 *
		 *  This function reads the actual current of the motor.
		 *  [qc]
		 *  To get load position just do:
		 * 	load_pos[º] = motor_pos[qc] / ( 4 * encoder_pulses [lines/rev] * reduction )
		 *
		 *  \return actual current
		 */
		int32_t readPosition		();

    	/**
		 * \brief function to read EPOS2 StatusWord
		 *
		 *  This function reads the StatusWord.
		 *  It doesn't give more information than just the returned StatusWord
		 *
		 *  \return StatusWord
		 */
		long readStatusWord		();

		/**
		 * \brief function to read the Encoder Counter
		 *
		 *  This function reads the internal counter register of the encoder
		 *  multiplied by Polarity
		 *  [qc]
		 *
		 *  \return Encoder Counter
		 */
		long readEncoderCounter	();

		/**
		 * \brief function to read the Encoder Counter at index pulse
		 *
		 *  This function reads the encoder counter reached at last detected
		 *  encoder index pulse.
		 *  [qc]
		 *
		 *  \return Encoder Counter at index pulse
		 */
		long readEncoderCounterAtIndexPulse ();

		/**
		 * \brief function to read the Hall Sensor Pattern
		 *
		 *  This function reads the actual state of the three hall sensors as a pattern.
		 *  bit 0: hallsensor 1
		 *  bit 1: hallsensor 2
		 *  bit 2: hallsensor 3
		 *
		 *  \return Hallsensor Pattern
		 */
		long readHallsensorPattern	();

		/**
		 * \brief function to read the Following Error
		 *
		 *  This function reads the actual Following Error
		 *  [qc]
		 *
		 *  \return Following Error
		 */
		long readFollowingError		();

		/**
		 * \brief prints information about movement
		 *
		 *  Motor position, velocity, averaged velocity, demanded velocity, current, averaged
		 *  current, demanded current.
		 *
		 */
		void getMovementInfo		();

	// Errors

		/**
		 * \brief function to read an Error information
		 *
		 *  This function reads the information of an error
		 *
		 *  \return StatusWord
		 */
		char readError			();

		/**
		 * \brief function to read last 5 errors
		 *
		 *  This function reads the information of last five errors in EPOS
		 *
		 *  \return Errors
		 */
		void readErrorHistory		(long *error[5]);

		/**
		 * \brief given an error code it returns its description
		 *
		 *  \return Errors
		 */
		std::string searchErrorDescription	(long error_code);

    /*! \brief Strings of generic error descriptions */
    static const std::string error_names[];

    /*! \brief error integer codes */
    static const int error_codes[];

    /*! \brief error descriptions */
    static const std::string error_descriptions[];

	// Versions

		/**
		 * \brief function to read the software version
		 *
		 *  This function reads the software version, it can be useful to know if EPOS2 has
		 *  the last firmware.
		 *
		 *  \return Software Version
		 */
		long readVersionSoftware		();

		/**
		 * \brief function to read the hardware version
		 *
		 *  This function reads the hardware version.
		 *
		 *  \return Hardware Version
		 */
		long readVersionHardware		();



///@}


/// @name Sensor configuration
/// @{
		/**
		 * \brief function to GET the Encoder Pulses
		 *
		 *  This function gets the Encoder Pulses
		 *
		 *  \return Encoder Pulses
		 */
		long getEncoderPulseNumber	();

		/**
		 * \brief function to SET the Encoder Pulses
		 *
		 *  This function sets the Encoder pulses
		 *  16...2500000
		 *  DISABLE STATE only
		 *
		 *  \param pulses
		 */
		void setEncoderPulseNumber	(long pulses);

		/**
		 * \brief function to GET the Encoder Pulses
		 *
		 *  This function gets the Encoder Pulses
		 *
		 *  \return Encoder Pulses
		 */
		long getEncoderType		();

		/**
		 * \brief function to SET the Encoder type
		 *
		 *  This function sets the Encoder type
		 *  0=unknown,1=incremental+index,2=incremental,3=Hall
		 *  DISABLE STATE only
		 *
		 *  \param type
		 */
		void setEncoderType		(long type);

		/**
		 * \brief function to GET the Encoder Pulses
		 *
		 *  This function gets the Encoder Pulses
		 *
		 *  \return Encoder Pulses
		 */
		long getEncoderPolarity		();

		/**
		 * \brief function to SET the Encoder polarity
		 *
		 *  This function sets the Encoder polarity
		 *  bit 0: Incremental Encoder (0:CCW, 1:CW (encoder mounted on motor shaft)
		 *  bit 1: Hall s (0:normal, 1:inverted)
		 *  bit 2: SSI Encoder (0: CCW, 1:inverted)
		 *
		 *  DISABLE STATE only
		 *
		 *  \param polarity
		 */
		void setEncoderPolarity		(long polarity);

	// General

		/**
		 * \brief function to GET encoder parameters
		 *
		 *  \retval &pulses
		 *  \retval &type
		 *  \retval &polarity
		 */
		void getEncoderParameters(long &pulses, long &type, long &polarity);

		/**
		 * \brief function to SET encoder parameters
		 *
		 *  \param pulses
		 *  \param type
		 *  \param polarity
		 */
		void setEncoderParameters(long pulses, long type, long polarity);


///@}


/// @name Motor
/// @{
		/**
		 * \brief function to GET Motor type
		 *
		 *  This function gets Motor type
		 *
		 *  \return Motor type
		 */
		long getMotorType		();

		/**
		 * \brief function to SET Motor type
		 *
		 *  This function sets Motor type
		 *  1: brushed DC motor
		 *  10: EC motor sinus commutated
		 *  11: EC motor block commutated
		 *
		 *  DISABLE STATE only
		 *
		 *  \param type
		 */
		void setMotorType	(long type);

		/**
		 * \brief function to GET Motor Continous Current Limit
		 *
		 *  This function gets Motor Continous Current Limit
		 *
		 *  \return Motor Continous Current Limit
		 */
		long getMotorContinuousCurrentLimit	();

		/**
		 * \brief function to SET Motor Continous Current Limit
		 *
		 *  This function sets Motor Continous Current Limit
		 *  [mA]
		 *
		 *  \param current_mA [mA]
		 */
		void setMotorContinuousCurrentLimit	(long current_mA);

		/**
		 * \brief function to GET Motor Output Current Limit
		 *
		 *  This function gets Motor Output Current Limit
		 *
		 *  \return Motor Output Current Limit
		 */
		long getMotorOutputCurrentLimit	();

		/**
		 * \brief function to SET Motor Output Current Limit
		 *
		 *  This function sets Motor Output Current Limit
		 *  Usually 2·Continous current limit [mA]
		 *
		 *  \param current_mA [mA]
		 */
		void setMotorOutputCurrentLimit	(long current_mA);

		/**
		 * \brief function to GET Motor Pole Pair Number
		 *
		 *  This function gets Motor Pole Pair Number
		 *
		 *  \return Motor Pole Pair Number
		 */
		long getMotorPolePairNumber	();

		/**
		 * \brief function to SET Motor Pole Pair Number
		 *
		 *  This function sets Motor Pole Pair Number
		 *  number of poles / 2
		 *
		 *  Disable State
		 *
		 *  \param pole_pairs
		 */
		void setMotorPolePairNumber	(char pole_pairs);

		/**
		 * \brief function to GET Motor Thermal Time Constant Winding
		 *
		 *  This function gets Motor Thermal Time Constant Winding
		 *
		 *  \return Motor Thermal Time Constant Winding
		 */
		long getThermalTimeCtWinding	();

		/**
		 * \brief function to SET Motor Thermal Time Constant Winding
		 *
		 *  This function sets Motor Thermal Time Constant Winding
		 *  [ds] = [s*10]
		 *
		 *  \param time_ds
		 */
		void setThermalTimeCtWinding	(long time_ds);

	// General

		/**
		 * \brief function to GET encoder parameters
		 *
		 *  \retval &type
		 *  \retval &current_c_mA
		 *  \retval &current_out_mA
		 *  \retval &pole_pairs
		 *  \retval &time_ds
		 */
		void getMotorParameters(long &type, long &current_c_mA, long &current_out_mA, char &pole_pairs, long &time_ds);

		/**
		 *  \brief function to SET motor parameters
		 *  \param type
		 *  \param current_c_mA
		 *  \param current_out_mA
		 *  \param pole_pairs
		 *  \param time_ds
		 */
		void setMotorParameters(long type, long current_c_mA, long current_out_mA, char pole_pairs, long time_ds);

///@}

		// CONFIGURATION


/// @name Communication configuration
/// @{
		/**
		 * \brief function to GET the RS232 Baudrate
		 *
		 *  This function gets the RS232 Baudrate of the controller
		 *
		 *  \return RS232 Baudrate
		 */
		long getRS232Baudrate		();

		/**
		 * \brief function to SET the RS232 Baudrate
		 *
		 *  This function sets the RS232 Baudrate (Firmware 14.36)
		 *  	0: 9.6 kBaud
		 *	1: 14.4 kBaud
		 *	2: 19.2 kBaud
		 *	3: 38.4 kBaud
		 *	4: 57.6 kBaud
		 *	5: 115.2 kBaud
		 *
		 *  It needs to saveParameters and then restart EPOS2
		 *
		 *  \param baudrate desired following the table
		 */
		void setRS232Baudrate		(long baudrate);

		/**
		 * \brief function to GET the RS232 Frame Timeout
		 *
		 *  This function gets the RS232 Frame Timeout of the controller
		 *
		 *  \return RS232 Frame Timeout [ms]
		 */
		long getRS232FrameTimeout	();

		/**
		 * \brief function to SET the RS232 Frame Timeout
		 *
		 *  This function sets the RS232 Frame Timeout
		 *  [ms] (500 by default)
		 *
		 *  \param timeout in [ms]
		 */
		void setRS232FrameTimeout	(long timeout);

		/**
		 * \brief function to GET the USB Frame Timeout
		 *
		 *  This function gets the USB Frame Timeout of the controller
		 *
		 *  \return USB Frame Timeout [ms]
		 */
		long getUSBFrameTimeout		();

		/**
		 * \brief function to SET the USB Frame Timeout
		 *
		 *  This function sets the USB Frame Timeout
		 *  [ms] (500 by default)
		 *
		 *  \param timeout in [ms]
		 */
		void setUSBFrameTimeout		(long timeout);
	///@}

/// @name Position parameters configuration
/// @{
		/**
		 * \brief function to GET the Max Following Error
		 *
		 *  This function gets the Max following error
		 *  [qc]
		 *
		 *  \return Max Following Error
		 */
		long getMaxFollowingError	();


		/**
		 * \brief function to SET the Max Following Error
		 *
		 *  This function sets the Max following error
		 *  [qc] 0..4294967295
		 *
		 *  \param follerror
		 */
		void setMaxFollowingError	(long follerror);

		/**
		 * \brief function to GET the Min Position Limit
		 *
		 *  This function gets the Min Position Limit
		 *  [qc]
		 *
		 *  \return Min Position Limit
		 */
		long getMinPositionLimit		();

		/**
		 * \brief function to SET the Min Position Limit
		 *
		 *  This function sets the Min Position Limit
		 *  [qc] -2147483648..2147483647
		 *
		 *  \param limit
		 */
		void setMinPositionLimit	(long limit);

		/**
		 * \brief function to GET the Max Position Limit
		 *
		 *  This function gets the Max Position Limit
		 *  [qc]
		 *
		 *  \return Max Position Limit
		 */
		long getMaxPositionLimit		();

		/**
		 * \brief function to SET the Max Position Limit
		 *
		 *  This function sets the Max Position Limit
		 *  [qc] -2147483648..2147483647
		 *
		 *  \param limit
		 */
		void setMaxPositionLimit	(long limit);

		/**
		 * \brief function to DISABLE Position Limits
		 *
		 *  This function disables Position Limits
		 *  it writes min= -2147483648
		 *  max= 2147483647
		 *
		 */
		void disablePositionLimits	(void);

		/**
		 * \brief function to get Position Window
		 *
		 *  This function gets Position Window
		 *
		 *  \return actual output
		 */
		long getPositionWindow		();

		/**
		 * \brief function to set Position Window
		 *
		 *  This function sets Position Window
		 *  Disable: 4’294’967’295
		 *  [qc] 0..2’147’483’647
		 *
		 *  \param window_qc [qc]
		 */
		void setPositionWindow		(long window_qc);

		/**
		 * \brief function to get Position Window Time
		 *
		 *  This function gets Position Window Time
		 *
		 *  \return actual output
		 */
		long getPositionWindowTime		();

		/**
		 * \brief function to set Position Window Time
		 *
		 *  This function sets Position Window Time
		 *  [ms] 0..65535
		 *
		 *  \param time_ms [ms]
		 */
		void setPositionWindowTime		(long time_ms);

		/**
		 * \brief function to get Velocity Window
		 *
		 *  This function gets Velocity Window
		 *
		 *  \return Velocity Window
		 */
		long getVelocityWindow		();

		/**
		 * \brief function to set Velocity Window
		 *
		 *  This function sets Velocity Window
		 *  [qc] 0..4’294’967’295
		 *
		 *  \param window_rm [rev/min]
		 */
		void setVelocityWindow		(long window_rm);

		/**
		 * \brief function to get Velocity Window Time
		 *
		 *  This function gets Velocity Window Time
		 *
		 *  \return actual output
		 */
		long getVelocityWindowTime		();

		/**
		 * \brief function to set Velocity Window Time
		 *
		 *  This function sets Velocity Window Time
		 *  [ms] 0..65535
		 *
		 *  \param time_ms [ms]
		 */
		void setVelocityWindowTime		(long time_ms);

///@}

/// @name Dimension and notation configuration
/// @{
		/**
		 * \brief function to get Position Notation Index
		 *
		 *  This function gets Position Notation Index
		 *
		 *  \return Position Notation Index
		 */
		long getPositionNotationIndex		();

		/**
		 * \brief function to set Position Notation Index
		 *
		 *  This function sets Position Notation Index
		 *
		 *  \param notation \ref factor_tables
		 */
		void setPositionNotationIndex		(long notation);

		/**
		 * \brief function to get Velocity Notation Index
		 *
		 *  This function gets Velocity Notation Index
		 *
		 *  \return Velocity Notation Index
		 */
		long getVelocityNotationIndex		();

		/**
		 * \brief function to set Velocity Notation Index
		 *
		 *  This function sets Velocity Notation Index
		 *
		 *  \param notation \ref factor_tables
		 */
		void setVelocityNotationIndex		(long notation);

		/**
		 * \brief function to get Acceleration Notation Index
		 *
		 *  This function gets Acceleration Notation Index
		 *
		 *  \return Acceleration Notation Index
		 */
		long getAccelerationNotationIndex		();

		/**
		 * \brief function to set Acceleration Notation Index
		 *
		 *  This function sets Acceleration Notation Index
		 *
		 *  \param notation \ref factor_tables
		 */
		void setAccelerationNotationIndex		(long notation);


		/**
		 * \brief function to get Position Dimension Index
		 *
		 *  This function gets Position Dimension Index
		 *
		 *  \return Position Dimension Index
		 */
		long getPositionDimensionIndex		();

		/**
		 * \brief function to set Position Dimension Index
		 *
		 *  This function sets Position Dimension Index
		 *
		 *  \param Dimension \ref factor_tables
		 */
		void setPositionDimensionIndex		(long Dimension);

		/**
		 * \brief function to get Velocity Dimension Index
		 *
		 *  This function gets Velocity Dimension Index
		 *
		 *  \return Velocity Dimension Index
		 */
		long getVelocityDimensionIndex		();

		/**
		 * \brief function to set Velocity Dimension Index
		 *
		 *  This function sets Velocity Dimension Index
		 *
		 *
		 *  \param Dimension \ref factor_tables
		 */
		void setVelocityDimensionIndex		(long Dimension);

		/**
		 * \brief function to get Acceleration Dimension Index
		 *
		 *  This function gets Acceleration Dimension Index
		 *
		 *  \return Acceleration Dimension Index
		 */
		long getAccelerationDimensionIndex		();

		/**
		 * \brief function to set Acceleration Dimension Index
		 *
		 *  This function sets Acceleration Dimension Index
		 *
		 *
		 *  \param Dimension \ref factor_tables
		 */
		void setAccelerationDimensionIndex		(long Dimension);
///@}


/// @name Utilities
/// @{

		/**
		 * \brief function to get the value in the Analog Output1
		 *
		 *  This function gets the value in the Analog Output1, to know which Voltage is
		 *  giving this output
		 *  [mV]
		 *
		 *  \return actual output
		 */
		long getAnalogOutput1		();

		/**
		 * \brief function to set analog output 1
		 *
		 *  This function sets the analog output1 voltage.
		 *  <b>ONLY in 50/5 Controller!!</b>
		 *  Resolution 2.40 mV, Bandwith 20 kHz
		 *  [mV] 0..10000
		 *
		 * \image html "analog_output_connector_hardware_reference_pg29.png"
		 *
		 *  \param voltage_mV
		 */
		void setAnalogOutput1		(long voltage_mV);

		/**
		 * \brief function to save all parameters in EEPROM
		 *
		 *  This function save all actual parameters to EEPROM of EPOS2. Its necessary if you
		 *  do changes you want to save such as new controller parameters or profiles.
		 */
		void saveParameters		();

		/**
		 * \brief function to restore default parameters of EPOS2.
		 *
		 *  This function restores all default parameters of EPOS2.
		 *  \warning It removes its actual values! (ALL)
		*/
		void restoreDefaultParameters	();

    bool getVerbose();

    void setVerbose(bool verbose);

    /*!
    \brief gets the position marked by the sensor

    \param buffer 0: captured position, 1: history 1, 2: history 2
    */
    long getPositionMarker(int buffer = 0);

    /*!
    \brief Sets the configuration of a marker position

    \param digitalIN The pin which has the Marker position switch property
    \param polarity 0: true on high, 1: true on low
    \param edge_type 0 : rising, 1: falling, 2: both
    \param mode 0: continuous 1: single 2: multiple

    */
    void setPositionMarker(char polarity = 0, char edge_type = 0,
                           char mode = 0, char digitalIN = 2);

    /*! \brief wait for marker position reached
    *
    *  \param param
    */
    void waitPositionMarker();

    /*!
    \brief Sets the configuration of a homing operation

    It can use a sensor to reach the home, this sensor can be set as
    positive (18,2), negative(17,1) or home(27,23,11,7) switch. If the system
    doesn't have a switch, setting actual position as home (35) and Index methods
    (34,33) can be used.

    \param home_method
    \code
    #  Description
    35 Set Actual Position as home position
    34 Index Negative / Positive Speed
    33 Index Negative / Positive Speed
    27 Home Switch Negative Speed
    23 Home Switch Positive Speed
    18 Positive Limit Switch
    17 Negative Limit Switch
    11 Home Switch Negative Speed & Index
     7 Home Switch Positive Speed & Index
     2 Positive Limit Switch & Index
     1 Negative Limit Switch & Index
     0 No homing operation required
    -1 Current Threshold Positive Speed & Index
    -2 Current Threshold Negative Speed & Index
    -3 Current Threshold Positive Speed
    -4 Current Threshold Negative Speed
    \endcode
    \param speed_pos Speed for first positioning
    \param speed_zero Speed for find home near 0
    \param acc Acceleration
    \param digitalIN The pin which has the home switch property

    */
    void setHoming(int home_method = 11, int speed_pos = 100,
                   int speed_zero = 100, int acc = 1000, int digitalIN = 2);

    /*!
    \brief starts a homing operation

    Options had to be set with setHoming
    */
    void doHoming(bool blocking=false);

    /*! \brief Thread listens to target reached
    *
    *  \param param
    */
    static void *threadTargetReached(void *param);

    /*!
    \brief stops a homing operation

    */
    void stopHoming();

    /*!
    \brief Gives the state of a certain digital Input

    \return 0: inactive, 1: active(depending on polarity=0)
    */
    int getDigInState(int digitalIN = 2);

    /*!
    \brief Gives Digital Inputs State Mask

    \return state mask [FW table 98]
    */
    int getDigInStateMask();

    /*!
    \brief Gives Digital Inputs Functionalities Mask

    \return functionalities mask [FW table 99]
    */
    int getDigInFuncMask();

    /*!
    \brief Gives Digital Inputs Polarity Mask

    \return functionalities polarity mask [FW table 100]
    */

    int getDigInPolarity();

    /*!
    \brief Gives Digital Inputs Execution Mask

    \return execution mask [FW table 111]
    */
    int getDigInExecutionMask();

///@}
};

class EPOS2OpenException : public std::runtime_error
{
  public:
    EPOS2OpenException(const std::string& error_description)
      : std::runtime_error (error_description) {}
};

class EPOS2IOException : public std::runtime_error
{
  public:
    EPOS2IOException(const std::string& error_description)
      : std::runtime_error (error_description) {}
};

class EPOS2UnknownStateException : public std::runtime_error
{
  public:
    EPOS2UnknownStateException(const std::string& error_description)
      : std::runtime_error (error_description) {}
};

#endif
