//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!

    \file
     \section sdhlibrary_cpp_sdh_demo_velocity_acceleration_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple demonstration program using the SDHLibrary-CPP: Make an attached %SDH move one finger
       in "velocity with acceleration ramp" control mode.
       See \ref demo_velocity_acceleration__help__ "__help__" and online help ("-h" or "--help") for available options.

     \section sdhlibrary_cpp_sdh_demo_velocity_acceleration_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_velocity_acceleration_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-06-18 18:28:14 +0200 (Di, 18 Jun 2013) $
         \par SVN file revision:
           $Id: demo-velocity-acceleration.cpp 10351 2013-06-18 16:28:14Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_velocity_acceleration_changelog Changelog of this file:
         \include demo-velocity-acceleration.cpp.log

*/
/*!
  @}
*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_velocity_acceleration_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_velocity_acceleration__help__
char const* __help__      =
    "Make the SDH move one finger in \"velocity with acceleration ramp\" control mode.\n(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "  - Make SDH connected via Ethernet move.\n"
    "    The SDH has IP-Address 192.168.1.42 and is attached to TCP port 23.\n"
    "    (Requires at least SDH-firmware v0.0.3.1)\n"
    "    > demo-velocity-acceleration --tcp=192.168.1.42:23\n"
    "     \n"
    "- Example usage:\n"
    "  - Make SDH connected to port 2 = COM3 move:\n"
    "    > demo-velocity-acceleration -p 2\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 move:\n"
    "    > demo-velocity-acceleration --sdh_rs_device=/dev/ttyUSB0 \n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-velocity-acceleration --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of an SDH connected to port 2 = COM3 \n"
    "    > demo-velocity-acceleration --port=2 -v\n";

char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-velocity-acceleration.cpp 10351 2013-06-18 16:28:14Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_velocity_acceleration_cpp_vars
//  @}

char const* usage =
  "usage: demo-velocity-acceleration [options]\n";


int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-velocity-acceleration", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
    //
    //---------------------

    //---------------------
    // initialize debug message printing:
    cDBG cdbg( options.debug_level > 0, "red", options.debuglog );
    g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    // reduce debug level for subsystems
    options.debug_level-=1;
    //---------------------

    // Pack the rest of this demo into a try block so that we can recover in case of an error:
    try
    {
        // Create an instance "hand" of the class cSDH:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        //##############
        // Preparations: Move the hand to a pose that is adequate for this demo:
        std::cout << "Preparation:\n";
        std::cout << " Moving to start position with \"pose\" controller type:\n\n";

        // Switch to "pose" controller mode first, then move to "home"
        hand.SetController( hand.eCT_POSE );
        hand.SetAxisTargetVelocity( hand.All, 40.0 );
        hand.SetAxisTargetAngle(    hand.All, 0.0 );
        hand.MoveHand();
        //
        //##############


        //##############
        // Do some movements with "velocity with acceleration ramp" controller type,
        // move with different velocities and accelerations.
        std::cout << "Moving in \"velocity with acceleration ramp\" controller type:\n";
        std::cout << "  Now move back and forth with increasing acceleration\n";
        std::cout << "  and target velocities of alternating sign:\n";

        // Now switch to "velocity control with acceleration ramp" controller mode.
        hand.SetController( hand.eCT_VELOCITY_ACCELERATION );

        // Use one axis only:
        int axis_index = 2;

        // In this controller mode we must switch the power on explicitly:
        // (OK, here the power is switched on already since we used hand.MoveHand() before.)
        hand.SetAxisEnable( axis_index, true );

        double accelerations[] =  { 10.0, 20.0, 40.0, 80.0, 160.0, 0.0 };
        double velocity      = 40.0;
        int i = 0;
        do
        {
            // set desired acceleration (must be done first!):
            hand.SetAxisTargetAcceleration( axis_index, accelerations[i] );

            for ( double sign = -1.0; sign <= 1.0; sign += 2.0 ) // generate an alternating sign: -1.0 / +1.0
            {
                std::cout << "Setting target acceleration,velocity= " << std::setw(7) << accelerations[i] << " deg/(s*s)  " << std::setw(7) << sign * velocity << " deg/s\n";

                // set the desired target velocity. This will make the axis move!
                hand.SetAxisTargetVelocity( axis_index, sign * velocity );

                // keep current velocity (and acceleration) as long as the axis |angle| < 10 deg,
                // when 10 deg are exceeded then invert the velocity sign (and probably use next acceleration)
                bool position_reached = false;
                while ( !position_reached )
                {
                    // print out some debug data while moving:
                    cdbg << "  Actual angle: " << std::setw(7) << hand.GetAxisActualAngle( axis_index ) << " deg";
                    cdbg << ",  actual velocity: " << std::setw(7) << hand.GetAxisActualVelocity(axis_index) << " deg/s";
                    cdbg << ",  reference velocity: " << std::setw(7) << hand.GetAxisReferenceVelocity(axis_index) << " deg/s\n";
                    if ( sign > 0.0 )
                        position_reached = (hand.GetAxisActualAngle(axis_index) >= 10.0);
                    else
                        position_reached = (hand.GetAxisActualAngle(axis_index) <= -10.0);
                    SleepSec(0.05);
                }
            }
            i++;
        } while ( accelerations[i] != 0.0);
        //
        //##############


        //##############
        // Stop movement:

        std::cout << "Setting target acceleration,velocity= " << std::setw(7) << 100.0 << " deg/(s*s)  " << std::setw(7) << 0.0 << " deg/s (for stopping)\n";

        // set a default acceleration:
        hand.SetAxisTargetAcceleration( axis_index, 100.0 );

        // set the desired target velocity to 0.0. This will make the axis slow down until stop.
        hand.SetAxisTargetVelocity(     axis_index, 0.0 );

        // wait until the joint has stopped to give the SDH time for slowing down:
        //   Solution one:
        //     Simply wait for the time needed to reach velocity 0.0
        //     (v = a * t hence t= v/a)
        //SleepSec( hand.GetAxisReferenceVelocity( axis_index ) / 100.0 );

        //   Solution two:
        //     Wait until SDH reports "IDLE":
        hand.WaitAxis( axis_index, 5.0 );

        //
        //##############


        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-velocity-acceleration main(): An exception was caught: " << e->what() << "\n";
        delete e;
    }
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored)
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================]
