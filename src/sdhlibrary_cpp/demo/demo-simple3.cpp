//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_simple_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple C++ programm to make an attached %SDH move. With non-sequential call of move and WaitAxis.
       See \ref demo_simple3__help__ "__help__" and online help ("-h" or "--help") for available options.

       This code contains only the very basicst use of the features
       provided by the SDHLibrary-CPP. For more sophisticated
       applications see the other demo-*.cpp programms, or of course
       the html/pdf documentation.

     \section sdhlibrary_cpp_sdh_demo_simple_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_simple_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-06-18 18:28:14 +0200 (Di, 18 Jun 2013) $
         \par SVN file revision:
           $Id: demo-simple3.cpp 10351 2013-06-18 16:28:14Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_simple_changelog Changelog of this file:
         \include demo-simple3.cpp.log

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
    \anchor sdhlibrary_cpp_demo_simple3_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_simple3__help__
char const* __help__      =
    "Move axes 1,2 and 3 to a specific point.\n(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "- Example usage:\n"
    "  - Make SDH connected via Ethernet move.\n"
    "    The SDH has IP-Address 192.168.1.42 and is attached to TCP port 23.\n"
    "    (Requires at least SDH-firmware v0.0.3.1)\n"
    "    > demo-simple3 --tcp=192.168.1.42:23\n"
    "     \n"
    "  - Make SDH connected to port 2 = COM3 move:\n"
    "    > demo-simple3 -p 2\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 move:\n"
    "    > demo-simple3 --sdh_rs_device=/dev/ttyUSB0 \n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-simple3 --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of an SDH connected to port 2 = COM3 \n"
    "    > demo-simple3 --port=2 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-simple3.cpp 10351 2013-06-18 16:28:14Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_simple3_cpp_vars
//  @}



int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-simple3", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
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

    try
    {
        // Create an instance "hand" of the class cSDH:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        // Switch to "pose" controller mode and set default velocities first:
        hand.SetController( hand.eCT_POSE );
        hand.SetAxisTargetVelocity( hand.All, 40.0 );


        // Set a new target pose for axis 1,2 and 3
        std::vector<int> axes123;
        axes123.push_back( 1 );
        axes123.push_back( 2 );
        axes123.push_back( 3 );

        std::vector<double> angles123;
        angles123.push_back( -20.0 );
        angles123.push_back( -30.0 );
        angles123.push_back( -40.0 );

        hand.SetAxisTargetAngle( axes123, angles123 );

        // Move axes there non sequentially:
        hand.MoveAxis( axes123, false );

        // The last call returned immediately so we now have time to
        // do something else while the hand is moving:

        // ... insert any calculation here ...

        std::cout << "waiting while moving to " << hand.GetAxisTargetAngle(hand.all_axes) << "\n"; std::cout.flush();
        // Before doing something else with the hand make sure the
        // selected axes have finished the last movement:
        //
        // \attention With SDH firmwares prior to 0.0.2.6 this did not work as expected!
        //   Hack: We have to wait a very short time to give the joint controller
        //   a chance to react and start moving.
        // => Resolved with SDH firmware 0.0.2.6
        //SleepSec(0.1); // no longer needed
        hand.WaitAxis( axes123 );


        // go back home (all angles to 0.0):
        hand.SetAxisTargetAngle( hand.All, 0.0 );

        // Move all axes there non sequentially:
        hand.MoveAxis( hand.All, false );

        // ... insert any other calculation here ...

        std::cout << "waiting while moving to " << hand.GetAxisTargetAngle(hand.all_axes) << "\n"; std::cout.flush();
        // Wait until all axes are there, with a timeout of 10s:
        //
        // \attention With SDH firmwares prior to 0.0.2.6 this did not work as expected!
        //   Hack: We have to wait a very short time to give the joint controller
        //   a chance to react and start moving.
        // => Resolved with SDH firmware 0.0.2.6
        //SleepSec(0.1); // no longer needed
        hand.WaitAxis( hand.All, 10.0 );


        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-simple3 main(): An exception was caught: " << e->what() << "\n";
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
