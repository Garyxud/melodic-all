//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_temperature_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Print measured temperatures of an attached %SDH. (C++ demo application using the SDHLibrary-CPP library.)
       See \ref demo_temperature__help__ "__help__" and online help ("-h" or "--help") for available options.

     \section sdhlibrary_cpp_sdh_demo_temperature_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_temperature_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-06-18 18:28:14 +0200 (Di, 18 Jun 2013) $
         \par SVN file revision:
           $Id: demo-temperature.cpp 10351 2013-06-18 16:28:14Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_temperature_changelog Changelog of this file:
         \include demo-temperature.cpp.log

*/
/*!
  @}
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <getopt.h>
#include <assert.h>

#include <iostream>
#include <vector>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/sdh.h"
#include "sdh/simpletime.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
    \anchor sdhlibrary_cpp_demo_temperature_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_temperature__help__
char const* __help__      =
    "Print measured temperatures of SDH.\n"
    "(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "A vector of temperatures is reported. The first 7 temperatures\n"
    "are from sensors close to the corresponding axes motors.\n"
    "The 8th value is the temperature of the FPGA, the controller chip (CPU).\n"
    "The 9th value is the temperature of the PCB (Printed circuit board)\n"
    "in the body of the SDH.\n"
    ""
    "- Example usage:\n"
    "  - Print temperatures of an SDH connected via Ethernet.\n"
    "    The SDH has IP-Address 192.168.1.42 and is attached to TCP port 23.\n"
    "    (Requires at least SDH-firmware v0.0.3.1)\n"
    "    > demo-GetAxisActualAngle --tcp=192.168.1.42:23\n"
    "     \n"
    "  - Print temperatures of an SDH connected to port 2 = COM3 once:\n"
    "    > demo-temperature -p 2\n"
    "\n"
    "  - Print temperatures of an SDH connected to port 2 = COM3 every 500ms:\n"
    "    > demo-temperature -p 2 -t 0.5\n"
    "\n"
    "  - Print temperatures of an SDH connected to USB to RS232 converter 0 move:\n"
    "    > demo-temperature --sdh_rs_device=/dev/ttyUSB0 \n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-GetAxisActualAngle --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of an SDH connected to port 2 = COM3 \n"
    "    > demo-temperature --port=2  -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-temperature.cpp 10351 2013-06-18 16:28:14Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_temperature_cpp_vars
//  @}



//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( SDHUSAGE_DEFAULT " sdhother" );

    options.Parse( argc, argv, __help__, "demo-temperature", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
    //
    //----------------------------------------------------------------------

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
        // cSDH instance "hand" of the class cSDH according to the given options:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";


        cdbg << "Caption:\n";
        if (options.period)
            cdbg << "  times are reported in seconds\n";

        cdbg << "  temperatures are reported in " << hand.uc_temperature->GetName()
             << " [" << hand.uc_temperature->GetSymbol() << "]\n";


        //??? a second try block to catch keyboard interrupts
        //try:
#if SDH_USE_VCC
        double elapsed = 0.0;
#else
        cSimpleTime start;
#endif


        while (true)
        {
            vector<double> temps = hand.GetTemperature( hand.all_temperature_sensors );

            if (options.period > 0)
            {
                // print time only if reporting periodically
#if SDH_USE_VCC
                cout << elapsed << " ";
                elapsed += options.period;
#else
                cout << start.Elapsed() << " ";
#endif
            }

            for ( vector<double>::const_iterator vi = temps.begin();
                  vi != temps.end();
                  vi++ )
                cout << *vi << " ";

            //print "%6.*f" % (hand.uc_temperature.decimal_places, v),

            cout << "\n";
            cout.flush();

            if (options.period <= 0)
                break;

            SleepSec( options.period );
        }

        hand.Close();
        cdbg << "Successfully disabled controllers of SDH and closed connection\n";
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "demo-temperature main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================
