//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
   \file
   \section sdhlibrary_cpp_demo_dsa_cpp_general General file information

     \author   Dirk Osswald, Winfried Baum (IPA)
     \date     2008-06-12

   \brief
     Simple program to test class cDSA.
     See \ref demo_dsa__help__ "__help__" and online help ("-h" or "--help") for available options.

   \section sdhlibrary_cpp_demo_dsa_cpp_copyright Copyright

   - Copyright (c) 2008 SCHUNK GmbH & Co. KG

   <HR>
   \internal

     \subsection sdhlibrary_cpp_demo_dsa_cpp_details SVN related, detailed file specific information:
       $LastChangedBy: Osswald2 $
       $LastChangedDate: 2013-06-18 18:28:14 +0200 (Di, 18 Jun 2013) $
       \par SVN file revision:
         $Id: demo-dsa.cpp 10351 2013-06-18 16:28:14Z Osswald2 $

   \subsection sdhlibrary_cpp_demo_dsa_cpp_changelog Changelog of this file:
       \include demo-dsa.cpp.log

*/
/*!
  @}
*/
//======================================================================

#include "sdh/sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

//#include <getopt.h>
//#include <assert.h>

#include <iostream>
#include <vector>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/sdh.h"
#include "sdh/dsa.h"
#include "sdh/basisdef.h"
#include "sdh/util.h"
#include "sdhoptions.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
  \anchor sdhlibrary_cpp_demo_dsa_cpp_vars
  \name   Some informative variables

  Some definitions that describe the demo program

  @{
*/
//! \anchor demo_dsa__help__
char const* __help__      =
    "Simple demo to test cDSA class of SDHLibrary-cpp.\n"
    "\n"
    "Remarks:\n"
    "- You must specify at least one of these options to see some output:\n"
    "  -f | --fullframe  \n"
    "  -r | --resulting\n"
    "  -c | --controllerinfo \n"
    "  -s | --sensorinfoinfo\n"
    "  -m | --matrixinfo=N\n"
    "  \n"
    "- Example usage:\n"
    "  - Read a single full frame from tactile sensors connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f\n"
    "     \n"
    "  - Read a single full frame from tactile sensors connected via TCP/IP (ethernet):\n"
    "    The tactile sensors have IP-Address 192.168.1.42 and use TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-dsa --dsa_tcp=192.168.1.42:13000 -f\n"
    "     \n"
    "  - Read full frames continuously once per second from tactile sensors\n"
    "    connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f -r 1\n"
    "     \n"
    "  - Read full frames continuously 10 times per second from tactile sensors\n"
    "    connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f -r 10\n"
    "     \n"
    "  - Read full frames continuously as fast as possible (DSA push-mode)\n"
    "    from tactile sensors connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f -r 30\n"
    "     \n"
    "  - Read a single full frame from tactile sensors connected to USB\n"
    "    to RS232 converter 0:\n"
    "    > demo-dsa --dsa_rs_device=/dev/ttyUSB0 -f \n"
    "     \n"
    "  - Read the sensor, controller, matrix 0 infos \n"
    "    from tactile sensors connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -s -c -m 0 \n"
    "    \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-dsa --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected to \n"
    "    - port 2 = COM3 (joint controllers) and \n"
    "    - port 3 = COM4 (tactile sensor controller) \n"
    "    > demo-dsa -p 2 --dsaport=3 -v\n"
    "  - Set the sensitivity of all tactile sensor matrixes to 0.75 temporarily.\n"
    "    The value will be used only temporarily (until reset or power cycle). \n"
    "    > demo-dsa --dsaport=3 --sensitivity=0.75 \n"
    "  \n"
    "  - Set the sensitivity of all tactile sensor matrixes to 0.75 persistently.\n"
    "    The value will be stored persistently (i.e. will remain after reset or \n"
    "    power cycle). \n"
    "    > demo-dsa --dsaport=3 --sensitivity=0.75 --persistent\n"
    "  \n"
    "  - Reset the sensitivity of all tactile sensor matrixes to factory default.\n"
    "    > demo-dsa --dsaport=3 --sensitivity=0.75 --reset\n"
    "  \n"
    "  - Set the sensitivity of tactile sensor matrices 1 and 4 to individual\n"
    "    values temporarily.\n"
    "    The value will be used only temporarily (until reset or power cycle).\n"
    "    Sensor 1 (distal sensor of finger 1) will be set to 0.1\n"
    "    Sensor 4 (proximal sensor of finger 3) will be set to 0.4\n"
    "    > demo-dsa --dsaport=3 --sensitivity1=0.1 --sensitivity4=0.4 \n"

    "  - Like for the sensitivity the thresholds can be adjusted via \n"
    "    the --threshold=VALUE parameter.\n"
    "\n"
    "- Known bugs:\n"
    "  - see the bug description for \"cDSAException: Checksum Error on Windows\n"
    "    console\" in the Related Pages->Bug List section of the doxygen\n"
    "    documentation\n"
    ;

char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-dsa.cpp 10351 2013-06-18 16:28:14Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2008 SCHUNK GmbH & Co. KG";

char const* usage =
  "usage: demo-dsa [options]\n"
  ;

//  end of doxygen name group sdhlibrary_cpp_demo_dsa_cpp_vars
//  @}
//----------------------------------------------------------------------



int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    int i;

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( "general sdhcom_common dsacom dsaother dsaadjust" );

    options.Parse( argc, argv, __help__, "demo-dsa", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

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

    //---------------------
    // start actual processing:
    cDSA* ts = NULL;
    try
    {

        if ( options.dsa_use_tcp )
        {
            cdbg << "debug_level=" << options.debug_level << " tcp_adr=" << options.tcp_adr << " dsa_tcp_port=" << options.dsa_tcp_port << "\n";
            ts = new cDSA( options.debug_level, options.tcp_adr.c_str(), options.dsa_tcp_port, options.timeout );
        }
        else
        {
            cdbg << "debug_level=" << options.debug_level << " dsaport=" << options.dsaport << " dsa_rs_device=" << options.dsa_rs_device << "\n";
		    ts = new cDSA( options.debug_level, options.dsaport, options.dsa_rs_device );
        }


        //---------------------
        // print requested info:
        if ( options.controllerinfo )
        {
            cout << "Controller Info:\n";
            cout << ts->GetControllerInfo();
        }

        if ( options.sensorinfo )
        {
            cout << "Sensor Info:\n";
            cout << ts->GetSensorInfo();
        }

        for ( i=0; i<6; i++ )
        {
            if ( options.matrixinfo[i] < 0 )
                continue;

            cout << "Matrix Info " << options.matrixinfo[i] << ":\n";
            cout << ts->GetMatrixInfo( options.matrixinfo[i] );

            if ( ts->GetControllerInfo().sw_version >= 268 )
            {
                cDSA::sSensitivityInfo sensitivity_info;

                sensitivity_info = ts->GetMatrixSensitivity(i);
                cout << "  sensitivity         = " << sensitivity_info.cur_sens  << "\n";
                cout << "  factory_sensitivity = " << sensitivity_info.fact_sens << "\n";
                cout << "  threshold           = " << ts->GetMatrixThreshold(i)   << "\n";
            }
        }
        //---------------------

        if ( options.showdsasettings )
        {
            if ( ts->GetControllerInfo().sw_version < 268 )
            {
                cerr <<  "To be able to read the sensitivity/threshold settings you must update\n";
                cerr <<  "the firmware of the DSACON32m (tactile sensor controller in the SDH)\n";
                cerr <<  "to at least release R268.\n";
            }
            else
            {
                cDSA::sSensitivityInfo sensitivity_info;
                char const* descr[] = { " proximal", " distal" };
                for ( i=0; i<6; i++ )
                {
                    sensitivity_info = ts->GetMatrixSensitivity(i);
                    cout << "Sensor " << i << " (finger " << i/2+1 << descr[i%2] << ")\n";
                    cout << "  sensitivity         = " << sensitivity_info.cur_sens  << "\n";
                    cout << "  factory_sensitivity = " << sensitivity_info.fact_sens << "\n";
                    cout << "  threshold           = " << ts->GetMatrixThreshold(i)   << "\n";
                }
            }
        }


        //---------------------
        // Set sensitivities if requested
        for ( i=0; i<6; i++ )
        {
            if ( options.sensitivity[i] < 0.0 )
                continue;
            ts->SetMatrixSensitivity(i, options.sensitivity[i], false, options.reset_to_default, options.persistent );
        }

        // Set thresholds if requested
        for ( i=0; i<6; i++ )
        {
            if ( options.threshold[i] > 4095 )
                continue;
            ts->SetMatrixThreshold(i, options.threshold[i], false, options.reset_to_default, options.persistent );
        }
        //---------------------


        //---------------------
        // Read and show actual tactile sensor data ("full frame")
        if ( options.fullframe )
        {
            bool do_single_frames = options.framerate < 30;

            if ( do_single_frames )
            {
                // Make remote tactile sensor controller stop sending data automatically as fast as possible (prepare for DSA pull-mode):
                cdbg << "Starting DSA pull-mode, framerate=0 do_rle=" << options.do_RLE << " do_data_acquisition=false" <<  "\n";
                ts->SetFramerate( 0, options.do_RLE, false );
            }
            else
            {
                // Make remote tactile sensor controller send data automatically as fast as possible (DSA push-mode):
                cdbg << "Starting DSA push-mode, framerate=1 do_rle=" << options.do_RLE << " do_data_acquisition=true" <<  "\n";
                ts->SetFramerate( 1, options.do_RLE );
            }


            //-----------
            // start periodic or one time processing of full frames if requested:
            double period_s = 0.0;
            if ( options.framerate > 0)
                period_s = 1.0 / double(options.framerate);
            double remaining_s;
            int nb_errors = 0;
            int nb_frames = 0;
            cSimpleTime start;
            cSimpleTime now;
            cSimpleTime last;
            int nb_last = 0;
            do
            {
                //-----------
                try
                {
                    if ( do_single_frames )
                    {
                        start.StoreNow();
                        ts->SetFramerateRetries( 0, options.do_RLE, true, 3 );
                    }
                    ts->UpdateFrame();
                    now.StoreNow();
                    nb_frames++;
                    cout << *ts;
                    if ( options.framerate > 0 )
                        cout << "Actual framerate=" << ((nb_frames-nb_last)/last.Elapsed( now )) << "Hz nb_frames=" << nb_frames << " nb_errors=" << nb_errors << " (" << ((100.0*nb_errors)/nb_frames) << "%)\n";

                    if ( last.Elapsed( now ) > 3.0 )
                    {
                        last = now;
                        nb_last = nb_frames;
                    }
                    cout.flush();
                }
                catch ( cDSAException* e)
                {
                    nb_errors++;
                    cerr << "Caught and ignored cDSAException: " << e->what() << " nb_errors=" << nb_errors << "\n";
                    delete e;
                }
                //-----------

                //-----------
                if ( do_single_frames )
                {
                    remaining_s = period_s - (start.Elapsed());
                    if ( remaining_s > 0.0 )
                        SleepSec(remaining_s);
                }
            } while ( options.framerate > 0 );
            //---------------------
        }
        //---------------------

        ts->Close();
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "\ndemo-dsa main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "\ncaught unknown exception, giving up\n";
    }
    delete ts;
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
