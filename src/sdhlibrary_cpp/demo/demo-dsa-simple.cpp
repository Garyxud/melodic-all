//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
   \file
   \section sdhlibrary_cpp_demo_dsa_simple_cpp_general General file information

     \author   Dirk Osswald
     \date     2014-04-03

   \brief
     Simple program to test class cDSA (tactile sensor reading).
     See \ref demo_dsa_simple__help__ "__help__" and online help ("-h" or "--help") for available options.

   \section sdhlibrary_cpp_demo_dsa_simple_cpp_copyright Copyright

   - Copyright (c) 2014 SCHUNK GmbH & Co. KG

   <HR>
   \internal

     \subsection sdhlibrary_cpp_demo_dsa_simple_cpp_details SVN related, detailed file specific information:
       $LastChangedBy: Osswald2 $
       $LastChangedDate: 2014-09-30 10:28:44 +0200 (Tue, 30 Sep 2014) $
       \par SVN file revision:
         $Id: demo-dsa-simple.cpp 12284 2014-09-30 08:28:44Z Osswald2 $

   \subsection sdhlibrary_cpp_demo_dsa_simple_cpp_changelog Changelog of this file:
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
#include <fstream>
#include <string>
#include <vector>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/sdh.h"
#include "sdh/dsa.h"
#include "sdh/basisdef.h"
#include "sdh/util.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
  \anchor sdhlibrary_cpp_demo_dsa_simple_cpp_vars
  \name   Some informative variables

  Some definitions that describe the demo program

  @{
*/
//! \anchor demo_dsa_simple__help__
char const* __help__      =
    "Simple demo to test cDSA class of SDHLibrary-cpp for tactile sensor data\n"
    "reading only.\n"
    "\n"
    "This program does not use command line options. Instead all communication\n"
    "parameters are fixed here in the source code\n"
    ;

char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-dsa-simple.cpp 12284 2014-09-30 08:28:44Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2014 SCHUNK GmbH & Co. KG";

char const* usage =
  "usage: demo-dsa-simple\n"
  ;

//  end of doxygen name group sdhlibrary_cpp_demo_dsa_simple_cpp_vars
//  @}
//----------------------------------------------------------------------



int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //string tcp_adr   = "192.168.1.42";  // TODO: adjust this to your needs!
    string tcp_adr   = "192.168.100.200";  // TODO: adjust this to your needs!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    int debug_level  = 5;
    int dsa_tcp_port = 13000;
    double timeout   = -1.0;            // no timeout
    int framerate    = 5;              // max framerate for data reading ( < 30 => pull mode; 30 => push mode)
    bool do_RLE      = true;

    //---------------------
    // initialize debug message printing:
    //ios_base::openmode mode = ios_base::app;
    ostream* debuglog = NULL;
    debuglog = new ofstream( "demo-dsa-simple.log", ios_base::app );//mode );
    cDBG cdbg( debug_level>0, "red", debuglog );
    g_sdh_debug_log = debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    //---------------------

    //---------------------
    // start actual processing:
    cDSA* ts = NULL;
    try
    {
        // Create and opent cDSA object to communicate with tactile sensors
        cdbg << "debug_level=" << debug_level << " tcp_adr=" << tcp_adr << " dsa_tcp_port=" << dsa_tcp_port << "\n";
        ts = new cDSA( debug_level, tcp_adr.c_str(), dsa_tcp_port, timeout );

        //---------------------
        // Read and show actual tactile sensor data ("full frame")
        bool do_single_frames = framerate < 30;

        if ( do_single_frames )
        {
            // Make remote tactile sensor controller stop sending data automatically as fast as possible (prepare for DSA pull-mode):
            cdbg << "Starting DSA pull-mode, framerate=0 do_rle=" << do_RLE << " do_data_acquisition=false" <<  "\n";
            ts->SetFramerate( 0, do_RLE, false );
        }
        else
        {
            // Make remote tactile sensor controller send data automatically as fast as possible (DSA push-mode):
            cdbg << "Starting DSA push-mode, framerate=1 do_rle=" << do_RLE << " do_data_acquisition=true" <<  "\n";
            ts->SetFramerate( 1, do_RLE );
        }


        //-----------
        // start periodic or one time processing of full frames if requested:
        double period_s = 0.0;
        if ( framerate > 0)
            period_s = 1.0 / double(framerate);
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
                    ts->SetFramerateRetries( 0, do_RLE, true, 3 );
                }


                /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                 * Here the tactile sensor data is read from the DSACON32m tactile sensor controller:
                 */
                ts->UpdateFrame();
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


                now.StoreNow();
                nb_frames++;


                /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                 * Here the tactile sensor data is put to screen:
                 */
                cout << *ts;
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


                if ( framerate > 0 )
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
                cdbg << "Caught and ignored cDSAException: " << e->what() << " nb_errors=" << nb_errors << "\n";
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
        } while ( true );
        //---------------------
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "\ndemo-dsa-simple main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        cdbg << "\ndemo-dsa-simple main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "\ncaught unknown exception, giving up\n";
        cdbg << "\ncaught unknown exception, giving up\n";
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
