//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_mimic_general General file information

       \author   Dirk Osswald
       \date     2007-03-07

     \brief
       Print measured actual axis angles of an attached %SDH. (C++ demo application using the SDHLibrary-CPP library.)
       See \ref demo_mimic__help__ "__help__" and online help ("-h" or "--help") for available options.

     \section sdhlibrary_cpp_sdh_demo_mimic_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_mimic_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2011-03-01 18:35:45 +0100 (Di, 01 Mrz 2011) $
         \par SVN file revision:
           $Id: demo-mimic.cpp 6501 2011-03-01 17:35:45Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_mimic_changelog Changelog of this file:
         \include demo-mimic.cpp.log

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

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
    \anchor sdhlibrary_cpp_demo_mimic_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_mimic__help__
char const* __help__      =
    "In case you have 2 SDHs you can operate one of them by moving the first\n"
    "hand manualy. You must give parameters for 2 hands on the command line.\n"
    "(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "- Example usage:\n"
    "  - Mimic the manual movements of SDH on port 2 = COM3 \n"
    "    with the SDH on port 5 = COM6:\n"
    "    > demo-mimic -p 2  -p 5\n"
    "    \n"
    "  - Mimic the manual movements of SDH with CAN IDs 0x01 and 0x11 on\n"
    "    ESD CAN bus with the SDH with CAN IDs 0x02 and 0x2 on the same\n"
    "    ESD CAN bus\n"
    "    > demo-mimic --can --id_read 0x1 --id_write 0x11  --can --id_read 0x2 --id_write 0x22\n"
    "     \n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-mimic.cpp 6501 2011-03-01 17:35:45Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_mimic_cpp_vars
//  @}

//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

// create a cSDH object according to \a options and return a ptr to it
cSDH* GetHand( cSDHOptions& options, cDBG& cdbg, int nb )
{
    // cSDH instance "hand" of the class cSDH according to the given options:
    cSDH* hand = new cSDH( options.use_radians, options.use_fahrenheit, options.debug_level );
    cdbg << "Successfully created cSDH instance " << nb << "\n";

    // Open configured communication to the SDH device
    options.OpenCommunication( *hand );
    cdbg << "Successfully opened communication to SDH " << nb << "\n";

    return hand;
}


int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // this script is special since it uses 2 SDHs,

    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options1( SDHUSAGE_DEFAULT " sdhother" );
    cSDHOptions options2( SDHUSAGE_DEFAULT " sdhother" );

    options1.period = 0.5;

    // so detect where the options for the 2nd hand begin
    int i = 0;
    int nb_sdh = 0;
    while ( i<argc && nb_sdh < 2 )
    {
        if ( !strncmp( argv[i], "-p", strlen( "-p" ) ) ||
                        !strncmp( argv[i], "--port", strlen( "--port" ) ) ||
                        !strncmp( argv[i], "--sdhport", strlen( "--sdhport" ) ) ||
                        !strncmp( argv[i], "--sdh_rs_device", strlen( "--sdh_rs_device" ) ) ||
                        !strncmp( argv[i], "-c", strlen( "-c" ) ) ||
                        !strncmp( argv[i], "--can", strlen( "--can" ) ) ||
                        !strncmp( argv[i], "--canesd", strlen( "--canesd" ) ) ||
                        !strncmp( argv[i], "--canpeak", strlen( "--canpeak" ) ) ||
                        !strncmp( argv[i], "--sdh_canpeak_device", strlen( "--sdh_canpeak_device" ) ) )
            nb_sdh++;
        i++;
    }
    // parse options for 1st hand:
    options1.Parse( i-((nb_sdh==2)*1), argv, __help__, "demo-mimic", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

    if ( nb_sdh != 2 )
    {
        cerr << "You must provide parameters for exactly 2 SDHs. See --help\n";
        exit(1);
    }

    // parse options for 2nd hand:
    // add fake argv[0]
    argv[i-2] = argv[0];
    optind = 1;
    options2.Parse( argc-i+2, argv+i-2, __help__, "demo-mimic", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
    //
    //---------------------

    //---------------------
    // initialize debug message printing:
    if ( options2.debuglog != &cerr)
        g_sdh_debug_log = options2.debuglog;
    else
        g_sdh_debug_log = options1.debuglog;

    cDBG cdbg( (options1.debug_level + options2.debug_level) > 0, "red", g_sdh_debug_log );

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    // reduce debug level for subsystems
    options1.debug_level-=1;
    options2.debug_level-=1;
    //---------------------

    try
    {
        cSDH* hand[2];

        hand[0] = GetHand( options1, cdbg, 0 );
        hand[1] = GetHand( options2, cdbg, 1 );


        hand[0]->SetAxisEnable( cSDH::All, false );

        hand[1]->SetController( cSDH::eCT_POSE );
        hand[1]->SetVelocityProfile( cSDH::eVP_RAMP );
        hand[1]->SetAxisTargetVelocity( cSDH::All, 40.0 );
        hand[1]->SetAxisEnable( cSDH::All, true );

        vector<double> min_angles = hand[1]->GetAxisMinAngle( hand[1]->all_real_axes );
        vector<double> max_angles = hand[1]->GetAxisMaxAngle( hand[1]->all_real_axes );
        vector<double> angles;

#if SDH_USE_VCC
        double elapsed = 0.0;
#else
        cSimpleTime start;
#endif

        angles = hand[0]->GetAxisActualAngle( hand[0]->all_real_axes );
        ToRange( angles, min_angles, max_angles );
        hand[1]->SetAxisEnable( cSDH::All, true );
        hand[1]->MoveHand(true);

        while (true)
        {
            angles = hand[0]->GetAxisActualAngle( hand[0]->all_real_axes );
            ToRange( angles, min_angles, max_angles );
            hand[1]->SetAxisTargetAngle( hand[1]->all_real_axes, angles );
            hand[1]->MoveHand(false);

#if SDH_USE_VCC
            cout << elapsed << " ";
            elapsed += options1.period;
#else
            cout << start.Elapsed() << " ";
#endif

            for ( vector<double>::const_iterator ai = angles.begin();
                  ai != angles.end();
                  ai++ )
                cout << *ai << " ";

            cout << "\n";
            cout.flush();

            SleepSec( options1.period );
        }

        hand[0]->Close();
        hand[1]->Close();
        cdbg << "Successfully disabled controllers of SDHs and closed connections\n";
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "demo-mimic main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "caught unexpected exception!\n";
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
