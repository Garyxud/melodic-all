//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_simple_withtiming_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple C++ programm to make an attached %SDH move

       This code contains only the very basicst use of the features
       provided by the SDHLibrary-CPP. For more sophisticated
       applications see the other demo-*.cpp programms, or of course
       the html/pdf documentation.

     \section sdhlibrary_cpp_sdh_demo_simple_withtiming_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_simple_withtiming_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2010-12-02 14:59:45 +0100 (Do, 02 Dez 2010) $
         \par SVN file revision:
           $Id: demo-simple-withtiming.cpp 6265 2010-12-02 13:59:45Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_simple_withtiming_changelog Changelog of this file:
         \include demo-simple-withtiming.cpp.log

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
#include "sdh/simpletime.h"
#include "sdh/dbg.h"

USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_simple_withtiming_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Move proximal and distal joints of finger 1 three times by 10 degrees and measure time for these actions.\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-simple-withtiming.cpp 6265 2010-12-02 13:59:45Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_simple_withtiming_cpp_vars
//  @}

char const* usage =
  "usage: demo-simple-withtiming [options]\n"
  ;


int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-simple-withtiming", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
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

    int    iFinger        = 0;     // The index of the finger to move

    try
    {
        // Create an instance "hand" of the class cSDH:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        // Now perform some action:
        //   get the current actual axis angles of finger iFinger:
        std::vector<double> faa = hand.GetFingerActualAngle( iFinger );

        //   sometimes the actual angles are reported slightly out of range
        //   (Like -0.001 for axis 0 ). So limit the angles to the allowed range:
        ToRange( faa, hand.GetFingerMinAngle( iFinger ), hand.GetFingerMaxAngle( iFinger ) );

        //   modify faa by decrementing the proximal and the distal axis angles
        //   (make a copy fta of faa and modify that to keep actual pose available)
        std::vector<double> fta = faa;

        std::vector<double> faa2;

        fta[1] -= 10.0;
        fta[2] -= 10.0;

        //   keep fta in range too:
        ToRange( fta, hand.GetFingerMinAngle( iFinger ), hand.GetFingerMaxAngle( iFinger ) );

        std::cout << "Moving finger " << iFinger << " between faa=" << faa << " and fta=" << fta << "\n";

        cSimpleTime start;
        double dt1, dt2, dt3, T;
        //   now move for 3 times between these two poses:
        std::cout << "SetFingerTargetAngle\t";
        std::cout << "GetFingerActualAngle\t";
        std::cout << "MoveHand\t";
        std::cout << "\n";
        for (int i=0; i<20; i++ )
        {
            // set a new target angles
            start.StoreNow();
            hand.SetFingerTargetAngle( iFinger, fta );
            hand.SetFingerTargetAngle( iFinger, fta );
            hand.SetFingerTargetAngle( iFinger, fta );
            hand.SetFingerTargetAngle( iFinger, fta );
            hand.SetFingerTargetAngle( iFinger, fta );
            dt1 = start.Elapsed() / 5.0;

            // get actual angles
            start.StoreNow();
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            dt2 = start.Elapsed() / 5.0;

            // and make the finger move there:
            start.StoreNow();
            T = hand.MoveHand( false );
            dt3 = start.Elapsed();

            SleepSec( T );

            std::cout << dt1 << "\t" << dt2 << "\t" << dt3 << "\n";

            // set a new target angles
            start.StoreNow();
            hand.SetFingerTargetAngle( iFinger, faa );
            hand.SetFingerTargetAngle( iFinger, faa );
            hand.SetFingerTargetAngle( iFinger, faa );
            hand.SetFingerTargetAngle( iFinger, faa );
            hand.SetFingerTargetAngle( iFinger, faa );
            dt1 = start.Elapsed() / 5.0;

            // get actual angles
            start.StoreNow();
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            faa2 = hand.GetFingerActualAngle( iFinger );
            dt2 = start.Elapsed() / 5.0;

            // and make the finger move there:
            start.StoreNow();
            T = hand.MoveHand( false );
            dt3 = start.Elapsed();

            SleepSec( T );

            std::cout << dt1 << "\t" << dt2 << "\t" << dt3 << "\n";

        }


        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-simple-withtiming main(): An exception was caught: " << e->what() << "\n";
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
