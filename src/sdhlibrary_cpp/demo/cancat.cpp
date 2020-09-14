//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_cancat_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Yet incomplete tool to send and receive data via CAN.
       See \ref cancat__help__ "__help__" and online help ("-h" or "--help") for available options.

     \section sdhlibrary_cpp_sdh_cancat_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_cancat_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-02-04 16:39:29 +0100 (Mon, 04 Feb 2013) $
         \par SVN file revision:
           $Id: cancat.cpp 9739 2013-02-04 15:39:29Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_cancat_changelog Changelog of this file:
         \include cancat.cpp.log

*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#if WITH_ESD_CAN
# include "sdh/canserial-esd.h"
#endif
#if WITH_PEAK_CAN
# if ! SDH_USE_VCC
#  include <unistd.h>
# endif
# include "sdh/canserial-peak.h"
#endif
#if ! WITH_ESD_CAN  && ! WITH_PEAK_CAN
# error "At least support for ESD-CAN or PEAK-CAN must be enabled to compile cancat"
#endif
#include "sdh/sdh.h"
#include "sdh/dbg.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdhoptions.h"

using namespace std;
USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_cancat_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor cancat__help__
char const* __help__      = "Send data from command line via ESD or PEAK CAN and display replies until CTRL-C is pressed.";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: cancat.cpp 9739 2013-02-04 15:39:29Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_cancat_cpp_vars
//  @}

int main( int argc, char** argv )
{
    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( "general sdhcom_common sdhcom_esdcan sdhcom_peakcan sdhcom_cancommon" );

    options.timeout = 0.0;

    options.Parse( argc, argv, __help__, "cancat", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

    //
    //---------------------

    cDBG dbg( options.debug_level > 0, "red", options.debuglog );
    g_sdh_debug_log = options.debuglog;

    try
    {
        cSerialBase* canserial;

#if WITH_ESD_CAN
        if ( options.use_can_esd )
            canserial = new cCANSerial_ESD( options.net, options.can_baudrate, options.timeout, options.id_read, options.id_write );
#endif
#if WITH_PEAK_CAN
        if ( options.use_can_peak )
            canserial = new cCANSerial_PEAK( options.can_baudrate, options.timeout, options.id_read, options.id_write, options.sdh_canpeak_device );
#endif
        canserial->dbg.SetFlag( options.debug_level > 2 );

        canserial->Open();

        if ( optind < argc)
        {
            char buffer[ 512 ];
            buffer[0] = '\0';
            char const* sep = "";
            while (optind < argc)
            {
                strncat( buffer, argv[ optind++ ], 511 );
                strncat( buffer, sep, 511 );
                sep = " ";
            }
            strncat( buffer, "\r\n", 511 );

            dbg << "Sending String \"" << buffer << "\" via CAN\n";


            canserial->write( buffer, strlen( buffer ) );
        }
        while ( 1 )
        {
            char reply;
            if ( 1 == canserial->Read( &reply, 1, long(options.timeout*1E6), false ) )
                printf( "%c", reply );
            else
                dbg << "nutin\n";
            fflush( stdout );
        }
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "cancat main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "cancat main(): caught unexpected exception!\n";
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
