//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!

    \file
     \section sdhlibrary_cpp_sdh_demo_benchmark_general General file information

       \author   Dirk Osswald
       \date     2011-01-31

     \brief
       Simple script to benchmark communication speed of the SDH.
       See \ref demo_benchmark__help__ "__help__" and online help ("-h" or "--help") for available options.



     \section sdhlibrary_cpp_sdh_demo_benchmark_copyright Copyright

     Copyright (c) 2011 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_benchmark_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-11-11 15:15:08 +0100 (Mon, 11 Nov 2013) $
         \par SVN file revision:
           $Id: demo-benchmark.cpp 10895 2013-11-11 14:15:08Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_benchmark_changelog Changelog of this file:
         \include demo-benchmark.cpp.log

*/
/*!
  @}
*/
//======================================================================

#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdh/dsa.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

#define DEMO_BENCHMARK_USE_COMBINED_SET_GET 1

/*!
    \anchor sdhlibrary_cpp_demo_benchmark_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_benchmark__help__
char const* __help__      =
    "Simple script to benchmark communication speed of the SDH:\n"
    "The hand will move to a start position in coordinated position control\n"
    "mode first. Then periodic movements are performed using the velocity\n"
    "with acceleration ramp controller while the communication and control\n"
    "rate is printed.\n"
    "\n"
    "- Example usage:\n"
    "  - Make SDH connected via Ethernet move.\n"
    "    The SDH has IP-Address 192.168.1.42 and is attached to TCP port 23.\n"
    "    (Requires at least SDH-firmware v0.0.3.1)\n"
    "    > demo-benchmark --tcp=192.168.1.42:23\n"
    "     \n"
    "  - Make SDH connected to port 2 = COM3 move:\n"
    "    > demo-benchmark -p 2  --dsaport=3\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 move:\n"
    "    > demo-benchmark --sdh_rs_device=/dev/ttyUSB0  --dsa_rs_device=/dev/ttyUSB1\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-benchmark --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected to:\n"
    "    - port 2 = COM3 (joint controllers) and \n"
    "    - port 3 = COM4 (tactile sensor controller) \n"
    "    > demo-benchmark --port=2 --dsaport=3 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-benchmark.cpp 10895 2013-11-11 14:15:08Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_benchmark_cpp_vars
//  @}

char const* usage =
  "usage: demo-benchmark [options]\n"
  ;

cDBG cdbg( false, "red" );

//! structure to hold current hand state while recording with demo-benchmark
struct sRecordedData
{
    double t;
    std::vector<double> aaa;
    std::vector<double> aav;
    std::vector<double> atv;

    sRecordedData( double _t, std::vector<double> const& _aaa, std::vector<double> const& _aav, std::vector<double> const& _atv ) :
        t( _t ),
        aaa( _aaa ),
        aav( _aav ),
        atv( _atv )
        {};
};
//-----------------------------------------------------------------

/*!
 * Move all axes of \a hand to position \a ta
 * using the coordinated pose controller
 *
 * @param hand - reference to the SDH to operate on
 * @param ta - axis target angles
 */
void GotoPose( cSDH& hand, std::vector<double>& ta )
{
    hand.SetController( hand.eCT_POSE );
    hand.SetAxisTargetVelocity( hand.All, 50 );

    hand.SetAxisTargetAngle( hand.all_real_axes, ta );
    hand.MoveAxis( hand.All );
}
//-----------------------------------------------------------------


int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( SDHUSAGE_DEFAULT );

    options.Parse( argc, argv, __help__, "demo-benchmark", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

    //
    //---------------------

    //---------------------
    // initialize debug message printing:
    cdbg.SetFlag( options.debug_level > 0 );
    cdbg.SetOutput( options.debuglog );

    g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    // reduce debug level for subsystems
    options.debug_level-=1;
    //---------------------

    try
    {
        //###
        // Create an instance "hand" of the class cSDH:
        cdbg << "Connecting to joint controller...\n";
        cSDH hand( false, false, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        std::vector<double> max_vel = hand.GetAxisMaxVelocity( hand.all_real_axes );
        unsigned int nb_axes = hand.all_real_axes.size();
        //###


        //###
        // Prepare movements:

        // the given start positions:
        std::vector<double> start_pose;  // axis start positions in deg
        start_pose.push_back( 10 );
        start_pose.push_back( -10 );
        start_pose.push_back( 0 );
        start_pose.push_back( -10 );
        start_pose.push_back( 0 );
        start_pose.push_back( -10 );
        start_pose.push_back( 0 );

        // the given end positions:
        std::vector<double> end_pose;  // axis end positions in deg
        end_pose.push_back( 80.0 );
        end_pose.push_back( -80.0 );
        end_pose.push_back( -80.0 );
        end_pose.push_back( -80.0 );
        end_pose.push_back( -80.0 );
        end_pose.push_back( -80.0 );
        end_pose.push_back( -80.0 );

        // we want to make every axis move independently with velocity
        //   v(t)=a*sin(2*PI * t/p)
        // for given period p and amplitude a
        // the position is determined by:
        //   s(t)= a * p / (2*PI) * (1-cos(2*PI * t/p))
        // The maximum position m is at p/2
        //   m= a*p/PI
        // For fixed maximum position the amplitude thus calculates to
        //   a= m*PI/p

        // the given periods p:
        std::vector<double> aperiod;  // axis movement cycle periods in s
        aperiod.push_back( 10.0 );
        aperiod.push_back( 5.0 );
        aperiod.push_back( 4.0 );
        aperiod.push_back( 3.0 );
        aperiod.push_back( 4.0 );
        aperiod.push_back( 5.0 );
        aperiod.push_back( 3.0 );


        // the calculated amplidutes p of the sinusoidal
        std::vector<double> aamplitude( aperiod.size(), 0.0 );  // axis movement velocity amplitude in deg/s
        unsigned int ai;
        for ( ai=0; ai<nb_axes; ai++ )
        {
            aamplitude[ai] = (end_pose[ai]-start_pose[ai]) * M_PI / aperiod[ai];
            // limit to allowed range:
            ToRange( aamplitude[ai], -max_vel[ai]+1.0, +max_vel[ai]-1.0 );
        }
        //###


        //###
        // move to start pose:
        GotoPose( hand, start_pose );
        //###


        //###
        // start moving sinusoidal:
        cdbg << "  Moving with velocity with acceleration ramp controller.\n";
        cdbg.flush();

        // switch to velocity with acceleration ramp controller type:
        hand.SetController( hand.eCT_VELOCITY_ACCELERATION );
        hand.SetAxisTargetAcceleration( hand.All, 100 );

        std::vector<double> aaa(nb_axes, 0.0);  // axis actual angles
        std::vector<double> aav(nb_axes, 0.0);  // axis actual velocities
        std::vector<double> atv(nb_axes, 0.0);  // axis target velocities


        std::vector<double> min_angles = hand.GetAxisMinAngle( hand.all_real_axes );
        std::vector<double> max_angles = hand.GetAxisMaxAngle( hand.all_real_axes );
        std::vector<double> max_velocities = hand.GetAxisMaxVelocity( hand.all_real_axes );


        std::vector<sRecordedData> recorded_data;

        //###
        // perform motion for duration seconds
        double t;
        double const duration = 10.0; // duration in s
        cSimpleTime start_time;
        do
        {
            // get time elapsed since start_time
            t = start_time.Elapsed();

            // Get current position of axes:
            aaa = hand.GetAxisActualAngle( hand.all_real_axes );

            // calculate new target velocities for axes:
            for ( ai=0; ai<nb_axes; ai++ )
            {
                atv[ai] = aamplitude[ai] * sin( 2.0*M_PI/aperiod[ai] * t );
            }

#if DEMO_BENCHMARK_USE_COMBINED_SET_GET
            // Set target velocities and get current velocities of axes:
            aav = hand.SetAxisTargetGetAxisActualVelocity( hand.all_real_axes, atv );
#else
            // Get current velocities of axes:
            aav = hand.GetAxisActualVelocity( hand.all_real_axes );

            hand.SetAxisTargetVelocity( hand.all_real_axes, atv );
#endif
            //cdbg << t << ", " << aaa << ", " << aav << ", " << atv << "\n";
            //cdbg << t << ", " << atv << "\n";
            //std::cerr.flush();

            recorded_data.push_back( sRecordedData( t, aaa, aav, atv ) );
        } while ( t < duration );

        cSimpleTime end_time;
        //###

        //###
        // stop motion softly within brake_time second
        double brake_time = 0.5;
        do {
            t = end_time.Elapsed();

            for ( ai=0; ai<nb_axes; ai++ )
            {
                atv[ai] = aav[ai] * ( 1.0-t/brake_time);
            }
            hand.SetAxisTargetVelocity( hand.all_real_axes, atv );
        } while ( t < 0.5 );
        //###


        //###
        // open up again
        GotoPose( hand, start_pose );
        //
        //###


        //###
        // Close the connection to the SDH
        hand.Close();
        cdbg << "Successfully disabled joint controllers of SDH and closed connection\n";
        //###

        //###
        // do evaluation
        double dt_min = 1000.0;
        double t_dt_min;
        double dt_max = 0.0;
        double t_dt_max;
        double dt = 0.0;
        int r = 0;
        std::cout << "# combined gnuplot commands + data. Use plot.py for easy viewing\n";
        std::cout << "## plot using 2:3 with lines title 'dt [s]'\n";
        std::cout << "## plot using 2:4 with lines title 'aaa[0] [deg]'\n";
        std::cout << "## plot using 2:5 with lines title 'aaa[1] [deg]'\n";
        std::cout << "## plot using 2:6 with lines title 'aaa[2] [deg]'\n";
        std::cout << "## plot using 2:7 with lines title 'aaa[3] [deg]'\n";
        std::cout << "## plot using 2:8 with lines title 'aaa[4] [deg]'\n";
        std::cout << "## plot using 2:9 with lines title 'aaa[5] [deg]'\n";
        std::cout << "## plot using 2:10 with lines title 'aaa[6] [deg]'\n";
        std::cout << "## plot using 2:11 with lines title 'aav[0] [deg/s]'\n";
        std::cout << "## plot using 2:12 with lines title 'aav[1] [deg/s]'\n";
        std::cout << "## plot using 2:13 with lines title 'aav[2] [deg/s]'\n";
        std::cout << "## plot using 2:14 with lines title 'aav[3] [deg/s]'\n";
        std::cout << "## plot using 2:15 with lines title 'aav[4] [deg/s]'\n";
        std::cout << "## plot using 2:16 with lines title 'aav[5] [deg/s]'\n";
        std::cout << "## plot using 2:17 with lines title 'aav[6] [deg/s]'\n";
        std::cout << "## plot using 2:18 with lines title 'atv[0] [deg/s]'\n";
        std::cout << "## plot using 2:19 with lines title 'atv[1] [deg/s]'\n";
        std::cout << "## plot using 2:20 with lines title 'atv[2] [deg/s]'\n";
        std::cout << "## plot using 2:21 with lines title 'atv[3] [deg/s]'\n";
        std::cout << "## plot using 2:22 with lines title 'atv[4] [deg/s]'\n";
        std::cout << "## plot using 2:23 with lines title 'atv[5] [deg/s]'\n";
        std::cout << "## plot using 2:24 with lines title 'atv[6] [deg/s]'\n";
        std::cout << "## set xlabel 'Time [s]'\n";
        std::cout << "## set ylabel 'Control-Period / Position / Velocity [s] / [deg] / [deg/s]'\n";
        std::cout << "## set grid\n";
        std::cout << "## set title  \"demo-benchmark: SDH moving in acceleration + velocity control mode\"\n";

        std::vector<sRecordedData>::iterator it0 = recorded_data.begin();
        std::vector<sRecordedData>::iterator it1 = recorded_data.begin();
        std::cout << "# i, t, dt, aaa[0..6], aav[0..6], atv[0..6]\n";
        std::cout << r << ", " << it1->t << ", " << dt << ", " << it1->aaa << ", " << it1->aav << ", " << it1->atv << "\n";
        for ( it1++; it1 < recorded_data.end(); it1++, it0++, r++ )
        {
            dt = it1->t - it0->t;
            if (  dt < dt_min )
            {
                dt_min = dt;
                t_dt_min = it1->t;
            }
            if (  dt > dt_max )
            {
                dt_max = dt;
                t_dt_max = it1->t;
            }

            std::cout << r << ", " << it1->t << ", " << dt << ", " << it1->aaa << ", " << it1->aav << ", " << it1->atv << "\n";
        }
        double dt_avg = start_time.Elapsed( end_time ) / recorded_data.size();
        std::cout << "## set label 'dt_min=" << dt_min << "' at " << t_dt_min << "," << dt_min << " front point\n";
        std::cout << "## set label 'dt_max=" << dt_max << "' at " << t_dt_max << "," << dt_max << " front point\n";
        std::cout << "## set label 'dt_avg=" << dt_avg << "' at " << it0->t << "," << dt_avg << " front point\n";
        std::cout << "## set title  \"demo-benchmark.cpp: SDH moving in acceleration + velocity control mode\\ndt_avg = " << dt_avg << "   fps = " << 1.0/dt_avg;
        std::cout << "\\ndebug_level = " << options.debug_level+1 << "   DEMO_BENCHMARK_USE_COMBINED_SET_GET = " << DEMO_BENCHMARK_USE_COMBINED_SET_GET << "\"\n";
        std::cout.flush();
        //###
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-benchmark main(): An exception was caught: " << e->what() << "\n";
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
