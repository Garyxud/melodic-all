//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!

    \file
     \section sdhlibrary_cpp_sdh_demo_contact_grasping_general General file information

       \author   Dirk Osswald
       \date     2009-08-02

     \brief
       Simple script to do grasping using tactile sensor info feedback.
       See \ref demo_contact_grasping__help__ "__help__" and online help ("-h" or "--help") for available options.



     \section sdhlibrary_cpp_sdh_demo_contact_grasping_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_contact_grasping_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2013-11-11 14:58:38 +0100 (Mon, 11 Nov 2013) $
         \par SVN file revision:
           $Id: demo-contact-grasping.cpp 10894 2013-11-11 13:58:38Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_contact_grasping_changelog Changelog of this file:
         \include demo-contact-grasping.cpp.log

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
#include "sdh/dsa.h"
#include "sdhoptions.h"
#include "dsaboost.h"

USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_contact_grasping_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_contact_grasping__help__
char const* __help__      =
    "Simple script to do grasping with tactile sensor info feedback:\n"
    "The hand will move to a pregrasp pose (open hand). You can then\n"
    "reach an object to grasp into the hand. The actual grasping\n"
    "is started as soon as a contact is detected. The finger\n"
    "joints then try to move inwards until a certain\n"
    "force is reached on the corresponding tactile sensors.\n"
    "\n"
    "- Example usage:\n"
    "  - Make SDH and tactile sensors connected via Ethernet grasp.\n"
    "    The SDH and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    TCP port 23 and the tactile sensors to TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-contact-grasping --tcp=192.168.1.42:23 --dsa_tcp=:13000\n"
    "     \n"
    "  - Make SDH connected to port 2 = COM3 with tactile sensors\n"
    "    connected to port 3 = COM4 grasp:\n"
    "    > demo-contact-grasping -p 2  --dsaport=3\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 and with tactile sensors\n"
    "    connected to USB to RS232 converter 1 grasp:\n"
    "    > demo-contact-grasping --sdh_rs_device=/dev/ttyUSB0  --dsa_rs_device=/dev/ttyUSB1\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected via Ethernet.\n"
    "    The joint controllers and the tactile sensors have a common IP-Address,\n"
    "    here 192.168.1.42. The SDH controller is attached to the \n"
    "    default TCP port 23 and the tactile sensors to the default TCP port 13000.\n"
    "    (Requires at least SDH-firmware v0.0.3.2)\n"
    "    > demo-contact-grasping --tcp=192.168.1.42 --dsa_tcp -v\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected to:\n"
    "    - port 2 = COM3 (joint controllers) and \n"
    "    - port 3 = COM4 (tactile sensor controller) \n"
    "    > demo-contact-grasping --port=2 --dsaport=3 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-contact-grasping.cpp 10894 2013-11-11 13:58:38Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_contact_grasping_cpp_vars
//  @}

char const* usage =
  "usage: demo-contact-grasping [options]\n"
  ;

cDBG cdbg( false, "red" );

//-----------------------------------------------------------------

void GotoStartPose( cSDH& hand, char const* msg )
{
    printf( "%s", msg );
    hand.SetController( hand.eCT_POSE );
    hand.SetAxisTargetVelocity( hand.All, 50 );
    std::vector<double> fa;
    fa.push_back( 90 );
    fa.push_back( -45 );
    fa.push_back( 0 );
    fa.push_back( -90 );
    fa.push_back( 0 );
    fa.push_back( -45 );
    fa.push_back( 0 );

    hand.SetAxisTargetAngle( hand.all_real_axes, fa );
    hand.MoveAxis( hand.All );
    printf( " OK\n" );
    fflush( stdout );
}
//-----------------------------------------------------------------

void AxisAnglesToFingerAngles( std::vector<double> aa, std::vector<double>& fa0, std::vector<double>& fa1, std::vector<double>& fa2 )
{
   fa0[0] = aa[0];
   fa0[1] = aa[1];
   fa0[2] = aa[2];

   fa1[0] = 0.0;
   fa1[1] = aa[3];
   fa1[2] = aa[4];

   fa2[0] = aa[0];
   fa2[1] = aa[5];
   fa2[2] = aa[6];
}
//-----------------------------------------------------------------


int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( SDHUSAGE_DEFAULT " dsacom" );

#if 0
    int unused_opt_ind =
#endif
    options.Parse( argc, argv, __help__, "demo-contact-grasping", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

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

    cDSA* ts;
    try
    {
        //###
        // Create an instance "hand" of the class cSDH:
        printf( "Connecting to joint controller..." );
        cSDH hand( false, false, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        printf( "OK\n" );
        //###


        //###
        // Create a global instance at "ts" (tactile sensor) of the class cDSA according to the given options:

        // overwrite user given value
        options.framerate = 30;
        options.timeout = 1.0; // a real timeout is needed to make the closing of the connections work in case of errors / interrupt

        printf( "Connecting to tactile sensor controller. This may take up to 8 seconds..." );

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
        printf( "OK\n" );
        //###


        //###
        // Prepare for grasping: open hand:
        GotoStartPose( hand, "\nPreparing for grasping, opening hand (using POSE controller)..." );
        //###


        // Start reading tactile sensor info in a thread:
        cDSAUpdater dsa_updater( ts, 8 );
        boost::this_thread::sleep(boost::posix_time::milliseconds(200)); // give the updater a chance to read in first frame

        cDSA::sContactInfo contact_info;

        //###
#if 0
        // for debugging, just output the local preprocessing results:
        cIsGraspedByArea is_grasped_obj( ts );
        double expected_area_ratio[6] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        for ( int i=0; i+unused_opt_ind<argc; i++ )
        {
            int rc = sscanf( argv[unused_opt_ind+i], "%lf", &expected_area_ratio[i] );
            assert( rc == 1 );
        }
        is_grasped_obj.SetCondition( expected_area_ratio );

        while  ( true )
        {
            for ( int m=0; m < 6; m++ )
            {
                contact_info = ts->GetContactInfo( m );
                printf( "  m=%d age=%ldms force=%6.1f x=%6.1f y=%6.1f area=%6.1f\n",
                        m,
                        ts->GetAgeOfFrame( (cDSA::sTactileSensorFrame*) &ts->GetFrame() ),
                        contact_info.force, contact_info.cog_x, contact_info.cog_y, contact_info.area );
            }
            printf( "=> IsGrasped: %d\n", is_grasped_obj.IsGrasped() );

            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
#endif
        //###

        //###
        // wait for contact to start grasping
        double desired_start_contact_area = 50.0;
        double contact_area;
        printf( "\nPress any tactile sensor on the SDH to start searching for contact..." ); fflush( stdout );
        do
        {
            contact_area = 0.0;

            for ( int m=0; m < 6; m++ )
            {
                contact_area += ts->GetContactArea( m );
                //printf( "  m=%d area2=%6.1f\n", m, ts->GetContactArea( m ) );
            }

            cdbg << "  contact area too small\n";
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        } while ( contact_area < desired_start_contact_area );

        // wait until that contact is released on the middle finger
        while ( ts->GetContactArea( 2 ) + ts->GetContactArea( 3 ) > desired_start_contact_area )
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(200)); // give the updater a chance to read in first frame
        }

        printf( "  OK, contact detected: %f mm*mm\n", contact_area ); fflush( stdout );
        //###

        //###
        // start grasping
        double desired_force = 5.0; //options.desired_force // the desired force that every sensor patch should reach
        double vel_per_force = 2.5;     // factor for converting force to velocity
        double loop_time = 0.01;
        int loop_time_ms = (int) (loop_time * 1000.0);

        bool finished = false;
        printf( "Simple force based grasping starts:\n" );
        printf( "  Joints of the opposing fingers 1 and 3 (axes 1,2,5,6) will move in\n" );
        printf( "  positive direction (inwards) as long as the contact force measured\n" );
        printf( "  by the tactile sensor patch of that joint is less than\n" );
        printf( "  the desired force %f\n", desired_force );
        printf( "  This will use the velocity with acceleration ramp controller.\n" );
        printf( "\n" );
        printf( "  Press a tactile sensor on the middle finger 2 to stop!\n" );
        fflush( stdout );


        // switch to velocity with acceleration ramp controller type:
        hand.SetController( hand.eCT_VELOCITY_ACCELERATION );
        hand.SetAxisTargetAcceleration( hand.All, 100 );
        cdbg << "max=" << hand.GetAxisMaxVelocity( hand.all_real_axes ) << "\n";

        std::vector<double> aaa;  // axis actual angles
        std::vector<double> ata;  // axis target angles
        std::vector<double> atv;  // axist target velocities
        std::vector<double> fta0(3); // finger target angles
        std::vector<double> fta1(3); // finger target angles
        std::vector<double> fta2(3); // finger target angles
        std::vector<double> fta[3];
        fta[0] = fta0;
        fta[1] = fta1;
        fta[2] = fta2;

        std::vector<double> min_angles = hand.GetAxisMinAngle( hand.all_real_axes );
        std::vector<double> max_angles = hand.GetAxisMaxAngle( hand.all_real_axes );
        std::vector<double> max_velocities = hand.GetAxisMaxVelocity( hand.all_real_axes );

        atv = hand.GetAxisTargetVelocity( hand.all_real_axes );
        while (true) //!finished:
        {
            int nb_ok = 0;

            //###
            // check for stop condition (contact on middle finger)
            cDSA::sContactInfo contact_middle_prox = ts->GetContactInfo( 2 );
            cDSA::sContactInfo contact_middle_dist = ts->GetContactInfo( 3 );
            if ( contact_middle_prox.area + contact_middle_dist.area > desired_start_contact_area )
            {
                printf( "\nContact on middle finger detected! Stopping demonstration.\n" ); fflush( stdout );
                finished = true;
                break;
            }
            //
            //###


            // Get current position of axes:
            // (Limited to the allowed range. This is necessary since near the
            //  range limits an axis might report an angle slightly off range.)
            std::vector<double> aaa = hand.GetAxisActualAngle( hand.all_real_axes );
            ToRange( aaa, min_angles, max_angles );
            ata = aaa;

            // for the selected axes:
            int ais[] = { 1,2, 5,6 }; // indices of used motor axes
            int mis[] = { 0,1, 4,5 }; // indices of used tactile sensors
            for ( int i=0; i < (int) (sizeof( ais ) / sizeof( int )); i++ )
            {
                int ai = ais[i];
                int mi = mis[i];
                // read the contact force of the tactile sensor of the axis
                contact_info = ts->GetContactInfo( mi );
                //cdbg << "%d,%d,%d  force=%6.1f x=%6.1f y=%6.1f area=%6.1f\n" % (ai, fi, part, force, cog_x, cog_y, area) # pylint: disable-msg=W0104
                cdbg << mi << " force=" << contact_info.force << "\n";

                // translate the measured force into a new velocity for the axis
                // in order to reach the desired_force
                atv[ai] = (desired_force - contact_info.force) * vel_per_force;

                // limit the calculated target velocities to the allowed range:
                atv[ai] = ToRange( atv[ai], -max_velocities[ai], max_velocities[ai] );

                if ( contact_info.force < desired_force )
                {
                    cdbg << "closing axis " << ai << " with " << atv[ai] << " deg/s\n";
                }
                else if ( contact_info.force > desired_force )
                {
                    cdbg << "opening axis " << ai << " with " << atv[ai] << " deg/s\n";
                }
                else
                {
                    cdbg << "axis " << ai << " ok\n";
                    nb_ok += 1;

                }

                //###
                // check if the fingers would get closer than 10mm with the calculated position:
                // calculate future position roughly according to determined velocity:
                ata[ai] += atv[ai] * loop_time; // s = v * t  =>  s(t') = s(t) + v * dt


                AxisAnglesToFingerAngles( ata, fta[0], fta[1], fta[2] );

                // TODO: CheckFingerCollisions not implemented in SDHLibrary-C++
#if 0
                (cxy, (c01,d01), (c02,d02), (c12,d12)) = hand.CheckFingerCollisions( fta[0], fta[1], fta[2] );
                d_min = min( d01, d02, d12);
                if ( (cxy || d_min < 2.0)  && force < desired_force )
                {
                    // if there would be a collision then do not move the current axis there
                    printf( "not moving axis %d further due to internal collision (d_min=%f)\n", ai, d_min );
                    ata[ai] = aaa[ai];
                    atv[ai] = 0.0;
                }
#else
                // simple approach for now: dont let any finger move into the other fingers half
                int fi = ( (ai<=2) ? 0 : 2 );
                std::vector<double> xyz = hand.GetFingerXYZ( fi, fta[fi] );

                switch (fi)
                {
                case 0:
                    if ( xyz[0] <= 6.0 )
                    {
                        // if there would be a collision then do not move the current axis there
                        printf( "not moving axis %d further due to quadrant crossing of finger %d xyz= %6.1f,%6.1f,%6.1f\n", ai, fi, xyz[0],xyz[1],xyz[2] );
                        ata[ai] = aaa[ai];
                        atv[ai] = 0.0;
                    }
                    break;
                case 2:
                    if ( xyz[0] >= -6.0 )
                    {
                        // if there would be a collision then do not move the current axis there
                        printf( "not moving axis %d further due to quadrant crossing of finger %d xyz= %6.1f,%6.1f,%6.1f\n", ai, fi, xyz[0],xyz[1],xyz[2] );
                        ata[ai] = aaa[ai];
                        atv[ai] = 0.0;
                    }
                    break;
                default:
                    assert( fi == 0  || fi == 2 );
                    break;
                }
#endif
                //
                //###

            } //
            // a new target velocity has been calculated from the tactile sensor data, so move accordingly
            cdbg.PDM( "moving with %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f deg/s\n", atv[0], atv[1], atv[2], atv[3], atv[4], atv[5], atv[6] );
            hand.SetAxisTargetVelocity( hand.all_real_axes, atv );
            finished = (nb_ok == 6);

            fflush( stdout );
            boost::this_thread::sleep(boost::posix_time::milliseconds(loop_time_ms));
        }
        cdbg << "after endless loop\n";



        //###
        // open up again
        GotoStartPose( hand, "\nReopening hand (using POSE controller)..." );
        //
        //###


        //###
        // stop sensor:
        dsa_updater.interrupt();

        ts->Close();
        cdbg << "Successfully disabled tactile sensor controller of SDH and closed connection\n";

        // Close the connection to the SDH and DSA
        hand.Close();
        cdbg << "Successfully disabled joint controllers of SDH and closed connection\n";


    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-contact-grasping main(): An exception was caught: " << e->what() << "\n";
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
