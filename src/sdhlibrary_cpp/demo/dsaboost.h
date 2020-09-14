//======================================================================
/*!

    \file
     \section dhlibrary_cpp_sdh_dsaboost_h_general General file information

       \author   Dirk Osswald
       \date     2009-08-03

     \brief
       helper stuff for the DSA using boost



     \section dhlibrary_cpp_sdh_dsaboost_h_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection dhlibrary_cpp_sdh_dsaboost_h_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2011-02-07 09:15:17 +0100 (Mo, 07 Feb 2011) $
         \par SVN file revision:
           $Id: dsaboost.h 6424 2011-02-07 08:15:17Z Osswald2 $

     \subsection dhlibrary_cpp_sdh_dsaboost_h_changelog Changelog of this file:
         \include dsaboost.h.log

*/
//======================================================================
#ifndef DSABOOST_h
#define DSABOOST_h

#include <iostream>
#include <vector>
#include <assert.h>

// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdh/dsa.h"

#include "boost/thread.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"


//-----------------------------------------------------------------


NAMESPACE_SDH_START

/*!
 * \brief Class to create an updater thread for continuously updating tactile sensor data.
 *
 * Uses boost::thread from the boost library (http://www.boost.org)!
 */
class cDSAUpdater
{
private:
    boost::thread updater_thread;

    int   error_threshold;
    cDSA* ts;

    //! the actual run function of the updater thread
    void Updater();

public:
    //! default error threshold, see parameter \a error_threshold in CTOR
    static int const DEFAULT_ERROR_THRESHOLD = 16;

    /*!
     * CTOR: start an updater thread for the connected tactile sensor \a _ts
     *
     * Make remote DSA send frames. Create a thread that updates _ts->frame continuously
     *
     * \param _ts - ptr to already initialized cDSA tactile sensor object
     * \param _error_threshold - the number of errors that causes a "reset" of the connection to the remote DSA:
     * communication errors with the remote DSACON32m controllers are counted,
     * - every error increments the error level,
     * - every correct communication decrements the error level (down to 0)
     * if the error level rises above ERROR_THRESHOLD then
     * the connection is closed and the reopened. This is usefull to recover
     * from emergency stop since the hands loose power temporarily in that case.
     */
    cDSAUpdater( cDSA* _ts, int _error_threshold=DEFAULT_ERROR_THRESHOLD );

    //! interrupt the updater thread
    void interrupt()
    { updater_thread.interrupt(); };
};
//-----------------------------------------------------------------

/*!
 * \brief abstract base class for calculation of IsGrasped condition using tactile sensor information
 */
class cIsGraspedBase
{
protected:
    SDH::cDSA* ts;  //!< ptr to the cDSA tactile sensor object to use

public:
    cIsGraspedBase( SDH::cDSA* _ts )
    :
        // init member objects:
        ts( _ts )
    {
    }

    virtual bool IsGrasped(void) = 0;
};
//-----------------------------------------------------------------


/*!
 * \brief class for calculation of IsGrasped condition using an expected area of contact measured by the tactile sensors
 */
class cIsGraspedByArea : public cIsGraspedBase
{
private:
    double expected_area[ 6 ]; //!< array of expected contact area of sensor patch in mm*mm

    //! helper function, return full area of sensor patch \a m in mm*mm
    double FullArea( int m );

public:
    /*! set the is grasped condition
     *
     * @param eaf0p - expected area of finger 0 proximal for detecting grasp condition, value [0..1] with 0=no area, 1=full area
     * @param eaf0d - expected area of finger 0 distal for detecting grasp condition, value [0..1] with 0=no area, 1=full area
     * @param eaf1p - ...
     * @param eaf1d - ...
     * @param eaf2p - ...
     * @param eaf2d - ...
     */
    void SetCondition( double eaf0p, double eaf0d, double eaf1p, double eaf1d, double eaf2p, double eaf2d );

    //! overloaded variant which uses an array of doubles instead of 6 single double parameters
    void SetCondition( double* eafx );

    //! default constructor which initializes the internal date
    cIsGraspedByArea( SDH::cDSA* _ts );

    /*! Implementation of is grasped condition.
     *
     * \return true if the object is grasped according to the actual tactile sensor
     *         data in \a ts and the condition set with SetCondition()
     */
    virtual bool IsGrasped(void);
};

NAMESPACE_SDH_END

#endif

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
