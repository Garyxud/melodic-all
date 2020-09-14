//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_util_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of auxilliary utility functions for SDHLibrary-CPP.

  \section sdhlibrary_cpp_util_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_util_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2014-02-20 16:00:30 +0100 (Thu, 20 Feb 2014) $
      \par SVN file revision:
        $Id: util.cpp 11372 2014-02-20 15:00:30Z Osswald2 $

  \subsection sdhlibrary_cpp_util_cpp_changelog Changelog of this file:
      \include util.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <assert.h>

#include <iostream>
#if SDH_USE_VCC
# include <windows.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "util.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

bool InIndex( int v, int max )
{
    return 0 <= v  &&  v < max;
}
//-----------------------------------------------------------------

bool InRange( double v, double min, double max )
{
    return min <= v  &&  v <= max;
}
//-----------------------------------------------------------------

bool InRange( int n, double const* v, double const* min, double const* max )
{
    for ( int i=0; i < n; i++ )
    {
        if (! InRange( v[i], min[i], max[i] ))
            return false;
    }
    return true;
}
//-----------------------------------------------------------------

double ToRange( double v, double min, double max )
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}
//-----------------------------------------------------------------

void ToRange( int n, double* v, double const* min, double const* max )
{
    for ( int i=0; i < n; i++ )
    {
        v[i] = ToRange( v[i], min[i], max[i] );
    }
}
//-----------------------------------------------------------------

void ToRange( std::vector<double>& v, std::vector<double> const& min, std::vector<double> const& max )
{
    ToRange( int(v.size()), &(v[0]), &(min[0]), &(max[0]) );
}
//-----------------------------------------------------------------

void ToRange( cSimpleVector& v, std::vector<double> const& min, std::vector<double> const& max )
{
    ToRange( cSimpleVector::eNUMBER_OF_ELEMENTS, &(v[0]), &(min[0]), &(max[0]) );
}
//-----------------------------------------------------------------

double Approx( double a, double b, double eps )
{
    return fabs( a - b ) < eps;
}
//-----------------------------------------------------------------

bool Approx( int n, double* a, double* b, double* eps )
{
    for ( int i=0; i < n; i++ )
    {
        if (! Approx( a[i], b[i], eps[i] ))
            return false;
    }
    return true;
}
//-----------------------------------------------------------------

double DegToRad( double d )
{
    return d*M_PI/180.0;
}
//-----------------------------------------------------------------

double RadToDeg( double r )
{
    return r*180.0/M_PI;
}
//-----------------------------------------------------------------

void SleepSec( double t )
{
#if SDH_USE_VCC
    ::Sleep( static_cast<int>(1000.0*t) );
#else
    timespec sleeptime;
    sleeptime.tv_sec  = (time_t) floor( t );
    sleeptime.tv_nsec = (long)   ((t - floor( t )) * 1E9);

    ////std::cout << "Sleeping for " << sleeptime.tv_sec << "s and " << sleeptime.tv_nsec << "ns\n";

    nanosleep( &sleeptime, NULL );
#endif
}
//-----------------------------------------------------------------


/*!
    return a vector of integer numbers for a release string \a rev
    \param rev release string like "0.0.1.11-a"
    \return a vector of integer numbers like [0,0,1,11,1]
*/
std::vector<int> NumerifyRelease( char const* rev )
{
    std::vector<int> result;
    char const* rev_orig = rev;

    while ( rev != NULL && *rev != '\0' )
    {
        int r;
        int chars_scanned = 0;
        int rc;
        rc = sscanf( rev, "%d%n", &r, &chars_scanned );

        //printf ( "Scanning rev='%s' r=%d chars_scanned=%d rc=%d\n", rev, r, chars_scanned, rc );

        if ( chars_scanned > 0 )
        {
            if ( rc==1 )
                result.push_back(r);
            rev += chars_scanned;
        }
        else if ( chars_scanned == 0  &&  'a' <= *rev && *rev <= 'z' )
        {
            result.push_back( *rev - 'a' + 1 );
            rev++;
        }
        else if ( chars_scanned == 0  &&  'A' <= *rev && *rev <= 'Z' )
        {
            result.push_back( *rev - 'A' + 1 );
            rev++;
        }
        else if ( chars_scanned == 0  &&  ( *rev == '.' || *rev == '-' ) )
        {
            rev++;
        }
        else
        {
            std::cerr << "NumerifyRelease( " << rev_orig << " ) Could not be handled!\n";
            std::cerr.flush();
            assert( "invalid rev string!" == NULL );
        }
    }
    return result;
}
//-----------------------------------------------------------------


/*!
    compare release strings \a rev1 and \a rev2.
    \return -1,0, or 1 if \a rev1 is older, equal or newer than \a rev2

    \param rev1 - a release string like "0.0.1.5" or "0.0.1.11-a"
    \param rev2 - another release string

    Example:
    - CompareReleases( "0.0.1.5", "0.0.1.5" )   ==>  0
    - CompareReleases( "0.0.1.5", "0.0.1.4" )   ==>  1
    - CompareReleases( "0.0.1.5", "0.0.2.1" )   ==> -1
    - CompareReleases( "0.0.1.5", "0.0.1.5-a" ) ==> -1
*/
int CompareReleases( char const* rev1, char const* rev2 )
{
    assert( rev1 != NULL );
    assert( rev2 != NULL );

    std::vector<int> nums1 = NumerifyRelease( rev1);
    std::vector<int> nums2 = NumerifyRelease( rev2 );

    std::vector<int>::const_iterator n1 = nums1.begin();
    std::vector<int>::const_iterator n2 = nums2.begin();

    for ( ; n1 != nums1.end() && n2 != nums2.end();  n1++, n2++ )
    {
        if ( *n1 < *n2 )
            return -1;
        else if ( *n1 > *n2 )
            return 1;
    }
    // elements existing in both lists are all the same

    if ( nums1.size() < nums2.size() )
        return -1;
    if ( nums1.size() > nums2.size() )
        return 1;
    return 0;
}
//-----------------------------------------------------------------


NAMESPACE_SDH_END


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
