/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#ifndef TUW_GRID_MAP_H
#define TUW_GRID_MAP_H

#include <memory>
#include <tuw_geometry/world_scoped_maps.h>

namespace tuw
{

template <class T>
class GridMap : public WorldScopedMaps
{
public:

    static const int8_t SPACE_NA       = -1;
    static const int8_t SPACE_FREE     =  0;
    static const int8_t SPACE_OCCUPIED = 100;
    //special class member functions
    GridMap ( )
    : threshold_occupyied_(SPACE_OCCUPIED)
    , threshold_unknown_(SPACE_OCCUPIED / 2)
    , threshold_free_(SPACE_OCCUPIED / 2)
    {
        bool read_only_ = true;
    }
    virtual ~GridMap()                 = default;
    GridMap ( const GridMap& ) = default;
    GridMap& operator= ( const GridMap& ) = default;
    GridMap ( GridMap&& )      = default;
    GridMap& operator= ( GridMap&& )      = default;

    template <typename MapMetaData, class ARRAY>
    void init ( const MapMetaData &metadata, ARRAY *data )
    {
        WorldScopedMaps::init ( metadata );
        data_ = cv::Mat_<T> ( height(), width(), data );
        read_only_ = false;
    }
    template <typename MapMetaData, class ARRAY>
    void init ( MapMetaData &metadata, ARRAY &data )
    {
        WorldScopedMaps::init ( metadata );
        data_ = cv::Mat_<T> ( height(), width(), ( T * ) &data[0] );
        read_only_ = false;
    }
    template <typename MapMetaData>
    void init ( const MapMetaData &metadata, const T &data, bool copy = false)
    {
        WorldScopedMaps::init ( metadata );
        if(copy){
            data_ = cv::Mat_<T> ( height(), width());
            std::copy(data.begin(), data.end(), data_.begin());
        }else {
            data_ = cv::Mat_<T> ( height(), width(), ( T * ) &data[0] );
        }
        read_only_ = true;
    }
    
    /**
     * draws a circle given in the visualization space (meter, ....) into a pixel map
     * @param map opencv matrix
     * @param p location [m]
     * @param radius radius [m].
     * @param value 
     * @param thickness line thickness --> @see opencv. Negative thickness means that a filled circle is to be drawn.
     * @param lineType line type --> @see opencv
     **/
    void circle (const Point2D &p, double radius, int8_t value, int thickness=1, int lineType = CV_AA ) {        
        cv::circle ( data_, w2m ( p ).cv(), scale_w2m(radius), cv::Scalar(value), thickness, lineType );
    }
    void erode (double distance, const GridMap &src = GridMap()) {  
        int erosion_size = scale_w2m(distance);
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );
    
        cv::Mat I(rows(), cols(), CV_8U, data_.ptr(0));
        cv::dilate( I, I, element);
    }
    T& operator () ( double x, double y )
    {
        return data_ ( w2m ( x, y ).cv() );
    }
    const T& operator () ( double x, double y ) const
    {
        return data_ ( w2m ( x, y ).cv() );
    }
    T& operator () ( const Point2D& _world_coordinates )
    {
        return data_ ( w2m ( _world_coordinates ).cv() );
    }
    const T& operator () ( const Point2D& _world_coordinates ) const
    {
        return data_ ( w2m ( _world_coordinates ).cv() );
    }
    T& get ( const Point2D& _world_coordinates )
    {
        return data_ ( w2m ( _world_coordinates ).cv() );
    }
    const T& get ( const Point2D& _world_coordinates ) const
    {
        return data_ ( w2m ( _world_coordinates ).cv() );
    }
    bool isOccupyied ( const Point2D& _world_coordinates ) const
    {
        Point2D p = w2m ( _world_coordinates );
        T v = data_ ( p.cv() );
        return  v >= threshold_occupyied_;
    }
    bool isFree ( const Point2D& _world_coordinates ) const
    {
        Point2D p = w2m ( _world_coordinates );
        T v = data_ ( p.cv() );
        return  (v > SPACE_NA) && (v < threshold_free_);
    }
    const cv::Mat_<T> &mat() const
    {
        return data_;
    }
    void setThresholdOccupied ( const T& threshold )
    {
        threshold_occupyied_ = threshold;
    }
    const T &getThresholdOccupied()
    {
        return threshold_occupyied_;
    }
    void setThresholdFree ( const T& threshold )
    {
        threshold_free_ = threshold;
    }
    const T &getThresholdFree()
    {
        return threshold_free_;
    }
    void setThresholdUnknown ( const T& threshold )
    {
        threshold_unknown_ = threshold;
    }
    const T &getThresholdUnknown()
    {
        return threshold_unknown_;
    }
    int rows() const {
        return data_.rows;
    }
    int cols() const {
        return data_.cols;
    }
    T& grid ( int row, int col )
    {
        return data_ ( row , col );
    }
    const T& grid ( int row, int col ) const
    {
        return data_ ( row , col );
    }
private:
    bool read_only_;
    cv::Mat_<T> data_;
    T threshold_occupyied_;
    T threshold_unknown_;
    T threshold_free_;
};
}
#endif // TUW_GRID_MAP_H
