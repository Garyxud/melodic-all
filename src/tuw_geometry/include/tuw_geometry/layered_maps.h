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


#ifndef LAYERED_MAPS_H
#define LAYERED_MAPS_H 

#include <memory>
#include <tuw_geometry/world_scoped_maps.h>

namespace tuw {

class LayeredMaps;
using LayeredMapsPtr     = std::shared_ptr<       LayeredMaps >;
using LayeredMapsCostPtr = std::shared_ptr< const LayeredMaps >;

class LayeredMaps : public WorldScopedMaps {
public:
    enum Interpolation {
	SIMPLE = 0,
	BILINEAR = 1
    };
    
    //special class member functions
    LayeredMaps ( );
    virtual ~LayeredMaps()                     = default;
    LayeredMaps           (const LayeredMaps&) = default;
    LayeredMaps& operator=(const LayeredMaps&) = default;
    LayeredMaps           (LayeredMaps&&)      = default;
    LayeredMaps& operator=(LayeredMaps&&)      = default;
    
    void initLayers  ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double rotation = 0 );
    void clearLayers  ();
    void clearLayer   ( const size_t& _layer );
    void resizeLayers ( const size_t& _n );
    size_t sizeLayers () const;
    
    cv::Mat&       mapLayer( const size_t& _layer );
    const cv::Mat& mapLayer( const size_t& _layer ) const;
    double   getVal  ( const size_t& _layer, const tuw::Point2D& _worldPos, Interpolation interpolationType = BILINEAR) const;
    
    void computeDistanceField ( cv::Mat& _mDst, std::vector< Point2D >& _pSrc, const double& _radius, bool _flipDistance  = false, bool connectPoints = false ) const;
    void computeDistanceField ( cv::Mat& _mDst,                cv::Mat& _mSrc, const double& _radius, bool _flipDistance  = false ) const;
    
protected:
    virtual void initLayers();
    
    
private:
    std::vector<cv::Mat> mapLayers_;
};
}
#endif // LAYERED_MAPS_H
