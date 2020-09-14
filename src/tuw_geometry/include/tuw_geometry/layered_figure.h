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

#ifndef LAYERED_FIGURE_H
#define LAYERED_FIGURE_H

#include <memory>

#include <tuw_geometry/figure.h>
#include <tuw_geometry/layered_maps.h>

#include <opencv2/opencv.hpp>

namespace tuw {

class LayeredFigure;
using LayeredFigurePtr      = std::shared_ptr<       LayeredFigure >;
using LayeredFigureConstPtr = std::shared_ptr< const LayeredFigure >;
class LayeredFigure : public Figure {
public:
    //special class member functions
    LayeredFigure ( const std::string &title );
    virtual ~LayeredFigure()                       = default;
    LayeredFigure           (const LayeredFigure&) = default;
    LayeredFigure& operator=(const LayeredFigure&) = default;
    LayeredFigure           (LayeredFigure&&)      = default;
    LayeredFigure& operator=(LayeredFigure&&)      = default;
    
    void outputPlot();
    void init( int width_pixel, int height_pixel, 
	       double min_y, double max_y, 
	       double min_x, double max_x, 
	       double rotation = 0, 
	       double grid_scale_x = -1, double grid_scale_y = -1, 
	       const std::string &background_image = std::string() ) override;
    
    LayeredMaps layeredMaps;
    
protected:
    int view_idx_;
    size_t sizeLayers_;
    static void callbackTrkbar1 ( int flags, void* param );
};


}
#endif // LAYERED_FIGURE_H