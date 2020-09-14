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

#include <tuw_geometry/layered_figure.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>

using namespace cv;
using namespace std;
using namespace tuw;



LayeredFigure::LayeredFigure( const std::string &_title ) : Figure(_title), view_idx_(0), sizeLayers_(0) {
    namedWindow( title(), CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
    std::string namet1 = "MapLayer";
    
//     cv::setMouseCallback ( title(), GlobalInterface::onMouseMap, this );
    createTrackbar( namet1, title(), &view_idx_, layeredMaps.sizeLayers()+1, LayeredFigure::callbackTrkbar1 );
}

void LayeredFigure::callbackTrkbar1( int flags, void* param ) {
    
}

void LayeredFigure::init ( int width_pixel, int height_pixel, 
			   double min_y, double max_y, 
			   double min_x, double max_x, double rotation, 
			   double grid_scale_x, double grid_scale_y, const string& background_image ) {
    tuw::Figure::init ( width_pixel, height_pixel, min_y, max_y, min_x, max_x, rotation, grid_scale_x, grid_scale_y, background_image );
    layeredMaps.initLayers(width_pixel, height_pixel, min_y, max_y, min_x, max_x, rotation);
    
    if(sizeLayers_ != layeredMaps.sizeLayers()+1){
	sizeLayers_ = layeredMaps.sizeLayers()+1;
	destroyWindow(title());
	namedWindow( title(), CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
	std::string namet1 = "MapLayer";
	createTrackbar( namet1, title(), &view_idx_, sizeLayers_, LayeredFigure::callbackTrkbar1 );
    }
}

void LayeredFigure::outputPlot() {
    if(view_idx_>0) {
// 	setView(layeredMaps.mapLayer(view_idx_-1));
	appendToView(layeredMaps.mapLayer(view_idx_-1), black, white, 0);
    }
    imshow ( title(), view()                            );
//     switch (view_idx_) {
// 	case  (0): imshow ( title(), view()                            ); break;
// 	default  : imshow ( title(), layeredMaps.mapLayer(view_idx_-1) ); break;
//     }
    waitKey( 10 );
    clear();
}

