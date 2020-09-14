#include <cfloat>
#include <tgmath.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "tuw_geometry/figure.h"


using namespace tuw;

const cv::Scalar Figure::green ( 0, 255,   0 );
const cv::Scalar Figure::green_bright ( 51, 255,  51 );
const cv::Scalar Figure::green_dark ( 0, 102,   0 );
const cv::Scalar Figure::red ( 0,   0, 255 );
const cv::Scalar Figure::blue ( 255,   0,   0 );
const cv::Scalar Figure::blue_bright ( 255,  51,  51 );
const cv::Scalar Figure::blue_dark ( 139,   0,   0 );
const cv::Scalar Figure::orange ( 0, 128, 255 );
const cv::Scalar Figure::yellow ( 0, 255, 255 );
const cv::Scalar Figure::cyan ( 255, 255,   0 );
const cv::Scalar Figure::magenta ( 255,   0, 255 );
const cv::Scalar Figure::gray_bright ( 224, 224, 224 );
const cv::Scalar Figure::gray ( 128, 128, 128 );
const cv::Scalar Figure::black ( 0,   0,   0 );
const cv::Scalar Figure::white ( 255, 255, 255 );

const cv::Scalar Figure::niceBlue        (155, 100,   59);
const cv::Scalar Figure::niceMustard     ( 28, 174,  184);
const cv::Scalar Figure::niceMagenta     (147,  32,  220);
const cv::Scalar Figure::niceGreenBlue   (181, 196,   36);
const cv::Scalar Figure::niceRed         ( 82,  77,  204);
const cv::Scalar Figure::niceRedDark     ( 52,  48,  158);
const cv::Scalar Figure::niceGreen       ( 55, 142,   84);
const cv::Scalar Figure::niceGrey        (132, 109,  106);
const cv::Scalar Figure::niceGreyLight   (179, 165,  153);
const cv::Scalar Figure::niceGreyPurple  (155, 135,  149);
const cv::Scalar Figure::niceGreenWashed (151, 166,  125);
const cv::Scalar Figure::niceGreyDark    ( 85,  79,   74);
const cv::Scalar Figure::niceLime        ( 23, 176,  154);
const cv::Scalar Figure::niceDirtyPink   ( 78,   0,  120);

Figure::Figure ( const std::string &title )
    : title_ ( title )
    , label_format_x_ ( "x=%f" )
    , label_format_y_ ( "y=%f" ) {

}

void Figure::setLabel ( const std::string &label_format_x, const std::string &label_format_y ) {
    label_format_x_ = label_format_x, label_format_y_ = label_format_y;
}
const std::string& Figure::backgroundFileName() const {
    return background_filename_;
}
const cv::Mat& Figure::view() const {
    return view_;
}
cv::Mat& Figure::view() {
    return view_;
}
const cv::Mat& Figure::background() const {
    return background_;
}
cv::Mat& Figure::background() {
    return background_;
}
void Figure::setView ( const cv::Mat& view ) {
    if ( view.empty() ) return;
    view_.create ( view.cols, view.rows, CV_8UC3 );
    int depth = view.depth(), cn = view.channels();
    int type = CV_MAKETYPE ( depth, cn );
    if ( type == CV_8UC3 ) {
        view.copyTo ( view_ );
    } else if ( ( view.channels() == 1 ) && ( view.depth() == CV_8U ) ) {
        cv::cvtColor ( view, view_, CV_GRAY2BGR );
    }
}

void Figure::init ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double rotation, double grid_scale_y, double grid_scale_x, const std::string &background_image ) {
    WorldScopedMaps::init(width_pixel, height_pixel, min_x, max_x, min_y, max_y, rotation);
    grid_scale_x_ = grid_scale_x, grid_scale_y_ = grid_scale_y;
    background_filename_ = background_image;
    drawBackground();
    clear();
}

void Figure::drawBackground() {

    background_.create ( height(), width(), CV_8UC3 );
    if ( !background_filename_.empty() ) {
        cv::Mat image = cv::imread ( background_filename_, CV_LOAD_IMAGE_COLOR );
        cv::resize ( image, background_, cv::Size ( background_.cols, background_.rows ), cv::INTER_AREA );
    } else {
        background_.setTo ( 0xFF );
    }
    if ( ( grid_scale_y_ > 0 ) && ( grid_scale_x_ > 0 ) ) {
        char txt[0xFF];
        Point2D p0, p1, pm0, pm1;
        double min_y = round ( WorldScopedMaps::min_y() /grid_scale_y_ ) *  grid_scale_y_;
        double max_y = round ( WorldScopedMaps::max_y() /grid_scale_y_ ) *  grid_scale_y_;
        for ( p0.y() = min_y; p0.y() <= max_y; p0.y() +=grid_scale_y_ ) {
            p0.x() = round ( max_x() /grid_scale_x_ )  *  grid_scale_x_;
            p1.y() = p0.y();
            p1.x() = round ( min_x() /grid_scale_x_ )  *  grid_scale_x_;
            if ( fabs ( p0.y() ) > FLT_MIN ) WorldScopedMaps::line ( background_, p0, p1, gray_bright, 1, 8 );
            else WorldScopedMaps::line ( background_, p0, p1, gray, 1, 8 );
        }
        p0.set ( 0, max_y - grid_scale_y_/2.0 );
        sprintf ( txt, label_format_y_.c_str(), max_y );
        cv::putText ( background_, txt, w2m ( p0 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, white, 3, CV_AA );
        cv::putText ( background_, txt, w2m ( p0 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, gray, 1, CV_AA );
        p1.set ( 0, min_y + grid_scale_y_/2.0 );
        sprintf ( txt, label_format_y_.c_str(), min_y );
        cv::putText ( background_, txt, w2m ( p1 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, white, 3, CV_AA );
        cv::putText ( background_, txt, w2m ( p1 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, gray, 1, CV_AA );

        double min_x = round ( WorldScopedMaps::min_x() /grid_scale_x_ ) *  grid_scale_x_;
        double max_x = round ( WorldScopedMaps::max_x() /grid_scale_x_ ) *  grid_scale_x_;
        for ( p0.x() = min_x; p0.x() <= max_x; p0.x() +=grid_scale_x_ ) {
            p0.y() = round ( WorldScopedMaps::max_y() /grid_scale_y_ )  *  grid_scale_y_;
            p1.x() = p0.x();
            p1.y() = round ( WorldScopedMaps::min_y() /grid_scale_y_ )  *  grid_scale_y_;
            pm0 = w2m ( p0 );
            pm1 = w2m ( p1 );
            if ( fabs ( p0.x() ) > FLT_MIN ) WorldScopedMaps::line ( background_, p0, p1, gray_bright, 1, 8 );
            else WorldScopedMaps::line ( background_, p0, p1, gray, 1, 8 );
        }
        p0.set ( max_x - grid_scale_x_/2.0, 0 );
        sprintf ( txt, label_format_x_.c_str(), max_x );
        cv::putText ( background_, txt, w2m ( p0 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, white, 3, CV_AA );
        cv::putText ( background_, txt, w2m ( p0 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, gray, 1, CV_AA );
        p1.set ( min_x + grid_scale_x_/2.0, 0 );
        sprintf ( txt, label_format_x_.c_str(), min_x );
        cv::putText ( background_, txt, w2m ( p1 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, white, 3, CV_AA );
        cv::putText ( background_, txt, w2m ( p1 ).cv(), cv::FONT_HERSHEY_PLAIN, 0.6, gray, 1, CV_AA );
    }
}
void Figure::clear () {
    view_.create ( background_.cols, background_.rows, CV_8UC3 );
    background_.copyTo ( view_ );
}

void Figure::line ( const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness, int lineType ) {
    WorldScopedMaps::line ( view_, p0, p1, color, thickness, lineType );
}

void Figure::symbol ( const Point2D &p, const cv::Scalar &color ) {
    symbol ( view_, p, color );
}
void Figure::symbol ( cv::Mat &view,  const Point2D &p, const cv::Scalar &color ) {
    cv::Point pi = w2m ( p ).cv();
    if ( ( pi.x < 0 ) || ( pi.x >= view.cols ) || ( pi.y < 0 ) || ( pi.y >= view.rows ) ) return;
    cv::Vec3b &pixel = view.at<cv::Vec3b> ( pi );
    pixel[0] = color[0], pixel[1] = color[1], pixel[2] = color[2];
}
const std::string Figure::title() const {
    return title_;
}

void Figure::circle ( const Point2D &p, int radius, const cv::Scalar &color, int thickness, int lineType ) {
    WorldScopedMaps::circle ( view_, p, radius, color, thickness, lineType );
}

void Figure::symbol ( const Pose2D &p, double radius, const cv::Scalar &color, int thickness, int lineType ) {
    symbol ( view_, p, radius, color, thickness, lineType );
}

void Figure::symbol ( cv::Mat &view, const Pose2D &p, double radius, const cv::Scalar &color, int thickness, int lineType ) {
    circle ( p.position(), radius * (scale_x() + scale_y())/2., color, thickness, lineType );
    line ( p.position(), p.point_ahead ( radius ), color, thickness, lineType );

}
void Figure::putText ( const std::string& text, const Point2D &p, int fontFace, double fontScale, cv::Scalar color, int thickness, int lineType, bool bottomLeftOrigin ) {
    putText ( view_, text, p, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin );
}
void Figure::putText ( cv::Mat &view, const std::string& text, const Point2D &p, int fontFace, double fontScale, cv::Scalar color, int thickness, int lineType, bool bottomLeftOrigin ) {
    cv::putText ( view, text, w2m ( p ).cv(), fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin );
}

void Figure::appendToView ( const cv::Mat& _mat, const cv::Scalar& _colMin, const cv::Scalar& _colMax, u_int8_t _truncateLayerVal ) {
    if( view().empty() || _mat.empty() || !initialized() ) { return; }
    
    CV_Assert ( _mat.depth() == CV_8U );
    int channels = _mat.channels();
    int nRows    = _mat.rows;
    int nCols    = _mat.cols * channels;

    uint8_t const* p_s; cv::Vec3b* p_d;
    for(int i = 0; i < nRows; ++i) {
        for (p_s = _mat.ptr<const uint8_t>(i), p_d =view().ptr<cv::Vec3b>(i); p_s != _mat.ptr<uint8_t>(i+1); p_d++, p_s++){
	    if ( (*p_d == cv::Vec3b(255,255,255) ) && (*p_s < 255 - _truncateLayerVal ) ) {  
		double scale = *p_s / (255. - (double)_truncateLayerVal);
		*p_d =  cv::Vec3b( _colMin[0] + scale * (_colMax[0] - _colMin[0]),
				   _colMin[1] + scale * (_colMax[1] - _colMin[1]),
				   _colMin[2] + scale * (_colMax[2] - _colMin[2]) );
	    }
	}
    }
}
