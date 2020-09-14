#ifndef FIGURE_H
#define FIGURE_H
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_geometry/world_scoped_maps.h>

namespace tuw {
class Figure; /// Prototype
using FigurePtr      = std::shared_ptr< Figure > ;
using FigureConstPtr = std::shared_ptr< Figure const>;

/**
 * class to visualize information using OpenCV matrices
 **/
class Figure : public WorldScopedMaps {
    std::string title_;               /// window name
    std::string label_format_x_;      /// label format string
    std::string label_format_y_;      /// label format string
    cv::Mat view_;                    /// canvas
    cv::Mat background_;              /// background data, grid or image
    std::string background_filename_; /// if empty no file will be used
    double grid_scale_x_, grid_scale_y_; /// dimension of the drawn grid, if -1 no grid will be drawn 

    void drawBackground (); /// draws the background image
public:
    //special class member functions
    /**
     * constructor
     * @param title title of the displayed windows
     **/
    Figure ( const std::string &title );
    virtual ~Figure()                = default;
    Figure           (const Figure&) = default;
    Figure& operator=(const Figure&) = default;
    Figure           (Figure&&)      = default;
    Figure& operator=(Figure&&)      = default;

    /**
     * used to initialize the figure
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param rotation rotation of the visualized space
     * @param grid_scale_x dimension of the drawn grid, if -1 no grid will be drawn
     * @param grid_scale_y dimension of the drawn grid, if -1 no grid will be drawn
     * @param background_image file name of an image for the background, it can be empty as well
     **/
    virtual void init ( int width_pixel, int height_pixel, 
			double min_y, double max_y, 
			double min_x, double max_x, 
			double rotation = 0, 
			double grid_scale_x = -1, double grid_scale_y = -1, 
			const std::string &background_image = std::string() );
    
    /**
     * @return title of the window
     **/
    const std::string title() const;
    /**
     * can be used to define the x and y label format
     * @param label_x format of the x label, default "x=%f"
     * @param label_y format of the y label, default "y=%f"
     **/
    void setLabel ( const std::string &label_x = std::string("x=%f"), const std::string &label_y = std::string("y=%f"));

    /**
     * @return Name of the background file
     **/
    const std::string& backgroundFileName() const;
    /**
     * @return the matrix related to the foreground canvas
     **/
    const cv::Mat& view() const;
    /**
     * @return the matrix related to the foreground canvas
     **/
    cv::Mat& view();
    /**
     * @return the matrix related to the background canvas
     **/
    const cv::Mat& background() const;
    /**
     * @return the matrix related to the background canvas
     **/
    cv::Mat& background();
    /**
     * can be used to clone an image into the foreground independent to the image format (gray, color, ...)
     * @param view source image
     **/
    void setView ( const cv::Mat& view );
    /**
     * draws a line given in the visualization space (meter, ....) into the foreground image
     * @param p0 start point
     * @param p1 end point 
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void line ( const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    using WorldScopedMaps::line;
    /**
     * draws a circle given in the visualization space (meter, ....) into the foreground image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void circle ( const Point2D &p, int radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    using WorldScopedMaps::circle;
    /**
     * draws a symbol (dot) given in the visualization space (meter, ....) into a pixel map
     * @param view image
     * @param p location
     * @param color color --> @see opencv
     **/
    void symbol ( cv::Mat &view, const Point2D &p, const cv::Scalar &color );
    /**
     * draws a symbol (dot) given in the visualization space (meter, ....)  into the foreground image
     * @param p location
     * @param color color --> @see opencv
     **/
    void symbol ( const Point2D &p, const cv::Scalar &color );
    /**
     * draws a symbol (pose) given in the visualization space (meter, ....) into the image
     * @param view image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void symbol ( cv::Mat &view, const Pose2D &p, double radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a symbol (pose) given in the visualization space (meter, ....) into the foreground image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void symbol ( const Pose2D &p, double radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a text (pose) given in the visualization space (meter, ....) into the image
     * @param view image
     * @param text text
     * @param p location
     * @param fontFace fontFace --> @see opencv
     * @param fontScale fontScale --> @see opencv
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void putText ( cv::Mat &view, const std::string& text, const Point2D &p, int fontFace = cv::FONT_HERSHEY_PLAIN, double fontScale = 0.6, cv::Scalar color = cv::Scalar ( 128,0,0 ), int thickness=1, int lineType=CV_AA, bool bottomLeftOrigin=false );
    /**
     * draws a text (pose) given in the visualization space (meter, ....) into the foreground image
     * @param text text
     * @param p location
     * @param fontFace fontFace --> @see opencv
     * @param fontScale fontScale --> @see opencv
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void putText ( const std::string& text, const Point2D &p, int fontFace = cv::FONT_HERSHEY_PLAIN, double fontScale = 0.6, cv::Scalar color = cv::Scalar ( 128,0,0 ), int thickness=1, int lineType=CV_AA, bool bottomLeftOrigin=false );
   
    /**
     * overwrites the foreground with the background
     **/
    void clear ();
    
    /**
     * Appends the contents of a single channel CV_8U map into the foreground image as a (BGR) colour gradient on foreground pixels that were previously white (255,255,255).
     * @param _mat Map (CV_8U) to be appended to the foreground image
     * @param _colMin Minimum (BGR) colour vector value of the appended map (a value of 0 in the input map will be appended with the colour @ref _colMin)
     * @param _colMax Maximum (BGR) colour vector value of the appended map (a value of 255 in the input map will be appended with the colour @ref _colMax)
     * @param _truncateLayerVal Value to be truncated from the source map. For example, a value of 10 will not append any pixels with value greater than 245 ( = 255-10 )
     **/
    void appendToView(const cv::Mat& _mat, const cv::Scalar& _colMin, const cv::Scalar& _colMax, u_int8_t _truncateLayerVal = 0);
    
    /// color to use with the drawing functions
    static const cv::Scalar green;      
    static const cv::Scalar green_bright;
    static const cv::Scalar green_dark;
    static const cv::Scalar red;
    static const cv::Scalar blue;
    static const cv::Scalar blue_bright;
    static const cv::Scalar blue_dark;
    static const cv::Scalar orange;
    static const cv::Scalar yellow;
    static const cv::Scalar cyan;
    static const cv::Scalar magenta;
    static const cv::Scalar gray_bright;
    static const cv::Scalar gray;
    static const cv::Scalar black;
    static const cv::Scalar white;
    
    static const cv::Scalar niceBlue;
    static const cv::Scalar niceMustard;
    static const cv::Scalar niceMagenta;
    static const cv::Scalar niceGreenBlue;
    static const cv::Scalar niceRed;
    static const cv::Scalar niceRedDark;
    static const cv::Scalar niceGreen;
    static const cv::Scalar niceGrey;
    static const cv::Scalar niceGreyLight;
    static const cv::Scalar niceGreyPurple;
    static const cv::Scalar niceGreenWashed;
    static const cv::Scalar niceGreyDark;
    static const cv::Scalar niceLime;
    static const cv::Scalar niceDirtyPink;
    
};
}
#endif // FIGRUE_H
