#ifndef WORLD_SCOPED_MAPS_H
#define WORLD_SCOPED_MAPS_H
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/pose2d.h>

namespace tuw {
class WorldScopedMaps; /// Prototype
using WorldScopedMapsPtr      = std::shared_ptr< WorldScopedMaps > ;
using WorldScopedMapsConstPtr = std::shared_ptr< WorldScopedMaps const>;

/**
 * class to visualize information using OpenCV matrices
 **/
class WorldScopedMaps {
    cv::Matx33d Mw2m_;                ///< transformation world to map
    cv::Matx33d Mm2w_;                ///< transformation map to world
    int width_pixel_,  height_pixel_; ///< dimensions of the canvas in pixel
    double min_x_, max_x_, min_y_, max_y_, rotation_; ///< area and rotation of the visualized space
    double dx_, dy_; ///< dimension of the visualized space
    double ox_, oy_; ///< image offset
    double mx_, my_; ///< offset of the visualized space
    double sx_, sy_; ///< scale

    void init();     ///< initializes the transformation matrices
public:
    
    //special class member functions
    WorldScopedMaps ( );
    virtual ~WorldScopedMaps()                         = default;
    WorldScopedMaps           (const WorldScopedMaps&) = default;
    WorldScopedMaps& operator=(const WorldScopedMaps&) = default;
    WorldScopedMaps           (WorldScopedMaps&&)      = default;
    WorldScopedMaps& operator=(WorldScopedMaps&&)      = default;

    /**
     * used to initialize the figure
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param rotation rotation of the visualized spaces
     **/
    void init ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double rotation = 0  );
    
    /**
     * used to initialize the figure based on a ROS nav_msgs/MapMetaData
     * @param T nav_msgs/MapMetaData
     **/
    template <typename T> void init(const T &metadata){
        width_pixel_ = metadata.width,   height_pixel_ = metadata.height;        
        dx_ = metadata.resolution * (double) metadata.width;
        dy_ = metadata.resolution * (double) metadata.height;
        sx_ = 1.0/metadata.resolution;
        sy_ = 1.0/metadata.resolution;
        ox_ = 0.;
        oy_ = 0.;
        double roll = 0, pitch = 0, yaw = 0;
        QuaternionToEuler ( metadata.origin.orientation, roll, pitch, yaw );
        rotation_ = -yaw;
        rotation_ = 0;
        double ca = cos ( rotation_ ), sa = sin ( rotation_ );
        mx_ = metadata.origin.position.x;
        my_ = metadata.origin.position.y;
        cv::Matx<double, 3, 3 > Tw ( 1, 0, -mx_, 0, 1, -my_, 0, 0, 1 ); // translation
        cv::Matx<double, 3, 3 > Sc ( sx_, 0, 0, 0, sy_, 0, 0, 0, 1 );   // scaling
        cv::Matx<double, 3, 3 > R ( ca, -sa, 0, sa, ca, 0, 0, 0, 1 );   // rotation
        Mw2m_ = R * Sc * Tw;
        Mm2w_ = Mw2m_.inv();
        Point2D p = m2w(width_pixel_, height_pixel_);
        min_y_ = mx_;
        min_x_ = my_;
        max_x_ = p.x();
        max_y_ = p.y();  
    }

    /**
     *  @returns true if the figure is initialized
     **/
    bool initialized(); 
    /**
     * draws a line given in the visualization space (meter, ....) into a pixel map
     * @param map opencv matrix
     * @param p0 start point
     * @param p1 end point 
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    template <typename T>
    void line ( T &map, const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness=1, int lineType = CV_AA ) const {
        cv::line ( map, w2m ( p0 ).cv(), w2m ( p1 ).cv(), color, thickness, lineType );
    }
    /**
     * draws a circle given in the visualization space (meter, ....) into a pixel map
     * @param map opencv matrix
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    template <typename T>
    void circle ( T &map, const Point2D &p, int radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA ) const{
        cv::circle ( map, w2m ( p ).cv(), radius, color, thickness, lineType );
    }
    
    /**
     * return a copy of the value located at p in the visual space (meter, ....)
     * @param map opencv matrix
     * @param p location
     **/
    template <typename T>
    cv::Scalar_<T> get ( cv::Mat_<T> &map, const Point2D &p) const{
        return map.at(w2m ( p ).cv() );
    }

    /**
     * @return transformation matrix from the visualization space to image space (world -> map)
     **/
    const cv::Matx33d  &Mw2m () const;
    /**
     * @return transformation matrix from the image space to visualization space (map -> world)
     **/
    const cv::Matx33d  &Mm2w () const;

    /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @return point in image space (map [pixel])
     **/
    Point2D w2m ( const Point2D &src ) const ;
    /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param x x coordinate in visualization space (world) eg. [m]
     * @param y y coordinate in visualization space (world) eg. [m]
     * @return point in image space  eg. [pixel]
     **/
    Point2D w2m ( double x, double y ) const ;
    /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @param des point in image space (map [pixel])
     * @return reference to des
     **/
    Point2D &w2m ( const Point2D &src, Point2D &des ) const;
    /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @return point in visualization space (world)
     **/
    Point2D m2w ( const Point2D &src ) const ;
    /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param x x coordinate in image space  eg. [pixel]
     * @param y y coordinate in image space  eg. [pixel]
     * @return point in visualization space (world) eg. [m]
     **/
    Point2D m2w ( double x, double y ) const ;
    /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @param des  point in visualization space (world)
     * @return reference to des
     **/
    Point2D &m2w ( const Point2D &src, Point2D &des ) const;

    /**
     * @return canvas (image) width 
     **/
    int width () const ;
    /**
     * @return canvas (image) height 
     **/
    int height () const ;
    /**
     * @return computed x scale
     **/
    double scale_x () const ;
    /**
     * @return computed y scale
     **/
    double scale_y () const ;
    /**
     * @return minimal x of the visualized space
     **/
    double min_x () const ;
    /**
     * @return maximal x of the visualized space
     **/
    double max_x () const ;
    /**
     * @return minimal y of the visualized space
     **/
    double min_y () const ;
    /**
     * @return maximal y of the visualized space
     **/
    double max_y () const ;    
    /**
    * returns a distance measure scaled
    * @param v value to be scaledt
    * @return distance
    **/ 
    double scale_w2m (double v) const ;    
    /**
    * returns information about the maps metadata
    * @param format using printf format
    * @return string
    **/  
    std::string infoHeader() const;
};

}
#endif // WORLD_SCOPED_MAPS_H
