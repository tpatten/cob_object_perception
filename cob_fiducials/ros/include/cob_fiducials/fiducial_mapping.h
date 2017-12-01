#ifndef FIDUCIAL_MAPPING_H
#define FIDUCIAL_MAPPING_H

// Standard ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <cob_object_detection_msgs/DetectObjects.h>
#include <cob_object_detection_msgs/Detection.h>

#define TAG_0_ "tag_0"
#define TAG_1_ "tag_1"
#define TAG_2_ "tag_2"
#define TAG_3_ "tag_3"
#define NUM_TAGS_ 4

/** \brief Fiducial mapping namespace */
namespace fiducial_mapping
{

/** \brief Client class for Fiducial mapping */  
class FiducialMapping
{
public:
  
  /** \brief Struct to keep marker information */
  struct MarkerInfo
  {
    bool visible;                                   // Marker visibile in actual image?
    int marker_id;                                  // Marker ID
    int previous_marker_id;                         // Used for chaining markers
    geometry_msgs::Pose geometry_msg_to_previous;   // Position with respect to previous marker
    geometry_msgs::Pose geometry_msg_to_world;      // Position with respect to world's origin
    tf::StampedTransform tf_to_previous;            // TF with respect to previous marker
    tf::StampedTransform tf_to_world;               // TF with respect to world's origin
    geometry_msgs::Pose current_camera_pose;        // Position of camera with respect to the marker
    tf::Transform current_camera_tf;                // TF of camera with respect to the marker
  };

public:
  
  /** \brief Construct a client for EZN64 USB control*/  
  FiducialMapping(ros::NodeHandle *nh);
    
  ~FiducialMapping();

  /** \brief Callback function to handle image processing*/
  void imageCallback(const sensor_msgs::ImageConstPtr &original_image);

private:

  /** \brief Function to compute the camera pose*/
  void updateCameraPose();

  /** \brief Function to publish all known TFs*/
  void publishTfs(bool world_option);

  /** \brief Function to publish all known markers for visualization purposes*/
  void publishMarker(geometry_msgs::Pose markerPose, int MarkerID, int rank);

  /** \brief Compute TF from marker detector result*/
  //tf::Transform fiducial2Tf(const aruco::Marker &marker);
  tf::Transform fiducial2Tf(const cob_object_detection_msgs::Detection &fiducial);

  //Launch file params
  std::string calib_filename_;                    
  std::string space_type_;                        
  bool roi_allowed_;
  int  roi_x_;                                      
  int  roi_y_;                                      
  int  roi_w_;                                     
  int  roi_h_;
  
  /** \brief Service client to the /fiducials/get_fiducials service*/
  ros::ServiceClient get_fiducials_client_;

  /** \brief The list of fiducial detections*/
  std::vector<cob_object_detection_msgs::Detection> fiducial_detections_;
  
  /** \brief Publisher of visualization_msgs::Marker message to "aruco_markers" topic*/
  ros::Publisher marker_visualization_pub_;
  
  /** \brief Container holding MarkerInfo data about all detected markers */
  std::vector<MarkerInfo> markers_;
   
  /** \brief Actual TF of camera with respect to world's origin */
  tf::StampedTransform world_position_transform_;
  
  /** \brief Actual Pose of camera with respect to world's origin */
  geometry_msgs::Pose world_position_geometry_msg_;

  bool has_valid_pose_;
  int marker_counter_;
  int marker_counter_previous_;
  int closest_camera_index_;
  int lowest_marker_id_;
  bool first_marker_detected_;
  
  tf::TransformListener *listener_;
  tf::TransformBroadcaster broadcaster_;

  //Consts
  static const int CV_WAIT_KEY = 10;
  static const int CV_WINDOW_MARKER_LINE_WIDTH = 2;

  static const double WAIT_FOR_TRANSFORM_INTERVAL = 2.0;
  static const double BROADCAST_WAIT_INTERVAL = 0.0001;
  static const double INIT_MIN_SIZE_VALUE = 1000000;

  static const double RVIZ_MARKER_HEIGHT = 0.01;
  static const double RVIZ_MARKER_LIFETIME = 0.2;
  static const double RVIZ_MARKER_COLOR_R = 0.0;
  static const double RVIZ_MARKER_COLOR_G = 1.0;
  static const double RVIZ_MARKER_COLOR_B = 0.0;
  static const double RVIZ_MARKER_COLOR_A = 1.0;

  static const double THIS_IS_FIRST_MARKER = -2;

}; //FiducialMapping class

}  //fiducial_mapping namespace

#endif //FIDUCIAL_MAPPING_H
