#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include "threshold.cpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_listener/Num.h>

class ImageListener
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport     transporter_;
  image_transport::Subscriber         camera_sub_;
  image_transport::CameraSubscriber   caminfo_sub_;

  tf::StampedTransform                tf_transform_;
  tf::TransformListener               tf_listener_;

  ros::Publisher                      object_publisher_;  

public:
  /* CONSTRUCTOR */
  ImageListener() : transporter_(nh_)
  {
    camera_sub_ = transporter_.subscribe("/cameras/left_hand_camera/image", 1, &ImageListener::image_callback, this);
    //caminfo_sub_ = transporter_.subscribeCamera("/cameras/left_hand_camera/camera_info", 1, &ImageListener::info_callback, this);
    //object_publisher_ = nh_.advertise<std_msgs::String>("detected_objects", 1000);
    object_publisher_ = nh_.advertise<image_listener::Num>("detected_objects", 1000);
   

    std::cout << "Subscribed to left_hand_camera" << std::endl;
    ros::Duration(10).sleep();
  }

  void info_callback(const sensor_msgs::ImageConstPtr, const sensor_msgs::CameraInfoConstPtr msg){

    std::cout<< "info_callback" << std::endl;

  }



  /* /cameras/left_hand_camera/image callback */
  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {

    std::cout << "image_callback" << std::endl;

    /* PARAMETERS FROM CAMERA_INFO ROSTOPIC */
    double fx = 410.11718731515947;
    double fy = 410.51120062506004;
    double cx = 666.1770143433905;
    double cy = 447.51876027944303;
    

    cv_bridge::CvImagePtr camera_image;
    try
    {
      camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int width, height;
    std::vector<std::tuple<int,int,int> > image = get_image_cvMat(camera_image->image, width, height);
    std::vector<int> greyscale = convert_to_greyscale(image, width, height);

    cv::Mat output = threshold_image(greyscale, width, height);
    
    std::vector<uint16_t> components;
    Mat component_output = connected_components(output, components);
    //imwrite(std::string(argv[1]) + std::string(".connected_output.png"), component_output);

    Mat orientation = camera_image->image; // imread(argv[1], 1);
    for (uint16_t c: components) {
      Moments m = moments(mask_by_color(component_output, 7001*c), true);

      if (m.m00 < 1000) {
        continue;
      }
      int thickness = -1;
      int lineType = 8;

      double c_x = m.m10/m.m00;
      double c_y = m.m01/m.m00;

      circle( orientation,
         Point(c_x, c_y),
         10,
         Scalar( 0, 0, 255 ),
         thickness,
         lineType );

      double axis = m.mu11/(m.mu20 - m.mu02);
      //std::cout << "centroid: " << c_x << ", " << c_y << std::endl;
      //std::cout << "axis of orientation: " << axis << std::endl; 
    }

    cv::imshow("threshold_image", output);
    cv::imshow("component_output", component_output);
    cv::imshow("orientation", orientation);
    cv::waitKey(5);
  



    /* APPLYING HOMO_TRANSFOMS FROM /TF */

    //p_0 = H^0_1 *p_1     Point in frame 0 is the multiplication of the homogen_matrix times the point in frame1
    try{
      tf_listener_.lookupTransform("/left_hand_camera", "/base", ros::Time(0), tf_transform_);
      std::cout << "~~SUCCESS" << std::endl;
    }
      catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    /* Homoegenous transformation (camera -> base) parameters */
    //Individual parameters
    double x, y, z; 
    double qw, qx, qy, qz; 
    x = tf_transform_.getOrigin().x();
    y = tf_transform_.getOrigin().y();
    z = tf_transform_.getOrigin().z();
    qx = tf_transform_.getRotation().x();
    qy = tf_transform_.getRotation().y();
    qz = tf_transform_.getRotation().z();
    qw = tf_transform_.getRotation().w();

    // The 3x3 rotation matrix and 3x1 translation vector.
    tf::Matrix3x3 homo_rotation = tf_transform_.getBasis();
    tf::Vector3 homo_translation = tf_transform_.getOrigin();
    //std::cout << tf_transform_.getOrigin().x() <<" "<< tf_transform_.getOrigin().y() << " "<< tf_transform_.getOrigin().z() << std::endl;





    /* PUBLISH INFO ON OBJECT LOCATION AND COLOR TO ROSTOPIC /DETECTED_OBJECTS */
    // std_msgs::String message;
    // std::stringstream ss;
    // ss << "Hello";
    // message.data = ss.str();

    // object_publisher_.publish(message);

    image_listener::Num message;
    message.num = 5;

    object_publisher_.publish(message);


    std::cout << "End image_callback" << std::endl;   
  } 



};
  

int main(int argc, char **argv)
{
  std::cout << "Image Listener initiated" << std::endl;
  ros::init(argc, argv, "image_listener");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ImageListener il;

  ros::spin();

}
