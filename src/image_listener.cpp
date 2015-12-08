#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include "threshold.cpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>



class ImageListener
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport transporter_;
  image_transport::Subscriber camera_sub_;

  tf::StampedTransform tf_target_;
  tf::StampedTransform tf_transform_;
  tf::TransformListener tf_listener_;


public:
  /* CONSTRUCTOR */
  ImageListener() : transporter_(nh_)
  {
    camera_sub_ = transporter_.subscribe("/cameras/left_hand_camera/image", 1, &ImageListener::image_callback, this);
    std::cout << "Subscribed to images" << std::endl;

    //tf::TransformListener tf_listener;
    ros::Duration(10).sleep();
  }


  /* /image callback */
  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {

    /* PARAMETERS FROM CAMERA_INFO ROSTOPIC */
    double fx = 410.11718731515947;
    double fy = 410.51120062506004;
    double cx = 666.1770143433905;
    double cy = 447.51876027944303;
    
    std::cout << "imageCallback" << std::endl;

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
  



    //tf::StampedTransform transform;

    try{
      tf_listener_.lookupTransform("/left_hand_camera", "/base", ros::Time(0), tf_transform_);
      std::cout << "~~SUCCESS" << std::endl;
    }
      catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    std::cout << tf_transform_.getOrigin().y() << std::endl;
    

    std::cout << "End imageCallback" << std::endl;   
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
