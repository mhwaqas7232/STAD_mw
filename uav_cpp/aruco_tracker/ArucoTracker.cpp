#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <string>
#include <iostream>

class ArucoTrackerNode : public rclcpp::Node
{
public:
  ArucoTrackerNode() : Node("aruco_tracker_node")
  {
    // Subscriber for the image topic
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_topic", 10, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("/coordinates_topic", 1);
  }


 void publish_message(const std::string &message_content)
  {
    auto message = std_msgs::msg::String();
    message.data = message_content;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }


std::string format_tvec_to_string(double x, double y, double z)
{
    std::ostringstream oss;
    oss << "[" << x << " " << y << " " << z << "]";
    return oss.str();
}


private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {

RCLCPP_ERROR(this->get_logger(), "img received");


    // Convert the ROS Image message to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // Define ArUco dictionary and detector parameters
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

    std::vector<int> markerIds;  // Variable to store the marker IDs
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::ArucoDetector detector(dictionary, parameters);

    // Detect ArUco markers
    detector.detectMarkers(gray, markerCorners, markerIds, rejectedCandidates);    // Print detected markers' details
       float marker_length = 0.1f;

    // Camera matrix based on image size
    int height = gray.rows;
    int width = gray.cols;
    cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) << width, 0, width / 2,
                                                    0, height, height / 2,
                                                    0, 0, 1);

    // Iterate over detected markers
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      // Prepare the object points (marker corners in 3D space)
      std::vector<cv::Point3f> object_points = {
        {-marker_length / 2, -marker_length / 2, 0},  // Bottom left
        { marker_length / 2, -marker_length / 2, 0},  // Bottom right
        { marker_length / 2,  marker_length / 2, 0},  // Top right
        {-marker_length / 2,  marker_length / 2, 0}   // Top left
      };

      // Get the corners of the detected marker
      std::vector<cv::Point2f> marker_corners = markerCorners[i];

      // Ensure marker_corners is a 2D array of type float32
      std::vector<cv::Point2f> marker_corners_float(marker_corners.begin(), marker_corners.end());

      // SolvePnP to get the rotation and translation vectors
      cv::Mat rvec, tvec;
      bool success = cv::solvePnP(object_points, marker_corners_float, camera_matrix, cv::Mat(), rvec, tvec);

      if (success)
      {
        // Print the translation vector (X, Y, Z)
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d - Translation Vector (X, Y, Z): [%.2f, %.2f, %.2f]",
                    markerIds[i], tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        std::string formatted_tvec = format_tvec_to_string(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

	this->publish_message(formatted_tvec);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Could not solvePnP for Marker ID: %d", markerIds[i]);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
