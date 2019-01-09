#include "spoon_perception/food_tracker.h"

cv::Point2d mock_get_food_pixel_center(const cv::Mat &image)
{
  cv::Point2d pixel(400,400);
  return pixel;
}

FoodTracker::FoodTracker(std::string image_topic, std::string plane_frame) : it_(nh_), plane_frame_(plane_frame)
{  
  ROS_WARN("[food_tracker] subscribing to camera");
  sub_ = it_.subscribeCamera(image_topic, 1, &FoodTracker::imageCb, this);
  ROS_WARN("[food_tracker] advertizing food topic");
  food_loc_pub_ = nh_.advertise<geometry_msgs::PointStamped>("food",1);
}

void FoodTracker::StartTracking()
{
  active_ = true;
}

void FoodTracker::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (!pix_proj_)
  {
    ROS_WARN("[food_tracker] Initializing pixel projector");
    camera_frame_ = info_msg->header.frame_id;
    std::shared_ptr<PixelProjector> pix_proj(new PixelProjector(*info_msg, camera_frame_, plane_frame_)); 
    pix_proj_ = pix_proj;
  }

  // give some time for the newly initialized tf listener to hear the tf transform
  if (!active_)
  {
    ROS_WARN("[food_tracker] Waiting because not active yet...");
    return;
  }

  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = input_bridge->image;
  }
  catch (cv_bridge::Exception& ex){
    ROS_ERROR("[food_tracker] Failed to convert image");
    return;
  }
  cv::Point2d food_pixel = mock_get_food_pixel_center(image);

 
  ROS_WARN("[food_tracker] Actually using tf");
  geometry_msgs::PointStamped food_loc_msg = pix_proj_->PixelProjectedOnXYPlane(food_pixel, info_msg->header.stamp);

  food_loc_pub_.publish(food_loc_msg);
}

