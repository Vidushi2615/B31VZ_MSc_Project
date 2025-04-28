#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <interactive_robot/ObjectLocalisationAction.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include "interactive_robot/tracker_with_cloud_node.h"

class ObjectLocalisationAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<interactive_robot::ObjectLocalisationAction> as_;
    std::string action_name_;
    interactive_robot::ObjectLocalisationFeedback feedback_;
    interactive_robot::ObjectLocalisationResult result_;

    ros::Subscriber detection_sub_, cloud_sub_, caminfo_sub_;
    std::string target_object_;
    bool goal_active_;

    sensor_msgs::PointCloud2ConstPtr latest_cloud_;
    sensor_msgs::CameraInfoConstPtr latest_camera_info_;

public:
    ObjectLocalisationAction(std::string name)
        : as_(nh_, name, boost::bind(&ObjectLocalisationAction::executeCB, this, _1), false),
          action_name_(name), goal_active_(false)
    {
        detection_sub_ = nh_.subscribe("/detected_objects_2d", 10, &ObjectLocalisationAction::detectionCallback, this);
        cloud_sub_ = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &ObjectLocalisationAction::cloudCallback, this);
        caminfo_sub_ = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &ObjectLocalisationAction::cameraInfoCallback, this);
        as_.start();
        ROS_INFO("Object Localisation Action Server ready.");
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        latest_camera_info_ = msg;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        latest_cloud_ = msg;
    }

    void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
        if (!goal_active_ || target_object_.empty() || !latest_cloud_ || !latest_camera_info_) return;

        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            std::string class_name = det.header.frame_id;
            if (class_name != target_object_) continue;

            geometry_msgs::PointStamped result;
            bool success = projectCloud(det, *latest_cloud_, *latest_camera_info_, result);

            if (success) {
                result_.position = result;
                as_.setSucceeded(result_, "3D position obtained.");
                goal_active_ = false;
                ROS_INFO_STREAM("Successfully localised object: " << class_name);
            } else {
                feedback_.status = "3D projection failed";
                as_.publishFeedback(feedback_);
                ROS_WARN("3D projection failed for: %s", class_name.c_str());
            }
        }
    }

    void executeCB(const interactive_robot::ObjectLocalisationGoalConstPtr &goal) {
        target_object_ = goal->object_name;
        goal_active_ = true;
        feedback_.status = "Waiting for detection of " + target_object_;
        ROS_INFO_STREAM("Received localisation request for: " << target_object_);

        ros::Rate rate(10);
        int timeout = 100; // 10 seconds
        int count = 0;

        while (ros::ok() && goal_active_ && count++ < timeout) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                as_.setPreempted();
                goal_active_ = false;
                return;
            }
            as_.publishFeedback(feedback_);
            rate.sleep();
        }

        if (goal_active_) {
            as_.setAborted(result_, "Timeout while waiting for detection.");
            goal_active_ = false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_localisation_server");
    ObjectLocalisationAction server("object_localisation");
    ros::spin();
    return 0;
}
