#include <ros/ros.h>
#include <interactive_robot/GetHand3D.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Hand3DService {
public:
    Hand3DService(ros::NodeHandle& nh) : nh_(nh) {
        service_ = nh_.advertiseService("/get_hand_position_3d", &Hand3DService::callback, this);
        cloud_sub_ = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Hand3DService::cloudCallback, this);
        caminfo_sub_ = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &Hand3DService::cameraInfoCallback, this);
        ROS_INFO("Service [/get_hand_position_3d] ready.");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Subscriber cloud_sub_, caminfo_sub_;
    sensor_msgs::PointCloud2ConstPtr latest_cloud_;
    sensor_msgs::CameraInfoConstPtr latest_camera_info_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        latest_cloud_ = msg;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        latest_camera_info_ = msg;
    }

    bool callback(interactive_robot::GetHand3D::Request &req,
                  interactive_robot::GetHand3D::Response &res) {
        if (!latest_cloud_ || !latest_camera_info_) {
            ROS_WARN("No camera info or point cloud available.");
            return false;
        }

        if (req.detections.detections.empty()) {
            ROS_WARN("No detections received.");
            return false;
        }

        const auto& detection = req.detections.detections[0];
        geometry_msgs::PointStamped result;

        if (!projectCloud(detection, *latest_cloud_, *latest_camera_info_, result)) {
            ROS_WARN("3D projection failed.");
            return false;
        }

        res.point = result;
        return true;
    }

    bool projectCloud(const vision_msgs::Detection2D& detection,
                      const sensor_msgs::PointCloud2& cloud_msg,
                      const sensor_msgs::CameraInfo& cam_info,
                      geometry_msgs::PointStamped& output) {
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(boost::make_shared<sensor_msgs::CameraInfo const>(cam_info));

        int u = static_cast<int>(detection.bbox.center.x);
        int v = static_cast<int>(detection.bbox.center.y);

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(cloud_msg, pcl_cloud);

        if (u < 0 || v < 0 || u >= pcl_cloud.width || v >= pcl_cloud.height) {
            ROS_WARN("Pixel out of cloud bounds.");
            return false;
        }

        int index = v * pcl_cloud.width + u;
        const pcl::PointXYZ& pt = pcl_cloud.points[index];

        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            ROS_WARN("Invalid point at (u,v) = (%d, %d)", u, v);
            return false;
        }

        output.header = cloud_msg.header;
        output.point.x = pt.x;
        output.point.y = pt.y;
        output.point.z = pt.z;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_hand_position_3d_server");
    ros::NodeHandle nh;
    Hand3DService server(nh);
    ros::spin();
    return 0;
}
