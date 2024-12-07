#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>

class Vector2D {
public:
    double x, y;
    
    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    
    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }
    
    double norm() const {
        return std::sqrt(x*x + y*y);
    }
};

class StanleyController {
private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Publisher drive_pub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher target_point_pub_;
    
    std::vector<Vector2D> trajectory_;
    geometry_msgs::Pose current_pose_;
    double yaw_ = 0.0;
    double current_speed_ = 0.0;
    bool pose_received_ = false;
    bool trajectory_received_ = false;
    
    const double k_ = 0.4 ;  // Stanley gain
    const double wheelbase_ = 0.3302;
    const double max_steer_ = 0.5;
    const double target_speed_ = 1;

public:
    StanleyController() : rate_(10) {
        drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
        trajectory_sub_ = nh_.subscribe("/global_path/optimal_trajectory", 1, 
            &StanleyController::trajectoryCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, 
            &StanleyController::odomCallback, this);
        
        target_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_point", 1);
    }
    

    void trajectoryCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        trajectory_.clear();
        for (const auto& point : msg->points) {
            trajectory_.push_back(Vector2D(point.x, point.y));
        }
        trajectory_received_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
        current_speed_ = msg->twist.twist.linear.x;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw_);
        pose_received_ = true;
    }

    Vector2D findNearestPoint() {
        Vector2D current_pos(current_pose_.position.x, current_pose_.position.y);
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        const double look_ahead_dist = 1.0; // 1미터 앞을 보도록 설정
        
        for (size_t i = 0; i < trajectory_.size(); ++i) {
            Vector2D to_waypoint = trajectory_[i] - current_pos;
            double heading_to_waypoint = atan2(to_waypoint.y, to_waypoint.x);
            
            double heading_diff = heading_to_waypoint - yaw_;
            if (heading_diff > M_PI) heading_diff -= 2*M_PI;
            if (heading_diff < -M_PI) heading_diff += 2*M_PI;
            
            if (std::abs(heading_diff) < M_PI/2) {
                double dist = to_waypoint.norm();
                // look_ahead_dist와의 차이가 가장 작은 포인트를 선택
                double dist_diff = std::abs(dist - look_ahead_dist);
                if (dist_diff < min_dist) {
                    min_dist = dist_diff;
                    closest_idx = i;
                }
            }
        }
        
        return trajectory_[closest_idx];
    }
    void publishTargetPoint(const Vector2D& point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        target_point_pub_.publish(marker);
    }

    double calculateSteeringAngle() {
        Vector2D nearest = findNearestPoint();
        Vector2D current(current_pose_.position.x, current_pose_.position.y);
        
        // Cross track error
        double e = (nearest - current).norm();
        double path_heading = atan2(nearest.y - current.y, nearest.x - current.x);
        
        // Heading error
        double heading_error = path_heading - yaw_;
        if (heading_error > M_PI) heading_error -= 2*M_PI;
        if (heading_error < -M_PI) heading_error += 2*M_PI;
        
        // Stanley control law
        double steering = (heading_error + atan2(k_ * e, current_speed_ + 0.1)) / (2 * M_PI) ;
        return std::max(-max_steer_, std::min(max_steer_, steering));
    }

    void run() {
        while (ros::ok()) {
            if (!pose_received_ || !trajectory_received_) {
                ROS_INFO("Waiting for pose and trajectory data...");
                ros::spinOnce();
                rate_.sleep();
                continue;
            }
            Vector2D nearest = findNearestPoint();
            publishTargetPoint(nearest);
            double steering = calculateSteeringAngle();
            
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.steering_angle = steering;
            drive_msg.drive.speed = target_speed_;
            
            drive_pub_.publish(drive_msg);
            
            ros::spinOnce();
            rate_.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_controller");
    try {
        StanleyController controller;
        controller.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    return 0;
}
