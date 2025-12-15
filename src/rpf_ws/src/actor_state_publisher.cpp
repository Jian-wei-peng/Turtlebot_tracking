#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>

class ActorStatePublisher
{
public:
    ActorStatePublisher(ros::NodeHandle& nh)
        : has_last_pose_(false),
          marker_id_(0)
    {
        nh.param<std::string>("actor_name", actor_name_, "actor0");
        nh.param<std::string>("frame_id", frame_id_, "map");
        nh.param<double>("yaw_offset", yaw_offset_, -M_PI_2);
        nh.param<double>("trajectory_duration", traj_duration_, 8.0);
        nh.param<double>("trajectory_width", traj_width_, 0.05);

        sub_model_states_ = nh.subscribe(
            "/gazebo/model_states", 1,
            &ActorStatePublisher::modelStatesCallback, this);

        pub_pose_   = nh.advertise<geometry_msgs::PoseStamped>("actor/pose", 1);
        pub_marker_ = nh.advertise<visualization_msgs::Marker>("actor/marker", 10);
        pub_traj_   = nh.advertise<visualization_msgs::Marker>("actor/trajectory", 100);

        ROS_INFO("ActorStatePublisher started.");
    }

private:
    /* ================= Callback ================= */
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        auto it = std::find(msg->name.begin(), msg->name.end(), actor_name_);
        if (it == msg->name.end())
            return;

        size_t idx = std::distance(msg->name.begin(), it);

        geometry_msgs::Pose fixed_pose = fixActorPose(msg->pose[idx]);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose = fixed_pose;

        pub_pose_.publish(pose_msg);
        publishActorMarker(fixed_pose);
        publishTrajectorySegment(pose_msg);
    }

    /* ================= Pose Fix ================= */
    geometry_msgs::Pose fixActorPose(const geometry_msgs::Pose& pose)
    {
        geometry_msgs::Pose out = pose;

        tf2::Quaternion q_orig;
        tf2::fromMsg(pose.orientation, q_orig);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

        yaw += yaw_offset_;

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);
        q_yaw.normalize();

        out.orientation = tf2::toMsg(q_yaw);
        return out;
    }

    /* ================= Actor Marker ================= */
    void publishActorMarker(const geometry_msgs::Pose& pose)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = ros::Time::now();
        m.ns = "actor_body";
        m.id = 0;
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;

        m.scale.x = 0.4;
        m.scale.y = 0.4;
        m.scale.z = 1.7;

        m.pose.position = pose.position;
        m.pose.position.z = m.scale.z * 0.5;

        // --- orientation: yaw only ---
        tf2::Quaternion q;
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)).getRPY(roll, pitch, yaw);

        q.setRPY(0.0, 0.0, yaw);
        m.pose.orientation = tf2::toMsg(q);

        m.color.r = 0.1;
        m.color.g = 0.1;
        m.color.b = 1.0;
        m.color.a = 0.8;

        m.lifetime = ros::Duration(0);
        pub_marker_.publish(m);
    }

    /* ================= Trajectory ================= */
    void publishTrajectorySegment(const geometry_msgs::PoseStamped& pose)
    {
        if (!has_last_pose_)
        {
            last_pose_ = pose;
            has_last_pose_ = true;
            return;
        }

        visualization_msgs::Marker line;
        line.header.frame_id = frame_id_;
        line.header.stamp = ros::Time::now();
        line.ns = "actor_trajectory";
        line.id = marker_id_++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;

        // 初始化 orientation 防止 RViz 警告
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;

        line.scale.x = traj_width_;

        line.points.push_back(last_pose_.pose.position);
        line.points.push_back(pose.pose.position);

        line.color.r = 1.0;
        line.color.g = 0.3;
        line.color.b = 0.0;
        line.color.a = 1.0;

        line.lifetime = ros::Duration(traj_duration_);

        pub_traj_.publish(line);

        last_pose_ = pose;
    }

private:
    ros::Subscriber sub_model_states_;
    ros::Publisher  pub_pose_;
    ros::Publisher  pub_marker_;
    ros::Publisher  pub_traj_;

    std::string actor_name_;
    std::string frame_id_;
    double yaw_offset_;
    double traj_duration_;
    double traj_width_;

    geometry_msgs::PoseStamped last_pose_;
    bool has_last_pose_;
    int marker_id_;
};

/* ================= main ================= */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "actor_state_publisher");
    ros::NodeHandle nh("~");

    ActorStatePublisher node(nh);

    ros::spin();
    return 0;
}
