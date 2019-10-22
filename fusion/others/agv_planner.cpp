
/* frenet_optimal_trajectory_planner.cpp

    Copyright (C) 2019 SS47816, alexiscatnip, 
	& Advanced Robotics Center, National University of Singapore 
	& Micron Technology

    Local Planner ROS Node
    Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <obstacle_detector/Obstacles.h>
#include <agv/LaneInfo.h>

#include <vector>
#include <math.h>

#include <frenet_planner/frenet_optimal_trajectory_planner.h>

#include "frenet_planner/quintic_polynomial.h"
#include "frenet_planner/quartic_polynomial.h"
#include "frenet_planner/spline.h"
#include "frenet_planner/frenet_path.h"
#include "frenet_planner/lane.h"

using std::vector;
using std::cout;
using std::endl;

#define FRONTLINK_TO_BASELINK 2.2

class AgvPlanner
{
public:
	// Constructor
	AgvPlanner();

	// Destructor
	virtual ~AgvPlanner() {};

private:
	// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Private Variables $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    ros::NodeHandle nh;

	// debug switch
	bool debug = true;
	// Regnerate path flag
	bool regenerate_flag_ = false;

	// Hyperparameters
	double planning_frequency_;
	double steering_frequency_;
	double stanley_overall_gain_;
	double track_error_gain_;
	double max_steering_angle_;
	int output_path_max_size_;
	int output_path_min_size_;
	double distance_bound_;
 
	//obstacles: 2d array like: [x,y,radius,vel_x,vel_y]
	vector<vector<double>> obstacles; // from /obstacles topic
	vector<vector<double>> obstacles_2; // from /obstacles2 topic

	// Maps (All the waypoints) { x0, x1, x2, ...}
	vector<double> maps_x; // from /lane_info topic
	vector<double> maps_y; // from /lane_info topic
	vector<double> maps_dx; // from /lane_info topic
	vector<double> maps_dy; // from /lane_info topic
	vector<double> maps_s; // from /lane_info topic
	vector<double> maps_left_width; // from /lane_info topic
	vector<double> maps_right_width; // from /lane_info topic
	vector<double> maps_far_right_width;

	// Selected Waypoints { x0, x1, x2, ...}
	vector<double> waypoints_x; // from /lane_info topic
	vector<double> waypoints_y; // from /lane_info topic
	vector<double> waypoints_dx; // from /lane_info topic
	vector<double> waypoints_dy; // from /lane_info topic
	vector<double> waypoints_s; // from /lane_info topic
	vector<double> waypoints_left_width; // from /lane_info topic
	vector<double> waypoints_right_width; // from /lane_info topic
	vector<double> waypoints_far_right_width;

	// Vehicle's current state
	// Received from odom data
	double current_x = 0.0;
	double current_y = 0.0;
	double current_yaw = 0.0;
	double current_speed = 0.0;
	double frontlink_x;
	double frontlink_y;
	
	// derived from odom data
	double current_s; 
	double current_s_d; // need to get from /current_speed topic
	double current_d; // need to get from localization data
	double current_d_d;

	// Starting States for sampling
	double start_s;
	double start_s_d;
	double start_d;
	double start_d_d;
	double start_d_dd = 0.0;

	// output path
	vector<double> output_path_x;
	vector<double> output_path_y;
	vector<double> output_path_yaw;

	// output steering angle
	double steering_angle;

	//subscriber and publishers
	ros::Subscriber odom_sub;
	ros::Subscriber obstacle_sub;
	ros::Subscriber obstacle_2_sub;
	ros::Subscriber lane_info_sub;

	ros::Publisher output_path_pub;
	ros::Publisher next_path_pub;
	ros::Publisher ref_path_pub;
	ros::Publisher wp_array_pub;
	ros::Publisher current_pose_pub;
	ros::Publisher steering_angle_pub;

	//timer
	ros::Timer timer;
	//ros::Timer timer_2;

	// parameters
	double MAX_SPEED_;              // maximum speed [m/s]
	double MAX_ACCEL_;              // maximum acceleration [m/ss]
	double MAX_DECCEL_;             // maximum deceleration [m/ss]
	double MAX_CURVATURE_;          // maximum curvature [1/m]

	double LEFT_ROAD_WIDTH_;        // maximum left road width [m]
	double RIGHT_ROAD_WIDTH_;       // maximum right road width [m]
	double DELTA_WIDTH_;            // road width sampling length [m]
	
	double MAX_T_;                  // max prediction time [m]
	double MIN_T_;                  // min prediction time [m]
	double DELTA_T_;                // time tick [s]

	double TARGET_SPEED_;           // target speed [m/s]
	double DELTA_SPEED_;            // target speed sampling length [m/s]
	double NUM_SPEED_SAMPLE_;       // sampling number of target speed
	
	double NUM_OF_CIRCLES_;         // number of circles that used to represent the vehicle
	double CIRCLE_RADIUS_;          // circle radius [m]
	double CIRCLE_POSITIONS_1_;   // longitudinal position of circles wrt baselink frame
	double CIRCLE_POSITIONS_2_;
	double CIRCLE_POSITIONS_3_;
	// double VEHICLE_LENGTH_;          // vehicle length [m]
	// double VEHICLE_WIDTH_;           // vehicle width [m]

	// Cost Weights
	double KJ_;
	double KT_;
	double KD_;
	double KLAT_;
	double KLON_;

	// Params
	vector<double> PARAMS_;

	//Instantiate the Frenet compute object 
	FrenetOptimalTrajectoryPlanner frenet_planner_instance;// = FrenetOptimalTrajectoryPlanner();
	
	// Variables and Functions for subscribe to odom topic
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

	// ###################################### Private Functions ######################################

    // Functions for subscribing
	void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
	void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
	void laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info);
	
	// Functions fo publishing results
	void publishNextPath(const FrenetPath &frenet_path);
	void publishOutputPath(const vector<double> &output_path_x, const vector<double> &output_path_y, 
							const vector<double> &output_path_yaw);
	void publishRefSpline(const vector<double> &x, const vector<double> &y, const vector<double> &yaw);
	void publishWpArray(const vector<double> &wps_x, const vector<double> &wps_y);
	void publishPose(double current_x, double current_y, double current_yaw);
	
	// Odom Helper Functions
	void updateVehicleState();

	// Planner Helper Functions
	void updateStartState();
	void feedWaypoints();
	void concatPath(FrenetPath &frenet_path, int path_size, double bound);
	void updateOutputPath();

	// Stanley Steeing Functions
	double calculateSteeringAngle(int next_wp_id, double frontlink_x, double frontlink_y);
	void publishSteeringAngle(double angle);
	

	// Main Function in ROS running primary logics
	void mainTimerCallback(const ros::TimerEvent& timer_event);
	void steeringTimerCallback(const ros::TimerEvent& timer_event);
};

// Constructor
AgvPlanner::AgvPlanner() : tf_listener(tf_buffer)
{
	ros::NodeHandle private_nh("~");

	// topics
	std::string odom_topic_;
	std::string obstacle_topic_;
	std::string obstacle_topic_2_;
	std::string lane_info_topic_;
	std::string output_path_topic_;
	std::string next_path_topic_;
	std::string ref_path_topic_;
	std::string wp_array_topic_;
	std::string current_pose_topic_;
	std::string steering_angle_topic_;


	// parameters from launch file: Hyperparameters
	ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));
	ROS_ASSERT(private_nh.getParam("output_path_max_size", output_path_max_size_));
	ROS_ASSERT(private_nh.getParam("output_path_min_size", output_path_min_size_));
	ROS_ASSERT(private_nh.getParam("distance_bound", distance_bound_));

	// Parameters from launch file: topic names
	ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
	ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));       
	ROS_ASSERT(private_nh.getParam("obstacle_topic_2", obstacle_topic_2_));
	ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic_));
	ROS_ASSERT(private_nh.getParam("output_path_topic", output_path_topic_));
	ROS_ASSERT(private_nh.getParam("next_path_topic", next_path_topic_));
	ROS_ASSERT(private_nh.getParam("ref_path_topic", ref_path_topic_));
	ROS_ASSERT(private_nh.getParam("wp_array_topic", wp_array_topic_));
	ROS_ASSERT(private_nh.getParam("current_pose_topic", current_pose_topic_));
	
	// Steering Related Parameters
	ROS_ASSERT(private_nh.getParam("steering_frequency", steering_frequency_));
	ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic_));
	ROS_ASSERT(private_nh.getParam("stanley_overall_gain", stanley_overall_gain_));
	ROS_ASSERT(private_nh.getParam("track_error_gain", track_error_gain_));
	ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));

	// Parameters from launch file: Planner Parameters
	ROS_ASSERT(private_nh.getParam("max_speed", MAX_SPEED_)); // maximum speed [m/s]    //MAX_SPEED_ = 15.0 / 3.6;      
	ROS_ASSERT(private_nh.getParam("max_accel", MAX_ACCEL_)); // maximum acceleration [m/ss]
	ROS_ASSERT(private_nh.getParam("max_deccel", MAX_DECCEL_)); // maximum deceleration [m/ss]
	ROS_ASSERT(private_nh.getParam("max_curvature", MAX_CURVATURE_)); // maximum curvature [1/m]

	ROS_ASSERT(private_nh.getParam("left_road_width", LEFT_ROAD_WIDTH_)); // maximum left road width [m]
	ROS_ASSERT(private_nh.getParam("right_road_width", RIGHT_ROAD_WIDTH_)); // maximum right road width [m]
	ROS_ASSERT(private_nh.getParam("delta_width", DELTA_WIDTH_)); // road width sampling length [m]

	ROS_ASSERT(private_nh.getParam("max_t", MAX_T_)); // max prediction time [m]
	ROS_ASSERT(private_nh.getParam("min_t", MIN_T_)); // min prediction time [m]
	ROS_ASSERT(private_nh.getParam("delta_t", DELTA_T_)); // time tick [s]

	ROS_ASSERT(private_nh.getParam("target_speed", TARGET_SPEED_)); // target speed [m/s]
	ROS_ASSERT(private_nh.getParam("delta_speed", DELTA_SPEED_)); // target speed sampling length [m/s]
	ROS_ASSERT(private_nh.getParam("num_max_sample", NUM_SPEED_SAMPLE_)); // sampling number of target speed

	ROS_ASSERT(private_nh.getParam("num_of_circles", NUM_OF_CIRCLES_)); // number of circles that used to represent the vehicle
	ROS_ASSERT(private_nh.getParam("circle_radius", CIRCLE_RADIUS_)); // circle radius [m]
	ROS_ASSERT(private_nh.getParam("circle_position_1", CIRCLE_POSITIONS_1_)); // longitudinal position of circles wrt baselink frame
	ROS_ASSERT(private_nh.getParam("circle_position_2", CIRCLE_POSITIONS_2_)); // longitudinal position of circles
	ROS_ASSERT(private_nh.getParam("circle_position_3", CIRCLE_POSITIONS_3_)); // longitudinal position of circles
					
	// VEHICLE_LENGTH = 3.2;        // vehicle length [m]
	// VEHICLE_WIDTH = 1.4;         // vehicle width [m]

	// PLanner Cost weights
	ROS_ASSERT(private_nh.getParam("kj", KJ_));
	ROS_ASSERT(private_nh.getParam("kt", KT_));
	ROS_ASSERT(private_nh.getParam("kd", KD_));
	ROS_ASSERT(private_nh.getParam("klat", KLAT_));
	ROS_ASSERT(private_nh.getParam("klon", KLON_));

	// Pass the params to the planner constructor
	PARAMS_ = {
		MAX_SPEED_,              // maximum speed [m/s]
		MAX_ACCEL_,              // maximum acceleration [m/ss]
		MAX_DECCEL_,             // maximum deceleration [m/ss]
		MAX_CURVATURE_,          // maximum curvature [1/m]

		LEFT_ROAD_WIDTH_,        // maximum left road width [m]
		RIGHT_ROAD_WIDTH_,       // maximum right road width [m]
		DELTA_WIDTH_,            // road width sampling length [m]
		
		MAX_T_,                  // max prediction time [m]
		MIN_T_,                  // min prediction time [m]
		DELTA_T_,                // time tick [s]

		TARGET_SPEED_,           // target speed [m/s]
		DELTA_SPEED_,            // target speed sampling length [m/s]
		NUM_SPEED_SAMPLE_,       // sampling number of target speed
		
		CIRCLE_RADIUS_,          // circle radius [m]
		CIRCLE_POSITIONS_1_,	 // longitudinal position of circles wrt baselink frame
		CIRCLE_POSITIONS_2_,
		CIRCLE_POSITIONS_3_,

		// Cost Weights
		KJ_,
		KT_,
		KD_,
		KLAT_,
		KLON_
	};

	// Instantiate the Frenet compute object 
	frenet_planner_instance = FrenetOptimalTrajectoryPlanner(PARAMS_);

	// Subscribe & Advertise
	odom_sub = nh.subscribe(odom_topic_, 1, &AgvPlanner::odomCallback, this);
	lane_info_sub = nh.subscribe(lane_info_topic_, 1, &AgvPlanner::laneInfoCallback, this);
	obstacle_sub = nh.subscribe(obstacle_topic_, 1, &AgvPlanner::obstacleCallback, this);
	obstacle_2_sub = nh.subscribe(obstacle_topic_2_, 1, &AgvPlanner::obstacle2Callback, this);

	output_path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_, 1, true);
	next_path_pub = nh.advertise<nav_msgs::Path>(next_path_topic_, 1, true);
	ref_path_pub = nh.advertise<nav_msgs::Path>(ref_path_topic_, 1);
	wp_array_pub = nh.advertise<geometry_msgs::PoseArray>(wp_array_topic_, 1);
	current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(current_pose_topic_, 1);
	steering_angle_pub = nh.advertise<std_msgs::Float64>(steering_angle_topic_, 1);

	//timer
	timer = nh.createTimer(ros::Duration(1.0/planning_frequency_), &AgvPlanner::mainTimerCallback, this);
	//timer_2 = nh.createTimer(ros::Duration(1.0/steering_frequency_), &AgvPlanner::steeringTimerCallback, this);
};

void AgvPlanner::mainTimerCallback(const ros::TimerEvent& timer_event)
{
	ROS_DEBUG("timer start");

	// Update Waypoints
	feedWaypoints();
	updateStartState();
	
	if (regenerate_flag_)
	{
		// Clear the last output path
		output_path_x.clear();
		output_path_y.clear();
		output_path_yaw.clear();
	}


	// Check if all required data are in position
	if (waypoints_x.size() == 0 || waypoints_y.size() == 0) 
	{
		ROS_WARN("Waypoints XY Are Empty");
		return;
	}
	else if (waypoints_dx.size() == 0 || waypoints_dy.size() == 0) 
	{
		ROS_WARN("Waypoints dx dy are Empty");
		return;
	}
	if ((obstacles.size() == 0))
	{
		ROS_WARN("Obstacles List Is Empty");
	}
	if ((obstacles_2.size() == 0))
	{
		ROS_WARN("Obstacles_2 List Is Empty");
	}

	// Get the reference lane's centerline as a spline
	FrenetOptimalTrajectoryPlanner::ResultType result = frenet_planner_instance.generateReferenceCurve(waypoints_x, waypoints_y);
	//Spline2D ref_spline = result.cubic_spline;
	ROS_DEBUG("timer1");
	// Get the planning result (best path)
	FrenetPath best_path = frenet_planner_instance.frenetOptimalPlanning(result.cubic_spline, 
							start_s, start_s_d, start_d, start_d_d, start_d_dd, obstacles, obstacles_2);
	ROS_DEBUG("timer2");

	// flag for stopping the planner
	bool stop = false;

	// if cannot find best_trajectory, stop
	if (best_path.cf != NULL)
	{
		double dist_to_goal = (best_path.x[1] - result.rx[result.rx.size() - 1]) * (best_path.x[1] - result.rx[result.rx.size() - 1]) 
							+ (best_path.y[1] - result.ry[result.ry.size() - 1]) * (best_path.y[1] - result.ry[result.ry.size() - 1]);
		if (dist_to_goal <= 2.0)
		{
			stop = true;
		}
	}
	else
	{
		stop = true;
	}

	// Concatenate the best path into output_path
	concatPath(best_path, output_path_max_size_, distance_bound_);
	
	// Publish the best path to Stanley Controller via /current_path topic
	publishPose(current_x, current_y, current_yaw);
	publishNextPath(best_path);
	publishOutputPath(output_path_x, output_path_y, output_path_yaw);
	publishRefSpline(result.rx, result.ry, result.ryaw);
	//publishWpArray(waypoints_x, waypoints_y);

	// Publish steeing angle
	updateOutputPath();
	publishSteeringAngle(steering_angle);

}

void AgvPlanner::steeringTimerCallback(const ros::TimerEvent& timer_event)
{
	// Publish steeing angle
	//updateOutputPath();
	//publishSteeringAngle(steering_angle);
}

// This function calls the appropriate functions in the right order to obtain a navMsgs::path.
// It uses class variables that the callbacks will write to, such as obstacles and obstacles2.

void AgvPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	current_speed = odom_msg->twist.twist.linear.x;

	geometry_msgs::TransformStamped transform_stamped;
	try
	{
	transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
	}
	catch (tf2::TransformException& ex)
	{
	ROS_WARN("%s", ex.what());
	return;
	}

	geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
	pose_before_transform.header.frame_id = odom_msg->header.frame_id;
	pose_before_transform.header.stamp = odom_msg->header.stamp;
	pose_before_transform.pose = odom_msg->pose.pose;
	tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

	tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
					pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, current_yaw);

	// Current XY of robot (map frame)
	current_x = pose_after_transform.pose.position.x;
	current_y = pose_after_transform.pose.position.y;

	updateVehicleState();
}

void AgvPlanner::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles.clear();

	for (int i =0; i < obstacle_msg->circles.size(); i++) 
	{
		double x = obstacle_msg->circles[i].center.x;
		double y = obstacle_msg->circles[i].center.y;
		double r = obstacle_msg->circles[i].radius;
		double vx = obstacle_msg->circles[i].velocity.x;
		double vy = obstacle_msg->circles[i].velocity.y;
		obstacles.push_back({x, y, r, vx, vy});
	}
	return;
}

void AgvPlanner::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_2.clear();
	
	for (int i =0; i < obstacle_msg->circles.size(); i++) 
	{
		double x = obstacle_msg->circles[i].center.x;
		double y = obstacle_msg->circles[i].center.y;
		double r = obstacle_msg->circles[i].radius;
		double vx = obstacle_msg->circles[i].velocity.x;
		double vy = obstacle_msg->circles[i].velocity.y;
		obstacles_2.push_back({x, y, r, vx, vy});
	}
	return;
}

void AgvPlanner::laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info)
{
	int num_maps = lane_info->num_waypoints;

	maps_x.clear();
	maps_y.clear();
	maps_dx.clear();
	maps_dy.clear();
	maps_s.clear();
	maps_left_width.clear();
	maps_right_width.clear();
	maps_far_right_width.clear();

	for (int i =0; i < lane_info->waypoints.size(); i++) 
	{
		maps_x.push_back(lane_info->waypoints[i].x);
		maps_y.push_back(lane_info->waypoints[i].y);
		maps_dx.push_back(lane_info->waypoints[i].dx);
		maps_dy.push_back(lane_info->waypoints[i].dy);
		maps_s.push_back(lane_info->waypoints[i].s);
		maps_left_width.push_back(lane_info->waypoints[i].left_width);
		maps_right_width.push_back(lane_info->waypoints[i].right_width);
		maps_far_right_width.push_back(lane_info->waypoints[i].far_right_width);

		// maps_is_exit_waypoint.push_back(lane_info->waypoints[i].is_exit_waypoint);
		// maps_speed_cmd.push_back(lane_info->waypoints[i].speed_cmd);
	//TODO: each waypoint has a dx, dx, s , is_exit_point. These are not used here yet.
	}
}

void AgvPlanner::publishNextPath(const FrenetPath &frenet_path) 
{
	bool out_of_bound = false;
	double bound = 50.0;
	
	nav_msgs::Path output_path;
	output_path.header.frame_id = "map";

	for (int i = 0; i < frenet_path.x.size(); i++)
	{
		if (distance(current_x, current_y, frenet_path.x[i], frenet_path.y[i]) > bound)
		{
			//ROS_ERROR("Output Path Out of Bound");
			//out_of_bound = true;
			break;
		}
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "map";
		pose.pose.position.x = frenet_path.x[i];
		pose.pose.position.y = frenet_path.y[i];
		
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(frenet_path.yaw[i]);
		pose.pose.orientation = pose_quat;
		output_path.poses.emplace_back(pose);
	}
	
	if (!out_of_bound)
	{
		next_path_pub.publish(output_path);
	}
}

void AgvPlanner::publishOutputPath(const vector<double> &output_path_x, const vector<double> &output_path_y, 
									const vector<double> &output_path_yaw) 
{
	nav_msgs::Path output_path;
	output_path.header.frame_id = "map";

	for (int i = 0; i < output_path_x.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "map";
		pose.pose.position.x = output_path_x[i];
		pose.pose.position.y = output_path_y[i];
		
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(output_path_yaw[i]);
		pose.pose.orientation = pose_quat;
		output_path.poses.emplace_back(pose);
	}

	output_path_pub.publish(output_path);
}

void AgvPlanner::publishRefSpline(const vector<double> &x, const vector<double> &y, 
								const vector<double> &yaw) 
{
	bool out_of_bound = false;
	double bound = 1000000000.0;
	
	nav_msgs::Path ref_path;
	ref_path.header.frame_id = "map";

	for (int i = 0; i < yaw.size(); i++)
	{
		if (x[i] > bound || y[i] > bound)
		{
			ROS_ERROR("Output Path Out of Bound");
			out_of_bound = true;
			break;
		}
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "map";
		pose.pose.position.x = x[i];
		pose.pose.position.y = y[i];
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(yaw[i]);
		pose.pose.orientation = pose_quat;
		ref_path.poses.emplace_back(pose);
	}
	
	if (!out_of_bound)
	{
		ref_path_pub.publish(ref_path);
	}
}

void AgvPlanner::publishWpArray(const vector<double> &wps_x, const vector<double> &wps_y)
{
	geometry_msgs::PoseArray wp_array;
	wp_array.header.frame_id = "map";

	for (int i = 0; i < wps_x.size(); i++)
	{
		geometry_msgs::Pose pose;
		pose.position.x = wps_x[i];
		pose.position.y = wps_y[i];

		double d_x, d_y;
		if (i < wps_x.size() - 1)
		{
			d_x = wps_x[i + 1] - wps_x[i];
			d_y = wps_y[i + 1] - wps_y[i];
		}
		else
		{
			d_x = wps_x[i] - wps_x[i - 1];
			d_y = wps_y[i] - wps_y[i - 1];
		}
		
		double theta = atan2(d_y, d_x);
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(theta);
		pose.orientation = pose_quat;
		wp_array.poses.emplace_back(pose);
	}
	
	wp_array_pub.publish(wp_array);
}

void AgvPlanner::publishPose(double current_x, double current_y, double current_yaw)
{
	geometry_msgs::PoseStamped current_pose;
	current_pose.header.frame_id = "map";
	current_pose.pose.position.x = current_x;
	current_pose.pose.position.y = current_y;
	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(current_yaw);
	current_pose.pose.orientation = pose_quat;

	current_pose_pub.publish(current_pose);
}

// Odom Helper Functions
void AgvPlanner::updateVehicleState()
{
	if (waypoints_x.size() > 0)
	{
		vector<double> sd = getFrenet(current_x, current_y, current_yaw, current_speed, waypoints_x, waypoints_y, 
									waypoints_dx, waypoints_dy);
		current_s = sd[0];
		current_d = sd[1];
		current_s_d = sd[2] > 1.0? sd[2] : 1.0;
		current_d_d = sd[3];
		
		cout << "S = " << current_s << " D = " << current_d << endl;
		cout << "S_d = " << current_s_d << " D_d = " << current_d_d << endl;

		// Current XY of robot (map frame)
		frontlink_x = current_x + (FRONTLINK_TO_BASELINK * std::cos(current_yaw));
		frontlink_y = current_y + (FRONTLINK_TO_BASELINK * std::sin(current_yaw));

		// start_s = current_s;
		// start_s_d = current_s_d;
		// start_d = current_d;
		// start_d_d = current_d_d;
		// start_d_dd = 0.0;
	}
}

// Planner Helper Functions
void AgvPlanner::feedWaypoints()
{
	if (maps_x.size() == 0 || maps_y.size() == 0)
	{
		ROS_WARN("MAP XY Are Empty");
	}
	else if (maps_dx.size() == 0 || maps_dy.size() == 0)
	{
		ROS_WARN("MAP dx dy Are Empty");
	}
	else
	{
		int start_id = waypointFeeder(current_x, current_y, current_yaw, maps_x, maps_y);
		
		double dist = distance(maps_x[start_id], maps_y[start_id], current_x, current_y);
		double heading_diff = fabs(atan2(maps_dy[start_id], maps_dx[start_id]) + M_PI/2 - current_yaw);

		if(dist > 10)
		{
			ROS_ERROR("Vehicle's Location Is Too Far From The Target Lane");
		}
		else if (heading_diff > M_PI/2)
		{
			ROS_ERROR("Vehicle's Is Heading In A Different Direction");
		}
		else
		{
			waypoints_x.clear();
			waypoints_y.clear();
			waypoints_dx.clear();
			waypoints_dy.clear();
			// waypoints_s.clear();
			// waypoints_left_width.clear();
			// waypoints_right_width.clear();
			// waypoints_far_right_width.clear();

			for (int i = 0; i < 5; i++)
			{
				waypoints_x.push_back(maps_x[start_id + i]);
				waypoints_y.push_back(maps_y[start_id + i]);
				waypoints_dx.push_back(maps_dx[start_id + i]);
				waypoints_dy.push_back(maps_dy[start_id + i]);
				// waypoints_s.push_back(maps_s[start_id + i]);
				// waypoints_left_width.push_back(maps_left_width[start_id + i]);
				// waypoints_right_width.push_back(maps_right_width[start_id + i]);
				// waypoints_far_right_width.push_back(maps_far_right_width[start_id + i]);
			}
		}
	}
}

void AgvPlanner::updateStartState()
{
	if (waypoints_x.size() > 0)
	{
		if (output_path_x.size() < output_path_min_size_)
		{
			vector<double> sd = getFrenet(current_x, current_y, current_yaw, current_speed, waypoints_x, waypoints_y, 
										waypoints_dx, waypoints_dy);
			start_s = sd[0];
			start_d = sd[1];
			start_s_d = sd[2] > 1.0? sd[2] : 1.0; //13.0/3.6;
			start_d_d = sd[3];
			start_d_dd = 0.0;

			// start_s = current_s;
			// start_s_d = current_s_d;
			// start_d = current_d;
			// start_d_d = current_d_d;
			// start_d_dd = 0.0;
		}
		else
		{
			vector<double> sd = getFrenet(output_path_x[output_path_x.size() - 1], output_path_y[output_path_y.size() - 1], 
										output_path_yaw[output_path_yaw.size() - 1], current_speed, waypoints_x, waypoints_y, 
										waypoints_dx, waypoints_dy);
			start_s = sd[0];
			start_d = sd[1];
			start_s_d = sd[2] > 1.0? sd[2] : 1.0; //13.0/3.6;
			start_d_d = sd[3];
			start_d_dd = 0.0;
		}
	}
}

void AgvPlanner::concatPath(FrenetPath &frenet_path, int path_size, double bound)
{
	int diff = std::min(path_size - output_path_x.size(), frenet_path.x.size());
	cout << "Output Path Size: " << output_path_x.size() << " List Size: " << path_size << " Diff: " << diff << endl;

	for (int i = 0; i < diff; i++)
	{
		if (distance(current_x, current_y, frenet_path.x[i], frenet_path.y[i]) > bound)
		{
			break;
		}
		output_path_x.push_back(frenet_path.x[i]);
		output_path_y.push_back(frenet_path.y[i]);
		output_path_yaw.push_back(frenet_path.yaw[i]);
	}
}

void AgvPlanner::updateOutputPath()
{
	if (output_path_x.size() > 0 && output_path_y.size() > 0)
	{
		int next_wp_id = nextWaypoint(frontlink_x, frontlink_y, current_yaw, output_path_x, output_path_y);

		steering_angle = calculateSteeringAngle(next_wp_id, frontlink_x, frontlink_y);

		for (int i = 0; i < next_wp_id; i++)
		{
			output_path_x.erase(output_path_x.begin());
			output_path_y.erase(output_path_y.begin());
			output_path_yaw.erase(output_path_yaw.begin());
		}
	}
	else
	{
		ROS_ERROR("Output Path is Empty, Cannot Update");
	}
}

// Steeing Help Function
double AgvPlanner::calculateSteeringAngle(int wp_id, double frontlink_x, double frontlink_y)
{	
	int look_ahead = 0;
	wp_id += look_ahead;

	if (output_path_x.size() > wp_id + 1)
	{
		// First Term
		double delta_yaw = output_path_yaw[wp_id] - current_yaw;
		if (delta_yaw > M_PI)
		{
			delta_yaw -= (2 * M_PI);
		}
		else if (delta_yaw < -M_PI)
		{
			delta_yaw += (2 * M_PI);
		}

		// Second Term
		double a = distance(frontlink_x, frontlink_y, output_path_x[wp_id], output_path_y[wp_id]);
		double b = distance(frontlink_x, frontlink_y, output_path_x[wp_id + 1], output_path_y[wp_id + 1]);
		double c = distance(output_path_x[wp_id], output_path_y[wp_id], 
								output_path_x[wp_id + 1], output_path_y[wp_id + 1]);
		double p = (a + b + c) / 2.0;
		double triangle_area = sqrt(p * (p-a) * (p-b) * (p-c));
		double x = triangle_area * 2.0 / c;
		double u = std::max(0.3, current_speed);
		int direction = current_d >= 0 ? -1 : 1;

		// Final Angle
		steering_angle = stanley_overall_gain_ * ( delta_yaw + direction * atan(track_error_gain_ * x / u) );

		cout << "First Term: " << delta_yaw << " X(t) Term: " << x << " Second Term: " << direction * atan(track_error_gain_ * x / u) << endl;
		cout << "Steering Angle: " << steering_angle / M_PI * 180.0 << endl;

		// Check if exceeding max steering angle
		if (steering_angle >= max_steering_angle_)
		{
			return max_steering_angle_;
		}
		else if (steering_angle <= -max_steering_angle_)
		{
			return -max_steering_angle_;
		}
		else
		{
			return steering_angle;
		}
	}
	else
	{
		return 0.0;
	}
}

void AgvPlanner::publishSteeringAngle(double angle)
{
	if (output_path_x.size() <= 0)
	{
		ROS_ERROR("Output Path Empty! No output steering angle");
	}
	else
	{
		std_msgs::Float64 steering_angle;
		steering_angle.data = angle;
		steering_angle_pub.publish(steering_angle);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "frenet_optimal_trajectory_planner");
	// Construct a planner object
	AgvPlanner node_instance;// = AgvPlanner();
	ros::spin(); //spin the ros node.
	return 0;
}