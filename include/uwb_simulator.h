#ifndef UWB_SIMULATOR_H
#define UWB_SIMULATOR_H

#include <vector>
#include <algorithm>
#include <map>
#include <random>
#include <queue>
#include <set>
#include <cmath>
#include <ostream>

#include <boost/geometry.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>

#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <sensor_msgs/Range.h>

#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/ModelStates.h>

// Custom UWB messages
#include <nexus_swarm_sim/UwbRange.h>
#include <nexus_swarm_sim/RawUWBSignal.h>

/*
    Struct to store a range to be calculated
    including two models (nodes) and the ID of
    the UWB node in each of them.
*/
struct range_publisher
{
    std::string ori_node;
    std::string end_node;
    int ori_id;
    int end_id;

    range_publisher(std::string n1, std::string n2, int id1, int id2)
    {
        ori_node = n1;
        end_node = n2;
        ori_id = id1;
        end_id = id2;
    }

    bool const operator==(const range_publisher &o) const {
        return ori_node + std::to_string(ori_id) + end_node + std::to_string(end_id) == o.ori_node + std::to_string(o.ori_id) + o.end_node + std::to_string(o.end_id);
    }

    bool const operator<(const range_publisher &o) const {
        return ori_node + std::to_string(ori_id) + end_node + std::to_string(end_id) < o.ori_node + std::to_string(o.ori_id) + o.end_node + std::to_string(o.end_id);
    }

};

std::ostream &operator<<(std::ostream &out, const range_publisher &o)
{
    out << "/from/" + o.ori_node + "/" + std::to_string(o.ori_id) + "/to/" + o.end_node + "/" + std::to_string(o.end_id);
    return out;
}

/*
    Struct for DW3000-realistic UWB range measurement
*/
struct UWBRangeMeasurement
{
    std::string src_id;
    std::string dst_id;
    double distance_3d;
    double distance_2d;
    double sigma_m;
    bool los;
    float rssi;
    float quality;
    uint16_t nlos_bias_mm;
    float fp_index;
    ros::Time timestamp;
};

/*
    Struct for K-nearest neighbor tracking with hysteresis
*/
struct NeighborTrackingState
{
    std::string drone_id;
    std::set<std::string> active_neighbors;        // Currently tracked neighbors
    std::map<std::string, double> distances;       // Last known distances
    std::map<std::string, int> dropout_consecutive; // Count consecutive dropouts
};

/*
    UWB Simulator class.
    Subscribes to Gazebo model states (ground truth).
    Publishes UWB ranges between nodes specified in the config.
    Implements DW3000-realistic features: dropout, latency, outliers, LOS/NLOS, scheduling.
*/
class UwbSimulator
{

    public:
        // Constructors
        UwbSimulator();
        UwbSimulator(ros::NodeHandle& nh);

        // Timers
        void publish_ranges(const ros::TimerEvent& event);

    private:
        ros::NodeHandle nh_;  // Store node handle for dynamic publisher creation

        // UWB
        double duty_cycle_;
        double max_twr_freq_;

        // Publish settings
        bool publish_as_float_;

        // Topic names
        std::string ground_truth_topic_;
        std::string pub_topic_prefix_;
        std::string model_prefix_;  // Prefix to discover (e.g., "iris")

        // Discovery
        std::set<std::string> discovered_models_;

        // Configuration
        std::vector<std::string> model_names_;
        std::map<std::string, std::vector<std::vector<double>>> uwb_nodes_;
        std::map<std::string, std::vector<std::string>> uwb_node_names_;
        std::vector<std::string> uwb_ranges_param_;
        std::map<std::string, std::string> uwb_ranges_;

        // Model states
        std::map<std::string, geometry_msgs::Pose> models_gt_;  // Ground truth for the models

        // ===== DW3000-Realistic Parameters =====
        
        // Update rate scheduling
        double update_rate_hz_;
        double debug_rate_hz_;
        std::map<range_publisher, int> publish_counters_;  // Per-pair counter to throttle updates
        
        // Dropout model
        double p_drop_base_;
        double p_drop_slope_;
        double p_drop_max_distance_;
        
        // Latency model
        double latency_mu_ms_;
        double latency_sigma_ms_;
        bool jitter_enabled_;
        std::map<range_publisher, ros::Time> measurement_timestamps_;  // When measurement was "taken"
        
        // Outlier model
        double outlier_probability_;
        double outlier_bias_min_m_;
        double outlier_bias_max_m_;
        
        // LOS/NLOS detection
        bool enable_los_check_;
        int nlos_bias_mm_;
        double nlos_sigma_increase_;
        
        // K-nearest neighbor selection
        int k_neighbors_;
        double hysteresis_enter_m_;
        double hysteresis_exit_m_;
        bool hysteresis_enabled_;
        std::map<std::string, NeighborTrackingState> neighbor_tracking_;
        
        // Debug options
        bool enable_debug_topics_;
        bool debug_verbose_;
        
        // Random number generators
        std::mt19937 rng_;
        std::uniform_real_distribution<double> uniform_dist_;
        
        // ROS subscribers
        ros::Subscriber ground_truth_sub_;

        // ROS publishers
        std::map<range_publisher, ros::Publisher> uwb_publishers_;  // Legacy debug publishers (optional)
        
        // Drone-specific publishers (each drone publishes its own TX - processed distances)
        std::map<std::string, ros::Publisher> drone_tx_publishers_;  // /uwb/iris0/range, /uwb/iris1/range, etc.
        
        // Drone-specific raw signal publishers (PRIMARY - for signal processing algorithms)
        std::map<std::string, ros::Publisher> drone_raw_signal_publishers_;  // /uwb/iris0/raw_signal, etc.

        // Topic callbacks
        void ground_truth_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void check_for_new_models(const std::vector<std::string>& models);

        // ===== DW3000-Realistic Modeling Functions =====
        
        // Dropout model: returns true if measurement should be dropped
        bool should_dropout(double distance_m);
        
        // Latency model: returns latency in milliseconds
        double sample_latency();
        
        // Outlier model: returns true if outlier and sets bias
        bool should_inject_outlier(double& bias_m);
        
        // LOS/NLOS detection via raycast
        bool is_los(const geometry_msgs::Pose& src_pose, const geometry_msgs::Pose& dst_pose);
        
        // K-nearest neighbor update and filtering
        bool should_range_neighbor(const std::string& src_id, const std::string& dst_id, double distance_m);
        void update_neighbor_tracking(const std::string& src_id, const std::string& dst_id, double distance_m);
        
        // Scheduling decision: should we publish this measurement now?
        bool should_publish_measurement(const range_publisher& rp);
        
        // Payload generation (custom data from sender)
        std::vector<uint8_t> generate_payload(const std::string& src_id);
        
        // Raw signal generation (low-level ToA-based measurement)
        void generate_raw_signal(double true_distance_m, bool los, 
                                 float& toa_ns, float& snr_db, float& rssi);

        // Utils
        boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> calc_uwb_node_pose(
            geometry_msgs::Pose vehicle_pose, std::vector<double> uwb_relative_pos);
};

#endif
