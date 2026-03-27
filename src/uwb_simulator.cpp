#include "uwb_simulator.h"
#include <ctime>
#include <cstdlib>
#include <algorithm>

UwbSimulator::UwbSimulator(ros::NodeHandle& nh) : nh_(nh)
{
    ROS_INFO_STREAM("[UWBSim] Initializing UWB Simulator with Dynamic Discovery...");
    
    // Seed random number generator
    rng_.seed(std::time(nullptr));
    
    // Topics
    nh_.param<std::string>("/uwb_simulator/ground_truth_topic", ground_truth_topic_, "/gazebo/model_states");
    nh_.param<std::string>("/uwb_simulator/pub_topic_prefix", pub_topic_prefix_, "/uwb");
    nh_.param<std::string>("/drone_prefix", model_prefix_, std::string("nexus"));
    nh_.param<std::string>("/uwb_simulator/model_prefix", model_prefix_, model_prefix_);

    // UWB global params
    nh_.param<double>("/uwb_simulator/duty_cycle", duty_cycle_, 1);
    nh_.param<double>("/uwb_simulator/max_twr_freq", max_twr_freq_, 400);
    nh_.param<bool>("/uwb_simulator/publish_as_float", publish_as_float_, false);

    // ===== DW3000-Realistic Parameters =====
    nh_.param<double>("/uwb_simulator/update_rate_hz", update_rate_hz_, 15.0);
    nh_.param<double>("/uwb_simulator/debug_rate_hz", debug_rate_hz_, 100.0);
    
    nh_.param<double>("/uwb_simulator/p_drop_base", p_drop_base_, 0.01);
    nh_.param<double>("/uwb_simulator/p_drop_slope", p_drop_slope_, 0.005);
    nh_.param<double>("/uwb_simulator/p_drop_max_distance", p_drop_max_distance_, 100.0);
    
    nh_.param<double>("/uwb_simulator/latency_mu_ms", latency_mu_ms_, 5.0);
    nh_.param<double>("/uwb_simulator/latency_sigma_ms", latency_sigma_ms_, 2.0);
    nh_.param<bool>("/uwb_simulator/jitter_enabled", jitter_enabled_, true);
    
    nh_.param<double>("/uwb_simulator/outlier_probability", outlier_probability_, 0.02);
    nh_.param<double>("/uwb_simulator/outlier_bias_min_m", outlier_bias_min_m_, 0.5);
    nh_.param<double>("/uwb_simulator/outlier_bias_max_m", outlier_bias_max_m_, 2.0);
    
    nh_.param<bool>("/uwb_simulator/enable_los_check", enable_los_check_, true);
    nh_.param<bool>("/uwb_simulator/use_gazebo_raycast", use_gazebo_raycast_, true);
    nh_.param<int>("/uwb_simulator/nlos_bias_mm", nlos_bias_mm_, 150);
    nh_.param<double>("/uwb_simulator/nlos_sigma_increase", nlos_sigma_increase_, 1.5);
    nh_.param<std::string>("/uwb_simulator/los_service_name", los_service_name_, "/uwb_simulator/check_los");
    
    nh_.param<int>("/uwb_simulator/k_neighbors", k_neighbors_, 3);
    nh_.param<double>("/uwb_simulator/hysteresis_enter_m", hysteresis_enter_m_, 80.0);
    nh_.param<double>("/uwb_simulator/hysteresis_exit_m", hysteresis_exit_m_, 100.0);
    nh_.param<bool>("/uwb_simulator/hysteresis_enabled", hysteresis_enabled_, true);
    
    nh_.param<bool>("/uwb_simulator/enable_debug_topics", enable_debug_topics_, false);
    nh_.param<bool>("/uwb_simulator/debug_verbose", debug_verbose_, false);

    // Subscribers
    ground_truth_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(ground_truth_topic_, 1, &UwbSimulator::ground_truth_callback, this);
    if (use_gazebo_raycast_)
    {
        los_service_client_ = nh_.serviceClient<nexus_swarm_sim::CheckLineOfSight>(los_service_name_);
    }

    ROS_INFO_STREAM("[UWBSim] Initialization complete. Discovery prefix: '" << model_prefix_ << "'");
}

// ===== DW3000-Realistic Modeling Functions =====

bool UwbSimulator::should_dropout(double distance_m)
{
    // Dropout probability increases with distance
    double clamped_distance = std::min(distance_m, p_drop_max_distance_);
    double p_drop = p_drop_base_ + p_drop_slope_ * clamped_distance;
    p_drop = std::min(p_drop, 1.0);  // Clamp to [0,1]
    
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng_) < p_drop;
}

double UwbSimulator::sample_latency()
{
    std::normal_distribution<double> latency_dist(latency_mu_ms_, latency_sigma_ms_);
    return std::max(0.0, latency_dist(rng_));  // Ensure non-negative
}

bool UwbSimulator::should_inject_outlier(double& bias_m)
{
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    if (prob_dist(rng_) < outlier_probability_)
    {
        std::uniform_real_distribution<double> bias_dist(outlier_bias_min_m_, outlier_bias_max_m_);
        bias_m = bias_dist(rng_);
        return true;
    }
    bias_m = 0.0;
    return false;
}

bool UwbSimulator::is_los(const std::string& src_id, const std::string& dst_id,
                         const geometry_msgs::Pose& src_pose, const geometry_msgs::Pose& dst_pose)
{
    if (enable_los_check_ && use_gazebo_raycast_ && los_service_client_.isValid())
    {
        nexus_swarm_sim::CheckLineOfSight srv;
        srv.request.src_model = src_id;
        srv.request.dst_model = dst_id;

        if (!srv.request.src_model.empty() && !srv.request.dst_model.empty() &&
            srv.request.src_model != srv.request.dst_model &&
            los_service_client_.call(srv) && srv.response.success)
        {
            return srv.response.los;
        }
    }

    // Improved LOS model: combines altitude difference and 3D distance
    // In a real Gazebo implementation this would use ray casting against collision objects.
    // Here we use a probabilistic approach:
    //   - Small altitude difference AND short range → likely LOS
    //   - Large altitude difference OR long range → NLOS probability increases
    double alt_diff = std::abs(src_pose.position.z - dst_pose.position.z);
    double dx = src_pose.position.x - dst_pose.position.x;
    double dy = src_pose.position.y - dst_pose.position.y;
    double dz = src_pose.position.z - dst_pose.position.z;
    double dist_3d = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Hard threshold: if alt_diff > 5m, very likely blocked by terrain
    if (alt_diff > 5.0) return false;

    // Probabilistic NLOS: P(NLOS) increases with distance (0% @ 0m → 40% @ 100m)
    // and with altitude difference (0% @ 0m diff → 30% @ 5m diff)
    double p_nlos_dist = std::min(0.4, 0.004 * dist_3d);
    double p_nlos_alt  = std::min(0.3, 0.06  * alt_diff);
    double p_nlos = std::min(1.0, p_nlos_dist + p_nlos_alt);

    std::uniform_real_distribution<double> d(0.0, 1.0);
    return d(rng_) >= p_nlos;  // LOS if survived the NLOS probability
}

bool UwbSimulator::should_range_neighbor(const std::string& src_id, const std::string& dst_id, double distance_m)
{
    if (!hysteresis_enabled_)
    {
        // Simple distance-based filtering
        return distance_m < hysteresis_enter_m_;
    }

    // K-nearest neighbor with hysteresis
    auto& tracker = neighbor_tracking_[src_id];
    bool is_active = tracker.active_neighbors.count(dst_id) > 0;
    
    if (!is_active && distance_m < hysteresis_enter_m_)
    {
        // Add new neighbor
        tracker.active_neighbors.insert(dst_id);
        tracker.distances[dst_id] = distance_m;
        return true;
    }
    else if (is_active && distance_m > hysteresis_exit_m_)
    {
        // Remove neighbor
        tracker.active_neighbors.erase(dst_id);
        return false;
    }
    else if (is_active)
    {
        // Update distance for active neighbor
        tracker.distances[dst_id] = distance_m;
        return true;
    }

    return false;
}

void UwbSimulator::update_neighbor_tracking(const std::string& src_id, const std::string& dst_id, double distance_m)
{
    auto& tracker = neighbor_tracking_[src_id];
    tracker.distances[dst_id] = distance_m;
}

bool UwbSimulator::should_publish_measurement(const range_publisher& rp)
{
    // Per-pair counter: each drone pair throttled independently to update_rate_hz
    int publish_interval = static_cast<int>(max_twr_freq_ / update_rate_hz_);
    if (publish_interval < 1) publish_interval = 1;
    auto& counter = publish_counters_[rp];  // Default-constructed to 0 on first access
    counter++;
    return (counter % publish_interval) == 0;
}

// ===== Core Simulator =====

void UwbSimulator::ground_truth_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    std::vector<std::string> all_models = msg->name;
    std::vector<geometry_msgs::Pose> all_poses = msg->pose;

    // Discovery phase
    check_for_new_models(all_models);

    for (const std::string &model: discovered_models_)
    {
        auto model_idx = std::find(all_models.begin(), all_models.end(), model);
        if (model_idx != all_models.end())
        {  
            int idx = std::distance(all_models.begin(), model_idx); 
            models_gt_[model] = all_poses[idx];
        }
    }
}

void UwbSimulator::check_for_new_models(const std::vector<std::string>& models)
{
    for (const std::string& model : models)
    {
        // Skip models that don't match the prefix or are already discovered
        if (model.find(model_prefix_) != 0 || discovered_models_.count(model) > 0)
        {
            continue;
        }

        ROS_INFO_STREAM("[UWBSim] Dynamic Discovery: New model detected: " << model);
        discovered_models_.insert(model);

        // Initialize ground truth storage
        models_gt_[model] = geometry_msgs::Pose();

        // Initialize neighbor tracking
        NeighborTrackingState nts;
        nts.drone_id = model;
        neighbor_tracking_[model] = nts;

        // Initialize UWB node positions for this model (default to center if not in YAML)
        if (uwb_nodes_.find(model) == uwb_nodes_.end())
        {
            uwb_nodes_[model] = { {0.0, 0.0, 0.0} };
            uwb_node_names_[model] = { "uwb_tx" };
        }

        // Create publishers for this new drone
        std::string range_topic = "/" + model + pub_topic_prefix_ + "/range";
        std::string raw_topic = "/" + model + pub_topic_prefix_ + "/raw_signal";
        
        drone_tx_publishers_[model] = nh_.advertise<nexus_swarm_sim::UwbRange>(range_topic, 100);
        drone_raw_signal_publishers_[model] = nh_.advertise<nexus_swarm_sim::RawUWBSignal>(raw_topic, 100);

        ROS_INFO_STREAM("  --> Created publishers for " << model);

        // Dynamicaly create range pairs with all OTHER discovered models
        for (const std::string& other : discovered_models_)
        {
            if (model == other) continue;

            // Add pairs in BOTH directions (TWR)
            // iris3 -> iris0, iris3 -> iris1...
            range_publisher rpub_new_src(model, other, 0, 0);
            uwb_publishers_.insert(std::pair<range_publisher, ros::Publisher>(rpub_new_src, ros::Publisher())); // Placeholder for logic compatibility
            
            // iris0 -> iris3, iris1 -> iris3...
            range_publisher rpub_new_dst(other, model, 0, 0);
            uwb_publishers_.insert(std::pair<range_publisher, ros::Publisher>(rpub_new_dst, ros::Publisher()));

            ROS_INFO_STREAM("  --> Initialized ranging links: " << model << " <--> " << other);
        }
    }
}

boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> UwbSimulator::calc_uwb_node_pose(
    geometry_msgs::Pose vehicle_pose, std::vector<double> uwb_relative_pos)
{
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_base(
        vehicle_pose.position.x,
        vehicle_pose.position.y,
        vehicle_pose.position.z
    );
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v(
        uwb_relative_pos[0],
        uwb_relative_pos[1],
        uwb_relative_pos[2]
    );
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> u(
        vehicle_pose.orientation.x,
        vehicle_pose.orientation.y,
        vehicle_pose.orientation.z
    );
    double s = vehicle_pose.orientation.w;

    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_final, part2, part3;
    v_final = u; 
    boost::geometry::multiply_value(v_final, 2 * boost::geometry::dot_product(u, v));
    part2 = v;
    boost::geometry::multiply_value(part2, (s * s - boost::geometry::dot_product(u, u)));
    part3 = boost::geometry::cross_product(u, v);
    boost::geometry::multiply_value(part3, 2 * s);
    boost::geometry::add_point(v_final, part2);
    boost::geometry::add_point(v_final, part3);
    boost::geometry::add_point(v_final, v_base);

    return v_final;
}

void UwbSimulator::publish_ranges(const ros::TimerEvent& event)
{
    for(const auto& elem : uwb_publishers_)
    {
        boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_ori, v_end;
        v_ori = calc_uwb_node_pose(models_gt_[elem.first.ori_node],
            uwb_nodes_[elem.first.ori_node][elem.first.ori_id]);
        v_end = calc_uwb_node_pose(models_gt_[elem.first.end_node],
            uwb_nodes_[elem.first.end_node][elem.first.end_id]);
    
        double true_distance_3d = boost::geometry::distance(v_ori, v_end);

        // DW3000-realistic models
        if (should_dropout(true_distance_3d))
        {
            if (debug_verbose_) ROS_WARN_STREAM("[UWBSim] Dropout: " << elem.first);
            continue;  // Skip this measurement
        }

        // Calculate 2D distance (horizontal plane)
        double dx = v_end.get<0>() - v_ori.get<0>();
        double dy = v_end.get<1>() - v_ori.get<1>();
        double true_distance_2d = std::sqrt(dx*dx + dy*dy);
        double dz = v_end.get<2>() - v_ori.get<2>();
        double measured_distance_3d = true_distance_3d;

        // Outlier injection
        double outlier_bias = 0.0;
        bool outlier_injected = should_inject_outlier(outlier_bias);
        if (outlier_injected)
        {
            measured_distance_3d += outlier_bias;
            if (debug_verbose_) ROS_WARN_STREAM("[UWBSim] Outlier injected: " << outlier_bias << "m");
        }

        // LOS/NLOS detection
        bool los = is_los(elem.first.ori_node, elem.first.end_node,
                          models_gt_[elem.first.ori_node], models_gt_[elem.first.end_node]);
        double sigma = 0.1;  // Base uncertainty
        uint16_t nlos_bias_mm = 0;
        
        if (!los)
        {
            nlos_bias_mm = static_cast<uint16_t>(nlos_bias_mm_);
            measured_distance_3d += nlos_bias_mm / 1000.0;  // Add excess path length bias
            sigma *= nlos_sigma_increase_;  // Increase uncertainty
        }

        std::normal_distribution<double> range_noise_dist(0.0, sigma);
        measured_distance_3d = std::max(0.0, measured_distance_3d + range_noise_dist(rng_));
        double measured_distance_2d = std::sqrt(std::max(0.0, measured_distance_3d * measured_distance_3d - dz * dz));

        // Neighbor filtering with K-nearest
        if (!should_range_neighbor(elem.first.ori_node, elem.first.end_node, measured_distance_3d))
        {
            if (debug_verbose_) ROS_WARN_STREAM("[UWBSim] Neighbor filtered out (distance threshold)");
            continue;  // Skip if not a tracked neighbor
        }

        // Update neighbor tracking
        update_neighbor_tracking(elem.first.ori_node, elem.first.end_node, measured_distance_3d);

        // Scheduling check
        if (!should_publish_measurement(elem.first))
        {
            continue;  // Skip this publish cycle (rate throttling)
        }

        ros::Time publish_stamp = ros::Time::now();
        if (jitter_enabled_)
        {
            publish_stamp -= ros::Duration(sample_latency() / 1000.0);
        }

        float toa_ns = 0.0f;
        float snr_db = 0.0f;
        float rssi = 0.0f;
        generate_raw_signal(measured_distance_3d, true_distance_3d, los, toa_ns, snr_db, rssi);
        float quality = estimate_quality(snr_db, los, outlier_injected);
        float fppl = estimate_fppl(rssi, los);
        float sts_quality = estimate_sts_quality(snr_db, los, outlier_injected);

        // Publish debug topic if enabled
        if (enable_debug_topics_)
        {
            if (publish_as_float_)
            {
                std_msgs::Float64 f = std_msgs::Float64();
                f.data = measured_distance_3d;
                elem.second.publish(f);
            }
            else 
            {
                sensor_msgs::Range r = sensor_msgs::Range();
                r.header.stamp = publish_stamp;
                r.radiation_type = 2;
                r.field_of_view = 0;
                r.min_range = 0.2;
                r.max_range = 42;
                r.range = measured_distance_3d;
                elem.second.publish(r);
            }
        }

        // Publish unified UwbRange message
        nexus_swarm_sim::UwbRange uwb_msg;
        uwb_msg.header.stamp = publish_stamp;
        uwb_msg.header.frame_id = "map";
        uwb_msg.src_id = elem.first.ori_node;
        uwb_msg.dst_id = elem.first.end_node;
        uwb_msg.distance_3d = measured_distance_3d;
        uwb_msg.distance_2d = measured_distance_2d;
        uwb_msg.sigma_m = sigma;
        uwb_msg.los = los;
        uwb_msg.rssi = rssi;
        uwb_msg.quality = quality;
        uwb_msg.nlos_bias_mm = nlos_bias_mm;
        uwb_msg.fp_index = 0.0f;
        
        // Generate and attach payload from sender (empty in base system)
        uwb_msg.payload = generate_payload(elem.first.ori_node);

        // Publish to drone-specific topic (only this drone's TX)
        // This ensures each drone's receiver sees only their own transmitted pings
        std::string src_drone = elem.first.ori_node;
        if (drone_tx_publishers_.find(src_drone) != drone_tx_publishers_.end())
        {
            drone_tx_publishers_[src_drone].publish(uwb_msg);
        }
        
        nexus_swarm_sim::RawUWBSignal raw_msg;
        raw_msg.header.stamp = publish_stamp;
        raw_msg.header.frame_id = "map";
        raw_msg.src_id = elem.first.ori_node;
        raw_msg.dst_id = elem.first.end_node;
        raw_msg.status_code = classify_raw_signal_status(snr_db, los, outlier_injected, sts_quality);
        raw_msg.valid = is_raw_signal_valid(raw_msg.status_code);
        raw_msg.toa_ns = toa_ns;
        raw_msg.snr_db = snr_db;
        raw_msg.rssi = rssi;
        raw_msg.channel = 5;  // DW3000 default channel
        raw_msg.prf_mhz = 64;     // DW3000 default PRF
        raw_msg.first_path_power_dbm = fppl;
        raw_msg.fp_index = static_cast<uint16_t>(std::max(0.0f, std::round(std::min(1023.0f, 18.0f + snr_db * 0.6f))));
        raw_msg.sts_quality = sts_quality;
        
        if (drone_raw_signal_publishers_.find(src_drone) != drone_raw_signal_publishers_.end())
        {
            drone_raw_signal_publishers_[src_drone].publish(raw_msg);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_simulator");
    ros::NodeHandle nh;
    UwbSimulator uwb_ranging_sim = UwbSimulator(nh);

    // Create timer for publishing ranges at base frequency
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 100.0), &UwbSimulator::publish_ranges, &uwb_ranging_sim);
    
    ROS_INFO_STREAM("[UWBSim] Simulator running. Press Ctrl+C to exit.");
    ros::spin();

    return 0;
}

std::vector<uint8_t> UwbSimulator::generate_payload(const std::string& src_id)
{
    /*
        Generate empty payload to be transmitted in UWB frame.
        External ROS packages will populate this field with custom data.
        
        This allows flexible payload handling from separate nodes:
        - Custom telemetry packages can subscribe and inject data
        - Algorithm nodes can embed processing results
        - Payload remains modular and decoupled from simulation
        
        Payload format: EMPTY (caller will fill)
        Maximum size: 127 bytes (DW3000 hardware limit)
    */
    
    std::vector<uint8_t> payload;
    // Return empty - external packages will populate
    return payload;
}

void UwbSimulator::generate_raw_signal(double measured_distance_m, double true_distance_m, bool los,
                                       float& toa_ns, float& snr_db, float& rssi)
{
    /*
        Generate raw UWB signal parameters (ToA, SNR, RSSI) from true distance.
        This simulates DW3000 raw measurement output BEFORE signal processing.
        
        DW3000 API provides:
        - dwt_getrangetimestamp(): ToA in nanoseconds
        - dwt_getlastsignalstatistics(): SNR, RSSI, path information
        - dwt_getrxdiag(): Diagnostic data (power levels, etc.)
        
        This function simulates these low-level measurements with realistic noise.
        
        Real system equivalent:
        DW3000 sends raw data → Driver reads → This function's output equivalent
    */
    
    // Speed of light in meters per nanosecond
    const double C = 0.299792458;  // m/ns
    
    // Convert true distance to ToA (round-trip time)
    double measured_toa_ns = 2 * measured_distance_m / C;
    
    // Add ToA measurement noise (DW3000 typical ±1-2ns)
    std::normal_distribution<double> toa_noise_dist(0, 0.8);  // ±0.8ns Gaussian
    double toa_error_ns = toa_noise_dist(rng_);
    
    // ToA might also have systematic bias (rare, but real)
    // double toa_bias_ns = 0.5;  // Constant bias
    
    toa_ns = static_cast<float>(measured_toa_ns + toa_error_ns);
    
    // SNR calculation (path loss + noise)
    // Free space path loss: SNR = Pt - Pl - Pn
    // Simplified DW3000 model:
    //   - LOS: -3dB/meter (typical for UWB)
    //   - NLOS: -4dB/meter (worse propagation)
    
    double path_loss_factor = los ? 3.0 : 4.0;  // dB/meter
    double ref_snr = los ? 20.0 : 15.0;  // dB @ 1m
    double path_loss_db = path_loss_factor * std::log10(std::max(1.0, true_distance_m));
    double mean_snr = ref_snr - path_loss_db;
    
    // Add SNR noise (measurement uncertainty)
    std::normal_distribution<double> snr_noise_dist(0, 1.5);  // ±1.5dB Gaussian
    double snr_error = snr_noise_dist(rng_);
    
    snr_db = static_cast<float>(mean_snr + snr_error);
    snr_db = std::max(-20.0f, std::min(30.0f, snr_db));  // Clamp [-20, 30]
    
    // RSSI (Received Signal Strength Indicator)
    // Related to SNR but also includes absolute power level
    // DW3000 typical range: -120 to -40 dBm
    
    double tx_power = los ? -62.0 : -65.0;  // dBm @ 1m
    double path_loss_linear = 20 * std::log10(true_distance_m + 0.01);
    double mean_rssi = tx_power - path_loss_linear;
    
    // Add RSSI noise
    std::normal_distribution<double> rssi_noise_dist(0, 2.0);  // ±2dBm Gaussian
    double rssi_error = rssi_noise_dist(rng_);
    
    rssi = static_cast<float>(mean_rssi + rssi_error);
    rssi = std::max(-120.0f, std::min(-40.0f, rssi));  // Clamp [-120, -40]
}

float UwbSimulator::estimate_quality(float snr_db, bool los, bool outlier_injected) const
{
    float normalized_snr = std::max(0.0f, std::min(1.0f, (snr_db + 5.0f) / 25.0f));
    float quality = normalized_snr * (los ? 1.0f : 0.78f);
    if (outlier_injected) quality *= 0.65f;
    return std::max(0.0f, std::min(1.0f, quality));
}

float UwbSimulator::estimate_fppl(float rssi_dbm, bool los) const
{
    float first_path_penalty = los ? 2.0f : 6.0f;
    return rssi_dbm - first_path_penalty;
}

float UwbSimulator::estimate_sts_quality(float snr_db, bool los, bool outlier_injected) const
{
    float normalized_snr = std::max(0.0f, std::min(1.0f, (snr_db - 4.0f) / 18.0f));
    float sts_quality = normalized_snr;
    if (!los) sts_quality *= 0.82f;
    if (outlier_injected) sts_quality *= 0.72f;
    return std::max(0.0f, std::min(1.0f, sts_quality));
}

uint8_t UwbSimulator::classify_raw_signal_status(float snr_db, bool los, bool outlier_injected, float sts_quality) const
{
    if (sts_quality < 0.45f) {
        return nexus_swarm_sim::RawUWBSignal::STATUS_INVALID_STS_QUALITY;
    }
    if (outlier_injected) {
        return nexus_swarm_sim::RawUWBSignal::STATUS_DEGRADED_OUTLIER;
    }
    if (snr_db < 8.0f) {
        return nexus_swarm_sim::RawUWBSignal::STATUS_DEGRADED_WEAK_SIGNAL;
    }
    if (!los) {
        return nexus_swarm_sim::RawUWBSignal::STATUS_DEGRADED_NLOS;
    }
    return nexus_swarm_sim::RawUWBSignal::STATUS_OK;
}

bool UwbSimulator::is_raw_signal_valid(uint8_t status_code) const
{
    return status_code != nexus_swarm_sim::RawUWBSignal::STATUS_INVALID_STS_QUALITY;
}
