#include "ex6/potential_field_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex6 {


controller_interface::return_type PotentialFieldController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Read the goal pose
    xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();

    /**
     * Implement attractive force/velocity
     */

    // Current pose of the end-effector using forward kinematics
    int fk_ok = solver_->computeFK(q_kdl_, x_kdl_);

    // Error handling for forward kinematics computation
    if (fk_ok < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Forward kinematics computation failed");
        return controller_interface::return_type::ERROR;
    } 

    // Compute the postion error between the current pose and the desired pose
    xe_kdl_ = KDL::diff(x_kdl_, xd_kdl_);

    // Get dynamics parameters and Jacobian
    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
    solver_->compute_jac(q_kdl_, J_kdl_);

    // Copy data to Eigen variables for matrix computations
    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    J_ = J_kdl_.data;
    q_ = q_kdl_.data;
    q_dot_ = q_dot_kdl_.data;

    xe_(0) = xe_kdl_.vel.x(); xe_(1) = xe_kdl_.vel.y(); xe_(2) = xe_kdl_.vel.z();
    xe_(3) = xe_kdl_.rot.x(); xe_(4) = xe_kdl_.rot.y(); xe_(5) = xe_kdl_.rot.z();

    // Jacobian pseudo-inverse
    solver_->get_damped_pseudo_inverse(J_, J_pinv_);

    // Desired end effector velocity
    xd_dot_ = Kp_.cwiseProduct(xe_);

    // Joint velocity command
    q_dot_cmd_ = J_pinv_ * xd_dot_;

    // Joint space PD + computed torque to produce attractive torque
    e_dot_ = q_dot_cmd_ - q_dot_;
    u_ = Kd_.cwiseProduct(e_dot_);
    tau_attr_ = M_ * u_ + G_ + C_;
     
    /**
     * Joint limits as APF
     */

    // Reset repulsive torque to zero before computing contributions from joint limits and obstacles
    tau_rep_.setZero();

    // Compute repulsive torque for joint limits
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Get the current joint position and limits
        const double q = q_(i);
        const double lower = joint_limits_[i].lower;
        const double upper = joint_limits_[i].upper;

        const double d1 = q - lower;
        const double d2 = q - upper;

        // Gradients of squared distances
        const double grad_lower = 2.0 * d1;
        const double grad_upper = 2.0 * d2;

        const bool moving_towards_lower = 2.0 * grad_lower * q_dot_(i) < 0.0;
        const bool moving_towards_upper = 2.0 * grad_upper * q_dot_(i) < 0.0;

        // Avoid division by zero by ensuring distances are not too small
        const double D1 = std::max(D_min, d1 * d1);
        const double D2 = std::max(D_min, d2 * d2);

        // Initialize repulsive forces for lower and upper limits
        double F_lower = 0.0;
        double F_upper = 0.0;

        // Lower limit repulsive force
        if (D1 <= Q_star_joint_ && moving_towards_lower) {
            F_lower = K_rep_joint_ * (1.0 / D1 - 1.0 / Q_star_joint_) * (2.0 * d1 / (D1 * D1));
        }

        // Upper limit repulsive force
        if (D2 <= Q_star_joint_ && moving_towards_upper) {
            F_upper = K_rep_joint_ * (1.0 / D2 - 1.0 / Q_star_joint_) * (2.0 * d2 / (D2 * D2));
        }

        // Compute the repulsive force and store it in the repulsive torque vector
        tau_rep_(i) = F_lower + F_upper - B_rep_ * q_dot_(i);
    }

    /**
     * Obstacle avoidance with APF
     */

     // Get the avoidance points from helper function
    std::vector<AvoidancePoint> avoidance_points = getAvoidancePoints();

    // Read the obstacle info
    const auto& obstacles = *rt_obstacles_.readFromRT();

    // Compute repulsive torque for every joint and obstacles
    for (const auto& point : avoidance_points) {

        // Get the position and Jacobian of the avoidance point in 3D space
        Eigen::Vector3d p_point = point.position;
        Eigen::Matrix<double, 3, NUM_JOINTS> J_p = point.jacobian;

        for (const auto& obs : obstacles) {
            Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

            // Spherical obstacle
            if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
                // Sphere radius
                double obs_r = obs.radius;

                // Vector from obstacle center to the point
                Eigen::Vector3d diff = p_point - p_obs;

                // Closest point on sphere surface
                p0_ = p_obs + obs_r * diff.normalized();

                // Vector from closest point to the point
                Eigen::Vector3d surface_diff = p_point - p0_;

                // distance to obstacle surface
                double D_obs = surface_diff.dot(surface_diff);

                // Gradient
                Vector7d gradD = 2.0 * J_p.transpose() * surface_diff;

                if (D_obs < Q_star_obs_) {

                    // Repulsive torque
                    Vector7d tau_obs = K_rep_obs_ * (1.0 / D_obs - 1.0 / Q_star_obs_) * (1.0 / (D_obs * D_obs)) * gradD;

                    tau_rep_ += tau_obs;
                }
            }

            // Box obstacle (such as wall)
            else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
                // Box dimensions vector
                Eigen::Vector3d box_size(obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));

                // Point position relative to box center
                Eigen::Vector3d p_rel = p_point - p_obs;

                // Box half dimensions
                Eigen::Vector3d half_size = box_size / 2.0;

                // Closest point on the box (relative to its center)
                Eigen::Vector3d p0_rel = p_rel.cwiseMax(-half_size).cwiseMin(half_size);

                // Closest point in world coordinates
                Eigen::Vector3d p0 = p_obs + p0_rel;

                // Vector from closest point to the point
                Eigen::Vector3d diff = p_point - p0;

                // Squared distance to box
                double D_obs = diff.dot(diff);

                // Gradient
                Vector7d gradD = 2.0 * J_p.transpose() * diff;      

                if (D_obs < Q_star_obs_) {

                    // Repulsive torque
                    Vector7d tau_obs = K_rep_obs_ * (1.0 / D_obs - 1.0 / Q_star_obs_) * (1.0 / (D_obs * D_obs)) * gradD;

                    tau_rep_ += tau_obs;
                }
            }
        }
    }

    /**
     *  Final torque command
     */
    tau_ = tau_attr_ + tau_rep_;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }

    publish_markers();
    publish_avoidance_markers(avoidance_points);

    return controller_interface::return_type::OK;
}

std::vector<PotentialFieldController::AvoidancePoint> PotentialFieldController::getAvoidancePoints() {
    std::vector<AvoidancePoint> avoidance_points;
    
    for (unsigned int i = 1; i < chain_.getNrOfSegments(); ++i) {

        KDL::Frame frame;
        KDL::Jacobian jacobian(NUM_JOINTS);

        // KDL segment number is 1-based
        unsigned int segment_nr = i + 1;

        solver_->computeFK(q_kdl_, frame, segment_nr);
        solver_->compute_jac(q_kdl_, jacobian, segment_nr);

        AvoidancePoint point;

        // Store the position of the segment in 3D space
        point.position = Eigen::Vector3d(
            frame.p.x(),
            frame.p.y(),
            frame.p.z()
        );

        // Store the first three rows of the Jacobian
        point.jacobian = jacobian.data.topRows<3>();

        avoidance_points.push_back(point);
    }

    return avoidance_points;
}

CallbackReturn PotentialFieldController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);

        // Read the joint gains from the parameters
        p_gains_ = auto_declare<std::vector<double>>("p_gains", p_gains_);
        d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            Kd_(i) = d_gains_.at(i);
        }
        for (std::size_t i = 0; i < NUM_TASK; ++i) {
            Kp_(i) = p_gains_.at(i);
        }
    } 

    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PotentialFieldController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration started");

    // Get the robot description parameter (effort_fr3_arm.urdf.xacro parsed into one URDF as a string)
    // Due to AsyncParametersClient, this has to be in on_configure() method.
    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();

    // Check if robot description was retrieved succesfully
    if (!result.empty()) {
        robot_description_ = result[0].value_to_string();
    }
    else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
        return CallbackReturn::ERROR;
    }

    // Parse URDF string into URDF object
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
    }

    // Get the joint angle limits from the URDF
    for (std::size_t i = 0; i < NUM_JOINTS; i++) {
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf) {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }

        // Read the joint limits from the URDF and store them into array
        joint_limits_[i] = {
            joint_urdf->limits->lower,
            joint_urdf->limits->upper
        };
    }

    // Construct a KDL tree object from the URDF
    if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct a KDL tree.");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Constructed a KDL tree.");
    }

    // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, tip_link_, chain_)) {
        kdl_chain_error_print();

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print();
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // Create a subscription for the joint command
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&PotentialFieldController::goal_pose_callback,
        this, std::placeholders::_1));

    init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local(), std::bind(&PotentialFieldController::init_obstacle_callback,
        this, std::placeholders::_1));

    sub_obj_pose_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "world/obstacle_world/pose/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&PotentialFieldController::object_pose_callback, this, std::placeholders::_1));

    // Publisher
    pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1000);
    pub_avoidance_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("avoidance_markers", 1000);

    pub_lower_limits_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "joint_limits/lower", 10);

    pub_upper_limits_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "joint_limits/upper", 10);

    // Start a timer to publish joint limits at a fixed rate (every 100 ms)
    joint_limits_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            publish_joint_limits();
        }
    );

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PotentialFieldController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : command_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
PotentialFieldController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn PotentialFieldController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);
    M_kdl_.resize(NUM_JOINTS);
    G_kdl_.resize(NUM_JOINTS);
    C_kdl_.resize(NUM_JOINTS);
    J_kdl_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(0.5, 0.0, 1.6);
    xd->M = KDL::Rotation::EulerZYX(0.0, 0.0, M_PI);
    rt_goal_pose_buffer_.writeFromNonRT(xd);

    // Activate publishers
    pub_markers_->on_activate();
    pub_avoidance_markers_->on_activate();
    pub_lower_limits_->on_activate();
    pub_upper_limits_->on_activate();

    return CallbackReturn::SUCCESS;
}

void PotentialFieldController::updateJointStates() {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
        return;
    }

    // Get the current joint positions
    auto* interfaces = state_interfaces_.data();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Access interfaces directly with pointer arithmetic
        const auto& position_interface = interfaces[2*i];
        const auto& velocity_interface = interfaces[2*i + 1];

        // Interface name comparison
        const auto& pos_name = position_interface.get_interface_name();
        if (pos_name != "position") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                         pos_name.c_str());
            return;
        }
        const auto& vel_name = velocity_interface.get_interface_name();
        if (vel_name != "velocity") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s",
                         vel_name.c_str());
            return;
        }
        
        // Direct value assignment
        position_interface_vals_(i) = position_interface.get_value();
        velocity_interface_vals_(i) = velocity_interface.get_value();
    }
}

// Joint command subscriber callback
void PotentialFieldController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    auto xd = std::make_shared<KDL::Frame>();

    xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    xd->M = KDL::Rotation::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    rt_goal_pose_buffer_.writeFromNonRT(xd);
}

void PotentialFieldController::init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg) {
    obstacle_map_.clear(); // In case a new scene was loaded, otherwise no new messages will be published
    std::vector<custom_msgs::msg::Obstacle> obs_vec;
    for (auto obs : msg.obstacles) {
        obstacle_map_.emplace(obs.name, obs);
        obs_vec.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(obs_vec);
}

void PotentialFieldController::object_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
    for (geometry_msgs::msg::TransformStamped t : msg.transforms) {
        auto it = obstacle_map_.find(t.child_frame_id);
        if (it != obstacle_map_.end()) {
            it->second.position.x = t.transform.translation.x;
            it->second.position.y = t.transform.translation.y;
            it->second.position.z = t.transform.translation.z;
            it->second.orientation = t.transform.rotation;
        }
    }

    // Build a fresh snapshot vector to write to rt buffer
    std::vector<custom_msgs::msg::Obstacle> snapshot;
    snapshot.reserve(obstacle_map_.size());
    for (const auto& [name, obs] : obstacle_map_) {
        snapshot.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(snapshot);
}

void PotentialFieldController::publish_markers() {
    visualization_msgs::msg::MarkerArray msg;

    const auto& obstacles = *rt_obstacles_.readFromRT();
    int i = 0;
    for (const auto& obs : obstacles){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = "";
        marker.id = i++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = obs.position;
        marker.pose.orientation = obs.orientation;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 2 * obs.radius;
            marker.scale.y = 2 * obs.radius;
            marker.scale.z = 2 * obs.radius;
        }

        else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obs.box_size.at(0);
            marker.scale.y = obs.box_size.at(1);
            marker.scale.z = obs.box_size.at(2);
        }

        // Push back to marker array
        msg.markers.push_back(marker);
    }

    pub_markers_->publish(msg);
}

// Debug print functions to reduce clutter in controller code
void PotentialFieldController::kdl_chain_error_print() {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link_);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void PotentialFieldController::publish_avoidance_markers(
    const std::vector<AvoidancePoint>& avoidance_points)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < avoidance_points.size(); ++i) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base";  // KDL base frame
        marker.header.stamp = get_node()->get_clock()->now();

        marker.ns = "avoidance_points";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = avoidance_points[i].position.x();
        marker.pose.position.y = avoidance_points[i].position.y();
        marker.pose.position.z = avoidance_points[i].position.z();
        marker.pose.orientation.w = 1.0;

        // Scale (size of the sphere) to visualize the avoidance radius
        double avoidance_radius = std::sqrt(Q_star_obs_);
        
        marker.scale.x = 2.0 * avoidance_radius;
        marker.scale.y = 2.0 * avoidance_radius;
        marker.scale.z = 2.0 * avoidance_radius;

        // Blue color
        marker.color.r = 0.0;
        marker.color.g = 0.3;
        marker.color.b = 1.0;
        marker.color.a = 0.2;

        marker_array.markers.push_back(marker);
    }

    pub_avoidance_markers_->publish(marker_array);
}

void PotentialFieldController::publish_joint_limits()
{
    std_msgs::msg::Float64MultiArray lower_msg;
    std_msgs::msg::Float64MultiArray upper_msg;

    for (const auto& limit : joint_limits_) {
        lower_msg.data.push_back(limit.lower);
        upper_msg.data.push_back(limit.upper);
    }

    pub_lower_limits_->publish(lower_msg);
    pub_upper_limits_->publish(upper_msg);
}

void PotentialFieldController::kdl_chain_success_print() {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
    RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex6
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex6::PotentialFieldController,
                       controller_interface::ControllerInterface)