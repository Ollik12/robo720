#include "ex5/visual_controller.hpp"
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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <urdf/model.h>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex5 {


controller_interface::return_type VisualController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();
    const auto& obstacles = *rt_obstacles_.readFromRT();

    const double dt = period.seconds();
    elapsed_time_ = elapsed_time_ + dt;
    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }
    initial_timeout_ = initial_timeout_ + dt;
    // Get the current EE pose
    solver_->computeFK(q_kdl_, x_kdl_);

    // Load the markerpose boolean to decide whether to follow the wave trajectory or track the marker pose
    const bool marker_received = marker_pose_received_.load();

    if (!use_marker_top_pose_.load()) {
        marker_tracking_active_ = false;
        if (goal_pose_received_.load()) {
            const auto goal = *rt_goal_pose_buffer_.readFromRT();
            xd_kdl_.p = KDL::Vector(
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
            xd_kdl_.M = KDL::Rotation::Quaternion(
                goal->pose.orientation.x, goal->pose.orientation.y,
                goal->pose.orientation.z, goal->pose.orientation.w);
        } else {
            xd_kdl_ = x_kdl_;
        }
    } else if (marker_received) {
        marker_lost_time_ = 0.0;
        if (initial_timeout_ > 1.0) {
            // Read the marker pose
            T_cam_marker_ = *rt_marker_pose_buffer_.readFromRT()->get();

            // As the marker pose is in camera optical frame, we need to transform it to the robot base frame
            // but first feature is extracted so lets get frame from base to camera optical frame as seen in lecture slides

            // now we calculate forwkinematics for the camera
            camera_chain_fk_solver_->computeFK(q_kdl_, T_base_cam);

            // Now we have frames from base to camera and from camera to marker,
            // we can compute the frame from base to marker by multiplying gotten frames
            KDL::Frame T_Base_Marker = T_base_cam * T_cam_marker_;

            // Lower the TCP slightly so the fingers can reach around the cube.
            xd_kdl_.p = T_Base_Marker.p +
                T_Base_Marker.M * KDL::Vector(0.0, -0.01, 0.0);
            xd_kdl_.p.z(xd_kdl_.p.z() + marker_approach_height_);

            // NOTE: end effector orientation is rotated by PI compared to the aruco marker
            // so let's rotate the marker orientation by PI around X axis to align with the end effector orientation
            xd_kdl_.M = T_Base_Marker.M * KDL::Rotation::RotX(M_PI);
            marker_tracking_active_ = true;
        }
    } else if (!marker_tracking_active_) {
        // Hold the goal or current pose until the first marker detection.
        if (goal_pose_received_.load()) {
            const auto goal = *rt_goal_pose_buffer_.readFromRT();
            xd_kdl_.p = KDL::Vector(
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
            xd_kdl_.M = KDL::Rotation::Quaternion(
                goal->pose.orientation.x, goal->pose.orientation.y,
                goal->pose.orientation.z, goal->pose.orientation.w);
        } else {
            xd_kdl_ = x_kdl_;
        }
    }

    // Gripping is done via terminal

    xe_kdl_ = KDL::diff(x_kdl_, xd_kdl_);

    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
    solver_->compute_jac(q_kdl_, J_kdl_);

    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    J_ = J_kdl_.data;
    q_ = q_kdl_.data;
    q_dot_ = q_dot_kdl_.data;

    xe_(0) = xe_kdl_.vel.x(); xe_(1) = xe_kdl_.vel.y(); xe_(2) = xe_kdl_.vel.z();
    xe_(3) = xe_kdl_.rot.x(); xe_(4) = xe_kdl_.rot.y(); xe_(5) = xe_kdl_.rot.z();

    solver_->get_damped_pseudo_inverse(J_, J_pinv_);
    xd_dot_ = Kp_.cwiseProduct(xe_);
    q_dot_cmd_ = J_pinv_ * xd_dot_;
    e_dot_ = q_dot_cmd_ - q_dot_;
    u_ = Kd_.cwiseProduct(e_dot_);
    tau_att_ = M_ * u_ + G_ + C_;

    // Joint limits as artificial potential field
    // We need two repulsive functions as functions of 
    // D1(q) = (q − am)^2 and D2(q) = (q − aM)^2
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // check wheter we are moving toward lower or upper in joint limits
        double D1, D2, F1, F2;
        double gradD1 = 2.0 * (position_interface_vals_(i) - lower_joint_limits_(i));
        double gradD2 = 2.0 * (position_interface_vals_(i) - upper_joint_limits_(i));
        bool moving_to_lower = gradD1 * velocity_interface_vals_(i) < 0.0;
        bool moving_to_upper = gradD2 * velocity_interface_vals_(i) < 0.0;
        // lets calculate the forcefield for joint 1 for ex.
        // F,1m = k*((1)/(q1-am)^2-(1)/(Q_star))*(((1)/(q1-am)^2)^2)*[2(q1-am)]
        // F,1M = k*((1)/(q1-aM)^2-(1)/(Q_star))*(((1)/(q1-aM)^2)^2)*[2(q1-aM)]
        // F_total = SUM(F,1m + F,1M)
        D1 = (position_interface_vals_(i) - lower_joint_limits_(i)) * (position_interface_vals_(i) - lower_joint_limits_(i));
        D2 = (position_interface_vals_(i) - upper_joint_limits_(i)) * (position_interface_vals_(i) - upper_joint_limits_(i));
        // guard against division by zero
        if (D1 < 1e-6) {
            D1 = 1e-6;
        }
        if (D2 < 1e-6) {
            D2 = 1e-6;
        }
        // then we can calculate
        if (D1 <= Q_star && moving_to_lower) {
            F1 = k_rep * ((1.0 / D1) - (1.0 / Q_star)) * (1.0 / (D1 * D1)) * (gradD1);
        }
        else {
            F1 = 0.0;
        }
        if (D2 <= Q_star && moving_to_upper) {
            F2 = k_rep * ((1.0 / D2) - (1.0 / Q_star)) * (1.0 / (D2 * D2)) * (gradD2);
        }
        else {
            F2 = 0.0;
        }
        // then we can sum the forces and add damping term for total repulsive torque
        tau_rep_(i) = F1 + F2 - B_rep * velocity_interface_vals_(i);

    }
    for (const auto& obs : obstacles) {
        if (obs.name != "obstacle_w1" && obs.name != "obstacle_w2") {
            // only take the walls
            continue;
        }

        Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

        if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            Eigen::Vector3d box_size(
                obs.box_size.at(0), obs.box_size.at(1), obs.box_size.at(2));
              for (int segm_nr = 1;
                  segm_nr <= static_cast<int>(chain_.getNrOfSegments()); ++segm_nr) {
                KDL::Frame ee_frame;
                solver_->computeFK(q_kdl_, ee_frame, segm_nr);
                Eigen::Vector3d p(
                    ee_frame.p.x(), ee_frame.p.y(), ee_frame.p.z());
                Eigen::Vector3d p_local = p - p_obs;
                Eigen::Vector3d half_size = box_size / 2.0;
                Eigen::Vector3d nearest;
                for (int i = 0; i < 3; ++i) {
                    nearest(i) = std::clamp(p_local(i), -half_size(i), half_size(i));
                }
                Eigen::Vector3d nearest_world = nearest + p_obs;

                Eigen::Vector3d diff = p - nearest_world;
                double D = diff.dot(diff);
                if (D < 1e-6) {
                    D = 1e-6;
                }

                KDL::Jacobian jac(NUM_JOINTS);
                solver_->compute_jac(q_kdl_, jac, segm_nr);
                Eigen::Matrix<double, 3, 7> Jp = jac.data.topRows<3>();
                Vector7d gradD = 2.0 * Jp.transpose() * diff;

                if (D < Q_star) {
                    Vector7d F_obs = k_rep * (1.0 / D - 1.0 / Q_star) *
                        (1.0 / (D * D)) * gradD;
                    tau_rep_ += F_obs;
                }
            }
        }
    }

    tau_ = tau_att_ + tau_rep_; // TODO: add repulsive control for obstacle avoidance from task space into here aswell!

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }
    // publish for RViz visualization
    publish_markers(obstacles);
    publish_marker_tf();

    return controller_interface::return_type::OK;
}

CallbackReturn VisualController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);
        // for camera chain lets create camera_link
        optical_link_ = auto_declare<std::string>("optical_link", optical_link_);

        p_gains_ = auto_declare<std::vector<double>>("p_gains", p_gains_);
        d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
        marker_approach_height_ = auto_declare<double>(
            "marker_approach_height", marker_approach_height_);
        // Parameter to switch between two modes: follow wave trajectory or track marker pose
        use_marker_top_pose_.store(auto_declare<bool>("use_marker_top_pose", false));

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

CallbackReturn VisualController::on_configure(
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
        // joint limits in datastructure from urdf file
        lower_joint_limits_(i) = joint_urdf->limits->lower;
        upper_joint_limits_(i) = joint_urdf->limits->upper;
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
        kdl_chain_error_print(tip_link_);

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print(tip_link_, chain_);
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // lets create new chain instance from base to camera
        // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, optical_link_, camera_chain_)) {
        kdl_chain_error_print(optical_link_);
        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print(optical_link_, camera_chain_);
    }
    // Lets also create a solver for this new chain as the chain_ is from base to End effector
    // this solver is from base to camera optical frame
    camera_chain_fk_solver_.reset(new Solver(camera_chain_));


    // Parameter callback for debugging
    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
        if (param.get_name() == "use_marker_top_pose") {
            use_marker_top_pose_.store(param.as_bool());
            RCLCPP_INFO(get_node()->get_logger(), "use_marker_top_pose_ set to %s",
                        param.as_bool() ? "true" : "false");
        }
        }
        return result;
    });


    // Create a subscription for the joint command
    marker_pose_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "marker_pose", rclcpp::SystemDefaultsQoS(), std::bind(&VisualController::marker_pose_callback,
        this, std::placeholders::_1));

    // subscriber for task-space commands [x y z r p y]
    trajectory_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "command", rclcpp::SystemDefaultsQoS(), std::bind(&VisualController::trajectory_callback,
        this, std::placeholders::_1));

    // Subscribe to task-space goals
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&VisualController::goal_pose_callback,
        this, std::placeholders::_1));

    init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local(),
        std::bind(&VisualController::init_obstacle_callback, this, std::placeholders::_1));

    obstacle_pose_subscriber_ = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "world/cam_scene/pose/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&VisualController::obstacle_pose_callback, this, std::placeholders::_1));

    // Transform broadcaster for debugging
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
    pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
        "markers", 1000);

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
VisualController::command_interface_configuration() const {
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
VisualController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn VisualController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;
    initial_timeout_ = 0.0;
    marker_lost_time_ = 0.0;
    marker_tracking_active_ = false;

    M_kdl_.resize(NUM_JOINTS);
    G_kdl_.resize(NUM_JOINTS);
    C_kdl_.resize(NUM_JOINTS);
    J_kdl_.resize(NUM_JOINTS);
    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
    }

    T_cam_marker_ = KDL::Frame::Identity();
    T_base_cam = KDL::Frame::Identity();

    const KDL::Frame initial_goal_pose(
        KDL::Rotation::RPY(M_PI, 0.0, 0.0),
        KDL::Vector(0.7, 0.0, 1.5));

    auto initial_goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    initial_goal->header.frame_id = root_link_;
    initial_goal->pose.position.x = initial_goal_pose.p.x();
    initial_goal->pose.position.y = initial_goal_pose.p.y();
    initial_goal->pose.position.z = initial_goal_pose.p.z();
    initial_goal_pose.M.GetQuaternion(
        initial_goal->pose.orientation.x,
        initial_goal->pose.orientation.y,
        initial_goal->pose.orientation.z,
        initial_goal->pose.orientation.w);
    xd_kdl_ = initial_goal_pose;
    goal_pose_received_.store(true);
    rt_goal_pose_buffer_.writeFromNonRT(initial_goal);
    pub_markers_->on_activate();

    return CallbackReturn::SUCCESS;
}

void VisualController::updateJointStates() {
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

void VisualController::trajectory_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(msg->data.at(0), msg->data.at(1), msg->data.at(2));
    xd->M = KDL::Rotation::RPY(msg->data.at(3), msg->data.at(4), msg->data.at(5));

    rt_trajectory_buffer_.writeFromNonRT(xd);
}

void VisualController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    rt_goal_pose_buffer_.writeFromNonRT(msg);
    goal_pose_received_.store(true);
}

void VisualController::init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg) {
    obstacle_map_.clear();
    std::vector<custom_msgs::msg::Obstacle> obstacles;
    obstacles.reserve(msg.obstacles.size());
    for (const auto& obstacle : msg.obstacles) {
        obstacle_map_.emplace(obstacle.name, obstacle);
        obstacles.push_back(obstacle);
    }

    rt_obstacles_.writeFromNonRT(obstacles);
}

void VisualController::obstacle_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
    for (const auto& transform : msg.transforms) {
        const auto obstacle = obstacle_map_.find(transform.child_frame_id);
        if (obstacle == obstacle_map_.end()) {
            continue;
        }

        obstacle->second.position.x = transform.transform.translation.x;
        obstacle->second.position.y = transform.transform.translation.y;
        obstacle->second.position.z = transform.transform.translation.z;
        obstacle->second.orientation = transform.transform.rotation;
    }

    std::vector<custom_msgs::msg::Obstacle> obstacles;
    obstacles.reserve(obstacle_map_.size());
    for (const auto& [name, obstacle] : obstacle_map_) {
        obstacles.push_back(obstacle);
    }

    rt_obstacles_.writeFromNonRT(obstacles);
}

// Marker pose in camera optical frame
void VisualController::marker_pose_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    if (msg->data.size() >= 12) {
        double xx, xy, xz, yx, yy, yz, zx, zy, zz, tx, ty, tz;
        xx = msg->data[0], xy = msg->data[1];  xz = msg->data[2];
        yx = msg->data[3], yy = msg->data[4];  yz = msg->data[5];
        zx = msg->data[6], zy = msg->data[7];  zz = msg->data[8];
        tx = msg->data[9], ty = msg->data[10]; tz = msg->data[11];

        auto T_cam_marker = std::make_shared<KDL::Frame>(
            KDL::Frame(KDL::Rotation(xx, xy, xz, yx, yy, yz, zx, zy, zz), KDL::Vector(tx, ty, tz)));
        rt_marker_pose_buffer_.writeFromNonRT(T_cam_marker);
        // Set the flag to true when a marker pose message is received

        marker_pose_received_.store(true);
    }
    else {
        // lets follow wave trajectory for atleast 1 second if marker not detected
        
        // If we dont see the wave, lets go back to following the wave trajectory
        marker_pose_received_.store(false);
    }
}

void VisualController::publish_marker_tf() {
    geometry_msgs::msg::TransformStamped t;

    KDL::Frame f = xd_kdl_;

    t.header.stamp = get_node()->get_clock()->now();
    t.header.frame_id = root_link_;
    t.child_frame_id = "goal_pose";

    t.transform.translation.x = f.p.x();
    t.transform.translation.y = f.p.y();
    t.transform.translation.z = f.p.z();

    double qx, qy, qz, qw;
    f.M.GetQuaternion(qx, qy, qz, qw);
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;

    tf_broadcaster_->sendTransform(t);
}

void VisualController::publish_markers(
    const std::vector<custom_msgs::msg::Obstacle>& obstacles) {
    visualization_msgs::msg::MarkerArray markers;

    int id = 0;
    for (const auto& obstacle : obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = "environment";
        marker.id = id++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = obstacle.position;
        marker.pose.orientation = obstacle.orientation;
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        if (obstacle.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 2.0 * obstacle.radius;
            marker.scale.y = 2.0 * obstacle.radius;
            marker.scale.z = 2.0 * obstacle.radius;
        } else if (obstacle.shape_type == custom_msgs::msg::Obstacle::BOX) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obstacle.box_size.at(0);
            marker.scale.y = obstacle.box_size.at(1);
            marker.scale.z = obstacle.box_size.at(2);
        } else {
            continue;
        }

        markers.markers.push_back(marker);
    }

    pub_markers_->publish(markers);
}

// Debug print functions to reduce clutter in controller code
void VisualController::kdl_chain_error_print(const std::string& tip_link) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void VisualController::kdl_chain_success_print(const std::string& tip_link, const KDL::Chain& chain) {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link.c_str());
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
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex5
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex5::VisualController,
                       controller_interface::ControllerInterface)