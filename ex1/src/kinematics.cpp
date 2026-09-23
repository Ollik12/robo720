#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    Eigen::Vector2f ee_pos;

    // Compute the end effectorx and y position using trigonometric functions
    float x = L1_ * cos(q1) + L2_ * cos(q1 + q2) + L3_ * cos(q1 + q2 + q3);
    float y = L1_ * sin(q1) + L2_ * sin(q1 + q2) + L3_ * sin(q1 + q2 + q3);

    ee_pos << x, y;

    return ee_pos;
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    T_base_1 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;


    T_1_2 << cos(q1), -sin(q1), L1_ * cos(q1),
              sin(q1),  cos(q1), L1_ * sin(q1),
              0,        0,       1;

    T_2_3 << cos(q2), -sin(q2), L2_ * cos(q2),
              sin(q2),  cos(q2), L2_ * sin(q2),
              0,        0,       1;

    T_3_ee << cos(q3), -sin(q3), L3_ * cos(q3),
              sin(q3),  cos(q3), L3_ * sin(q3),
              0,        0,       1;

    Eigen::Matrix3f T_base_ee = T_base_1 * T_1_2 * T_2_3 * T_3_ee;

    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    // Add chain segments, rotation joints around z-axis and translation along x-axis
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L1_, 0.0, 0.0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L2_, 0.0, 0.0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L3_, 0.0, 0.0))));
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    // Initialize KDL Frame object to store the end effector pose
    KDL::Frame ee_pose;

    // Solve forward kinematics using KDL FK solver
    KDL::ChainFkSolverPos_recursive solver(chain_);

    // Array for storing the joint positions
    KDL::JntArray joints(3);
    joints(0) = q1;
    joints(1) = q2;
    joints(2) = q3;

    solver.JntToCart(joints, ee_pose);

    return ee_pose;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
     // Array for storing the joint positions
    KDL::JntArray q_out(3);

    KDL::ChainIkSolverPos_LMA solver(chain_);

    // Array for storing the initial joint positions
    KDL::JntArray q_init(3);
    q_init(0) = q1_init;
    q_init(1) = q2_init;
    q_init(2) = q3_init;

    solver.CartToJnt(q_init, target_pose, q_out);

    return q_out;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    // Initalize jacobian object with 3 columns for 3 joints
    KDL::Jacobian jac_kdl(3);

    KDL::ChainJntToJacSolver solver(chain_);
    KDL::JntArray joints(3);
    joints(0) = q1;
    joints(1) = q2;
    joints(2) = q3;

    solver.JntToJac(joints, jac_kdl, segment_n);

    /**
     *  Replace the segment_n parameter with hard coded value to compute Jacobian 
     *  for e.g. the frame 2. main.cpp calls this with segment_n = -1, which means the Jacobian is computed for the end effector frame.
     */
    // solver.JntToJac(joints, jac_kdl, 2);

    return jac_kdl;
}
