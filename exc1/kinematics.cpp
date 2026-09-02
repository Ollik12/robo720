#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    // So we are given joint angled q1, q2 and q2 with private variables for the lengths
    // Lets calculate the x and y position with joint angles and lengths
    Eigen::Vector2f positionVector;
    float xPos = L1_ * cos(q1) + L2_ * cos(q1 + q2) + L3_ * cos(q1 + q2 + q3);
    float yPos = L1_ * sin(q1) + L2_ * sin(q1 + q2) + L3_ * sin(q1 + q2 + q3);
    positionVector(0) = xPos;
    positionVector(1) = yPos;
    // Then function just returns the position vector with x and y position of the end effector
    return positionVector;
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    // So the initial transformation matrix starts from base and ends in T_base_1 and the base to end effector
    // Is just the product of multiplying thsi

    // From the figure provided in the exercise sheet, from base to joint 1, we have just eye(3) as no change
    // Happens in pos / orient. Apparently we could just use Eigens on identity matrix.
    T_base_1 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;


    // NOTE: I am completely not sure whether this is correct, but results from plots seem to be correct.

    // from the trigonometric functions we get the idea on how x and y position changes here.
    T_1_2 <<  cos(q1), -sin(q1), L1_*cos(q1),
              sin(q1), cos(q1), L1_*sin(q1),
              0, 0, 1;

    // Then with this idea we can construct the other transformation matrices and with << we can 
    // Populate the matrices with values and we dont have to do operations such as T_base_1(0,0) = cos(q1).

    T_2_3 <<  cos(q2), -sin(q2), L2_*cos(q2),
              sin(q2), cos(q2), L2_*sin(q2),
              0, 0, 1;

    T_3_ee << cos(q3), -sin(q3), L3_*cos(q3),
              sin(q3), cos(q3), L3_*sin(q3),
              0, 0, 1;    


    Eigen::Matrix3f T_base_ee = T_base_1 * T_1_2 * T_2_3 * T_3_ee;
    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */

    // chain_ is already initialised as variable KDL::Chain, lets add correct segments to it
    // Segment consists of joint and a frame.
    // ROTz used because we have 2d robot
    KDL::Segment segm1 = KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L1_, 0, 0)));
    chain_.addSegment(segm1);
    KDL::Segment segm2 = KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L2_, 0, 0)));
    chain_.addSegment(segm2);
    KDL::Segment segm3 = KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L3_, 0, 0)));
    chain_.addSegment(segm3);
    // So basically we just add segments in a chain and segment consists of a joint which rotates around z axis
    // And a frame which basically just has a vector with lenght in x-dir.
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */
    // using some Forward kinematics solves, lets calculate the pose of end effector
    KDL::ChainFkSolverPos_recursive fkSolver(chain_);
    // input is joint positions and lets create joint array 
    KDL::JntArray joint_positions(3);
    joint_positions(0) = q1;
    joint_positions(1) = q2;
    joint_positions(2) = q3;
    // we should return KDL frame
    KDL::Frame end_effector_pose;
    // Now we can use the solver to calculate the end effector pose
    int result = fkSolver.JntToCart(joint_positions, end_effector_pose);

    // ALsio lets add a check whether the calculation is successfull or not.
    // this returns < 0 if something went wrong:
    if (result < 0) {
        std::cerr << "Error computing forward kinematics: " << result << std::endl;
    }

    return end_effector_pose;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{

    // Lets use chainiksolverpos_lma defined in the header file  and now we need to compute joint angles for
    // which the end effector will be in target pose
    KDL::ChainIkSolverPos_LMA ikSolver(chain_);
    // Initial values ad joint array
    KDL::JntArray jointArray(3);
    jointArray(0) = q1_init;
    jointArray(1) = q2_init;
    jointArray(2) = q3_init;
    // With this jntArr and the base goal we can use CartToJnt method to calculate
    KDL::JntArray resultArray(3);
    int Result  = ikSolver.CartToJnt(jointArray, target_pose, resultArray);

    // Lets add ssome error detection based oin the enum
    if (Result < 0) {
        std::cerr << "Error computing inverse kinematics: " << Result << std::endl;
    }
    return resultArray;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{

    // Lets use chainjnttojacsolver as it is defined in the header. This will result in joint -> jacobian calc.
    // Lets create joint array as the input
    KDL::JntArray JointArray(3);
    JointArray(0) = q1;
    JointArray(1) = q2;
    JointArray(2) = q3; 

    // Then as the result we will get same sized
    KDL::Jacobian resultJacobian(3);

    // Then we can use the ready function to call:
    KDL::ChainJntToJacSolver JacobianSolver(chain_);
    int result = JacobianSolver.JntToJac(JointArray, resultJacobian, segment_n);

    // Then based on the enum, failures are <0
    if (result < 0){
        std::cerr << "Joint to Jacobian calculation failure" << result << std::endl;
    }

    // There is also a task to implement calculating the jacobian of other frame than end effector
    // As a input variable to tha solver, there is the segment number and it seems to be -1 which would
    // Logically be the last element of and cpp list-type datastructure. So if we just call 

    // int result = JacobianSolver.JntToJac(JointArray, resultJacobian, 2); we can calculate the jacobian for different than end effector
    
    // Then lets return the result jacobian:
    return resultJacobian;
}
