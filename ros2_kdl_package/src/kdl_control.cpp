#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis();
}


Eigen::VectorXd KDLController::idCntr(
    KDL::Frame &_desPos,
    KDL::Twist &_desVel,
    KDL::Twist &_desAcc,
    double _Kpp, double _Kpo,
    double _Kdp, double _Kdo)
{

        //Calculation of gain matrices 
        //Both matrices are initialized to zero:
        Eigen::Matrix<double, 6, 6> Kp = Eigen::MatrixXd::Zero(6, 6);
        Eigen::Matrix<double, 6, 6> Kd = Eigen::MatrixXd::Zero(6, 6);
        //Update the 3×3 block in the upper left part of Kp: represents linear proportional gains
        Kp.block(0, 0, 3, 3) = _Kpp * Eigen::Matrix3d::Identity();
        //Update the 3×3 block in the lower right part of Kp: represents angular proportional gains 
        Kp.block(3, 3, 3, 3) = _Kpo * Eigen::Matrix3d::Identity();
        //Update the 3×3 block in the upper left part of Kd: represents linear derivative gains.
        Kd.block(0, 0, 3, 3) = _Kdp * Eigen::Matrix3d::Identity();
        //Update 3×3 block in the lower right part of Kd: represents angular derivative gains.
        Kd.block(3, 3, 3, 3) = _Kdo * Eigen::Matrix3d::Identity();

       //Jacobian
        KDL::Jacobian JEE = robot_->getEEJacobian();
        //Identity matrix
        Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
        //Inertia matrix 
        Eigen::Matrix<double, 7, 7> M = robot_->getJsim();
        //Pseudoinverse of the Jacobian: used to map forces from the operational space to the joints.
        Eigen::Matrix<double, 7, 6> Jpinv = pseudoinverse(robot_->getEEJacobian().data);

        //Position:
        //Current position and orientation of the EE
        KDL::Frame cart_pose = robot_->getEEFrame();
        //Desired position of the EE
        Eigen::Vector3d p_d(_desPos.p.data);
        //Current position of the EE
        Eigen::Vector3d p_e(cart_pose.p.data);

        //Desired rotation matrix:
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_d(_desPos.M.data);
        //Current rotation matrix:
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_e(cart_pose.M.data);
        R_d = matrixOrthonormalization(R_d);
        R_e = matrixOrthonormalization(R_e);

        //Velocity:
        //Linear and angular velocity of EE:
        KDL::Twist cart_twist = robot_->getEEVelocity();
        //Desired linear velocity:
        Eigen::Vector3d dot_p_d(_desVel.vel.data);
        //Current linear velocity:
        Eigen::Vector3d dot_p_e(cart_twist.vel.data);
        //Desired angular velocity: 
        Eigen::Vector3d omega_d(_desVel.rot.data);
        //Current angular velocity:
        Eigen::Vector3d omega_e(cart_twist.rot.data);

        //Acceleration:
        Eigen::Matrix<double, 6, 1> dot_dot_x_d;
        //Desired accelerations (linear and angular):
        Eigen::Matrix<double, 3, 1> dot_dot_p_d(_desAcc.vel.data);
        Eigen::Matrix<double, 3, 1> dot_dot_r_d(_desAcc.rot.data);

        //Linear errors:
        Eigen::Matrix<double, 3, 1> e_p = computeLinearError(p_d, p_e);
        Eigen::Matrix<double, 3, 1> dot_e_p = computeLinearError(dot_p_d, dot_p_e);

        //Orientation errors
        Eigen::Matrix<double, 3, 1> e_o = computeOrientationError(R_d, R_e);
        Eigen::Matrix<double, 3, 1> dot_e_o = computeOrientationVelocityError(omega_d, omega_e, R_d, R_e);

        //Construction of states
        //Combine linear (ep) and angular (eo) errors into a single vector: this is the global error in the operational space
        Eigen::Matrix<double, 6, 1> x_tilde;  
        x_tilde << e_p, e_o;
        //Combine the linear (dot_e_p) and angular (dot_e_o) velocity errors into a single vector: representing the time derivative of the error state
        Eigen::Matrix<double, 6, 1> dot_x_tilde;
        dot_x_tilde << dot_e_p, dot_e_o; 

        //Represent the desired acceleration for the end-effector in linear (dot_dot_p_d) and angular motion (dot_dot_r_d)
        dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

        //Inverse Dynamics
        Eigen::Matrix<double, 6, 1> y = dot_dot_x_d - robot_->getEEJacDot().data * robot_->getJntVelocities()
                                        + Kd * dot_x_tilde + Kp * x_tilde;

        //Torques
        return M * (Jpinv * y) + robot_->getCoriolis();

    }