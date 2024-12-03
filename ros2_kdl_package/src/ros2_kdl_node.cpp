// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>


#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
 
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
 
class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"),
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity or effort)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
 
            if (!(cmd_interface_ == "position" || "velocity" || "effort" || "effort_cartesian"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            // std::cout<<"INSERISCI kpp: ";
            // std::cin>>_Kpp;
            // _Kpo=_Kpp;
             
            // // std::cout<<"INSERISCI kd: ";
            // // std::cin>>_Kdp;
            // _Kdp=2*sqrt(_Kpp);
            // _Kdo=_Kdp;

 
            // Parameters added in the constructor
            declare_parameter("time_law", "trapezoidal");  // Values: "trapezoidal" or "cubic"
            declare_parameter("path_type", "linear");      // Values: "linear" or "circular"
 
            get_parameter("time_law", time_law_);   
            get_parameter("path_type", path_type_);
 
            RCLCPP_INFO(get_logger(),"Current time_law is: '%s'", time_law_.c_str());
 
            if (!(time_law_ == "trapezoidal" || time_law_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected time_law is not valid!"); return;
            }
 
            RCLCPP_INFO(get_logger(),"Current path_type is: '%s'", path_type_.c_str());
 
            if (!(path_type_ == "linear" || path_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected path_type_ is not valid!"); return;
            }
 
            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false;
 
            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});
 
            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj);
            joint_velocities_.resize(nj);
            joint_acceleration_d_.resize(nj);
            joint_velocity_old.resize(nj);
            torque_values.resize(nj);
            q_des.resize(nj);
            dq_des.resize(nj);

 
            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));
 
            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }
 
            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;
 
            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl;
            // std::cout << q.data <<std::endl;
 
            // Initialize controller
            KDLController controller_(*robot_);
 
            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) );
 
            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1]+0.2, init_position[2];

            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, trajRadius = 0.2;
 
            //planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
            
            //Example of planner configuration with a trajectory type
            if (path_type_ == "linear") {
 
                if(time_law_ == "cubic"){
                    planner_ = KDLPlanner(traj_duration, init_position, end_position);
                }
                else{
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
                }
            
            } else if (path_type_ == "circular") {
 
                if(time_law_ == "cubic"){
                    planner_ = KDLPlanner(traj_duration, init_position, trajRadius);
                }
                else{
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, trajRadius);
                }
            }
            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);
 
            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }else if(cmd_interface_ == "effort" || "effort_cartesian"){
                // Create cmd publisher
                
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                for (long int i = 0; i < nj; ++i) {
                    desired_commands_[i] = 0;
                    
                }
 
            }else{
 
                std::cout<<"Error!";
 
            }
          
 
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
           
            
 
            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }
 
    private:
 
        void cmd_publisher(){
            iteration_ = iteration_ + 1;
            // Define trajectory
            double total_time = 1.5; //
            int trajectory_len = 150; //
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
 
            if (t_ < total_time){
                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);
 
                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           
 
                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos);
 
                // Compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;
    
                KDLController controller_(*robot_);
 
 
                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt;
 
                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
                else if(cmd_interface_ == "effort"){
                    
                    //Store the current joint velocity as a reference to calculate the next acceleration:
                    joint_velocity_old.data=joint_velocities_.data;

                    /*Combine a desired velocity p.vel with an error term for correction:
                    NOTE: The three zeros represent rotation components not considered here!*/
                    Vector6d cartvel; cartvel << p.vel + error, 0,0,0;
                    
                    //Update joint velocities, using the pseudoinverse of the end-effector Jacobian to map the desired Cartesian velocity (cartvel) in joint space:
                    dq_des.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;

                    //Calculate the new joint positions by integrating the velocities (joint_velocities_) with the time step dt:
                    q_des.data = joint_positions_.data + joint_velocities_.data*dt;

                    //Calculate joint acceleration by discrete numerical derivative:
                    joint_acceleration_d_.data=(joint_velocities_.data-joint_velocity_old.data)/dt;
                    
                    //Use the first method (idCntr) to calculate the required joint torques:
                   torque_values = controller_.idCntr(q_des,dq_des,joint_acceleration_d_, _Kp, _Kd);
                }
                else if(cmd_interface_ == "effort_cartesian"){
                    
                    
                    Vector6d cartacc; cartacc << p.acc + error/dt, 0,0,0;
                    desVel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                    desAcc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
                    desPos.M = desFrame.M;
                    desPos.p = desFrame.p;
                    
                    //Use the second method (idCntr) to calculate the required joint torques:
                    torque_values=controller_.idCntr(desPos,desVel,desAcc,_Kpp,_Kpo,_Kdp,_Kdo);
 
                }
                else{
 
                    std::cout<<"Error!";
                }
 
                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort" || "effort_cartesian"){
                     // Send joint velocity commands
                    for (long int i = 0; i < torque_values.size(); ++i) {
                        desired_commands_[i] = torque_values(i);
                    }
 
                }else{
 
                    std::cout<<"Error!";
                }
 
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint effort commands
                if(cmd_interface_ == "effort" || "effort_cartesian"){
                
                    KDLController controller_(*robot_);
                    q_des.data=joint_positions_.data;
                    // Azzerare i valori di qd (velocità dei giunti)
                    dq_des.data = Eigen::VectorXd::Zero(7,1);
                    // // Azzerare i valori di qdd (accelerazioni dei giunti)
                    joint_acceleration_d_.data = Eigen::VectorXd::Zero(7,1);
 
                    torque_values = controller_.idCntr(q_des,dq_des,joint_acceleration_d_, _Kp, _Kd);
                    
                    // // Update KDLrobot structure
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));  
                    
                    for (long int i = 0; i < torque_values.size(); ++i) {
    
                        desired_commands_[i] = torque_values(i);

                    }
                }else{
                     // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
 
                
 
        }
 
 
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
 
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }
 
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }
 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;
 
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_acceleration_d_;
        KDL::JntArray joint_velocity_old;
        Eigen::VectorXd torque_values;
        KDL::JntArray q_des;
        KDL::JntArray dq_des;
        KDL::Twist desVel;
        KDL::Twist desAcc;
        KDL::Frame desPos;


        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
 
        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string time_law_;
        std::string path_type_;
        KDL::Frame init_cart_pose_;
         
        //Gains
        double _Kp = 170 ;  // Example value for proportional gain
        double _Kd =  30;   // Example value for derivative gain
        double _Kpp = 90;
        double _Kpo = 90;
        double _Kdp = 2*sqrt(_Kpp);
        double _Kdo = 2*sqrt(_Kpo);
    
};
 
 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}