#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
 
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
 
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
 
class VisionControlNode : public rclcpp::Node
{
public:
    VisionControlNode()
    :Node("ros2_kdl_node_vision_control"),
    node_handle_(std::shared_ptr<VisionControlNode>(this))
    {
        
        // declare cmd_interface parameter ( velocity, effort or effort_cartesian)
        declare_parameter<std::string>("cmd_interface", "velocity"); // defaults to "velocity"
        get_parameter("cmd_interface", cmd_interface_);
        
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "velocity" || "effort" || "effort_cartesian"))
        {
            RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
        }

        // Dichiarazione del parametro del task
        declare_parameter<std::string>("task", "positioning");
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());
 
            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }

        // Publisher per comandi di velocità
        // vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_available_=false;
 
 
        // Subscriber to jnt states
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&VisionControlNode::aruco_subscriber, this, std::placeholders::_1));
 
        //     // Wait for the joint_state topic 
        while(!aruco_available_){

            RCLCPP_INFO(this->get_logger(), "No data from aruco received yet! ...");
            rclcpp::spin_some(node_handle_);
        }
 
 
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
        dqd.resize(nj);
        qd.resize(nj);
        qdi.resize(nj);
        joint_acceleration_d_.resize(nj);
        joint_velocity_old.resize(nj);
        torque_values.resize(nj);
        q_des.resize(nj);
        dq_des.resize(nj);

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));

        //Wait for the joint_state topic
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Update KDLrobot object
        // robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        // KDL::Frame f_T_ee = KDL::Frame::Identity();
        // robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        //definizione frame della camera

        // Specify an end-effector: camera in flange transform
        // KDL::Frame ee_T_cam;
        // ee_T_cam.M = KDL::Rotation::RotY(-1.57)*KDL::Rotation::RotZ(-3.14);
        // ee_T_cam.p = KDL::Vector(0.02,0,0);
        // robot_->addEE(ee_T_cam);

        // Update robot
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        // Retrieve initial ee pose
        Fi = robot_->getEEFrame();
        Eigen::Vector3d init_position = toEigen(Fi.p);

        KDL::Chain chain = robot_->getChain();
        fkSol_ = new KDL::ChainFkSolverPos_recursive(chain);

        // Initialize controller
        KDLController controller_(*robot_);

        // compute current jacobians
        KDL::Jacobian J_cam = robot_->getEEJacobian();

        // From object to base frame with offset on rotation and position
        KDL::Frame cam_T_object(marker.M*KDL::Rotation::RotY(-1.57), 
        KDL::Vector(marker.p.data[0]+0.03,marker.p.data[1],marker.p.data[2]-0.15));
        base_T_object = robot_->getEEFrame() * cam_T_object;
        // double p_offset = 0.02;     // Position offset
        // double R_offset = 0.314/2;     // Orientation offset. Put at 0 to centre the aruco
        base_T_object.p = base_T_object.p; //+ KDL::Vector(0.2,0.04,p_offset);
        base_T_object.M = base_T_object.M;
        
        Eigen::Vector3d end_position;

        if(cmd_interface_=="velocity"){
        
            end_position = toEigen(base_T_object.p);
        
        }else{

            end_position << init_position[0], -init_position[1]+0.2, init_position[2];

        }

        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile

        trajectory_point p = planner_.compute_trajectory(t);

        // compute errors
        Eigen::Vector3d error = computeLinearError(p.pos, init_position);
        std::cout<<"error "<<error<<std::endl;

        robot_->getInverseKinematics(Fi, qdi);
        

        if(task_ == "positioning" && aruco_available_ && joint_state_available_ ){

            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Send joint position commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
            }

        }
        else if(task_ == "look-at-point" && aruco_available_ && joint_state_available_ ){
            
            if(cmd_interface_ == "velocity"){
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&VisionControlNode::cmd_publisher, this));
                
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                
                }else if(cmd_interface_ == "effort" || "effort_cartesian"){
                    // Create cmd publisher
                    
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&VisionControlNode::cmd_publisher, this));
                    
                    for (long int i = 0; i < nj; ++i) {
                        desired_commands_[i] = 0;
                        
                    }
    
                }else{
    
                    std::cout<<"Error!";
 
            }
        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");


    }
 
private:
    void cmd_publisher() {
        
    iteration_ = iteration_ + 1;

    // define trajectory
    double total_time = 1.5; 
    int trajectory_len = 150; //
    int loop_rate = trajectory_len / total_time;
    double dt = 1.0 / loop_rate;
    t_+=dt;
    Eigen::Vector3d sd;
    sd<<0, 0, 1;
    double k = 1.2;
    

    if (t_ < total_time){

            std::cout<< cmd_interface_.c_str()<< std::endl;
             std::cout<< task_.c_str()<< std::endl;

        trajectory_point p = planner_.compute_trajectory(t_);

        //Compute EE frame
        KDL::Frame cartpos = robot_->getEEFrame();           



        KDL::Frame cam_T_object(marker.M,marker.p); 
        base_T_object = robot_->getEEFrame() * cam_T_object;
        // Compute desired Frame
        KDL::Frame desFrame; desFrame.M = base_T_object.M; desFrame.p = base_T_object.p;

        // compute current jacobians
        KDL::Jacobian J_cam = robot_->getEEJacobian();
        
        //calcolo matrici
        Eigen::Matrix<double,3,1> c_Po = toEigen(cam_T_object.p);
        Eigen::Matrix<double,3,1> s = c_Po/c_Po.norm();
        Eigen::Matrix<double,3,3> Rc = toEigen(robot_->getEEFrame().M);
        Eigen::Matrix<double,3,3> L_block = (-1/c_Po.norm())*(Eigen::Matrix<double,3,3>::Identity()-s*s.transpose());
        Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();
        Eigen::Matrix<double,6,6> Rc_grande = Eigen::Matrix<double,6,6>::Zero(); 
        Rc_grande.block(0,0,3,3) = Rc;        
        Rc_grande.block(3,3,3,3) = Rc;
        L.block(0,0,3,3) = L_block;        
        L.block(0,3,3,3) = skew(s);
        L=L*(Rc_grande.transpose());

 
        //calcolo N
        Eigen::MatrixXd LJ = L*(J_cam.data);
        Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity()-(LJ_pinv*LJ);


        // compute errors
        Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(base_T_object.p.data), Eigen::Vector3d(cartpos.p.data));
        Eigen::Vector3d o_error = computeOrientationError(toEigen(cartpos.M), toEigen(base_T_object.M));
        // std::cout << "The error norm is : " << error.norm() << std::endl;
        // std::cout << "orientation error: " << o_error.norm() << std::endl;

        KDLController controller_(*robot_);

        if(task_ == "positioning"){
            // Next Frame
            // Compute differential IK
            Vector6d cartvel; cartvel << 0.05*p.vel + 5*error, o_error;
            joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
            joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
        }
        else if(task_ == "look-at-point"){
            

            if(cmd_interface_ == "velocity"){
                ///tarare i k dell look at point
                std::cout<<"riga 310\n";

                qdi.data=joint_positions_.data;
                //robot_->getInverseKinematics(base_T_object, joint_positions_);
                dqd.data=k*LJ_pinv*sd + 1.2*N*(-qdi.data + (joint_positions_.data))/dt;
            }
            else if(cmd_interface_ == "effort"){
                std::cout<<"riga 317\n";
                
                qdi.data=joint_positions_.data;
                //robot_->getInverseKinematics(base_T_object, joint_positions_);
                dqd.data=k*LJ_pinv*sd + 1.2*N*(-qdi.data + (joint_positions_.data))/dt;

                qd.data=qdi.data+dqd.data*dt;

                fkSol_->JntToCart(qd,fd);

                Eigen::Vector3d orientation_error = computeOrientationError(toEigen(fd.M), toEigen(cartpos.M));
                std::cout << "orientation error secondo look: " << orientation_error.norm() << std::endl;

                
                joint_velocity_old.data=joint_velocities_.data;

                /*Combine a desired velocity p.vel with an error term for correction:
                NOTE: The three zeros represent rotation components not considered here!*/
                Vector6d cartvel; cartvel << p.vel + error, orientation_error;
                
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




        }
        else{
        RCLCPP_WARN(this->get_logger(), "Unknown task: %s", task_.c_str());
        return;
        }

        // Update KDLrobot structure
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));   

       if(task_ == "positioning"){

            // Send joint velocity commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }
            
        }
        else if(task_ == "look-at-point"){
            

            if(cmd_interface_ == "velocity"){
                   
                    // Send joint velocity commands
                    for (long int i = 0; i < dqd.data.size(); ++i){
                        desired_commands_[i] = dqd(i);
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
    
        }           

        
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
                        std::cout << "torque commands " << torque_values << std::endl;

                    }
                }
                else{
                     // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                        std::cout << "velocity commands " << torque_values << std::endl;

                    }
                }
                
        // Create msg and publish
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);
        
        }

    }

    void aruco_subscriber(const geometry_msgs::msg::PoseStamped& pose_msg){ 
 
     aruco_available_ = true;
     double x,y,z,q1,q2,q3,q4;
     x=pose_msg.pose.position.x;
     y=pose_msg.pose.position.y;
     z=pose_msg.pose.position.z;
     q1=pose_msg.pose.orientation.x;
     q2=pose_msg.pose.orientation.y;
     q3=pose_msg.pose.orientation.z;
     q4=pose_msg.pose.orientation.w;
     KDL::Rotation rotation= KDL::Rotation::Quaternion(q1,q2,q3,q4);
     KDL::Vector trans(x,y,z);
 
     marker.p=trans;
     marker.M=rotation;
    }
 
    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }
 
 
 
 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;
 
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray dqd;
    KDL::JntArray qd;
    KDL::JntArray qdi;
    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    KDL::Tree robot_tree;
    std::string task_, cmd_interface_;
    KDL::Frame marker;
    KDL::Frame Fi;
    KDL::Frame fd;
    KDL::Frame base_T_object;
    KDL::JntArray joint_acceleration_d_;
    KDL::ChainFkSolverPos_recursive* fkSol_;
    KDL::JntArray joint_velocity_old;
    Eigen::VectorXd torque_values;
    KDL::JntArray q_des;
    KDL::JntArray dq_des;
    KDL::Twist desVel;
    KDL::Twist desAcc;
    KDL::Frame desPos;
    //Gains
    double _Kp = 170 ;  // Example value for proportional gain
    double _Kd =  30;   // Example value for derivative gain
    double _Kpp = 90;
    double _Kpo = 90;
    double _Kdp = 2*sqrt(_Kpp);
    double _Kdo = 2*sqrt(_Kpo);
    double t_;
    int iteration_;
    bool joint_state_available_;
    bool aruco_available_;

 
};
 
int main(int argc, char **argv) {
 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 1;
}
 
 