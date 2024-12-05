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
    :Node("ros2_kdl_vision_control"),
    node_handle_(std::shared_ptr<VisionControlNode>(this))
    {
        // declare cmd_interface parameter ( velocity, effort or effort_cartesian)
        declare_parameter<std::string>("cmd_interface", "effort"); // defaults to "velocity"
        get_parameter("cmd_interface", cmd_interface_);
    
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
 
        if (!(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"))
        {
            RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
        }
 
        // Dichiarazione del parametro del task
        declare_parameter<std::string>("task", "look-at-point");
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());
 
            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }

            // std::cout<<"KP: ";
            // std::cin>>_Kp;
            // std::cout<<"\nKd: ";
            // std::cin>>_Kd;
 
        // Publisher per comandi di velocità
        // vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_available_=false;
 
 
        // Subscriber to jnt states
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 1, std::bind(&VisionControlNode::aruco_subscriber, this, std::placeholders::_1));
 
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
        qd.resize(nj);
        dqd.resize(nj);
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


         for (long int i = 0; i < dq_des.data.size(); ++i) {
                        dq_des.data[i] = 0.0;
                    }
        // Specify an end-effector: camera in flange transform
        // KDL::Frame ee_T_cam;
        // ee_T_cam.M = KDL::Rotation::RotY(-1.57)*KDL::Rotation::RotZ(-3.14);
        // ee_T_cam.p = KDL::Vector(0.02,0,0);
        // robot_->addEE(ee_T_cam);
 
        /// Retrieve initial ee pose
        Fi = robot_->getEEFrame();
        Eigen::Vector3d init_position = toEigen(Fi.p);


        // Initialize controller
        //KDLController controller_(*robot_);
        // compute current jacobians
        KDL::Jacobian J_cam = robot_->getEEJacobian();

        // From object to base frame with offset on rotation and position
        KDL::Frame marker_wrt_camera(marker.M, 
        KDL::Vector(marker.p.data[0],marker.p.data[1]-0.12,marker.p.data[2]-0.27));
        cam_link_to_optical= KDL::Frame(KDL::Rotation::Quaternion(-0.5,0.5,-0.5,0.5),KDL::Vector(0,0,0));

        marker_wrt_world =  robot_->getEEFrame() * cam_link_to_optical* marker_wrt_camera;


        // double p_offset = 0.02;     // Position offset
        // double R_offset = 0.314/2;     // Orientation offset. Put at 0 to centre the aruco

        end_position<<marker_wrt_world.p.data[0],marker_wrt_world.p.data[1]-0.1,marker_wrt_world.p.data[2];
        //end_position << init_position[0], init_position[1], init_position[2]+0.1;        
        end_orientation = marker_wrt_world.M*KDL::Rotation::RotX(1.57)*KDL::Rotation::RotY(0.01)*KDL::Rotation::RotZ(2.16);
;


        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile

        trajectory_point p = planner_.compute_trajectory(t);

        // compute errors
        Eigen::Vector3d error = computeLinearError(p.pos, init_position);

        robot_->getInverseKinematics(Fi, qdi);
        
 
        if(task_ == "look-at-point" && aruco_available_ && joint_state_available_ ){
            
            if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"){
                    // Create cmd publisher
                    
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
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
        int trajectory_len = 150;
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;
        t_+=dt;
        Eigen::Vector3d sd;
        sd<<0, 0, 1;
        double k = 1;
    
        if (t_ < total_time){

        trajectory_point p = planner_.compute_trajectory(t_);
 
        KDL::Frame marker_wrt_camera(marker.M, 
        KDL::Vector(marker.p[0],marker.p[1],marker.p[2]));
        // std::cout<<robot_->getEEFrame()*marker_wrt_camera<<std::endl;

        //Compute EE frame
        KDL::Frame cartpos = robot_->getEEFrame();           

        // Compute desired Frame
        KDL::Frame desFrame; 
            // compute current jacobians
        KDL::Jacobian J_cam = robot_->getEEJacobian();
        
        //calcolo matrici
        Eigen::Matrix<double,3,1> c_Po = toEigen(marker_wrt_camera.p);
        Eigen::Matrix<double,3,1> s = c_Po/c_Po.norm();
        Eigen::Matrix<double,3,3> Rc = toEigen(robot_->getEEFrame().M);
        Eigen::Matrix<double,3,3> L_block = (-1/c_Po.norm())*(Eigen::Matrix<double,3,3>::Identity()-s*s.transpose());
        Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();
        Eigen::Matrix<double,6,6> Rc_grande = Eigen::Matrix<double,6,6>::Zero(); 
        Rc_grande.block(0,0,3,3) = Rc;        
        Rc_grande.block(3,3,3,3) = Rc;
        L.block(0,0,3,3) = L_block;        
        L.block(0,3,3,3) = skew(s);
        L=L*Rc_grande;

 
        //calcolo N
        Eigen::MatrixXd LJ = L*(J_cam.data);
        Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity()-(LJ_pinv*LJ);


        // compute errors
        Eigen::Vector3d error ;
        Eigen::Vector3d o_error;
        KDL::Frame des_cartpos;
        //Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(desFrame.p.data), Eigen::Vector3d(cartpos.p.data));
        //Eigen::Vector3d o_error = computeOrientationError(toEigen(cartpos.M), toEigen(desFrame.M));
        //std::cout << "The error norm is : " << error.norm() << std::endl;
        // std::cout << "orientation error norm " << o_error.norm() << std::endl;
        //std::cout << "s error norm " << (s-sd).norm() << std::endl;
 
        KDLController controller_(*robot_);
 
                    double cos_theta = sd.dot(s); // Prodotto scalare tra i due vettori
                    cos_theta = std::max(-1.0, std::min(1.0, cos_theta)); // Assicura che il valore sia tra -1 e 1 per evitare errori numerici
                     //double angular_error = std::acos(cos_theta);
                    double cam_o_error = std::acos(cos_theta);

                    Eigen::Vector3d orientation_error_cam = sd.cross(s);
                    KDL::Rotation axang = (KDL::Rotation::Rot(toKDL(orientation_error_cam), cam_o_error)); //costriusco la matrice di rotazione tra i due versori

                     // Trasformazione nel frame base
                    Eigen::Vector3d orientation_error = computeOrientationError( toEigen(robot_->getEEFrame().M*axang),toEigen(cartpos.M));
                    std::cout << "orientation norm " << orientation_error.norm() << std::endl;
                    desFrame.M =robot_->getEEFrame().M*axang; 
                    desFrame.p = toKDL(p.pos);
                
                if(cmd_interface_ == "effort"){
                
                
                   
                    error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                    //cam_o_error = computeOrientationError(toEigen(cartpos.M), toEigen(fd.M));
                    std::cout << "error norm " << error.norm() << std::endl;
 
                
    
    
                    //Eigen::Vector3d orientation_error = computeOrientationError(toEigen(desFrame.M), toEigen(cartpos.M));
                    //std::cout << "orientation error secondo look: " << orientation_error.norm() << std::endl;
    
                    
                    joint_velocity_old.data=dq_des.data;
    
                    Vector6d cartvel; cartvel << 0.05*p.vel + 5*error, 5*orientation_error;
                    //Update joint velocities, using the pseudoinverse of the end-effector Jacobian to map the desired Cartesian velocity (cartvel) in joint space:
                    dq_des.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel ;

                    //Calculate the new joint positions by integrating the velocities (joint_velocities_) with the time step dt:
                    q_des.data = q_des.data + dq_des.data*dt;
    
                    //Calculate joint acceleration by discrete numerical derivative:
                    joint_acceleration_d_.data=(dq_des.data-joint_velocity_old.data)/dt;
                    
                    //Use the first method (idCntr) to calculate the required joint torques:
                    torque_values = controller_.idCntr(q_des,dq_des,joint_acceleration_d_, _Kp, _Kd);
                }
                else if(cmd_interface_ == "effort_cartesian"){
                    
                 KDL::Twist cartvel; cartvel.vel=toKDL(p.vel);// des_cartvel2.rot=toKDL(o_error);
                
                //desired frame acceleration
                KDL::Twist cartacc; cartacc.vel=toKDL(p.acc); //des_cartacc.rot initialized to zero

                     error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                     std::cout << "The error norm is : " << error.norm() << std::endl;
                    
                    //Use the second method (idCntr) to calculate the required joint torques:
                    torque_values=controller_.idCntr(desFrame,cartvel,cartacc,_Kpp,_Kpo,_Kdp,_Kdo);
 
                }
                
            
                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));  
 
                if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"){
                    
                     // Send joint velocity commands
                    for (long int i = 0; i < torque_values.size(); ++i) {
                        desired_commands_[i] = torque_values(i);
                    }
 
                }else{
 
                    std::cout<<"Error!";
            }
 
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
 
 
        } 
        else{
 
        if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian" ){
                
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
                    //std::cout << "torque commands " << torque_values << std::endl;
 
                }
            }
            
             RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
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

    Eigen::Vector3d end_position;
 
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray dqd;
    KDL::JntArray qdi;
        KDL::JntArray qd;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::string task_;
        std::string cmd_interface_;

    KDL::Frame marker;
    KDL::Frame cam_link_to_optical;
    KDL::Frame Fi;
    KDL::Rotation end_orientation;
    KDL::Frame marker_wrt_world;
    KDL::Frame lap_frame_;
 
    KDL::JntArray joint_acceleration_d_;
    KDL::JntArray joint_velocity_old;
    Eigen::VectorXd torque_values;
    KDL::JntArray q_des;
    KDL::JntArray dq_des;
    KDL::Twist desVel;
    KDL::Twist desAcc;
    KDL::Frame desPos;
    //Gains
    double _Kp=0.05  ;  // Example value for proportional gain
    double _Kd=10 ;   // Example value for derivative gain
    double _Kpp = 80;
    double _Kpo = 80;
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
 
 
 