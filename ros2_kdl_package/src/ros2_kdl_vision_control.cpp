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
        qdi.resize(nj);

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


        // Initialize controller
        //KDLController controller_(*robot_);

        // compute current jacobians
        KDL::Jacobian J_cam = robot_->getEEJacobian();

        // From object to base frame with offset on rotation and position
        KDL::Frame cam_T_object(marker.M*KDL::Rotation::RotY(-1.57), 
        KDL::Vector(marker.p.data[0]+0.03,marker.p.data[1],marker.p.data[2]-0.24));
        base_T_object = robot_->getEEFrame() * cam_T_object;
        // double p_offset = 0.02;     // Position offset
        // double R_offset = 0.314/2;     // Orientation offset. Put at 0 to centre the aruco
        base_T_object.p = base_T_object.p; //+ KDL::Vector(0.2,0.04,p_offset);
        base_T_object.M = base_T_object.M;

        Eigen::Vector3d end_position = toEigen(base_T_object.p);


        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile

        trajectory_point p = planner_.compute_trajectory(t);

        // compute errors
        Eigen::Vector3d error = computeLinearError(p.pos, init_position);
        std::cout<<"error "<<error<<std::endl;

        robot_->getInverseKinematics(Fi, qdi);
        

        if((task_ == "positioning" || "look-at-point" ) && aruco_available_ && joint_state_available_  ){

            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Send joint position commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
            }

        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");


    }
 
private:


    // Variabili per la posizione precedente del marker ARUCO
    Eigen::Vector3d prev_marker_pos = Eigen::Vector3d::Zero();  // inizializzazione della posizione precedente
    double marker_velocity_threshold = 0.001;  // Soglia di velocità per il movimento del marker

    // Funzione che calcola la velocità del marker ARUCO
    bool is_marker_moving(const Eigen::Vector3d& current_pos) {
        
        Eigen::Vector3d delta_pos = current_pos - prev_marker_pos;
        double speed = delta_pos.norm();
        // Aggiorna la posizione precedente
        prev_marker_pos = current_pos;
        // Controlla se la velocità è superiore alla soglia
        return speed > marker_velocity_threshold;
    }


    void cmd_publisher() {
        
        iteration_ = iteration_ + 1;

        // define trajectory
        double total_time = 1.5; //
        int trajectory_len = 150; //
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;
        t_+=dt;
        Eigen::Vector3d sd;
        sd<<0, 0, 1;
        double k = -10;
    

        //Compute EE frame
        KDL::Frame cartpos = robot_->getEEFrame();           

        // Compute desired Frame
        KDL::Frame desFrame; desFrame.M = base_T_object.M; desFrame.p = base_T_object.p;

        KDL::Frame cam_T_object(marker.M,marker.p); 

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
        L=L*Rc_grande;

 
        //calcolo N
        Eigen::MatrixXd LJ = L*(J_cam.data);
        Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity()-(LJ_pinv*LJ);


        // compute errors
        Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(base_T_object.p.data), Eigen::Vector3d(cartpos.p.data));
        Eigen::Vector3d o_error = computeOrientationError(toEigen(cartpos.M), toEigen(base_T_object.M));
        std::cout << "The error norm is : " << error.norm() << std::endl;
        //std::cout << "s error norm " << (s-sd).norm() << std::endl;

        if(task_ == "positioning"){

            if (t_ < total_time){


                trajectory_point p = planner_.compute_trajectory(t_);

                // Next Frame
                // Compute differential IK
                Vector6d cartvel; cartvel << 0.05*p.vel + 5*error, 0.1*o_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                // Update KDLrobot structure
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data)); 
                    
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);

                    }

                        std_msgs::msg::Float64MultiArray cmd_msg;
                        cmd_msg.data = desired_commands_;
                        cmdPublisher_->publish(cmd_msg);
                }
                else{


                     // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                        //std::cout << "velocity commands " << joint_velocities_.data << std::endl;
                }
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                
                
            }    
        }
        else if(task_ == "look-at-point"){
            
            // Aggiungi la logica per verificare se l'ARUCO si muove
                if (is_marker_moving(toEigen(cam_T_object.p))) {
           
                dqd.data=k*LJ_pinv*sd -1*N*(-qdi.data+joint_positions_.data);

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                 // Send joint velocity commands
                    for (long int i = 0; i < dqd.data.size(); ++i){
                        desired_commands_[i] = dqd(i);
                    }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
  
             }
            
            else{

            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                    //std::cout << "velocity commands " << joint_velocities_.data << std::endl;

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

            }

            }
        }
        else{
        RCLCPP_WARN(this->get_logger(), "Unknown task: %s", task_.c_str());
        return;
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
    KDL::JntArray qdi;
    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::string task_;
    KDL::Frame marker;
    KDL::Frame Fi;
    KDL::Frame base_T_object;
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
 
 