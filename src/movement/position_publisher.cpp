#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <string>

const bool REAL_ROBOT = true;

class JointPositionPublisher {
    public:
        JointPositionPublisher(ros::Publisher publisher, ros::NodeHandle nh, int joint_positions_size, int gripper_positions_size) {
            pub_ = publisher;
            joint_sub_ = nh.subscribe("/arm_joint_position", 10, &JointPositionPublisher::setJointAnglesCallback, this);
            gripper_sub_ = nh.subscribe("/gripper_joint_position", 10, &JointPositionPublisher::setGripperAnglesCallback, this);

            this->joint_positions = Eigen::VectorXd(joint_positions_size);
            this->gripper_positions = Eigen::VectorXd(gripper_positions_size);
        }

        /// @brief sends the joint angles with a given publisher
        void send_joint_positions(){
            if(REAL_ROBOT){
                trajectory_msgs::JointTrajectory command;
                command.joint_names.resize(joint_positions.size()+gripper_positions.size());
                command.points.resize(1);
                command.points[0].positions.resize(joint_positions.size());
                std::vector<std::string> joint_names = {
                    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                };
                for (int i=0; i<joint_positions.size(); i++){
                    command.joint_names[i] = joint_names[i];
                    command.points[0].positions[i] = joint_positions[i];

                    ROS_INFO_STREAM("Joint " << joint_names[i] << " set to " << joint_positions[i]);
                }
                pub_.publish(command);
            } 
            else {
                std_msgs::Float64MultiArray command;
                command.data.resize(joint_positions.size()+gripper_positions.size());
                for(int i=0; i<joint_positions.size(); i++)
                    command.data[i] = (float)joint_positions[i];
                for(int i=joint_positions.size(); i<joint_positions.size()+gripper_positions.size(); i++)
                    command.data[i] = (float)gripper_positions[i-joint_positions.size()];
                pub_.publish(command);
            }
            ros::spinOnce();    // spin once to make sure the callback is processed
        }

    private:
        ros::Publisher pub_;
        ros::Subscriber joint_sub_;
        ros::Subscriber gripper_sub_;
        Eigen::VectorXd joint_positions;
        Eigen::VectorXd gripper_positions;

        void setJointAnglesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
            assert(msg->data.size() == joint_positions.size());
            for(int i=0; i<joint_positions.size(); i++)
                joint_positions[i] = msg->data[i];
            send_joint_positions();
        }

        void setGripperAnglesCallback(const std_msgs::Float64::ConstPtr& msg) {
            float gripper_angle = msg->data;
            for(int i=0; i<gripper_positions.size(); i++){
                gripper_positions[i] = gripper_angle;
            }
            send_joint_positions();
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "joint_position_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub;
    if (REAL_ROBOT) {
        pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);
    }
    else {
        pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    }  
    JointPositionPublisher joint_position_publisher(pub, nh, 6, 3);
    
    ros::spin();
    return 0;
}