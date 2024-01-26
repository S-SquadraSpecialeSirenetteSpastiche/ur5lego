#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <ros_impedance_controller/generic_float.h>


/// @brief this class is a layer of abstraction to control the real and simulated robot 
/// the same from the controller and the trajectory planner
/// it works by subscribing to two topics: the position of the arm and the position of the gripper
/// if the robot is simulated, when one of the two is updated, it sends Gazebo the updated joint vector,
/// since both the joints and the gripper are treated as joints in Gazebo, 
/// if the robot is real, it only serves as a wrapper for the gripper movement

const bool REAL_ROBOT = false;

class JointPositionPublisher {
    public:
        JointPositionPublisher(ros::NodeHandle nh, int joint_positions_size, int gripper_positions_size) {
            nh_ = nh;
            pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
            joint_sub_ = nh_.subscribe("/arm_joint_position", 10, &JointPositionPublisher::setJointAnglesCallback, this);
            gripper_sub_ = nh_.subscribe("/gripper_joint_position", 10, &JointPositionPublisher::setGripperAnglesCallback, this);

            this->joint_positions = Eigen::VectorXd(joint_positions_size);
            this->gripper_positions = Eigen::VectorXd(gripper_positions_size);
        }

        /// @brief sends the joint angles with a given publisher
        void send_joint_positions(){
            std_msgs::Float64MultiArray command;

            if(REAL_ROBOT){
                command.data.resize(joint_positions.size());
                for(int i=0; i<joint_positions.size(); i++)
                    command.data[i] = (float)joint_positions[i];
            } 
            else {
                command.data.resize(joint_positions.size()+gripper_positions.size());
                for(int i=0; i<joint_positions.size(); i++)
                    command.data[i] = (float)joint_positions[i];
                for(int i=joint_positions.size(); i<joint_positions.size()+gripper_positions.size(); i++)
                    command.data[i] = (float)gripper_positions[i-joint_positions.size()];
            }

            pub_.publish(command);
            ros::spinOnce();    // spin once to make sure the callback is processed
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber joint_sub_;
        ros::Subscriber gripper_sub_;
        Eigen::VectorXd joint_positions;
        Eigen::VectorXd gripper_positions;

        void setJointAnglesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
            // assert(msg->data.size() == joint_positions.size());
            for(int i=0; i<joint_positions.size(); i++)
                joint_positions[i] = msg->data[i];
            send_joint_positions();
        }

        void setGripperAnglesCallback(const std_msgs::Float64::ConstPtr& msg) {
            if(REAL_ROBOT){
                ROS_INFO_STREAM("Moving the real gripper to: " << msg->data << "mm");
                ros::ServiceClient client = nh_.serviceClient<ros_impedance_controller::generic_float>("move_gripper");

                ros_impedance_controller::generic_float srv;
                srv.request.data = msg->data;

                if (client.call(srv)) {
                    ROS_DEBUG("Service call successful");
                } else {
                    ROS_WARN("Failed to call service move_gripper");
                }
            }
            else {
                float gripper_millimeter_span = msg->data;
                float gripper_angle; // asin((gripper_millimeter_span-350)/(2*100));
                if(gripper_millimeter_span == 35.0){
                    gripper_angle = -0.08;
                }
                else{
                    gripper_angle = asin((gripper_millimeter_span-35)/(2*100));
                }
                ROS_INFO_STREAM("Moving the simulated gripper to: " << gripper_angle << "rad");
                for(int i=0; i<gripper_positions.size(); i++){
                    gripper_positions[i] = gripper_angle;
                }
                send_joint_positions();
            }
        }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "joint_position_publisher");
    ros::NodeHandle nh;
    JointPositionPublisher joint_position_publisher(nh, 6, 2);
    
    ros::spin();
    return 0;
}
