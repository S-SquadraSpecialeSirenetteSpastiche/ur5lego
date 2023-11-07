#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <ur5lego/ConnectLinks.h>

namespace gazebo
{
    class ConnectLinksPlugin : public WorldPlugin
    {
    public:
        /// @brief Constructor
        /// @param _world pointer to world object 
        /// @param _sdf pointer to sdf object
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            this->world = _world;
            // Initialize ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "link_connector_plugin", ros::init_options::NoSigintHandler);
            }

            // Create a ROS node
            this->rosNode.reset(new ros::NodeHandle("link_connector_plugin"));

            // Create a ROS subscriber
            this->rosSub = this->rosNode->subscribe("/connect_links", 1, &ConnectLinksPlugin::OnRequest, this);
        }

        void OnRequest(const ur5lego::ConnectLinks::ConstPtr& msg) {
            ConnectLinks(msg->model1, msg->link1, msg->model2, msg->link2);
        }

        /// @brief Connects two links from two different models
        /// @param model1_name Name of the first model
        /// @param link1_name Name of the first link
        /// @param model2_name Name of the second model
        /// @param link2_name Name of the second link
        void ConnectLinks(std::string model1_name, std::string link1_name, std::string model2_name, std::string link2_name) {
            physics::ModelPtr model1 = this->world->ModelByName(model1_name);
            physics::ModelPtr model2 = this->world->ModelByName(model2_name);

            if(model1 == NULL)
            {
                ROS_WARN("model1 not found");
                return;
            }
            if(model2 == NULL)
            {
                ROS_WARN("model2 not found");
                return;
            }

            physics::LinkPtr link1 = model1->GetLink(link1_name);
            physics::LinkPtr link2 = model2->GetLink(link2_name);

            if(link1 == NULL)
            {
                ROS_WARN("link1 not found");
                return;
            }
            if(link2 == NULL)
            {
                ROS_WARN("link2 not found");
                return;
            }

            // Create a joint between the two links
            physics::JointPtr joint = this->world->Physics()->CreateJoint("revolute", this->world->ModelByName(model1_name));
            joint->Load(link1, link2, ignition::math::Pose3d());

            // Initialize the joint
            joint->Init();
        }

    private:
        physics::WorldPtr world;    // pointer to the simulated world
        ros::NodeHandlePtr rosNode; // pointer to the ROS node
        ros::Subscriber rosSub;     // pointer to the ROS subscriber that listens for requests to connect links
    };

    GZ_REGISTER_WORLD_PLUGIN(ConnectLinksPlugin)
}