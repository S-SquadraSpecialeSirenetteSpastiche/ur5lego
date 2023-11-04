#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo
{
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              ROS_INFO("Hello World!\n");
              // ROS_INFO("world name is: %s", this->world->Name().c_str());
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
              ROS_INFO("world name is: %s", _world->Name().c_str());
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}