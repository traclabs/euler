#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class WorldFrictionPlugin : public WorldPlugin
  {
    public: WorldFrictionPlugin() : WorldPlugin()
            {
              printf("****************** Setting friction plugin ************ \n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)  {
      physics::PhysicsEnginePtr physics = _world->GetPhysicsEngine();
      const std::string frictionModel = "cone_model";
      physics->SetParam("friction_model", frictionModel);
      printf("************ Finished setting friction plugin ************* \n");
   }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldFrictionPlugin)
}


