#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <nexus_swarm_sim/CheckLineOfSight.h>

namespace gazebo
{
class NexusLosRaycastPlugin : public WorldPlugin
{
  public: NexusLosRaycastPlugin() = default;

  public: ~NexusLosRaycastPlugin() override = default;

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr) override
  {
    this->world_ = _world;
    if (!this->world_)
    {
      gzerr << "[LOSRaycastPlugin] World pointer is null\n";
      return;
    }

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&NexusLosRaycastPlugin::OnUpdate, this));
  }

  private: void OnUpdate()
  {
    this->EnsureRayShape();
    this->TryInitializeRos();
    if (this->service_ready_)
    {
      this->callback_queue_.callAvailable(ros::WallDuration(0.0));
    }
  }

  private: void EnsureRayShape()
  {
    if (this->ray_ || !this->world_)
    {
      return;
    }

    auto physics = this->world_->Physics();
    if (!physics)
    {
      return;
    }

    this->ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
        physics->CreateShape("ray", physics::CollisionPtr()));
    if (!this->ray_ && !this->ray_shape_error_logged_)
    {
      gzerr << "[LOSRaycastPlugin] Failed to create Gazebo ray shape\n";
      this->ray_shape_error_logged_ = true;
    }
  }

  private: void TryInitializeRos()
  {
    if (this->service_ready_ || !this->world_)
    {
      return;
    }

    if (!ros::isInitialized())
    {
      return;
    }

    this->ros_node_.reset(new ros::NodeHandle("~"));
    this->ros_node_->setCallbackQueue(&this->callback_queue_);
    this->service_ = this->ros_node_->advertiseService(
        "/uwb_simulator/check_los",
        &NexusLosRaycastPlugin::HandleCheckLineOfSight,
        this);
    this->service_ready_ = true;

    gzmsg << "[LOSRaycastPlugin] Advertised /uwb_simulator/check_los\n";
  }

  private: bool HandleCheckLineOfSight(
      nexus_swarm_sim::CheckLineOfSight::Request &_req,
      nexus_swarm_sim::CheckLineOfSight::Response &_res)
  {
    this->ComputeLineOfSight(_req, _res);
    return true;
  }

  private: void ComputeLineOfSight(
      const nexus_swarm_sim::CheckLineOfSight::Request &_req,
      nexus_swarm_sim::CheckLineOfSight::Response &_res)
  {
    if (!this->world_ || !this->ray_)
    {
      _res.success = false;
      _res.los = true;
      _res.hit_distance = 0.0;
      _res.hit_entity = "";
      return;
    }

    auto src = this->world_->ModelByName(_req.src_model);
    auto dst = this->world_->ModelByName(_req.dst_model);
    if (!src || !dst)
    {
      _res.success = false;
      _res.los = true;
      _res.hit_distance = 0.0;
      _res.hit_entity = "";
      return;
    }

    ignition::math::Vector3d src_pos = src->WorldPose().Pos();
    ignition::math::Vector3d dst_pos = dst->WorldPose().Pos();
    ignition::math::Vector3d ray_vec = dst_pos - src_pos;
    double total_distance = ray_vec.Length();

    if (total_distance < 1e-3)
    {
      _res.success = true;
      _res.los = true;
      _res.hit_distance = 0.0;
      _res.hit_entity = "";
      return;
    }

    ignition::math::Vector3d direction = ray_vec / total_distance;
    const double offset = std::min(0.35, total_distance * 0.25);
    ignition::math::Vector3d start = src_pos + direction * offset;
    ignition::math::Vector3d end = dst_pos - direction * offset;

    this->ray_->SetPoints(start, end);
    this->ray_->Update();

    double hit_distance = 0.0;
    std::string hit_entity;
    this->ray_->GetIntersection(hit_distance, hit_entity);

    std::string src_scoped = src->GetScopedName(false);
    std::string dst_scoped = dst->GetScopedName(false);
    bool hit_src = !hit_entity.empty() && hit_entity.find(src_scoped) == 0;
    bool hit_dst = !hit_entity.empty() && hit_entity.find(dst_scoped) == 0;

    _res.success = true;
    _res.hit_entity = hit_entity;
    _res.hit_distance = hit_distance;
    _res.los = hit_entity.empty() || hit_src || hit_dst;
  }

  private: physics::WorldPtr world_;
  private: physics::RayShapePtr ray_;
  private: std::unique_ptr<ros::NodeHandle> ros_node_;
  private: ros::CallbackQueue callback_queue_;
  private: ros::ServiceServer service_;
  private: event::ConnectionPtr update_connection_;
  private: bool service_ready_ = false;
  private: bool ray_shape_error_logged_ = false;
};

GZ_REGISTER_WORLD_PLUGIN(NexusLosRaycastPlugin)
}  // namespace gazebo
