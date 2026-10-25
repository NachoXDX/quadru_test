// Copyright 2026 Nacho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// PlanarBodyDriver: the "cheat" locomotion for a legged model.
//
// Pushes the model's canonical link with a horizontal force and a yaw torque
// proportional to the error between the commanded planar twist (gz.msgs.Twist:
// linear.x/y in the heading frame, angular.z) and the measured one. Height,
// roll and pitch are left entirely to physics, unlike gz VelocityControl,
// which overrides the full body velocity and keeps the robot from falling.
//
// SDF parameters (all optional):
//   <topic>          gz topic with the commanded twist      (default /cmd_vel)
//   <linear_gain>    N per m/s of velocity error            (default 800)
//   <angular_gain>   N*m per rad/s of yaw-rate error        (default 80)
//   <max_force>      clamp on the horizontal force, N       (default 150)
//   <max_torque>     clamp on the yaw torque, N*m           (default 30)
//   <cmd_timeout>    s without commands before stopping     (default 0.5)

#include <gz/msgs/twist.pb.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace quadruped_gz_plugins
{

class PlanarBodyDriver
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "PlanarBodyDriver must be attached to a model.\n";
      return;
    }
    link_ = gz::sim::Link(model.CanonicalLink(ecm));
    link_.EnableVelocityChecks(ecm, true);

    topic_ = sdf->Get<std::string>("topic", topic_).first;
    linear_gain_ = sdf->Get<double>("linear_gain", linear_gain_).first;
    angular_gain_ = sdf->Get<double>("angular_gain", angular_gain_).first;
    max_force_ = sdf->Get<double>("max_force", max_force_).first;
    max_torque_ = sdf->Get<double>("max_torque", max_torque_).first;
    cmd_timeout_ = sdf->Get<double>("cmd_timeout", cmd_timeout_).first;

    node_.Subscribe(topic_, &PlanarBodyDriver::OnCmd, this);
    gzmsg << "PlanarBodyDriver listening on [" << topic_ << "]\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !link_.Valid(ecm)) {
      return;
    }
    const double now = std::chrono::duration<double>(info.simTime).count();

    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (new_cmd_) {
        last_cmd_time_ = now;
        new_cmd_ = false;
      }
      const bool fresh = now - last_cmd_time_ < cmd_timeout_;
      vx = fresh ? cmd_vx_ : 0.0;
      vy = fresh ? cmd_vy_ : 0.0;
      wz = fresh ? cmd_wz_ : 0.0;
    }

    const auto pose = link_.WorldPose(ecm);
    const auto com = link_.WorldInertialPose(ecm);
    const auto lin_vel = link_.WorldLinearVelocity(ecm);
    const auto ang_vel = link_.WorldAngularVelocity(ecm);
    if (!pose || !com || !lin_vel || !ang_vel) {
      return;  // velocities become available after the first physics step
    }

    // Commanded velocity is in the heading frame (yaw only), like cmd_vel.
    const double yaw = pose->Rot().Yaw();
    const double vx_des = vx * std::cos(yaw) - vy * std::sin(yaw);
    const double vy_des = vx * std::sin(yaw) + vy * std::cos(yaw);

    gz::math::Vector3d force(
      linear_gain_ * (vx_des - lin_vel->X()),
      linear_gain_ * (vy_des - lin_vel->Y()),
      0.0);
    if (force.Length() > max_force_) {
      force = force.Normalized() * max_force_;
    }
    double torque_z = std::clamp(
      angular_gain_ * (wz - ang_vel->Z()), -max_torque_, max_torque_);

    // AddWorldWrench applies the force at the link origin; add the moment that
    // moves it to the center of mass, so the push doesn't pitch the body.
    const gz::math::Vector3d torque =
      gz::math::Vector3d(0, 0, torque_z) + (com->Pos() - pose->Pos()).Cross(force);
    link_.AddWorldWrench(ecm, force, torque);
  }

private:
  void OnCmd(const gz::msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_vx_ = msg.linear().x();
    cmd_vy_ = msg.linear().y();
    cmd_wz_ = msg.angular().z();
    new_cmd_ = true;
  }

  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::string topic_{"/cmd_vel"};
  double linear_gain_{800.0};
  double angular_gain_{80.0};
  double max_force_{150.0};
  double max_torque_{30.0};
  double cmd_timeout_{0.5};

  std::mutex mutex_;
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  bool new_cmd_{false};
  double last_cmd_time_{-1e9};
};

}  // namespace quadruped_gz_plugins

GZ_ADD_PLUGIN(
  quadruped_gz_plugins::PlanarBodyDriver,
  gz::sim::System,
  quadruped_gz_plugins::PlanarBodyDriver::ISystemConfigure,
  quadruped_gz_plugins::PlanarBodyDriver::ISystemPreUpdate)
