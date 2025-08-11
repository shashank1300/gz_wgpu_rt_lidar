#include <gz/plugin/Register.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

#include <memory>
#include <unordered_map>
#include <utility>

#include "rtsensor.hh"
#include "rtmanager.hh"

namespace wgpu_sensor
{
  /// \brief A plugin that validates target identification reports.
  class WGPURtSensor :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    public: void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

    private:
      std::unique_ptr<RTManager> rtManager;
      std::unordered_map<gz::sim::Entity,
          std::pair<std::shared_ptr<rtsensor::RtSensor>, gz::transport::Node::Publisher>> entitySensorMap;


      gz::transport::Node node; // Gazebo Transport Node
  };

  void WGPURtSensor::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr)
  {
    this->rtManager = std::make_unique<RTManager>();
    this->rtManager->Initialize();
  }

  void WGPURtSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm)
  {
    _ecm.EachNew<gz::sim::components::CustomSensor,
                 gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::CustomSensor *_custom,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        sdf::Sensor data = _custom->Data();

        if (data.Topic().empty())
        {
          data.SetTopic(gz::sim::scopedName(_entity, _ecm) + "/rt_sensor");
        }

        auto sensor = std::make_shared<rtsensor::RtSensor>();

        if (!sensor->Load(data))
        {
          gzerr << "Failed to load RtSensor for entity [" << _entity << "]" << std::endl;
          return true;
        }
        sensor->SetParentEntity(_parent->Data());

        this->rtManager->CreateSensorRenderer(_entity, sensor);

        gz::transport::Node::Publisher pub;
        if (sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA)
        {
            pub = this->node.Advertise<gz::msgs::Image>(sensor->TopicName());
        }
        else if (sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR)
        {
            pub = this->node.Advertise<gz::msgs::PointCloudPacked>(sensor->TopicName());
        }
        this->entitySensorMap[_entity] = std::make_pair(sensor, pub);

        return true;
      });

  }

  void WGPURtSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &_ecm)
  {
    using std::chrono::high_resolution_clock;
    if (_info.paused)
    {
      return;
    }

    if (!this->rtManager->IsSceneInitialized())
    {
      this->rtManager->BuildScene(_ecm);

    }
    else
    {
      bool shouldUpdate = false;
      for (auto const& [entityId, pair] : this->entitySensorMap)
      {
        auto const& sensor = pair.first;
        auto update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / sensor->UpdateRate()));
        if ((_info.simTime - sensor->lastUpdateTime) >= update_period)
        {
          shouldUpdate = true;
          break;

        }
      }

      if (!shouldUpdate)
      {
          return; // No sensors need updating, skip
      }

      this->rtManager->UpdateTransforms(_ecm);

    for (auto & [entityId, pair] : this->entitySensorMap)
    {
      auto & sensor = pair.first;
      auto & publisher = pair.second;
      auto update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / sensor->UpdateRate()));

      if ((_info.simTime - sensor->lastUpdateTime) >= update_period)
      {
        sensor->lastUpdateTime = _info.simTime;
        this->rtManager->RenderSensor(entityId, sensor, _info, _ecm, publisher);
      }
    }
  }
  }

  void WGPURtSensor::RemoveSensorEntities(
      const gz::sim::EntityComponentManager &_ecm)
  {
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
      [&](const gz::sim::Entity &_entity,
          const auto*) -> bool
      {
        if (this->entitySensorMap.count(_entity))
        {
          this->rtManager->RemoveSensorRenderer(_entity);
          this->entitySensorMap.erase(_entity);
          gzmsg << "Removed RtSensor for entity [" << _entity << "]." << std::endl;
        }
        return true;
      });
  }
}

GZ_ADD_PLUGIN(wgpu_sensor::WGPURtSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
