#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>


#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>

#include <gz/common/StringUtils.hh>

#include <sdf/Box.hh>

#include "rust_binding.h"
namespace wgpu_sensor { 
  /// \brief A plugin that validates target identification reports.
  class WGPURtSensor :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: WGPURtSensor();
    /// \brief Destructor
    public: ~WGPURtSensor();

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm) override;

    RtScene* rt_scene {nullptr};
    RtRuntime* rt_runtime;
  };


  WGPURtSensor::WGPURtSensor() {}

  WGPURtSensor::~WGPURtSensor() {
    if (rt_scene != nullptr) {
      free_rt_scene(rt_scene);
    }
    free_rt_runtime(rt_runtime);
  }

  Mesh* convertSDFModelToWGPU(const sdf::Geometry& geom) {
    
     if (geom.Type() == sdf::GeometryType::BOX) {
      auto bmesh = geom.BoxShape();
      auto mesh = create_mesh();

      // TOP
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      
      // BOTTOM
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
     
      // RIGHT
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);

      // LEFT
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);

      // FRONT
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, bmesh->Size().Y()/2, bmesh->Size().Z()/2);

      // BACK
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, -bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      add_mesh_vertex(mesh, bmesh->Size().X()/2, -bmesh->Size().Y()/2, -bmesh->Size().Z()/2);
      
      for (auto x =0; x < 21; x++) {
        add_mesh_face(mesh, x);
      }
      return mesh;
    }
    return nullptr;
  }

  void WGPURtSensor::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr) {
    gzmsg << "WGPURtSensor::Configure" << std::endl;
    this->rt_runtime = create_rt_runtime();
  }

  void WGPURtSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &_ecm) {
    gzmsg << "WGPURtSensor::PreUpdate" << std::endl;

    if( this->rt_scene == nullptr) {
      auto rt_scene_builder = create_rt_scene_builder();
       _ecm.EachNew<gz::sim::components::Visual, gz::sim::components::Geometry>
              ([this, &rt_scene_builder, &_ecm](auto& entity, auto&& visual, auto&& geometry) {
                  auto geom = geometry->Data();
                  auto model = convertSDFModelToWGPU(geom);
                  if (model)
                  {
                    auto mesh_id = add_mesh(rt_scene_builder, model);
                    auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
                    auto instance = create_instance_wrapper(mesh_id, pose->Data().Pos().X(), pose->Data().Pos().Y(), pose->Data().Pos().Z(), pose->Data().Rot().X(), pose->Data().Rot().Y(), pose->Data().Rot().Z(), pose->Data().Rot().W());
                    add_instance(rt_scene_builder, instance);
                    free_instance_wrapper(instance);
                    free_mesh(model);
                  }
                  return true;
              });
      this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
      free_rt_scene_builder(rt_scene_builder);
    }
    else {

    }
  }
}

GZ_ADD_PLUGIN(wgpu_sensor::WGPURtSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)