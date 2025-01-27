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
    RtDepthCamera* rt_depth_camera{nullptr};
    std::unordered_map<size_t, size_t> gz_entity_to_rt_instance;
    gz::sim::Entity camera_entity;
  };


  WGPURtSensor::WGPURtSensor() {}

  WGPURtSensor::~WGPURtSensor() {
    if (rt_scene != nullptr) {
      free_rt_scene(rt_scene);
    }
    if (rt_depth_camera != nullptr) {
      free_rt_depth_camera(rt_depth_camera);
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
    this->rt_depth_camera = create_rt_depth_camera(this->rt_runtime, 256, 256, 0.75);
    this->camera_entity = _entity;
  }

  void WGPURtSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &_ecm) {
    gzmsg << "WGPURtSensor::PreUpdate" << std::endl;

    if (_info.paused) {
      return;
    }

    if( this->rt_scene == nullptr) {
      gzerr << "Creating scene" << std::endl;
      auto rt_scene_builder = create_rt_scene_builder();
       _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
              ([this, &rt_scene_builder, &_ecm](auto& entity, auto&& visual, auto&& geometry) {
                  auto geom = geometry->Data();
                  auto model = convertSDFModelToWGPU(geom);
                  if (model)
                  {
                    auto mesh_id = add_mesh(rt_scene_builder, model);
                    auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
                    if (!pose) {
                      gzmsg << "No pose component found for entity " << entity << std::endl;
                      return true;
                    }
                    auto instance = create_instance_wrapper(mesh_id, pose->Data().Pos().X(), pose->Data().Pos().Y(), pose->Data().Pos().Z(), pose->Data().Rot().X(), pose->Data().Rot().Y(), pose->Data().Rot().Z(), pose->Data().Rot().W());
                    auto inst_id = add_instance(rt_scene_builder, instance);
                    free_instance_wrapper(instance);
                    gz_entity_to_rt_instance[entity] = inst_id;
                    free_mesh(model);
                  }
                  return true;
              });
      gzerr << "Creating scene" << std::endl;
      this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
      free_rt_scene_builder(rt_scene_builder);
    }
  

    else {
      auto rt_scene_update = create_rt_scene_update();
      _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
            ([this, &rt_scene_update, &_ecm](auto& entity, auto&& visual, auto&& geometry) {
        auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
        if (!pose) {
          gzmsg << "No pose component found for entity " << entity << std::endl;
          return true;
        }
        auto instance_handle = gz_entity_to_rt_instance.find(entity);
        if (instance_handle == gz_entity_to_rt_instance.end()) {
          gzmsg << "No instance found for entity " << entity << std::endl;
          return true;
        }
        gzerr << "Creating instance " << instance_handle->second << std::endl;
        auto instance = 
          create_instance_wrapper(instance_handle->second, 
            pose->Data().Pos().X(), pose->Data().Pos().Y(), pose->Data().Pos().Z(),
            pose->Data().Rot().X(), pose->Data().Rot().Y(), pose->Data().Rot().Z(), pose->Data().Rot().W());
        
        gzerr << "Updating instance " << instance_handle->second << std::endl;

        add_update(rt_scene_update, instance, instance_handle->second);
        
        gzerr << "Freeing instance" << std::endl;
        free_instance_wrapper(instance);

        return true;
      });

      gzerr << "Setting transforms" << std::endl;
      set_transforms(rt_scene, rt_runtime, rt_scene_update);

      auto camera_pose = _ecm.Component<gz::sim::components::Pose>(camera_entity);
      if (!camera_pose) {
        gzmsg << "No pose component found for camera entity " << camera_entity << std::endl;
        return;
      }
      auto view_matrix = create_view_matrix(camera_pose->Data().Pos().X(), camera_pose->Data().Pos().Y(), camera_pose->Data().Pos().Z(), camera_pose->Data().Rot().X(), camera_pose->Data().Rot().Y(), camera_pose->Data().Rot().Z(), camera_pose->Data().Rot().W());
      render_depth(rt_depth_camera, rt_scene, rt_runtime, view_matrix);
      free_rt_scene_update(rt_scene_update);
    }
    
  }
}

GZ_ADD_PLUGIN(wgpu_sensor::WGPURtSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)