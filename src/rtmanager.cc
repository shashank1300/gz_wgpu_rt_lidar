#include "rtmanager.hh"

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud.pb.h>

#include <gz/sim/Util.hh>

#include <sdf/Box.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <builtin_interfaces/msg/time.hpp>

using namespace wgpu_sensor;

RTManager::RTManager()
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  this->ros_node = std::make_shared<rclcpp::Node>("wgpu_rt_sensor_node");
  this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this->ros_node);

  this->workerThread = std::thread(&RTManager::RenderLoop, this);
}

RTManager::~RTManager()
{
  // Tell the thread to stop
  {
    std::unique_lock<std::mutex> lock(this->queueMutex);
    this->stopThread = true;
  }
  // Wake up the thread so it can see the stop flag
  this->condition.notify_one();
  // Wait for the thread to finish its work and exit
  if (this->workerThread.joinable())
  {
    this->workerThread.join();
  }
  if (this->rt_scene != nullptr)
  {
    free_rt_scene(this->rt_scene);
  }
  for (auto const& [entityId, rtCameraPtr] : this->rt_depth_cameras)
  {
    if (rtCameraPtr != nullptr)
    {
      free_rt_depth_camera(rtCameraPtr);
    }
  }
  this->rt_depth_cameras.clear();

  for (auto const& [entityId, rtLidarPtr] : this->rt_lidars)
  {
    if (rtLidarPtr != nullptr)
    {
      free_rt_lidar(rtLidarPtr);
    }
  }
  this->rt_lidars.clear();

  free_rt_runtime(this->rt_runtime);
}

void RTManager::Initialize()
{
  this->rt_runtime = create_rt_runtime();
}

bool RTManager::IsSceneInitialized() const
{
  return this->rt_scene != nullptr;
}

void RTManager::RenderLoop()
{
  while (true)
  {
    RenderJob job;

    // 1. Wait for a job to become available
    {
      std::unique_lock<std::mutex> lock(this->queueMutex);
      this->condition.wait(lock, [this] {
        return !this->jobQueue.empty() || this->stopThread;
      });

      // If we're stopping and there are no jobs, exit the loop
      if (this->stopThread && this->jobQueue.empty())
      {
        return;
      }

      // Get the next job from the queue
      job = this->jobQueue.front();
      this->jobQueue.pop();
    } // Mutex is unlocked here

    auto pubIt = this->publishers.find(job.entityId);
    if (pubIt == this->publishers.end())
    {
      continue; // No publisher for this entity, skip
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(job.updateInfo.simTime).count();
    transform_stamped.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(job.updateInfo.simTime).count() % 1000000000;
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = job.parentFrameId;

    transform_stamped.transform.translation.x = job.sensorWorldPose.Pos().X();
    transform_stamped.transform.translation.y = job.sensorWorldPose.Pos().Y();
    transform_stamped.transform.translation.z = job.sensorWorldPose.Pos().Z();
    transform_stamped.transform.rotation.x = job.sensorWorldPose.Rot().X();
    transform_stamped.transform.rotation.y = job.sensorWorldPose.Rot().Y();
    transform_stamped.transform.rotation.z = job.sensorWorldPose.Rot().Z();
    transform_stamped.transform.rotation.w = job.sensorWorldPose.Rot().W();

    this->tf_broadcaster->sendTransform(transform_stamped);

    auto view_matrix = create_view_matrix(
      static_cast<float>(job.sensorWorldPose.Pos().X()), static_cast<float>(job.sensorWorldPose.Pos().Y()), static_cast<float>(job.sensorWorldPose.Pos().Z()),
      static_cast<float>(job.sensorWorldPose.Rot().X()), static_cast<float>(job.sensorWorldPose.Rot().Y()), static_cast<float>(job.sensorWorldPose.Rot().Z()), static_cast<float>(job.sensorWorldPose.Rot().W())
    );

    if (job.sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA)
    {
      auto it = this->rt_depth_cameras.find(job.entityId);
      if (it == this->rt_depth_cameras.end() || !it->second) return;
      RtDepthCamera* currentRtCamera = it->second;

      ImageData imageData = render_depth(currentRtCamera, this->rt_scene, this->rt_runtime, view_matrix);

      gz::msgs::Image msg;
      msg.set_width(imageData.width);
      msg.set_height(imageData.height);
      msg.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT16);
      msg.set_step(imageData.width * sizeof(uint16_t));
      msg.set_data(imageData.ptr, imageData.len * sizeof(uint16_t));
      *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(job.updateInfo.simTime);

      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(job.parentFrameId);

      pubIt->second.Publish(msg);

      auto ros_pub_it = this->ros_image_publishers.find(job.entityId);
      if (ros_pub_it != this->ros_image_publishers.end())
      {
        sensor_msgs::msg::Image ros_msg;
        ros_msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(job.updateInfo.simTime).count();
        ros_msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(job.updateInfo.simTime).count() % 1000000000;
        ros_msg.header.frame_id = job.parentFrameId;
        ros_msg.height = imageData.height;
        ros_msg.width = imageData.width;
        ros_msg.encoding = "16UC1"; // 16-bit unsigned single channel
        ros_msg.is_bigendian = false;
        ros_msg.step = imageData.width * sizeof(uint16_t);
        ros_msg.data.resize(imageData.len * sizeof(uint16_t));
        std::memcpy(ros_msg.data.data(), imageData.ptr, imageData.len * sizeof(uint16_t));

        ros_pub_it->second->publish(ros_msg);
      }
    }
    else if (job.sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR)
    {
      auto it = this->rt_lidars.find(job.entityId);
      if (it == this->rt_lidars.end() || !it->second) return;
      RtLidar* currentRtLidar = it->second;

      RtPointCloud pointCloudData = render_lidar(currentRtLidar, this->rt_scene, this->rt_runtime, view_matrix);

      gz::msgs::PointCloudPacked msg;
      *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(job.updateInfo.simTime);

      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(job.parentFrameId);

      msg.set_height(1);
      const uint32_t num_points = pointCloudData.length / 4; // x,y,z,i
      msg.set_width(num_points);

      msg.add_field()->set_name("x");
      msg.add_field()->set_name("y");
      msg.add_field()->set_name("z");
      msg.add_field()->set_name("intensity");
      msg.set_point_step(sizeof(float) * 4);
      msg.set_data(pointCloudData.points, pointCloudData.length * sizeof(float));

      pubIt->second.Publish(msg);

      auto ros_pub_it = this->ros_pointcloud_publishers.find(job.entityId);
      if (ros_pub_it != this->ros_pointcloud_publishers.end())
      {
        sensor_msgs::msg::PointCloud2 ros_msg;
        ros_msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(job.updateInfo.simTime).count();
        ros_msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(job.updateInfo.simTime).count() % 1000000000;
        ros_msg.header.frame_id = job.parentFrameId;
        ros_msg.height = 1;
        ros_msg.width = pointCloudData.length / 4; // x,y,z,intensity
        ros_msg.is_bigendian = false;
        ros_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(ros_msg);
        modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

        modifier.resize(ros_msg.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");

        for (size_t i = 0; i < ros_msg.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
        {
          *iter_x = pointCloudData.points[i * 4 + 0];
          *iter_y = pointCloudData.points[i * 4 + 1];
          *iter_z = pointCloudData.points[i * 4 + 2];
          *iter_intensity = pointCloudData.points[i * 4 + 3];
        }

        ros_pub_it->second->publish(ros_msg);
      }
    }
    free_view_matrix(view_matrix);
  }
}

void RTManager::QueueRenderJob(const RenderJob& _job)
{
  {
    std::unique_lock<std::mutex> lock(this->queueMutex);
    this->jobQueue.push(_job);
  }
  this->condition.notify_one();
}

void RTManager::BuildScene(const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "RTManager: Building ray-tracing scene from world geometry." << std::endl;
  auto rt_scene_builder = create_rt_scene_builder();
  _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
    [this, &rt_scene_builder, &_ecm](const gz::sim::Entity &entity, const auto*, const auto* geometry)
    {
      auto geom = geometry->Data();
      auto model = this->convertSDFModelToWGPU(geom);
      if (model != nullptr)
      {
        auto mesh_id = add_mesh(rt_scene_builder, model);
        auto pose = _ecm.Component<gz::sim::components::Pose>(entity);
        if (pose)
        {
          auto instance = create_instance_wrapper(mesh_id, pose->Data().Pos().X(), pose->Data().Pos().Y(), pose->Data().Pos().Z(), pose->Data().Rot().X(), pose->Data().Rot().Y(), pose->Data().Rot().Z(), pose->Data().Rot().W());
          auto inst_id = add_instance(rt_scene_builder, instance);
          free_instance_wrapper(instance);
          this->gz_entity_to_rt_instance[entity] = inst_id;
        }
        free_mesh(model);
      }
      return true;
    });

  this->rt_scene = create_rt_scene(this->rt_runtime, rt_scene_builder);
  free_rt_scene_builder(rt_scene_builder);
  gzmsg << "RTManager: Scene built successfully." << std::endl;
}

void RTManager::UpdateTransforms(const gz::sim::EntityComponentManager &_ecm)
{
  auto rt_scene_update = create_rt_scene_update();
  _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>
    ([this, &rt_scene_update, &_ecm](auto& entity, auto&& visual, auto&& geometry)
    {
      auto pose = gz::sim::worldPose(entity, _ecm);
      auto instance_handle = gz_entity_to_rt_instance.find(entity);
      if (instance_handle == gz_entity_to_rt_instance.end())
      {
        gzwarn << "No instance found for entity " << entity << std::endl;
        return true;
      }
      auto instance =
        create_instance_wrapper(instance_handle->second,
          pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
          pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

      add_update(rt_scene_update, instance, instance_handle->second);
      free_instance_wrapper(instance);
      return true;
    });

  set_transforms(this->rt_scene, this->rt_runtime, rt_scene_update);
  free_rt_scene_update(rt_scene_update);
}

void RTManager::CreateSensorRenderer(const gz::sim::Entity &_entity, const std::shared_ptr<rtsensor::RtSensor>& _sensor)
{
  if (_sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA)
  {
    this->rt_depth_cameras[_entity] = create_rt_depth_camera(
      this->rt_runtime,
      _sensor->config.camera.width,
      _sensor->config.camera.height,
      static_cast<float>(_sensor->config.camera.horizontalFov)
      // ADD: verticalFOV, nearClip, far Clip
    );
  }
  else if (_sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR)
  {
    auto lidarConfig = new_lidar_config(
      _sensor->config.lidar.num_lasers,
      _sensor->config.lidar.num_steps,
      static_cast<float>(_sensor->config.lidar.min_vertical_angle),
      static_cast<float>(_sensor->config.lidar.max_vertical_angle),
      static_cast<float>(_sensor->config.lidar.step_vertical_angle),
      static_cast<float>(_sensor->config.lidar.min_horizontal_angle),
      static_cast<float>(_sensor->config.lidar.max_horizontal_angle),
      static_cast<float>(_sensor->config.lidar.step_horizontal_angle)
      // ADD: min_range, max_range
    );
    this->rt_lidars[_entity] = create_rt_lidar(this->rt_runtime, lidarConfig);
    free_lidar_config(lidarConfig);
  }
  if (_sensor->Type() == rtsensor::RtSensor::SensorType::LIDAR)
  {
    this->publishers[_entity] = this->node.Advertise<gz::msgs::PointCloudPacked>(_sensor->TopicName());
    this->ros_pointcloud_publishers[_entity] = this->ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
      _sensor->TopicName(),
      rclcpp::SensorDataQoS()
    );
  }
  else if (_sensor->Type() == rtsensor::RtSensor::SensorType::CAMERA)
  {
    this->publishers[_entity] = this->node.Advertise<gz::msgs::Image>(_sensor->TopicName());
    this->ros_image_publishers[_entity] = this->ros_node->create_publisher<sensor_msgs::msg::Image>(
      _sensor->TopicName(),
      rclcpp::SensorDataQoS()
    );
  }
}

void RTManager::RemoveSensorRenderer(const gz::sim::Entity &_entity)
{
  if (this->rt_depth_cameras.count(_entity))
  {
    free_rt_depth_camera(this->rt_depth_cameras.at(_entity));
    this->rt_depth_cameras.erase(_entity);
  }
  if (this->rt_lidars.count(_entity))
  {
    free_rt_lidar(this->rt_lidars.at(_entity));
    this->rt_lidars.erase(_entity);
  }
}

Mesh* RTManager::convertSDFModelToWGPU(const sdf::Geometry& geom)
{
  if (geom.Type() == sdf::GeometryType::BOX)
  {
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

    uint16_t indices[] = {
      0, 1, 2, 2, 3, 0, // top
      4, 5, 6, 6, 7, 4, // bottom
      8, 9, 10, 10, 11, 8, // right
      12, 13, 14, 14, 15, 12, // left
      16, 17, 18, 18, 19, 16, // front
      20, 21, 22, 22, 23, 20, // back
    };
    for (auto x: indices)
    {
      add_mesh_face(mesh, x);
    }
    return mesh;
  }
  else if (geom.Type() == sdf::GeometryType::PLANE)
  {
    auto pmesh = geom.PlaneShape();
    auto mesh = create_mesh();

    add_mesh_vertex(mesh, -pmesh->Size().X()/2, -pmesh->Size().Y()/2, 0.0f);
    add_mesh_vertex(mesh, pmesh->Size().X()/2, -pmesh->Size().Y()/2, 0.0f);
    add_mesh_vertex(mesh, pmesh->Size().X()/2, pmesh->Size().Y()/2, 0.0f);
    add_mesh_vertex(mesh, -pmesh->Size().X()/2, pmesh->Size().Y()/2, 0.0f);

    uint16_t indices[] = {
      0, 1, 2,
      2, 3, 0
    };
    for (auto x : indices)
    {
      add_mesh_face(mesh, x);
    }
    return mesh;
  }
  else if (geom.Type() == sdf::GeometryType::MESH)
  {
    auto mesh_shape = geom.MeshShape();
    auto mesh = create_mesh();
    const std::string &uri = mesh_shape->Uri();
    const gz::common::Mesh *mesh_data = gz::common::MeshManager::Instance()->Load(uri);
    if (!mesh_data) return nullptr;

    const gz::math::Vector3d scale = mesh_shape->Scale();
    uint16_t vertex_offset = 0;
    for (unsigned int i = 0; i < mesh_data->SubMeshCount(); ++i)
    {
      auto submesh_weak = mesh_data->SubMeshByIndex(i);
      auto submesh = submesh_weak.lock();
      if (!submesh) continue;
      for (unsigned int v = 0; v < submesh->VertexCount(); ++v)
      {
        const auto &vertex = submesh->Vertex(v);
        add_mesh_vertex(mesh,
          static_cast<float>(vertex.X() * scale.X()),
          static_cast<float>(vertex.Y() * scale.Y()),
          static_cast<float>(vertex.Z() * scale.Z()));
      }
      for (unsigned int x = 0; x < submesh->IndexCount(); ++x)
      {
        add_mesh_face(mesh, submesh->Index(x) + vertex_offset);
      }
      vertex_offset += submesh->VertexCount();
    }
    return mesh;
  }
  return nullptr;
}
