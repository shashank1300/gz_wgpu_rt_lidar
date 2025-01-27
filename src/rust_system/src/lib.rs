use glam::{Affine3A, Mat3A, Quat, Vec3A};
use wgpu::Device;
use wgpu_rt_lidar::{vertex, AssetMesh, Instance, Vertex};

#[no_mangle]
pub extern "C" fn create_mesh() -> *mut Mesh {
    Box::into_raw(Box::new(Mesh {
        vertices: Vec::new(),
        faces: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn free_mesh(ptr: *mut Mesh)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub extern "C" fn add_mesh_vertex(mesh: *mut Mesh, x: f32, y: f32, z: f32) {
    let mut mesh = unsafe {
        assert!(!mesh.is_null());
        &mut *mesh
    };
    mesh.vertices.push(vertex([x, y, z]));
}

#[no_mangle]
pub extern "C" fn add_mesh_face(mesh: *mut Mesh, face: u16) {
    let mut mesh = unsafe {
        assert!(!mesh.is_null());
        &mut *mesh
    };
    mesh.faces.push(face);
}

/// If the environment variable `WGPU_ADAPTER_NAME` is set, this function will attempt to
/// initialize the adapter with that name. If it is not set, it will attempt to initialize
/// the adapter which supports the required features.
async fn get_adapter_with_capabilities_or_from_env(
    instance: &wgpu::Instance,
    required_features: &wgpu::Features,
    required_downlevel_capabilities: &wgpu::DownlevelCapabilities,
) -> wgpu::Adapter {
    use wgpu::Backends;
    if std::env::var("WGPU_ADAPTER_NAME").is_ok() {
        let adapter = wgpu::util::initialize_adapter_from_env_or_default(instance, None)
            .await
            .expect("No suitable GPU adapters found on the system!");

        let adapter_info = adapter.get_info();
        println!("Using {} ({:?})", adapter_info.name, adapter_info.backend);

        let adapter_features = adapter.features();
        assert!(
            adapter_features.contains(*required_features),
            "Adapter does not support required features for this example: {:?}",
            *required_features - adapter_features
        );

        let downlevel_capabilities = adapter.get_downlevel_capabilities();
        assert!(
            downlevel_capabilities.shader_model >= required_downlevel_capabilities.shader_model,
            "Adapter does not support the minimum shader model required to run this example: {:?}",
            required_downlevel_capabilities.shader_model
        );
        assert!(
                downlevel_capabilities
                    .flags
                    .contains(required_downlevel_capabilities.flags),
                "Adapter does not support the downlevel capabilities required to run this example: {:?}",
                required_downlevel_capabilities.flags - downlevel_capabilities.flags
            );
        adapter
    } else {
        let adapters = instance.enumerate_adapters(Backends::all());

        let mut chosen_adapter = None;
        for adapter in adapters {
            let required_features = *required_features;
            let adapter_features = adapter.features();
            if !adapter_features.contains(required_features) {
                continue;
            } else {
                chosen_adapter = Some(adapter);
                break;
            }
        }

        chosen_adapter.expect("No Raytracing enabled GPU adapters found on the system!")
    }
}


#[repr(C)]
#[derive(Clone)]
pub struct Mesh {
    vertices: Vec<Vertex>,
    faces: Vec<u16>
}

#[repr(C)]
pub struct InstanceWrapper {
    instance: Instance
}


#[no_mangle]
pub extern "C" fn create_instance_wrapper(asset_index: usize, x: f32, y: f32, z:f32, qx:f32, qy:f32, qz:f32, qw:f32) -> *mut InstanceWrapper {
    Box::into_raw(Box::new(InstanceWrapper {
            instance: Instance{
                asset_mesh_index: asset_index,
                transform: Affine3A { matrix3: Mat3A::from_quat(Quat::from_xyzw(qx,qy,qz,qw)), translation: Vec3A::new(x, y, z)}
            }
    }))
}

#[no_mangle]
pub extern "C" fn free_instance_wrapper(ptr: *mut InstanceWrapper){
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}



#[repr(C)]
pub struct RtSceneBuilder {
    meshes: Vec<Mesh>,
    instances: Vec<Instance>
}

#[no_mangle]
pub extern "C" fn create_rt_scene_builder() -> *mut RtSceneBuilder {
    Box::into_raw(Box::new(RtSceneBuilder {
        meshes: Vec::new(),
        instances: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn free_rt_scene_builder(ptr: *mut RtSceneBuilder)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub extern "C" fn add_mesh(ptr: *mut RtSceneBuilder, mesh: &Mesh) -> usize
{
    let mut builder = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let index = builder.meshes.len();
    builder.meshes.push(mesh.clone());
    return index;
}

#[no_mangle]
pub extern "C" fn add_instance(ptr: *mut RtSceneBuilder, instance: *mut InstanceWrapper) -> usize
{
    let mut builder = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let mut instance = unsafe {
        assert!(!instance.is_null());
        &mut *instance
    };

    let id = builder.instances.len();
    builder.instances.push(instance.instance.clone());
    id
}


#[repr(C)]
pub struct RtRuntime {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

impl RtRuntime {
    pub async fn new() -> Self {
        let instance = wgpu::Instance::default();
        let required_features = wgpu::Features::TEXTURE_BINDING_ARRAY
            | wgpu::Features::STORAGE_RESOURCE_BINDING_ARRAY
            | wgpu::Features::VERTEX_WRITABLE_STORAGE
            | wgpu::Features::EXPERIMENTAL_RAY_QUERY
            | wgpu::Features::EXPERIMENTAL_RAY_TRACING_ACCELERATION_STRUCTURE;
        let required_downlevel_capabilities = wgpu::DownlevelCapabilities::default();
        let adapter = get_adapter_with_capabilities_or_from_env(
            &instance,
            &required_features,
            &required_downlevel_capabilities,
        )
        .await;

        let Ok((device, queue)) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features,
                    required_limits: wgpu::Limits::downlevel_defaults(),
                    memory_hints: wgpu::MemoryHints::MemoryUsage,
                },
                None,
            )
            .await
        else {
            panic!("Failed to create device");
        };

        println!("Found Raytracing enabled GPU");
        RtRuntime {
            device,
            queue
        }
    }
}
#[no_mangle]
pub extern "C" fn create_rt_runtime() -> *mut RtRuntime {
    Box::into_raw(Box::new(futures::executor::block_on(RtRuntime::new())))
}

#[no_mangle]
pub extern "C" fn free_rt_runtime(ptr: *mut RtRuntime)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtSceneUpdate {
    pub updates: Vec<Instance>,
    pub indices: Vec<usize>
}

#[no_mangle]
pub extern "C" fn create_rt_scene_update() -> *mut RtSceneUpdate {
    Box::into_raw(Box::new(RtSceneUpdate {
        updates: Vec::new(),
        indices: Vec::new()
    }))
}

#[no_mangle]
pub extern "C" fn add_update(ptr: *mut RtSceneUpdate, instance_wrapper: *mut InstanceWrapper, index: usize) {
    let mut update = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let instance = unsafe {
        assert!(!instance_wrapper.is_null());
        &mut *instance_wrapper
    };
    update.updates.push(instance.instance.clone());
    update.indices.push(index);
}

#[no_mangle]
pub extern "C" fn free_rt_scene_update(ptr: *mut RtSceneUpdate)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtScene {
    scene: wgpu_rt_lidar::RayTraceScene
}

#[no_mangle]
pub extern "C" fn create_rt_scene(runtime: *mut RtRuntime, builder: *mut RtSceneBuilder) -> *mut RtScene {
    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let builder = unsafe {
        assert!(!builder.is_null());
        &mut *builder
    };

    let scene = 
     futures::executor::block_on(
        wgpu_rt_lidar::RayTraceScene::new(
            &runtime.device, 
            &runtime.queue, 
            &builder.meshes.iter().map(|m| AssetMesh {vertex_buf: m.vertices.clone(), index_buf: m.faces.clone()}).collect::<Vec<AssetMesh>>(),
            &builder.instances));
    Box::into_raw(Box::new(RtScene {
        scene
    }))
}

#[no_mangle]
pub extern "C" fn set_transforms(ptr: *mut RtScene, device: *mut RtRuntime, updates: *mut RtSceneUpdate) {
    let scene = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let device = unsafe {
        assert!(!device.is_null());
        &mut *device
    };
    let updates = unsafe {
        assert!(!updates.is_null());
        &mut *updates
    };
    scene.scene.set_transform(&device.device, &device.queue, &updates.updates, &updates.indices);
}

#[no_mangle]
pub extern "C" fn free_rt_scene(ptr: *mut RtScene)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct ViewMatrix {
    pub view: glam::Mat4,
}

#[no_mangle]
pub extern "C" fn create_view_matrix(x: f32, y: f32, z: f32, qx:f32, qy:f32, qz:f32) -> *mut ViewMatrix {
    Box::into_raw(Box::new(ViewMatrix {
        view: glam::Mat4::from_translation(glam::Vec3::new(x, y, z)) * glam::Mat4::from_quat(glam::Quat::from_xyzw(qx, qy, qz, 1.0))
    }))
}

#[no_mangle]
pub extern "C" fn free_view_matrix(ptr: *mut ViewMatrix)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
}

#[repr(C)]
pub struct RtDepthCamera {
    camera: wgpu_rt_lidar::depth_camera::DepthCamera
}

#[no_mangle]
pub extern "C" fn create_rt_depth_camera(runtime: *mut RtRuntime, width: u32, height: u32, fov: f32) -> *mut RtDepthCamera {

    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let camera = wgpu_rt_lidar::depth_camera::DepthCamera::new(&runtime.device, width, height, fov);
    Box::into_raw(Box::new(RtDepthCamera {
        camera: futures::executor::block_on(camera)
    }))
}

#[no_mangle]
pub extern "C" fn render_depth(ptr: *mut RtDepthCamera, scene: *mut RtScene, runtime: *mut RtRuntime, view: *mut ViewMatrix) {
    let camera = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let scene = unsafe {
        assert!(!scene.is_null());
        &mut *scene
    };

    let runtime = unsafe {
        assert!(!runtime.is_null());
        &mut *runtime
    };

    let view = unsafe {
        assert!(!view.is_null());
        &mut *view
    };

    let res = futures::executor::block_on(camera.camera.render_depth_camera(&scene.scene, &runtime.device, &runtime.queue, view.view));
    println!("Res {:?}", res);
}

#[no_mangle]
pub extern "C" fn free_rt_depth_camera(ptr: *mut RtDepthCamera)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(ptr));
    }
} 