# WGPU based Raytracing Sensors for Gazebo

This project builds on the raytracing sensors provided by the wgpu sensors library. It is currently at a Proof-of-concept stage and can only render boxes.

## Requirements

This is package designed to be built on ros2 jazzy. You will need a recent version of [rust](https://www.rust-lang.org/tools/install), a non-apple ray-tracing enabled GPU (NVidia RTX or above, AMD RX6000 and above, steamdeck or intel battlemage)

You will also need [rerun](https://rerun.io/) as currently this prototype directly visuallizes the depth data on the GPU as a rerun object.
## Build from source
On ros2 jazzy:
```
git clone git@github.com:arjo129/gz_wgpu_rt_lidar.git
rosdep install --from-paths gz_wgpu_rt_lidar
colcon build
```


## Supported platform

This library is only available as a source build on ROS2 Jazzy.

## Distrobox usage
I personally love using Distrobox. Sure its overkill, but its great for quickly getting things going. I can spin up specific ROS 2 versions with much ease on a linux system. If you are building in distrobox and have an NVidia Graphics card, you will need the `--nvidia` flags.


## Rust Analyzer Config
For code completion to work in vscode you may need to specify the following
```json
{
    "rust-analyzer.linkedProjects": [
        "gz_wgpu_rt_lidar/src/rust_system/Cargo.toml"
    ]
}
```

## TODOs

- Support all sdf shapes (currently only cubes are supported)
- Move rendering of main thread via some queue.
- Explore LiDARs.
