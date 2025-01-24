# WGPU based Raytracing Sensors for Gazebo

This project builds on the raytracing sensors provided by the wgpu sensors library.

## Supported platform

This library is only available as a source build on ROS2 Jazzy.

## Distrobox usage
I personally love using Distrobox. Sure its overkill, but its great for quickly getting things going. I can spin up specific ROS 2 versions with much ease on a linux system. If you are building in distrobox and have an NVidia Graphics card, you will need the `--nvidia` flags.


## Rust Analyzer Config

```json
{
    "rust-analyzer.linkedProjects": [
        "gz_wgpu_rt_lidar/src/rust_system/Cargo.toml"
    ]
}
```