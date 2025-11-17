# Legacy Image Recorders

This directory contains legacy image recording modules that have been replaced by the ZED Direct SDK recorder.

## Moved Files

- `image_recorder.py` - ROS2 topic-based image recorder (legacy)
- `camera_info_recorder.py` - ROS2 topic-based camera info recorder (legacy)

## Why These Are Legacy

These recorders used ROS2 topics for image capture, which had several limitations:
- Lower frequency capture (limited by ROS2 topic publishing rates)
- Higher latency due to ROS2 message passing overhead
- Dependency on ROS2 camera nodes being properly configured and running
- Less reliable synchronization with the data collection system

## Current Solution

The system now uses `zed_direct_recorder.py` which:
- Directly accesses ZED cameras via the ZED SDK
- Provides higher frequency capture (30fps)
- Lower latency with direct SDK access
- Better synchronization with timestamps
- Auto-detection and configuration of ZED cameras

## Migration

If you need to use the legacy recorders for any reason, you can:
1. Move the files back to the main recorders directory
2. Modify `real_env.py` to import and use them
3. Note that this is not recommended as the ZED Direct SDK provides superior performance

## Compatibility

The legacy recorders may not be compatible with the current system architecture and are provided for reference only.

