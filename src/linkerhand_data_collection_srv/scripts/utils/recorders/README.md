# Recorders Package

This package contains separated recorder classes for different components of the LinkerHand data collection system.

## Structure

```
recorders/
├── __init__.py              # Package exports
├── image_recorder.py        # Camera image recording and synchronization
├── arm_recorder.py          # New arm data collection and control
├── linkerhand_recorder.py   # LinkerHand joint state recording
└── README.md               # This file
```

## Classes

### ImageRecorder
- **Purpose**: Handles camera image recording and synchronization
- **Dependencies**: `cv_bridge`, `sensor_msgs`, `time_sync`
- **Features**: 
  - Multi-camera support
  - Time synchronization
  - Image format conversion
  - Ready state monitoring

### ArmRecorder (New Arm)
- **Purpose**: Manages new arm data collection and control
- **Dependencies**: ROS2 `sensor_msgs`
- **Features**:
  - Joint position/velocity/effort reading
  - End effector pose calculation (TODO)
  - Joint position control (TODO)
  - ROS2 topic management

### LinkerHandRecorder
- **Purpose**: Handles LinkerHand joint state recording
- **Dependencies**: ROS2 `sensor_msgs`
- **Features**:
  - Joint state subscription
  - Command position tracking
  - Left/right hand support
  - ROS2 topic management

## Usage

### Backward Compatibility
Existing code continues to work with the old import:
```python
from utils.recorders.image_recorder import ImageRecorder
from utils.recorders.arm_recorder import ArmRecorder
from utils.recorders.linkerhand_recorder import LinkerHandRecorder
```

### New Import Style (Recommended)
For new code, use direct imports:
```python
from utils.recorders.image_recorder import ImageRecorder
from utils.recorders.arm_recorder import ArmRecorder
from utils.recorders.linkerhand_recorder import LinkerHandRecorder
```

Or import from the package:
```python
from utils.recorders import ImageRecorder, ArmRecorder, LinkerHandRecorder
```

## Migration Benefits

1. **Separation of Concerns**: Each recorder handles its specific component
2. **Easier Maintenance**: Changes to one component don't affect others
3. **Future Arm Migration**: Piper-specific code is isolated in `piper_recorder.py`
4. **Better Testing**: Each recorder can be tested independently
5. **Cleaner Dependencies**: Each file only imports what it needs

## Migration Status

✅ **Arm Migration Complete**: 
- Created `ArmRecorder` class with ROS2-based interface
- Updated all imports to use new arm recorder
- Maintained backward compatibility with legacy method names
- Ready for custom topic configuration

The `ImageRecorder` and `LinkerHandRecorder` classes remain unchanged.
