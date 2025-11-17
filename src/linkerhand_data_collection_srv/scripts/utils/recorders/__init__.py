#!/usr/bin/env python3
# -*-coding:utf8-*-

# Legacy image recorder moved to legacy/ directory
# from .image_recorder import ImageRecorder
# from .piper_recorder import Recorder
from .linkerhand_recorder import LinkerHandRecorder
from .arm_recorder import ArmRecorder
try:
    from .exhand_recorder import ExHandRecorder
    __all__ = ['LinkerHandRecorder', 'ArmRecorder', 'ExHandRecorder']
except ImportError:
    __all__ = ['LinkerHandRecorder', 'ArmRecorder']
