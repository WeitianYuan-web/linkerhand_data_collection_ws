#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
VideoWriter class for real-time video encoding with FFmpeg.

This module provides a high-performance video writer that streams frames
directly to FFmpeg subprocess for H.264 encoding, designed for robotics
data collection with multiple cameras.
"""

import os
import time
import threading
import subprocess
import numpy as np
from typing import Optional, Dict, List, Tuple, Union
from pathlib import Path
import queue
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VideoWriter:
    """
    Real-time video writer using FFmpeg subprocess for H.264 encoding.
    
    Features:
    - Streaming video encoding with minimal memory usage
    - Thread-safe frame writing
    - Automatic timestamp tracking
    - Multiple camera support
    - Configurable quality settings
    - Error handling and recovery
    """
    
    def __init__(self, 
                 output_path: str,
                 width: int,
                 height: int,
                 fps: int = 30,
                 quality: str = 'medium',
                 preset: str = 'fast',
                 pixel_format: str = 'rgb24',
                 codec: str = 'libx264',
                 camera_name: str = 'camera'):
        """
        Initialize video writer.
        
        Args:
            output_path: Path to output video file (e.g., 'cameras/cam_top.mp4')
            width: Video width in pixels
            height: Video height in pixels
            fps: Frames per second
            quality: Video quality ('low', 'medium', 'high', 'lossless')
            preset: FFmpeg preset ('ultrafast', 'fast', 'medium', 'slow')
            pixel_format: Input pixel format ('rgb24', 'bgr24', 'yuv420p')
            codec: Video codec ('libx264', 'libx265')
            camera_name: Name of camera for logging
        """
        self.output_path = Path(output_path)
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_name = camera_name
        self.pixel_format = pixel_format
        
        # Create output directory if it doesn't exist
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Quality settings
        self.quality_settings = {
            'low': {'crf': 28, 'bitrate': '500k'},
            'medium': {'crf': 23, 'bitrate': '1000k'},
            'high': {'crf': 18, 'bitrate': '2000k'},
            'lossless': {'crf': 0, 'bitrate': None}
        }
        
        self.quality_config = self.quality_settings.get(quality, self.quality_settings['medium'])
        
        # Store codec and preset
        self.codec = codec
        self.preset = preset
        
        # FFmpeg process
        self.process = None
        self._is_writing = False
        self._frame_count = 0
        self._timestamps = []
        self._timestamps_lock = threading.Lock()
        
        # Frame queue for buffering
        self._frame_queue = queue.Queue(maxsize=100)  # Buffer up to 100 frames
        self._queue_thread = None
        self._stop_queue = False
        
        # Error handling
        self._error_occurred = False
        self._error_message = None
        
        # Initialize FFmpeg process
        self._start_ffmpeg_process()
        
        # Start frame processing thread
        self._start_queue_processing()
        
        logger.info(f"VideoWriter initialized for {camera_name}: {width}x{height} @ {fps}fps, quality={quality}")
    
    def _start_ffmpeg_process(self):
        """Start FFmpeg subprocess for video encoding."""
        try:
            # Build FFmpeg command
            cmd = self._build_ffmpeg_command()
            
            logger.debug(f"Starting FFmpeg with command: {' '.join(cmd)}")
            
            # Start FFmpeg process
            self.process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0  # Unbuffered for real-time streaming
            )
            
            self._is_writing = True
            logger.info(f"FFmpeg process started for {self.camera_name}")
            
        except Exception as e:
            logger.error(f"Failed to start FFmpeg process: {e}")
            raise RuntimeError(f"Failed to start FFmpeg: {e}")
    
    def _build_ffmpeg_command(self) -> List[str]:
        """Build FFmpeg command for video encoding."""
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output file
            '-f', 'rawvideo',
            '-pix_fmt', self.pixel_format,
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', 'pipe:0',  # Read from stdin
            '-c:v', self.codec,
            '-preset', self.preset,
            '-pix_fmt', 'yuv420p',  # Output format for compatibility
            '-movflags', '+faststart',  # Optimize for streaming
        ]
        
        # Add quality settings
        if self.quality_config['bitrate']:
            cmd.extend(['-b:v', self.quality_config['bitrate']])
        else:
            cmd.extend(['-crf', str(self.quality_config['crf'])])
        
        # Add output file
        cmd.append(str(self.output_path))
        
        return cmd
    
    def _start_queue_processing(self):
        """Start background thread for processing frame queue."""
        self._queue_thread = threading.Thread(target=self._process_frame_queue, daemon=True)
        self._queue_thread.start()
        logger.debug(f"Started frame processing thread for {self.camera_name}")
    
    def _process_frame_queue(self):
        """Process frames from queue and write to FFmpeg."""
        while not self._stop_queue:
            try:
                # Get frame from queue with timeout
                frame_data = self._frame_queue.get(timeout=0.1)
                
                if frame_data is None:  # Shutdown signal
                    break
                
                frame, timestamp = frame_data
                
                # Write frame to FFmpeg stdin
                if self.process and self.process.stdin:
                    self.process.stdin.write(frame.tobytes())
                    self.process.stdin.flush()
                    
                    # Record timestamp
                    with self._timestamps_lock:
                        self._timestamps.append(timestamp)
                    
                    self._frame_count += 1
                    
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error processing frame for {self.camera_name}: {e}")
                self._error_occurred = True
                self._error_message = str(e)
                break
    
    def write_frame(self, frame: np.ndarray, timestamp: Optional[float] = None) -> bool:
        """
        Write a frame to the video.
        
        Args:
            frame: Image frame as numpy array (height, width, channels)
            timestamp: Optional timestamp for the frame
            
        Returns:
            True if frame was queued successfully, False otherwise
        """
        if self._error_occurred:
            logger.warning(f"Cannot write frame to {self.camera_name}: {self._error_message}")
            return False
        
        if not self._is_writing:
            logger.warning(f"Video writer for {self.camera_name} is not active")
            return False
        
        # Validate frame
        if not self._validate_frame(frame):
            logger.warning(f"Frame validation failed for {self.camera_name}: shape={frame.shape}, expected=({self.height}, {self.width}, 3)")
            return False
        
        # Use current time if no timestamp provided
        if timestamp is None:
            timestamp = time.time()
        
        try:
            # Add frame to queue (non-blocking)
            self._frame_queue.put_nowait((frame, timestamp))
            return True
            
        except queue.Full:
            logger.warning(f"Frame queue full for {self.camera_name}, dropping frame")
            return False
        except Exception as e:
            logger.error(f"Error queuing frame for {self.camera_name}: {e}")
            return False
    
    def _validate_frame(self, frame: np.ndarray) -> bool:
        """Validate frame dimensions and format."""
        if frame is None:
            logger.warning(f"None frame received for {self.camera_name}")
            return False
        
        if len(frame.shape) != 3:
            logger.warning(f"Invalid frame shape for {self.camera_name}: {frame.shape}")
            return False
        
        h, w, c = frame.shape
        if h != self.height or w != self.width:
            logger.warning(f"Frame size mismatch for {self.camera_name}: {frame.shape} vs expected ({self.height}, {self.width}, 3)")
            return False
        
        if c != 3:
            logger.warning(f"Invalid channel count for {self.camera_name}: {c}, expected 3")
            return False
        
        return True
    
    def get_timestamps(self) -> List[float]:
        """Get list of frame timestamps."""
        with self._timestamps_lock:
            return self._timestamps.copy()
    
    def get_frame_count(self) -> int:
        """Get number of frames written."""
        return self._frame_count
    
    def is_healthy(self) -> bool:
        """Check if video writer is healthy."""
        return self._is_writing and not self._error_occurred and self.process is not None
    
    def get_error_message(self) -> Optional[str]:
        """Get error message if any error occurred."""
        return self._error_message
    
    def close(self):
        """Close video writer and finalize video file."""
        logger.info(f"Closing video writer for {self.camera_name}")
        
        # Stop queue processing
        self._stop_queue = True
        if self._queue_thread:
            self._queue_thread.join(timeout=2.0)
        
        # Signal end of input to FFmpeg
        if self.process and self.process.stdin:
            try:
                self.process.stdin.close()
            except Exception as e:
                logger.warning(f"Error closing FFmpeg stdin: {e}")
        
        # Wait for FFmpeg to finish
        if self.process:
            try:
                # Check if process is still running
                if self.process.poll() is None:
                    stdout, stderr = self.process.communicate(timeout=10.0)
                    
                    if self.process.returncode != 0:
                        logger.error(f"FFmpeg process failed with return code {self.process.returncode}")
                        if stderr:
                            logger.error(f"FFmpeg stderr: {stderr.decode()}")
                    else:
                        logger.info(f"Video encoding completed for {self.camera_name}: {self._frame_count} frames")
                else:
                    logger.info(f"FFmpeg process already finished for {self.camera_name}")
                    
            except subprocess.TimeoutExpired:
                logger.error(f"FFmpeg process timed out for {self.camera_name}")
                self.process.kill()
            except Exception as e:
                # Don't log I/O errors on closed files as errors
                if "I/O operation on closed file" not in str(e):
                    logger.error(f"Error waiting for FFmpeg process: {e}")
                else:
                    logger.debug(f"FFmpeg process cleanup: {e}")
        
        self._is_writing = False
        logger.info(f"Video writer closed for {self.camera_name}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class MultiCameraVideoWriter:
    """
    Manager for multiple video writers.
    
    Handles multiple cameras simultaneously with synchronized writing.
    """
    
    def __init__(self, output_dir: str, camera_configs: Dict[str, Dict]):
        """
        Initialize multi-camera video writer.
        
        Args:
            output_dir: Base output directory for videos
            camera_configs: Dictionary mapping camera names to configurations
                Example: {
                    'cam_top': {
                        'width': 1280,
                        'height': 720,
                        'fps': 30,
                        'quality': 'medium'
                    }
                }
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create cameras subdirectory
        self.cameras_dir = self.output_dir / 'cameras'
        self.cameras_dir.mkdir(exist_ok=True)
        
        self.camera_configs = camera_configs
        self.writers = {}
        
        # Initialize video writers for each camera
        for cam_name, config in camera_configs.items():
            output_path = self.cameras_dir / f"{cam_name}.mp4"
            
            writer = VideoWriter(
                output_path=str(output_path),
                width=config['width'],
                height=config['height'],
                fps=config.get('fps', 30),
                quality=config.get('quality', 'medium'),
                preset=config.get('preset', 'fast'),
                pixel_format=config.get('pixel_format', 'rgb24'),
                camera_name=cam_name
            )
            
            self.writers[cam_name] = writer
            logger.info(f"Initialized video writer for {cam_name}")
    
    def write_frames(self, frames: Dict[str, np.ndarray], timestamps: Optional[Dict[str, float]] = None) -> Dict[str, bool]:
        """
        Write frames for all cameras.
        
        Args:
            frames: Dictionary mapping camera names to frames
            timestamps: Optional dictionary mapping camera names to timestamps
            
        Returns:
            Dictionary mapping camera names to success status
        """
        results = {}
        
        for cam_name, frame in frames.items():
            if cam_name in self.writers:
                timestamp = timestamps.get(cam_name) if timestamps else None
                success = self.writers[cam_name].write_frame(frame, timestamp)
                results[cam_name] = success
            else:
                logger.warning(f"No video writer found for camera {cam_name}")
                results[cam_name] = False
        
        return results
    
    def get_timestamps(self) -> Dict[str, List[float]]:
        """Get timestamps for all cameras."""
        return {name: writer.get_timestamps() for name, writer in self.writers.items()}
    
    def get_frame_counts(self) -> Dict[str, int]:
        """Get frame counts for all cameras."""
        return {name: writer.get_frame_count() for name, writer in self.writers.items()}
    
    def is_healthy(self) -> bool:
        """Check if all video writers are healthy."""
        return all(writer.is_healthy() for writer in self.writers.values())
    
    def get_errors(self) -> Dict[str, Optional[str]]:
        """Get error messages for all cameras."""
        return {name: writer.get_error_message() for name, writer in self.writers.items()}
    
    def close(self):
        """Close all video writers."""
        logger.info("Closing all video writers")
        for writer in self.writers.values():
            writer.close()
        logger.info("All video writers closed")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def test_video_writer():
    """Test function for VideoWriter class."""
    import cv2
    
    # Test single camera
    print("Testing single camera video writer...")
    
    with VideoWriter(
        output_path="test_output/test_camera.mp4",
        width=640,
        height=480,
        fps=30,
        quality='medium',
        camera_name='test_cam'
    ) as writer:
        
        # Generate test frames
        for i in range(100):
            # Create a test frame with moving pattern
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.circle(frame, (i * 6 % 640, 240), 50, (255, 255, 255), -1)
            
            success = writer.write_frame(frame, time.time())
            if not success:
                print(f"Failed to write frame {i}")
                break
            
            time.sleep(0.033)  # ~30 FPS
        
        print(f"Written {writer.get_frame_count()} frames")
        print(f"Timestamps: {len(writer.get_timestamps())}")
    
    print("Single camera test completed")
    
    # Test multi-camera
    print("Testing multi-camera video writer...")
    
    camera_configs = {
        'cam_left': {'width': 320, 'height': 240, 'fps': 30, 'quality': 'medium'},
        'cam_right': {'width': 320, 'height': 240, 'fps': 30, 'quality': 'medium'}
    }
    
    with MultiCameraVideoWriter("test_output", camera_configs) as multi_writer:
        
        for i in range(50):
            frames = {}
            for cam_name in camera_configs.keys():
                frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
                frames[cam_name] = frame
            
            results = multi_writer.write_frames(frames)
            print(f"Frame {i}: {results}")
            
            time.sleep(0.033)
        
        print(f"Frame counts: {multi_writer.get_frame_counts()}")
    
    print("Multi-camera test completed")


if __name__ == "__main__":
    test_video_writer()
