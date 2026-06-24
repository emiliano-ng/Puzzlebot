# Puzzlebot - Autonomous Navigation with Vision and TensorRT

A full autonomous-driving pipeline for a differential-drive **Puzzlebot** running **ROS 2 Humble** on a Jetson board. The robot follows a line, detects pedestrian crosswalks, classifies traffic lights, and reacts to road signs in real time using a custom **YOLOv4-tiny model accelerated with NVIDIA TensorRT (FP16)**.

Developed as a final project at Tecnológico de Monterrey, with Arturo Balboa, Oscar de la Rosa, Angel Hernandez, **Emiliano Niño**, and Rigoberto Soto

---

## Overview

The system integrates multiple perception inputs through a state machine that issues wheel-velocity commands. Everything runs onboard — no offloaded compute.

**What the robot does:**
- Follows a dark line using Otsu thresholding and contour analysis
- Detects pedestrian crosswalks and corrects alignment angle
- Classifies traffic lights (red / yellow / green) via HSV segmentation
- Recognizes 6 road signs in real time (turn left/right, stop, straight, give way, work ahead)
- Arbitrates all inputs through a state machine with PID-based motion control

---

## System Architecture

```
                      ┌─────────────┐
                      │  cam_publish │  GStreamer / NVMM (Jetson CSI)
                      └──────┬──────┘
                             │ /cam/img_raw
      ┌──────────────────────┼──────────────────────┐
      ▼                      ▼                       ▼
┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  line_follower  │  │  traffic_detect  │  │  sign_detect_trt │
│  (Python/OpenCV)│  │  (HSV, Python)   │  │  (C++/TensorRT)  │
└────────┬────────┘  └────────┬─────────┘  └────────┬─────────┘
         │                    │                       │
 /line_detector_error  /Traffic_light          /traffic_sign
 /crosswalk_bool                               /stop_area
 /crosswalk_ang
         │                    │                       │
         └────────────────────▼───────────────────────┘
                       ┌──────────────┐
                       │  line_follow  │  State machine + dual PID
                       └──────┬───────┘
                              │
                 /VelocitySetL  /VelocitySetR
```

---

## Repository Structure

```
src/
├── vision_puzzlebot/           # Python pkg — camera, line follower, traffic-light detector
│   ├── config/
│   │   ├── params.yaml         # All tunable parameters (HSV ranges, PID gains, speed limits)
│   │   └── crosswalk_debug_conf.yaml
│   ├── launch/
│   │   ├── final_launch.launch.py
│   │   └── puzzlebot_line_follower.launch.py
│   └── vision_puzzlebot/
│       ├── cam_publish.py          # GStreamer/NVMM camera node
│       ├── line_follower_camera.py # Line following + crosswalk detection
│       └── trafficlight_detect.py  # HSV traffic light classifier
├── vision_puzzlebot_trt/       # C++ pkg — TensorRT sign detection
│   ├── models/
│   │   ├── yolov4-tiny-signs.onnx
│   │   ├── yolov4-tiny-signs_best_fp16.engine
│   │   └── obj.names
│   └── src/
│       └── sign_detect_trt.cpp
└── waypoints_puzzlebot/        # Python pkg — state machine, PID controller
    └── waypoints_puzzlebot/
        ├── line_follow.py      # Main controller (state machine + dual PID)
        └── 8_waypoints.py      # Alternative waypoint-based controller
```

---

## State Machine

```
              ┌─────────────┐
      ┌──────▶│ LINE_FOLLOW │◀──────────────────────────┐
      │       └──────┬──────┘                           │
      │              │ crosswalk detected                │
      │              ▼                                   │
      │       ┌─────────────────┐                       │ done
      └───────│ ADVANCE_TO_CROSS│   sign detected:       │
              └─────────────────┘                       │
                                   stop ──────────▶  STOP (wait for green)
                                   turn_right ───▶  TURN_RIGHT
                                   turn_left ────▶  TURN_LEFT
                                   straight ─────▶  STRAIGHT
                                   give_way ─────▶  GIVE_WAY (speed reduce)
                                   work_ahead ───▶  (speed flag, no state change)
```

A **voting counter (min 3 detections)** filters noisy YOLO outputs before triggering any state transition.

---

## Detected Classes

| Class | Robot Behavior |
|---|---|
| `stop` | Halt until traffic light turns green |
| `turn_left` | Execute timed left turn, resume line following |
| `turn_right` | Execute timed right turn, resume line following |
| `straight` | Override intersection, continue forward |
| `give_way` | Reduce speed momentarily |
| `work_ahead` | Reduce speed for 2 s (0.06 m/s), no state change |

---

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/cam/img_raw` | `sensor_msgs/Image` | Raw camera frames (640×480, 20 fps) |
| `/line_detector_error` | `std_msgs/Float32` | Signed lateral error [−1, 1] |
| `/crosswalk_bool` | `std_msgs/Bool` | Crosswalk detected flag |
| `/crosswalk_ang` | `std_msgs/Float32` | Crosswalk alignment angle |
| `/Traffic_light` | `std_msgs/String` | `"red"` / `"yellow"` / `"green"` / `"none"` |
| `/traffic_sign` | `std_msgs/String` | Detected sign class |
| `/stop_area` | `std_msgs/Float32` | STOP sign bounding-box area (proxy for distance) |
| `/VelocitySetL` | `std_msgs/Float32` | Left wheel velocity setpoint |
| `/VelocitySetR` | `std_msgs/Float32` | Right wheel velocity setpoint |

---

## Key Technical Decisions

**Why TensorRT FP16 for sign detection?**  
The Jetson board has limited compute. Running YOLOv4-tiny as a standard ONNX model was too slow for real-time control. Compiling to a TensorRT FP16 engine cut inference latency enough to keep up with the 20 fps camera stream without dropping the control loop frequency.

**Why a voting counter for sign detection?**  
Single-frame YOLO detections are noisy — a sign can appear for one frame and disappear. Requiring 3 consecutive positive detections before triggering a state transition eliminated false positives almost entirely without adding meaningful delay.

**Why separate PID controllers for angular and linear?**  
The line follower and the waypoint maneuvers (turn left/right/straight) have fundamentally different control objectives. Mixing them into one controller caused instability during transitions. Splitting into angular PID (for line tracking) and linear PID (for distance-controlled maneuvers) made each independently tunable.

**Odometry without external localization:**  
Wheel encoder velocities are integrated using a differential-drive kinematic model to estimate robot pose during waypoint maneuvers. No external localization (GPS, LiDAR SLAM) is used — the robot navigates purely from encoder data and visual feedback.

---

## Configuration

All tunable parameters are in `src/vision_puzzlebot/config/params.yaml`. No recompilation needed:

```yaml
vision_traffic:
  red_hsv_lower1: [0, 100, 100]
  red_hsv_upper1: [10, 255, 255]
  # ... additional HSV ranges

control_line_follower:
  kp_angular: 0.7
  kd_angular: 0.6
  kp_linear: 0.65
  base_speed: 0.12

vision_YOLO:
  engine_path: "models/yolov4-tiny-signs_best_fp16.engine"
  confidence_threshold: 0.75
  nms_threshold: 0.45
```

---

## Build & Run

```bash
cd ~/ros2_ws
pip3 install numpy opencv-python
colcon build --symlink-install
source install/setup.bash

# Full pipeline (line following + sign detection + traffic lights)
ros2 launch vision_puzzlebot final_launch.launch.py
```

> **TensorRT engines** are pre-compiled for a specific Jetson hardware/TensorRT version. To rebuild from ONNX:
> ```bash
> trtexec --onnx=yolov4-tiny-signs.onnx \
>         --saveEngine=yolov4-tiny-signs_best_fp16.engine \
>         --fp16
> ```

---

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| OpenCV | ≥ 4.5 |
| NVIDIA TensorRT | ≥ 8 |
| CUDA | ≥ 11 |
| cv_bridge | — |
| micro_ros_agent | — |

---

## Limitations and Future Work

- Maneuver timing (turn duration) is open-loop and calibrated for a specific speed — changes in battery level or surface friction affect accuracy.
- Traffic light detection relies on HSV thresholds that are sensitive to lighting changes; a learned classifier would generalize better.
- Sign detection requires a Jetson with TensorRT; the `.onnx` models are provided for portability to other platforms.
- Next steps: SLAM integration for closed-loop waypoint navigation, dynamic speed adaptation based on battery state.