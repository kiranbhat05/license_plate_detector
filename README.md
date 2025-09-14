# license_plate_detector_pkg

ROS1 (Noetic) package for **license plate detection** using **YOLOv5-Nano** (custom `alpr_weights.pt`) + **OpenALPR**, with:
- frame **preprocessing** (split/crop to left/right),
- **GNSS** adjustment based on **IMU yaw** and left/right view,
- **RSA encryption** of plate text (pubkey),
- JSON publisher, and an optional **map plotter** (decrypts with privkey) that saves an HTML map.

---

## Quick Start

```bash
# Clone & build
cd ~/catkin_ws/src
git clone https://github.com/kiranbhat05/license_plate_detector.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

# Run
roslaunch license_plate_detector_pkg license_plate_detector.launch
```

---

## Nodes (what they do)

- `license_plate_detector_preprocessor.py`  
  Subscribes `/image_raw` → splits L/R → crops → publishes:
  `processed_left_frame`, `processed_right_frame`.

- `license_plate_detector.py` *(x2: left & right)*  
  Subscribes one of the processed topics → YOLOv5 (if `use_yolo`) → OpenALPR →  
  adjusts GNSS by orientation & view → **encrypts plate** → publishes CSV on `license_plate_data`.

- `license_plate_json_publisher.py`  
  Converts `license_plate_data` CSV → JSON on `number_plate_json`.

- `map_plotter.py` *(optional)*  
  Subscribes `license_plate_data`, **decrypts** plate using private key, and saves **`license_plate_map.html`** on shutdown.

---

## Topics (I/O)

**Input**
- `/image_raw` (`sensor_msgs/Image`)
- `/gps/fix` (`sensor_msgs/NavSatFix`)
- `/yaw` (`sensor_msgs/Imu`) — used for yaw → N/NE/E/… orientation buckets

**Output**
- `processed_left_frame`, `processed_right_frame` (`sensor_msgs/Image`)
- `license_plate_data` (`std_msgs/String`) → CSV:  
  `automobile_id,date_time,encrypted_plate_hex,lat,lon,alt`
- `number_plate_json` (`std_msgs/String`) → JSON object with same fields

---

## Parameters (main)

Preprocessor:
- `~left_width_percent` (default `50`)
- `~left_image_left/right/top/bottom_crop` (percent)
- `~right_image_left/right/top/bottom_crop` (percent)

Detector:
- `~topic_name` (`processed_left_frame` / `processed_right_frame`)
- `~gps_topic` (default `/gps/fix`)
- `~orientation_topic` (default `/yaw`)
- `~use_yolo` (`true`/`false`, default `true`)
- (internal path) YOLO weights: `data/alpr_weights.pt`

---

## File layout (key bits)

```
license_plate_detector_pkg/
├─ launch/
│  └─ license_plate_detector.launch
├─ src/
│  ├─ license_plate_detector_preprocessor.py
│  ├─ license_plate_detector.py
│  ├─ license_plate_json_publisher.py
│  └─ map_plotter.py
├─ data/
│  └─ alpr_weights.pt                # YOLOv5-Nano weights
├─ lib/
│  └─ runtime_data/                  # OpenALPR runtime data
├─ certs/
│  ├─ public_key.pem                 # for encryption (detector)
│  └─ private_key.pem                # for decryption (map_plotter)
├─ package.xml
└─ CMakeLists.txt
```

---

## Dependencies

ROS: `rospy`, `sensor_msgs`, `std_msgs`, `cv_bridge`

Python libs (runtime):
- `torch`, `opencv-python`, `numpy`
- `openalpr` (with runtime data in `lib/runtime_data`)
- `cryptography`, `plotly`

Example install:
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install opencv-python numpy cryptography plotly
# YOLOv5 hub dependency (used via torch.hub):
pip install git+https://github.com/ultralytics/yolov5.git
# OpenALPR: install per your OS; ensure runtime data path matches lib/runtime_data
```

---

## Launch (provided)

```xml
<launch>
  <node pkg="license_plate_detector_pkg" type="license_plate_detector_preprocessor.py" name="license_plate_detector_preprocessor" output="screen">
    <!-- tune crops -->
    <param name="left_width_percent" value="50"/>
    <param name="left_image_left_side_crop" value="25"/>
    <param name="left_image_right_side_crop" value="35"/>
    <param name="left_image_top_crop" value="33"/>
    <param name="left_image_bottom_crop" value="33"/>
    <param name="right_image_left_side_crop" value="32"/>
    <param name="right_image_right_side_crop" value="32"/>
    <param name="right_image_top_crop" value="33"/>
    <param name="right_image_bottom_crop" value="33"/>
  </node>

  <node pkg="license_plate_detector_pkg" type="license_plate_detector.py" name="license_plate_detector_left"  output="screen">
    <param name="topic_name" value="processed_left_frame"/>
    <param name="use_yolo"   value="true"/>
  </node>

  <node pkg="license_plate_detector_pkg" type="license_plate_detector.py" name="license_plate_detector_right" output="screen">
    <param name="topic_name" value="processed_right_frame"/>
    <param name="use_yolo"   value="true"/>
  </node>

  <node pkg="license_plate_detector_pkg" type="map_plotter.py" name="map_plotter" output="screen"/>
  <node pkg="license_plate_detector_pkg" type="license_plate_json_publisher.py" name="license_plate_json_publisher" output="screen"/>
</launch>
```

---

## Notes / Gotchas

- **Weights:** keep `data/alpr_weights.pt` in place; detector loads via `torch.hub` (`ultralytics/yolov5`, `custom`).
- **Keys:** `license_plate_detector.py` **encrypts** with `certs/public_key.pem`.  
  `map_plotter.py` **decrypts** with `certs/private_key.pem`.
- **OpenALPR runtime:** path is resolved as `src/../lib/runtime_data`. Ensure it exists & matches your region (package uses `"eu"`).
- **GNSS adjust:** small meter-level shifts applied based on **orientation** (derived from IMU yaw) and **left/right** topic to better associate detections to the viewing side.

---

## License

BSD (see `package.xml`).
