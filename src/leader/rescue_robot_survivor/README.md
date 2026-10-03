# rescue_robot_survivor

## 개요와 현재 상태

`rescue_robot_survivor`는 Leader의 Intel RealSense D435 RGB 영상에서 생존자 후보인
사람을 검출하는 독립 ROS 2 Humble 패키지다. AprilTag는 구호물품 인식과 접근을,
이 패키지는 생존자 인식을 담당하므로 모델 의존성, 실행 주기와 이후 depth 처리를 서로
섞지 않도록 별도 패키지로 분리했다.

**현재 상태: Stage 1 VERIFIED / Stage 3 Camera XYZ VERIFIED /
Stage 4 Map XYZ VERIFIED / Stage 5 RViz Visualization VERIFIED (PASS) /
Stage 6 Persistent Survivor Registry VERIFIED /
Stage 6.1 Registry Robustness + Visualization VERIFIED (실제 multi-person physical validation 포함)**

코드, package build와 자동 테스트를 완료했다. survivor 전용 Jetson Docker GPU runtime에서
YOLO11n 추론을 수행하고 실제 D435 화면에서 1명, 2명, 3명 검출을 확인했다. 각 사람의
bounding box와 confidence, 왼쪽에서 오른쪽 순서의 `person1..N` 표시 및
`/leader/survivor/debug_image`를 rqt_image_view에서 확인했다. `personN`은 persistent
tracking ID가 아닌 현재 프레임의 표시 번호다.

Stage 2의 aligned-depth 거리 코드는 구현되었지만 정식 줄자 기준 거리표는 아직 남아 있어
`IMPLEMENTED - HARDWARE VERIFICATION REQUIRED`다. Stage 3는 실제 Jetson + D435에서 사람의
camera optical XYZ, 좌/우 X 부호, distance와 Z의 일치, debug overlay, 다중 사람별 XYZ와
`/leader/survivor/camera_positions` PoseArray를 확인해 `VERIFIED`로 승격했다. Stage 1의
`VERIFIED` 판정도 유지된다.

Stage 4는 `survivor_map_transform_node`에서 원본 detection timestamp의 TF2를 조회해
`/leader/survivor/map_positions` (`PoseArray`, `header.frame_id=map`)를 발행한다. TF lookup가
실패하면 최신 TF로 대체하지 않고 해당 메시지를 skip한다. 실제 Jetson + D435에서 정지
상태와 고정된 사람의 A→B 저속 이동을 검증했다. Camera XYZ는 크게 변했고 map XYZ는
같은 사람 주변에 유지됐지만 raw A→B map 평균 차이는 약 0.115 m였다.

Stage 5는 `survivor_map_visualizer_node`가 map 후보를 sphere/text로 표시하고
`/leader/survivor/map_markers`를 발행한다. 실제 RViz에서 단일·다중 후보,
nvblox 3D mesh 공존, FOV 이탈 후 lifetime 삭제와 재진입, 통제된 2명→1명
old marker 제거를 수동 검증했다. VSLAM·local/global EKF·nvblox·Survivor
동시 실행도 확인했다. 상세 결과는
[Stage 5 validation](../../../docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE5_RVIZ_VISUALIZATION_VALIDATION.md)에
기록했다. `Survivor candidate N`은 현재 PoseArray 인덱스 기반 임시 번호다.
사람이 위치를 바꾸거나 검출 순서가 변하면 번호가 달라질 수 있다.

## Jetson GPU runtime

Host global Python에는 AI package를 설치하지 않는다. 재현 가능한 survivor 전용 image를
사용한다.

```bash
cd ~/damgc_robot
./scripts/build_survivor_runtime.sh
./scripts/run_survivor_detector.sh
```

Runtime smoke test:

```bash
docker run --rm --runtime=nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v ~/.cache/damgc-survivor-ultralytics:/root/.cache/ultralytics \
  damgc-survivor-yolo:humble 'check_survivor_ai_runtime --static'
```

검증된 조합은 JetPack 6.2.3/L4T R36.5.2, Python 3.10, NVIDIA CUDA PyTorch
`2.5.0a0+872d972e41.nv24.08`, CUDA-enabled source-built torchvision `0.20.0`,
Ultralytics `8.4.147`이다. torchvision은 torch를 교체하지 않고 `FORCE_CUDA=1` 및
Orin `sm_87` 대상으로 build한다. check script는 CUDA NMS와 YOLO11n GPU inference까지
확인한다.

## Stage 1 보호 기능과 Stage 2/3 범위

현재 구현은 다음 기능을 지원한다.

- `/leader/camera/color/image_rect`를 sensor-data QoS로 구독
- Ultralytics `yolo11n.pt` COCO pretrained detection 사용
- COCO class 0인 `person`만 confidence threshold로 필터링
- 한 프레임의 모든 사람을 중심 x 좌표 기준 왼쪽에서 오른쪽으로 정렬
- `person1 0.94 | 1.24 m` 형식의 번호, confidence, bounding box와 거리 표시
- `XYZ (-0.42, 0.08, 1.24) m` 형식의 camera optical position 표시
- 원본 `header.stamp`와 `header.frame_id`를 유지한 debug image 발행
- valid camera XYZ를 `/leader/survivor/camera_positions` `PoseArray`로 발행
- CUDA 사용 가능 시 GPU 선택, 불가능하면 CPU 선택(`device=auto`)

`person1`, `person2`는 **현재 프레임의 표시용 번호**다. tracking ID가 아니며 사람이
움직이거나 서로 교차하거나 검출이 누락되면 다음 프레임에서 번호가 달라질 수 있다.
ByteTrack, BoT-SORT, DeepSORT와 appearance re-identification은 구현하지 않는다. Persistent
mission-runtime ID는 별도 Registry node가 map 좌표 spatial association으로 관리한다.

Stage 2 범위는 bounding box 중심 ROI의 median 거리까지다. Stage 3는 같은 ROI 중심 pixel과
median Z, rectified RGB CameraInfo.P를 결합해 camera optical XYZ를 계산한다.
이 절의 Stage 2/3 처리와 별도로 Stage 4 TF2 map 변환과 Stage 5 RViz marker가
완료됐다. Stage 6에서 spatial association 기반 persistent ID, 위치 filtering, registry와
survivor confirmation을 추가했다. Mission Coordinator, custom training과 TensorRT
최적화, process/map-session 외부 ID 저장은 현재 구현 범위에 없다.

RGB, depth, aligned-depth를 포함한 D435 camera pipeline도 정상 기동 및 topic 발행을
확인했다. Stage 2는 YOLO bounding box와 aligned depth를 결합해 중심 ROI의 유효 depth만
추출하고, zero/NaN/Inf/range 밖 값을 제거한 median depth로 사람까지의 거리[m]를 계산한다.
aligned depth가 없거나 RGB와 timestamp 차이가 `sync_slop_sec`보다 크면 detection은 계속
발행하고 거리만 `N/A`로 표시한다.

Stage 3는 실제 확인된 rectified RGB, color-aligned depth와 CameraInfo가 모두 `640×480`,
`camera_color_optical_frame` geometry를 공유하는 조건에서 동작한다. CameraInfo가 없거나 P,
resolution 또는 frame ID가 유효하지 않으면 YOLO와 distance를 유지하고 `XYZ N/A` 및 빈
PoseArray로 graceful degradation한다.

## 구조

```text
rescue_robot_survivor/
├── rescue_robot_survivor/
│   ├── detection_logic.py       # person 필터, 좌표 보정, 정렬, drawing
│   ├── depth_logic.py           # ROI, scale, filtering, median
│   ├── geometry_logic.py        # P 검증, ROI 중심, camera deprojection
│   ├── person_detector_node.py  # ROS 구독/cache, YOLO/depth/XYZ, output
│   ├── survivor_map_transform_node.py # exact-time camera → map
│   ├── survivor_map_visualizer_node.py # map pose → sphere/text MarkerArray
│   ├── survivor_registry_core.py      # ROS-independent association/lifecycle core
│   ├── survivor_registry_node.py      # map_positions → typed tracks + reset
│   └── survivor_registry_visualizer_node.py # tracks → persistent MarkerArray
├── launch/person_detector.launch.py
├── launch/survivor_map_transform.launch.py
├── launch/survivor_map_visualizer.launch.py
├── config/person_detector.yaml
├── config/survivor_registry.yaml
├── test/                        # 순수 로직과 launch/config 계약 테스트
├── docs/SURVIVOR_DEVELOPMENT_PLAN.md
├── docs/STAGE1_PERSON_DETECTION_VALIDATION.md
├── docs/STAGE2_DEPTH_DISTANCE_VALIDATION.md
├── docs/STAGE3_CAMERA_XYZ_IMPLEMENTATION.md
├── docs/STAGE3_CAMERA_XYZ_VALIDATION.md
├── package.xml
├── setup.py
└── setup.cfg
```

Typed interfaces are provided by the sibling `rescue_robot_interfaces` package:
`SurvivorTrack.msg` and `SurvivorTrackArray.msg`.

각 launch는 detector, map transform, visualizer를 각각 실행한다. RealSense camera를
시작하거나 기존 AprilTag launch를 include하지 않는다. Detector YAML 값은 launch
argument의 기본값으로 읽히므로 config 수정과 실행 시 override를 모두 지원한다.

## ROS 인터페이스와 파라미터

| 구분 | 기본값 | 타입/설명 |
| --- | --- | --- |
| 입력 | `/leader/camera/color/image_rect` | `sensor_msgs/msg/Image`, 확인값 `rgb8` |
| 입력 | `/leader/camera/aligned_depth_to_color/image_raw` | `Image`, 확인값 `16UC1` |
| 입력 | `/leader/camera/color/camera_info` | `CameraInfo`, rectified P 사용 |
| 출력 | `/leader/survivor/debug_image` | `sensor_msgs/msg/Image`, `bgr8` |
| 출력 | `/leader/survivor/camera_positions` | `geometry_msgs/msg/PoseArray`, valid XYZ만 |
| 출력 | `/leader/survivor/map_positions` | `geometry_msgs/msg/PoseArray`, map frame, exact input stamp |
| 출력 | `/leader/survivor/map_markers` | `visualization_msgs/msg/MarkerArray`, sphere/text 및 DELETE |
| `image_topic` | 위 입력 | 다른 RGB 토픽으로 변경 |
| `debug_image_topic` | 위 출력 | debug 영상 토픽 변경 |
| `model_name` | `yolo11n.pt` | 모델 이름 또는 로컬 weight 경로 |
| `confidence_threshold` | `0.5` | 0.0–1.0 |
| `device` | `auto` | `auto`, `cpu`, `0` 등 |
| `aligned_depth_topic` | `/leader/camera/aligned_depth_to_color/image_raw` | RGB 정렬 depth |
| `depth_roi_width_ratio` / `height_ratio` | `0.25` / `0.25` | bbox 중심 ROI 비율 |
| `min_depth_m` / `max_depth_m` | `0.2` / `6.0` | 유효 거리 범위 |
| `min_valid_depth_pixels` | `20` | 최소 유효 pixel 수 |
| `depth_scale_m_per_unit` | `0.001` | integer depth raw 단위 변환값 |
| `show_depth_roi` | `false` | debug ROI 표시 |
| `sync_queue_size` | `5` | 최근 depth cache 크기 |
| `sync_slop_sec` | `0.12` | RGB/depth 허용 timestamp 차이 |
| `camera_info_topic` | `/leader/camera/color/camera_info` | RGB CameraInfo 입력 |
| `camera_positions_topic` | `/leader/survivor/camera_positions` | camera XYZ 출력 |
| `map_transform.input_topic` | `/leader/survivor/camera_positions` | Stage 4 input |
| `map_transform.output_topic` | `/leader/survivor/map_positions` | Stage 4 output |
| `map_transform.target_frame` | `map` | map target frame |
| `map_transform.tf_timeout_sec` | `0.6` | exact-time lookup timeout; fallback 없음 |
| `map_visualizer.marker_lifetime_sec` | `2.0` | finite lifetime; 무입력 시 RViz에서 만료 |
| `map_visualizer.text_z_offset` | `0.30` | text 표시 Z에만 적용, map XYZ 불변 |
| `show_camera_xyz` | `true` | debug image XYZ 둘째 줄 표시 |

image 입출력은 `BEST_EFFORT`, `VOLATILE`, `KEEP_LAST(1)`을 사용한다. CameraInfo는 실제
RealSense publisher와 호환되는 `RELIABLE`, `VOLATILE`, `KEEP_LAST(1)`, PoseArray는
`RELIABLE`, `VOLATILE`, `KEEP_LAST(1)`이다.

## Dependencies

ROS 의존성은 `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`,
`tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, `python3-opencv`, `python3-numpy`,
`launch`, `launch_ros`다. 추론에는 별도로 PyTorch, torchvision과 Ultralytics가 필요하다.

Jetson에서는 일반 PyPI torch/torchvision으로 기존 NVIDIA build를 교체하면 안 된다.
다음 확인을 먼저 수행한다.

```bash
python3 --version
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
python3 -c "import torchvision; print(torchvision.__version__)"
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

초기 host 조사에서는 Python 3.10.12, JetPack 6.2.3/CUDA 12.6 환경에 AI 패키지가
미설치였다. 이후 위 survivor 전용 container 조합을 구성·검증했다. 이 패키지의
build 과정은 AI 패키지를 자동 설치하지 않는다.

설치 기준은 [NVIDIA PyTorch for Jetson 설치 문서](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)와
[Ultralytics Jetson 가이드](https://docs.ultralytics.com/guides/nvidia-jetson/)다. 확인 시점의
Ultralytics native 예시는 JetPack 6.1용 조합까지만 명시하므로, 현재 장비의 JetPack
6.2.3에 그 wheel을 그대로 설치하지 않는다. 정확한 6.2.3 호환 조합을 확인하거나 격리된
JetPack 6 container를 선택한 뒤 사용자가 설치해야 한다.

`yolo11n.pt`가 로컬에 없으면 Ultralytics가 최초 실행 시 다운로드할 수 있다. Jetson이
오프라인이면 weight를 미리 복사하고 `model_name:=/absolute/path/yolo11n.pt`로 지정한다.

## Host camera와 container detector 실행

```bash
cd ~/damgc_robot
./scripts/build_survivor_runtime.sh
```

기존 camera launch는 host에서 먼저 실행한다. detector는 host global Python이 아니라 전용
container에서 실행한다.

```bash
# Host terminal
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=true enable_sync:=true align_depth.enable:=true

# Another terminal
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

파라미터 override 예시는 다음과 같다.

```bash
./scripts/run_survivor_detector.sh \
  confidence_threshold:=0.6 device:=cpu
```

화면은 다음 중 하나로 확인한다.

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
# GUI에서 /leader/survivor/debug_image 선택

ros2 run rqt_image_view rqt_image_view /leader/survivor/debug_image
```

정상 상태에서는 사람이 없을 때 원본 영상만 보이고, 사람이 있으면 각 사람 주위에 초록색
box, `personN confidence | distance m`와 `XYZ (x, y, z) m`가 표시된다. 사용할 수 있는
depth가 없으면 distance와 XYZ가 `N/A`, CameraInfo만 유효하지 않으면 distance는 유지되고
`XYZ N/A`가 표시된다.

camera position은 다음과 같이 확인한다.

```bash
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic echo /leader/survivor/camera_positions
```

PoseArray header는 원본 RGB stamp와 optical frame을 유지하고 orientation은 identity
quaternion이다. valid XYZ만 왼쪽→오른쪽 순서로 포함하므로 중간 person이 invalid이면 array
index가 debug image의 person 번호와 같지 않을 수 있다.

## Known limitations

- frame-local 번호이므로 프레임 간 ID가 유지되지 않는다.
- COCO pretrained 모델은 누운 사람, 심한 가림, 작은 사람 또는 마네킹을 놓칠 수 있다.
- 마네킹 실패 시 Stage 1에서 custom training을 시작하지 않고 검증 결과에 기록한다.
- GPU/CPU 성능, camera와 AprilTag의 동시 사용에 따라 debug FPS가 낮아질 수 있다.
- Stage 2의 정식 줄자 기반 거리표는 남아 있다. Stage 3 camera XYZ 실기 검증은 완료했다.
- ROI 중심 ray와 ROI median Z는 사람의 물리적 중심을 근사하며 같은 단일 pixel 측정은 아니다.
- bbox 변화에 따라 XYZ가 흔들릴 수 있고 temporal smoothing은 아직 적용하지 않는다.
- PoseArray는 confidence, bbox와 원래 person 번호를 포함하지 않는다.
- COCO person detector는 사람처럼 보이는 의류를 false positive로 검출할 수 있다.
- host Python에는 AI runtime이 없다. detector는 항상 survivor 전용 container에서 실행하고,
  host는 camera, ROS graph 확인과 rqt에 사용한다.

## 빠른 문제 해결

- RGB 없음: `rs-enumerate-devices`, `ros2 topic list | grep camera`, camera launch 로그 확인.
- callback 없음/QoS 의심: `ros2 topic info -v /leader/camera/color/image_rect`에서 endpoint와
  `BEST_EFFORT` 확인.
- debug 없음: `ros2 node list`, `ros2 topic info -v /leader/survivor/debug_image`, detector
  terminal의 model/import 오류 확인.
- Ultralytics/model load 실패: AI 패키지 import와 weight 경로/네트워크를 확인하되 torch를
  임의 업그레이드하지 않는다.
- CUDA false: JetPack용 torch인지 확인하고 우선 `device:=cpu`로 기능을 검증한다.
- rqt blank: debug topic을 명시하고 rqt의 QoS/transport 및 `ros2 topic hz`를 확인한다.
- 낮은 FPS/일부 사람 누락: 입력·출력 hz, GPU 사용 여부, 조명·거리·가림과 threshold를
  확인한다.
- aligned depth 없음: camera를 `enable_sync:=true align_depth.enable:=true`로 재실행한다.
- XYZ만 N/A: CameraInfo.P, RGB와 CameraInfo의 resolution/frame ID를 확인한다.

명령별 상세 진단은 [Stage 1 validation](docs/STAGE1_PERSON_DETECTION_VALIDATION.md)과
[Stage 2 validation](docs/STAGE2_DEPTH_DISTANCE_VALIDATION.md),
[Stage 3 implementation](docs/STAGE3_CAMERA_XYZ_IMPLEMENTATION.md),
[Stage 3 validation](docs/STAGE3_CAMERA_XYZ_VALIDATION.md), 이후 전체 개발 순서는
[survivor development plan](docs/SURVIVOR_DEVELOPMENT_PLAN.md)을 따른다.

## Stage 2 distance algorithm

RGB callback은 YOLO inference를 계속 수행하고, depth callback은 최근 aligned depth message를
`sync_queue_size`개 보관한다. RGB timestamp에 가장 가까운 depth와의 차이가 `sync_slop_sec`
이내일 때만 depth를 사용한다.
`16UC1`/`MONO16`은 `depth_scale_m_per_unit`을 곱하고 `32FC1`은 meter로 처리한다. bbox
중심 ROI에서 유효 pixel을 추출하고 NumPy median을 계산한다. 유효 pixel이 20개 미만이거나
depth가 없으면 `N/A`를 표시한다.

## Stage 3 camera XYZ algorithm

Stage 2 ROI의 실제 pixel 범위 중심을 `(u,v)`로 사용하고 같은 ROI의 median distance를 Z로
사용한다. rectified RGB이므로 CameraInfo의 `P[0], P[5], P[2], P[6]`에서
`fx,fy,cx,cy`를 얻어 `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy`를 계산한다. optical frame은
right/down/forward 축이다.

## Stage 4 — Camera XYZ → exact timestamp TF2 → map XYZ

실행 명령:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
```

입력은 `/leader/survivor/camera_positions`의 `header.frame_id`와 원본 RGB
`header.stamp`를 그대로 사용한다. node는 `target_frame=map`,
`source_frame=msg.header.frame_id`, `time=Time.from_msg(msg.header.stamp)`로
exact-time TF를 조회한다. PoseArray마다 lookup는 한 번만 수행하고 모든 position에
적용한다.

출력은 `/leader/survivor/map_positions`이며 `header.frame_id=map`, timestamp는 입력
stamp를 유지하고 orientation은 identity quaternion이다. 빈 frame/stamp, NaN/Inf,
TF lookup 실패·timeout·extrapolation·connectivity failure에서는 publish하지 않는다.

Stage 4는 완료됐으며 구현·실행·A/B 이동 검증 결과는
[`Stage 4 validation`](../../../docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE4_MAP_TRANSFORM_VALIDATION.md)에
기록되어 있다.

## Stage 5 — Map XYZ → RViz visualization

**RViz Visualization VERIFIED (Stage 5: PASS).** 현재 topic flow:

```text
/leader/camera/color/image_rect
+ /leader/camera/aligned_depth_to_color/image_raw
+ /leader/camera/color/camera_info
        ↓
Person Detector (YOLO + aligned depth + camera optical XYZ)
        ↓
/leader/survivor/camera_positions
        ↓
Survivor Map Transform (exact timestamp TF2)
        ↓
/leader/survivor/map_positions
        ↓
Survivor Map Visualizer
        ↓
/leader/survivor/map_markers
        ↓
RViz Survivors display + nvblox 3D map
```

Visualizer는 유효 pose마다 `SPHERE`와 `TEXT_VIEW_FACING` marker를 만들며,
후보 수가 줄면 사라진 인덱스의 sphere/text에 `DELETE`를 보낸다. 입력이
끊기면 기본 `2.0 s` finite lifetime으로 기존 marker가 만료된다.
기본 `text_z_offset=0.30 m`는 text 표시 위치에만 적용된다. 가까운
label은 겹칠 수 있고 text가 mesh 뒤에 있으면 가려질 수 있다.
이 제한은 map XYZ 변환 결과에 영향을 주지 않는다.

재현 실행 순서, 실제 측정값 및 최종 수동 검증 체크리스트는 위
[Stage 5 validation](../../../docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE5_RVIZ_VISUALIZATION_VALIDATION.md)을
따른다. Stage 2의 별도 정식 줄자 거리표는 아직 남아 있다.

## Stage 6 — Persistent Survivor Registry

**Persistent Survivor ID + Spatial Deduplication + Position Stabilization**을 구현했다.
Raw map detections를 spatial association으로 연결해 같은 사람은 기존 ID와 위치를
갱신하고, 새 후보는 tentative confirmation 후 새 ID를 부여한다.

```text
Raw map detections
        ↓
Spatial association
   ┌────┴────┐
same person  new person
   ↓             ↓
update       new persistent ID
   ↓             │
position averaging
   └──────┬──────┘
          ↓
Persistent Survivor Registry
```

구현 topic과 service:

- `/leader/survivor/tracks` — `rescue_robot_interfaces/msg/SurvivorTrackArray`
- `/leader/survivor/registry_markers` — persistent VISIBLE/LAST SEEN markers
- `/leader/survivor/registry/reset` — `std_srvs/srv/Trigger`

Stage 6.1에서 여러 사람의 distinct persistent ID, moving survivor same-ID 유지,
FOV→LOST, 마지막 위치와 yellow LAST SEEN marker, same-ID reassociation 및 Registry
visualization을 실제 Jetson + D435 + VSLAM + nvblox + RViz에서 검증했다. process
restart/map session 외부 ID 복원과 CSV/JSON 저장은 구현하지 않는다.

## Next Stage

다음 단계는 optional hardening 및 정량 검증이다. absolute survivor localization accuracy,
association threshold tuning, 장시간 안정성, close crossing/occlusion stress test와
CPU/GPU/runtime performance logging을 검토한다. Appearance Re-ID, disk persistence와
Mission Coordinator는 별도 후속 범위이며, spatial association 한계가 stress test에서
실제로 문제가 될 때 선택적으로 도입한다.
