# 생존자 인식 전체 개발 계획

## 상태와 최종 흐름

현재 상태는 **Stage 1 VERIFIED (2026-09-11)**,
**Stage 2 IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**, **Stage 3 VERIFIED
(2026-09-12)**, **Stage 4 PASS (2026-09-14, raw-coordinate caveat)**,
**Stage 5 PASS (2026-09-18, RViz 수동 검증 완료), Stage 6.1 VERIFIED (실제 multi-person
physical validation 및 Registry RViz 검증 완료)**다. Stage 3는 실제 Jetson + D435에서 사람 XYZ, 좌/우 X
부호, distance/Z 일치, debug overlay, 다중 사람과 PoseArray를 확인했다. Stage 2의 별도 정식
줄자 거리표 상태는 임의로 변경하지 않는다.

```text
D435 RGB → YOLO person bbox ─┐
D435 aligned depth ──────────┼→ distance → camera XYZ → map XYZ → RViz marker (Stage 5)
CameraInfo ──────────────────┘                          → association/registry (Stage 6.1 VERIFIED)
                                                        → confirmation/mission lifecycle (Stage 6.1 VERIFIED)
```

## Stage 1 — RGB YOLO person bounding box

- Purpose: COCO pretrained YOLO11n으로 한 프레임의 모든 사람을 시각화한다.
- Input: `/leader/camera/color/image_rect`, `sensor_msgs/Image`.
- Processing: BGR 변환, class 0/threshold 필터, 중심 x 좌우 정렬, frame-local 번호와 box 표시.
- Output: `/leader/survivor/debug_image`, 원본 header 유지.
- Completion: 0/1/다중 person 결과를 rqt에서 확인하고 비-person이 표시되지 않는다.
- Current: software/build/test, survivor 전용 Docker의 Jetson GPU 추론, 다중-person sample 및
  D435 실제 1명/2명/3명 검출을 확인했다. bounding box, confidence, 좌→우 frame-local
  numbering과 `/leader/survivor/debug_image`를 rqt에서 검증했다. Stage 2 이후도 같은
  image와 model cache를 재사용한다.

## Stage 2 — Bounding box + aligned depth → distance

**현재 상태: IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**

- Purpose: 각 검출 사람까지의 강건한 대표 거리를 계산한다.
- Input: Stage 1 bbox, `/leader/camera/aligned_depth_to_color/image_raw`, RGB/depth stamp.
- Processing: 최근 aligned depth cache에서 RGB와 가장 가까운 stamp를 찾아 `sync_slop_sec` 이내인지 검사하고,
  bbox 중심 ROI에서 0/non-finite/range 밖 값을 제거한 뒤 median을 계산한다.
- Output: frame-local person별 거리[m]를 debug image에 표시한다. depth가 없거나 불일치하면
  YOLO 결과를 유지하고 `N/A`를 표시한다.
- Completion: 자동 테스트, Docker 실행, 실제 D435의 알려진 거리·다중 인원·이동 시험이 모두
  기록되어야 한다. synthetic test만으로 VERIFIED로 승격하지 않는다.

## Stage 3 — Depth + CameraInfo → camera XYZ

**현재 상태: VERIFIED (2026-09-12)**

- Purpose: 2D pixel과 depth를 camera optical frame의 3D 위치로 변환한다.
- Input: Stage 2 중앙 ROI center `(u,v)`, median Z, rectified RGB CameraInfo.P.
- Processing: P의 `fx/fy/cx/cy` 검증, RGB/CameraInfo resolution·frame 일치 검사,
  pinhole deprojection과 finite/positive 검사.
- Output: debug image의 XYZ와 `/leader/survivor/camera_positions` `PoseArray`. 원본 RGB
  timestamp와 실제 optical frame을 유지하고 valid XYZ만 좌→우 순서로 포함한다.
- Dependency: Stage 1 frame-local detections, Stage 2 ROI/median Z, color CameraInfo.
- Completion: unit test와 Docker runtime, 실제 사람 XYZ, 좌/우 X 부호, distance/Z 일치,
  debug overlay, 다중 person/pose, frame ID와 identity quaternion을 확인했다.
- Verification: `/leader/survivor/camera_positions`의 여러 pose와 debug XYZ가 대응했으며
  `header.frame_id=camera_color_optical_frame`임을 확인했다.

## Stage 4 — Camera XYZ + exact timestamp TF2 → map XYZ

**현재 상태: PASS (2026-09-14, raw-coordinate caveat)**

- Purpose: 측정 시점의 생존자 후보를 전역 map 좌표로 변환한다.
- Input: `/leader/survivor/camera_positions`, `geometry_msgs/msg/PoseArray`.
- Processing: `Time.from_msg(msg.header.stamp)`를 사용한 원본 timestamp TF2 lookup/transform,
  timeout과 unavailable 처리. PoseArray마다 lookup는 한 번만 수행한다.
- Output: `/leader/survivor/map_positions`, `map` frame의 stamped survivor 후보 좌표.
- Parameters: `input_topic`, `output_topic`, `target_frame=map`, `tf_timeout_sec=0.6`.
- Failure policy: 빈 frame/stamp, NaN/Inf, lookup/connectivity/extrapolation/timeout 실패 시
  latest TF fallback 없이 메시지 전체를 skip한다.
- Dependency: Stage 3 PoseArray의 correct frame ID와 original RGB timestamp, 유효 TF tree.
- Completion: 정지 255 valid pairs, A/B 이동 129/135 valid pairs, map frame 및 동일 stamp,
  VSLAM/EKF/nvblox 회귀를 실제 Jetson + D435에서 확인했다. A→B raw map 평균 변화 약
  0.115 m는 제한사항이며 이번 Stage에서 filtering하지 않았다.

## Stage 5 — RViz survivor visualization

**현재 상태: PASS (2026-09-18, 수동 검증 완료)**

- Purpose: 현재 map-frame 후보를 RViz의 nvblox 3D map 위에 표시한다.
- Input: `/leader/survivor/map_positions` `PoseArray`.
- Processing: 후보별 sphere/text, finite lifetime, 후보 수 감소 시 DELETE.
- Output: `/leader/survivor/map_markers` `MarkerArray` 및 RViz `Survivors` display.
- Completion: 한 사람/다중 후보, FOV 이탈·재진입, 통제된 2명→1명 감소,
  VSLAM·dual EKF·nvblox 동시 실행을 실제 수동 검증했다.
- Limit: `Survivor candidate N`은 frame-local 인덱스이며 영구 ID가 아니다.
  공간 association, 중복 제거, 위치 평균/필터, registry와 저장은 미구현이다.

## Stage 6 — Persistent Survivor ID + Spatial Deduplication + Position Stabilization

**현재 상태: NOT IMPLEMENTED**

- Purpose: 같은 사람의 반복 map 관측을 한 persistent ID에 연결하고 위치를 안정화한다.
- Input: Stage 4의 raw map detections와 시각·좌표 metadata.
- Processing: spatial association, 새 사람 ID 발급, 기존 사람 위치 averaging/filtering,
  persistent survivor registry 정책 설계.
- Output: ID와 안정화된 위치를 가진 중복 제거 survivor 후보 목록(계획).
- Dependency: Stage 4 map XYZ. Stage 5의 frame-local candidate 번호를 ID로 재사용하지 않는다.
- Completion: 동일 사람 반복 관측과 서로 다른 사람을 구분하는 정책 및 실물 검증이 필요하다.
  persistent storage/CSV·JSON 저장과 장기 mission-level tracking도 아직 미구현이다.

## Stage 7 — Mission Coordinator integration

**현재 상태: NOT IMPLEMENTED**

- Purpose: confirmed survivor를 로봇 임무 판단과 구호물품 전달 흐름에 연결한다.
- Input: 향후 confirmation을 거친 survivor 위치/ID/상태와 coordinator 상태.
- Processing: 인터페이스 계약, 중복 임무 방지, 취소·복구와 안전 gate.
- Output: coordinator가 소비할 survivor event/target.
- Dependency: Stage 6 registry와 후속 confirmation 정책, mission state contract.
- Completion: simulation과 실기에서 한 후보가 한 번만 임무로 전환되고 실패를 복구한다.

## Stage 1에서 Stage 2로 전달된 계약

- bbox coordinates: color image pixel 좌표
- RGB timestamp/frame: 원본 `header.stamp`, `camera_color_optical_frame`
- aligned depth: `/leader/camera/aligned_depth_to_color/image_raw`, 실제 확인값 `16UC1`,
  640×480, `camera_color_optical_frame`
- raw depth 참고: `/leader/camera/depth/image_rect_raw`, `16UC1`, 640×480,
  `camera_depth_optical_frame`
- CameraInfo: `/leader/camera/color/camera_info`
- 실제 resolution, encoding과 timestamp 일치는 매 실행에서 재검증한다.

## Stage 2에서 Stage 3로 전달된 계약과 구현

- representative pixel: Stage 2 half-open central ROI의 실제 pixel 중심
- representative Z: 같은 ROI의 valid median distance[m]
- rectified intrinsics: `/leader/camera/color/camera_info`의 P 사용
- 실제 확인 geometry: RGB/depth/CameraInfo `640×480`, `camera_color_optical_frame`
- optical axes: right/down/forward
- invalid policy: detection/distance 유지, `XYZ N/A`, PoseArray에는 placeholder 미삽입

Stage 2는 `depth_logic.py`, Stage 3는 `geometry_logic.py`에 ROS-independent 순수 함수를 둔다.
`person_detector_node.py`가 depth cache, CameraInfo cache, frame별 계산과 ROS message 발행을
담당한다. 자세한 결정 근거는 [Stage 3 구현 문서](STAGE3_CAMERA_XYZ_IMPLEMENTATION.md), 실제
절차는 [Stage 3 검증 가이드](STAGE3_CAMERA_XYZ_VALIDATION.md)를 따른다.

## Stage 3에서 Stage 4로 전달할 계약

- topic: `/leader/survivor/camera_positions`
- type: `geometry_msgs/msg/PoseArray`
- header stamp: original RGB measurement time
- header frame: 검증된 RGB camera optical frame
- position: meter 단위 camera optical XYZ
- orientation: identity quaternion
- order: valid frame-local detections의 left-to-right 순서, persistent ID 아님

Stage 4는 이 header를 사용해 camera optical XYZ를 측정 시점의 exact-timestamp TF2
transform으로 map frame XYZ로 변환한다. map TF가 없거나 timestamp에 맞는 transform이
없을 때 최신 transform이나 fake origin으로 대체하지 않는다. Stage 5 RViz
Marker와 map-coordinate visualization은 완료됐으며 다음 단계는 Stage 6이다.
