# Leader d812a04 계산식 동기화

2026-10-04. 원본은 `rbgusrns/damgc_robot`의
[d812a04](https://github.com/rbgusrns/damgc_robot/commit/d812a04d94653b6941228c18fc24f301a3a71c29)와
[Follower 인계 계약](https://github.com/rbgusrns/damgc_robot/blob/d812a04d94653b6941228c18fc24f301a3a71c29/docs/COOP_TRANSPORT_FOLLOWER_HANDOFF.md)이다.
Follower fork의 `origin/main`에는 해당 커밋이 없으므로 Leader 저장소를 별도로 fetch했다.

## 적용 내용

현재 실행하는 `cooperative_transport/hinged_formation.py`의 반복 역산을 제거하고
리더 차축 곡률 `kL`에서 직접 힌지 각도를 계산한다. `a=axle_to_hinge`,
`b=hinge_to_contact+object_center_to_contact`일 때:

```text
q = atan(kL*a) + asin(kL*b / sqrt(1+(kL*a)^2))
object_yaw = leader_yaw + q
object_xy = leader_xy + a*unit(leader_yaw) + b*unit(object_yaw)
```

리더와 같은 `radius=min(5,max(1,len(path)//10))` 이동평균을 사용한다.
약 5cm 샘플에서 최대 11개 점을 참조한다. `iterations` 인자는 기존 호출 호환용으로
유지하지만 반복 계산에 사용하지 않는다. Leader와 동일한 원 입력 경로를 기준으로
Follower 차축 경로를 재계산하고, 전달 경로와 위치 5mm/yaw 0.01rad로 비교한다.

공유 peer의 Leader 역할 코드에도 5cm 간격 보간과 `body.leader`에 원 입력을 넣는
변경을 반영했다. Follower는 받은 배열을 재보간하지 않는다. 새 인계 계약에 맞춰
설정 속도 불일치와 이미 도착 허용 범위 7.5cm 안에 있는 목표도 거절한다.

Leader의 Nav2 준비 회전반경 1.5m 설정은 Leader mapping manager가 담당하며
Follower 호스트에는 Nav2 planner를 실행하지 않는다. 이전 조사 문서에 적힌
Leader selector/mapping 연결 미반영 상태는 당시의 기록이다. 최신 Leader 구현은
위 원본 인계 문서를 기준으로 판단한다.

## 확인과 실행 상태

원본 Leader 소스를 실행하여 생성한 반경 1.5m 원호와 곡률 전이 경로를 fixture로
저장했다. Follower 계산의 object/Follower pose 배열을 이 기준값과 대조한다.
격리 ROS 환경에서는 실제 Leader 계산으로 만든 전이 경로 PREPARE를 보내
STOP 서비스 성공 후 READY에 도달하고 최종 속도 출력이 0임을 확인한다.
이것은 실제 두 로봇 DDS에서 양쪽 READY가 관측됐다는 뜻은 아니다.

현재 Follower는 ROS domain 0, `motion_enabled=false`, `i2c_write_enabled=false`로
재기동한다. 이전 사용자가 요청한 `validate_curvature=false`는 유지한다.
엄격 검증 옵션을 켠 수학 대조도 별도로 수행하므로 Leader가 통과시킨 경로는
같은 계산값을 얻는다. 해시·geometry·횡이동·round-trip·정렬 검사는 유지한다.
기존 `cooperative_mission` 실행기는 이번 peer 흐름에서 사용하지 않는다.

재기동하면 이전 session/READY는 폐기된다. 실제 리더와 확인하려면 작업자가 B와
새 목표를 다시 지정한다. N/START, 모터 쓰기, Dynamixel 실행은 이번 작업에 없다.
