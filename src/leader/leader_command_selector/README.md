# Leader Command Selector

Leader의 최종 `/leader/cmd_vel` source를 하나로 선택한다.

| Mode | Input |
|---|---|
| `STOP` | 항상 zero |
| `TELEOP` | `/leader/teleop/cmd_vel` |
| `APPROACH` | `/leader/approach/cmd_vel_safe` |
| `NAV2` | `/nav2/cmd_vel` |
| `MISSION` | `/leader/mission/cmd_vel_safe` |

Output은 `/leader/cmd_vel`, status는 transient-local
`/leader/command_selector/status` (`std_msgs/msg/String`)다. Generic launch default는 `STOP`이다.

```bash
ros2 param get /leader/command_selector source_mode
ros2 param set /leader/command_selector source_mode STOP
ros2 param set /leader/command_selector source_mode TELEOP
ros2 param set /leader/command_selector source_mode APPROACH
ros2 param set /leader/command_selector source_mode NAV2
ros2 param set /leader/command_selector source_mode MISSION
ros2 topic echo /leader/command_selector/status \
  --qos-durability transient_local
```

Mode 변경 시 모든 command cache를 버리고 zero를 즉시 발행한다. 선택된 source에서 fresh
command를 받기 전까지 zero를 유지한다. Source별 timeout, 잘못된 Twist와 shutdown은 zero로
fail closed한다. Status는 `STOP`, `WAITING_*`, `ACTIVE_*`, `STALE_*`를 보고한다.
