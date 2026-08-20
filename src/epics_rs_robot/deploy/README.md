# robot_ioc — procServ + systemd(유저) 부팅 자동시작

`robot_ioc`(EPICS soft IOC)를 procServ 아래에서 데몬으로 돌리고 systemd로 부팅 시
자동 시작합니다. procServ가 iocsh 콘솔을 TCP 20001(localhost 전용)로 노출하므로,
실행 중인 IOC에 붙어서 `dbl`, `camonitor` 등 iocsh 명령을 쓸 수 있습니다.

## 1. release 바이너리 빌드 (서비스가 참조)

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd ~/ws/src/epics_rs_robot && cargo build --release -p robot_ioc
```

## 2. 설치 & 활성화 (**유저 유닛**, sudo 없음)

먼저 수동으로 돌던 IOC가 있으면 정지:
```bash
pkill -x robot_ioc
```

유닛 설치 → 부팅 자동시작 등록 → 지금 시작. **이 저장소의 사본을 설치하세요** —
`~/ws/src/epics_rs_robot/deploy/`에 남아 있는 구 트리 사본은 `ROBOT_DB`가
`~/ws/db`를 가리키고, 그 db에는 CalibMode 3-7 라벨도 `Robot:MapSource`도
없습니다(그립 널·홀더 간 이동이 쓰는 PV):
```bash
install -m 644 ~/work/robot-sample-changer/src/epics_rs_robot/deploy/robot-ioc.service \
     ~/.config/systemd/user/robot-ioc.service
systemctl --user daemon-reload
systemctl --user enable --now robot-ioc.service
```

이 호스트의 다른 IOC 둘(`ur-monitor-ioc`, `d405-ioc`)과 같은 유저 유닛입니다.
root가 필요한 항목이 하나도 없어서입니다 — TCP 20001도 CA 5064도 비특권
포트이고 경로는 전부 `/home/bl9b` 아래입니다. sudo를 뗀 값은 데스크톱
런처(`[0] Robot IOC`)가 IOC를 직접 띄울 수 있다는 것입니다.

로그인 없이도 유지되려면 linger가 켜져 있어야 합니다(이 호스트는 이미 켜짐):
```bash
loginctl show-user "$USER" -p Linger      # Linger=yes
```

**구 시스템 유닛을 지우세요.** `/etc/systemd/system/robot-ioc.service`에 2026-06-01
사본이 남아 있으면 같은 이름이 시스템·유저 양쪽에 존재하고, 그 사본은
`ROBOT_DB=~/ws/db`를 들고 있습니다. 지우는 것만 sudo가 필요합니다:
```bash
sudo systemctl disable --now robot-ioc.service      # 시스템 쪽
sudo rm /etc/systemd/system/robot-ioc.service
sudo systemctl daemon-reload
```

## 3. 상태 / 로그 / 제어

```bash
systemctl --user status robot-ioc.service
journalctl --user -u robot-ioc.service -f   # systemd 레벨 로그
tail -f ~/ws/src/epics_rs_robot/robot_ioc/ioc/procServ.log   # IOC 콘솔 출력

systemctl --user restart robot-ioc.service
systemctl --user stop robot-ioc.service
```

## 4. 실행 중 IOC의 iocsh 콘솔 붙기/나가기

```bash
telnet localhost 20001      # 또는: nc localhost 20001
# iocsh 프롬프트에서: dbl / camonitor Robot:CurrentStep / 등
# 나가기(IOC는 계속 실행): Ctrl-] 입력 후 'quit'  (telnet escape)
```

procServ 콘솔에서 `Ctrl-X` = IOC 자식 프로세스 재시작(restart), `Ctrl-T` = autorestart 토글.
`--ignore "^D^C"` 로 Ctrl-D/Ctrl-C 는 무시되어 실수로 IOC가 죽지 않습니다.

## 동작 방식 / 복원력

- procServ가 IOC 자식이 죽으면 **자동 재시작**(holdoff 후). `systemd --user`는
  procServ 자체가 죽으면 **재시작**(`Restart=always`). → 2중 복원력.
- IOC가 재시작돼도 autosave(`robot_ioc/autosave/robot_state.sav`)가 Robot:CurrentStep 등
  상태를 복원하므로, 크래시 후 재개 흐름이 그대로 유지됩니다 (CLAUDE.md 참고).

## 참고: 브리지(epics_ros_bridge)

브리지는 iocsh가 아니라 ROS2 노드라 procServ를 쓰지 않습니다. 부팅 자동시작이 필요하면
ROS 환경을 source 하는 별도 systemd 유닛으로 구성할 수 있습니다(요청 시 추가).
