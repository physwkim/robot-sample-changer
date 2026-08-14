# UR monitoring IOC (read-only)

epics-rs-iocs의 ur-robot IOC(urRobot 포트)에서 **dashboard + RTDE
receive** 두 포트만 로드하는 감시 전용 IOC입니다. `Robot:UR:` prefix로
로봇 모드/안전 상태/조인트·TCP 상태를 CA로 노출합니다.

control / io / jog / gripper db는 의도적으로 뺐습니다 — 해당 포트들은
프로그램 슬롯, RTDE 입력 레지스터, URCap 그리퍼 소켓에 **씁니다**.
robot-sequencer(ur-driver + robotiq-hande)가 그 셋을 쥐고 있습니다.

읽기는 배타적이지 않습니다. 데몬이 자기 RTDE 스트림을 물고 있는 상태에서
두 번째 RTDE 클라이언트가 자기 출력 레시피로 125 Hz를 받고, 대시보드
29999도 동시 접속 2개가 모두 `robotmode`에 답하는 것을 실측했습니다
(URControl 5.16.0.0). 그래서 이 IOC는 시퀀서와 동시에 떠 있어도 됩니다.

## 빌드/실행

epics-rs-iocs 체크아웃은 이 호스트에서 `~/work/epics-rs-iocs`입니다
(`~/epics-rs-iocs`는 없습니다).

```bash
cd ~/work/epics-rs-iocs && cargo build --release -p ur-robot-ioc
# 수동 실행:
~/work/epics-rs-iocs/target/release/ur-robot-ioc \
    ~/work/robot-sample-changer/deploy/ur_monitor_ioc/st.cmd
```

## systemd (유저 레벨, sudo 없음)

```bash
mkdir -p ~/.config/systemd/user
cp ur-monitor-ioc.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now ur-monitor-ioc
loginctl enable-linger bl9b   # 로그아웃/부팅 후에도 유지 (1회)
# 콘솔: telnet localhost 20002  (robot_ioc는 20001)
# 로그:  journalctl --user -u ur-monitor-ioc -f
```

읽기 전용이고 20002/5064만 쓰므로 시스템 유닛일 이유가 없습니다. linger를
켜지 않으면 로그아웃할 때 세션과 함께 내려갑니다.

URSim 리허설 시에는 st.cmd의 `IP`를 192.168.56.101로 바꿔서 수동 실행.
