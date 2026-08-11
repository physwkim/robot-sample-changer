# UR monitoring IOC (read-only)

epics-rs-iocs의 ur-robot IOC(urRobot 포트)에서 **dashboard + RTDE
receive** 두 포트만 로드하는 감시 전용 IOC입니다. `Robot:UR:` prefix로
로봇 모드/안전 상태/조인트·TCP 상태를 CA로 노출합니다.

control / io / jog / gripper db는 의도적으로 뺐습니다 — 해당 포트들은
프로그램 슬롯, RTDE 입력 레지스터, URCap 그리퍼 소켓을 점유해서
robot-sequencer(ur-driver + robotiq-hande)와 공존할 수 없습니다.
이 IOC는 읽기만 하므로 시퀀서와 동시에 떠 있어도 됩니다.

## 빌드/실행

epics-rs-iocs가 `/home/bl9b/epics-rs-iocs`(개발머신은
`~/work/epics-rs-iocs`)에 체크아웃돼 있어야 합니다.

```bash
cd ~/epics-rs-iocs && cargo build --release -p ur-robot-ioc
# 수동 실행:
~/epics-rs-iocs/target/release/ur-robot-ioc ~/ws/deploy/ur_monitor_ioc/st.cmd
```

## systemd

```bash
sudo cp ur-monitor-ioc.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ur-monitor-ioc
# 콘솔: telnet localhost 20002  (robot_ioc는 20001)
```

URSim 리허설 시에는 st.cmd의 `IP`를 192.168.56.101로 바꿔서 수동 실행.
