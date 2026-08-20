# D405 camera IOC

epics-rs-iocs의 `d435i-ioc`를 Intel RealSense **D405** 한 대에 물려 돌리는
배치 파일 모음입니다. IOC 코드 자체(드라이버, st.cmd, 레코드 템플릿)는
`~/work/epics-rs-iocs`에 있고, 여기 있는 것은 **이 워크스테이션에서 굴리는
데 필요한 것들** — systemd 유닛, 수동 실행 스크립트, 호스트 준비 스크립트입니다.

`RS405:` prefix로 컬러·깊이를 pvAccess로 내보냅니다. 데스크톱 런처
`[1] Camera IOC`가 이 유닛을 띄웁니다.

## 구성

| 파일 | 용도 |
|---|---|
| `d405-ioc.service` | systemd **유저** 유닛. procServ 경유, 콘솔 20003 |
| `run-d405-ioc.sh` | systemd 없이 손으로 띄울 때 |
| `run-d435i-ioc.sh` | D435i용. 지금 그 카메라는 쓰지 않음 |
| `run-camera-viewer.sh` | PyDM 뷰어. 현재는 rsdm(PVA)를 쓰므로 비주력 |
| `install-librealsense.sh` | librealsense2 SDK 설치 (호스트 1회) |
| `reset-usb-controller.sh` | USB가 먹통일 때 xHCI 컨트롤러 리셋 |

## systemd (유저 레벨, sudo 없음)

```bash
cp d405-ioc.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now d405-ioc
# 콘솔: telnet localhost 20003  (robot_ioc 20001, ur_monitor_ioc 20002)
# 로그:  journalctl --user -u d405-ioc -f
```

로그아웃해도 떠 있게 하려면 한 번만: `sudo loginctl enable-linger bl9b`

### 종료는 반드시 systemctl로

```bash
systemctl --user stop d405-ioc
```

유닛의 `ExecStop`이 **취득을 먼저 멈춥니다.** 스트리밍 중에 IOC를 강제
종료하면 RealSense 펌웨어가 물려서, USB 열거는 계속 되는데 프레임만 안
나오는 상태가 됩니다. USB 버스 리셋으로는 안 풀립니다(VBUS가 안 끊겨
카메라 ASIC 상태가 유지됨) — 실제로 한 대는 xHCI 컨트롤러 리셋까지, 다른
한 대는 물리적 재연결까지 갔습니다.

## 호스트 준비 (새 머신에서 1회)

```bash
sudo ./install-librealsense.sh
```

librealsense2는 Ubuntu 아카이브에 없고, 벤더 저장소(`librealsense.realsenseai.com`,
구 `librealsense.intel.com`)는 2026-08 기준 **InRelease 서명이 깨져 있어**
`apt update`가 저장소를 거부합니다. 같은 Release에 대한 분리 서명
`Release.gpg`는 정상이므로 콘텐츠는 진짜이고, Artifactory가 만든 InRelease만
망가진 상태입니다. 그래서 이 스크립트는 apt 저장소를 추가하는 대신
`Release.gpg → Release → Packages → *.deb` 서명 사슬을 직접 검증해 받은
`.deb`을 설치합니다.

받아둔 `.deb`은 저장소에 넣지 않았습니다(8 MB). 기본 경로는
`~/work/librealsense-debs/`이고, 없으면 스크립트 주석의 절차대로 다시
받으면 됩니다. 버전은 `Cargo.lock`의 `realsense-sys`(현재 2.56.5)에 맞춰
고정(`apt-mark hold`)합니다 — FFI 구조체가 그 헤더로 생성돼 있습니다.

## 설정 지속성

카메라 설정과 플러그인 활성 상태는 autosave가 들고 있습니다
(`epics-rs-iocs/iocs/d435i-ioc/auto_settings.req`, 저장 위치는
`iocs/d435i-ioc/autosave/RS405/`, 30초 주기). 재시작하면 스트림 모드와 PVA
플러그인이 그대로 복원됩니다.

`RSStreamMode`만 예외입니다. pass1 복원이 레코드 VAL은 채우지만 처리하지
않아 드라이버까지 값이 내려가지 않습니다 — `RSStreamMode`는 복원값을,
`RSStreamMode_RBV`는 드라이버 기본값을 가리키고 카메라는 후자로 돕니다.
st.cmd 안에서는 고칠 수 없습니다(프레임워크가 스크립트 종료 *후*에 복원을
실행). 그래서 유닛의 `ExecStartPost`와 `run-d405-ioc.sh`가 기동 후 한 번
강제로 처리합니다.

## USB 문제가 잦습니다

이 호스트에서 두 카메라 모두 SuperSpeed 링크가 끊긴 이력이 있습니다.
증상은 세 가지로 나뉩니다:

- **USB2로 강등** — `lsusb`상 480M, 커널에 `-75`(EOVERFLOW). 대역이 모자라
  프레임이 0. 케이블 교체가 유효했습니다.
- **재열거 후 무응답** — 링크는 SuperSpeed인데 커널에 `-32`(EPIPE). IOC가
  `try_wait_for_frames cannot be called before start()`로 반복합니다.
  드라이버가 장치 재열거를 복구하지 못하는 알려진 한계라, 지금은 서비스
  재시작이 가장 빠릅니다.
- **버스에서 소실** — `error -71`, 열거 자체 실패. `reset-usb-controller.sh`가
  풀어줄 때가 있고, 안 되면 물리적 재연결뿐입니다(VBUS 차단 불가).

과전류 로그는 한 번도 없었으므로 전원 부족은 아닙니다. 대역폭도 아닙니다 —
두 카메라 1280x720@30 동시 스트리밍을 무결점으로 통과했습니다.

## 성능 메모

PVA 이미지 서빙은 `epics-pva-rs`의 NTNDArray 변환이 픽셀마다 24바이트 enum을
만들던 탓에 640x480@15fps에서 CPU 1.5코어를 먹었습니다. 타입드 배열
(`ScalarArrayTyped`, `Arc<[T]>` + 벌크 memcpy)로 바꿔 9.5배 줄였고, 쓰지 않는
CA 경로(`CC1`/`image1`/`image2`)를 끄면 최종 17%입니다. 이 수정은 epics-rs
쪽에 있으며 릴리스 전까지는 `epics-rs-iocs/Cargo.toml`의 로컬
`[patch.crates-io]`로 물려 씁니다 — 그 patch는 커밋하지 마세요.
