# SBG Ellipse-D — 운영 명령 치트시트

실차 INS 브링업/RTK/진단에 쓰는 명령 모음. 배경·원리는
[`sbg_ellipse_d_bringup.md`](./sbg_ellipse_d_bringup.md), 검증 결과는
[`ins_odometry_validation.md`](./ins_odometry_validation.md) 참조.

- 장치: **ELLIPSE-D-G4A3-B2, SN 68336**, FTDI RS-232 → `/dev/ttyUSB0`, **115200 bps**
- 드라이버 config: `confWithRos: false` (장치가 진실원 — yaml `log_*`는 퍼블리셔 생성 게이팅용)

---

## 1. 드라이버 실행

```bash
# 기본 (INS만)
ros2 launch sbg_driver sbg_device_launch.py

# RTK 포함 (NTRIP 클라이언트도 같이 켬)  ← §3 참조
ros2 launch sbg_driver sbg_device_launch.py ntrip:=true \
  host:=RTS1.ngii.go.kr port:=2101 mountpoint:=VRS-RTCM31 \
  username:=shchon11 password:=ngii
```

포트 잡혀있으면 먼저 정지: `kill $(pgrep -x sbg_device)`

---

## 2. 장치 output config 적용 (롤백/공장초기화 후 재설정)

장치 출력 메시지는 flash에 저장된다. `airData`만 남는 상태로 롤백되면 아래로 재적용.
**REST POST는 스테이징만 → 반드시 `sbg_settings_action save`로 flash 저장(+리부트).**
안테나 lever arm·aiding은 건드리지 않음(출력 메시지만).

```bash
cd ~/fsk/src
API=tools/sbg-patches/bin/sbgEComApi
ACT=tools/sbg-patches/bin/sbg_settings_action
PORT=/dev/ttyUSB0; BAUD=115200

# 25 Hz: nav / euler / imu / rot_accel_body
for m in ekfNav ekfEuler imuData ekfRotAccelBody; do
  $API -s $PORT -r $BAUD -p -b '"40ms"'   /api/v1/settings/output/comA/messages/$m
done
# 1 Hz: status / utc
for m in status utcTime; do
  $API -s $PORT -r $BAUD -p -b '"1000ms"' /api/v1/settings/output/comA/messages/$m
done
# GPS 원시(진단용): 주기 아님 → onChange  (200ms 넣으면 422)
for m in gps1Pos gps1Vel gps1Hdt; do
  $API -s $PORT -r $BAUD -p -b '"onChange"' /api/v1/settings/output/comA/messages/$m
done

# flash 저장 + 리부트
$ACT $PORT $BAUD save        # ~10 s 대기

# 확인 (켜진 것만)
$API -s $PORT -r $BAUD /api/v1/settings/output/comA/messages -g \
  | python3 -c "import json,sys;print({k:v for k,v in json.load(sys.stdin).items() if v!='off'})"
```

전체 config 백업/복원 참고: `$API ... /api/v1/settings -g` (JSON 덤프).
`sbg_ellipse_d_settings_backup.json` = 커밋된 원본 스냅샷(안테나 실측값 포함).

---

## 3. NTRIP RTK — NGII 네트워크RTK (확정 동작값)

> ⚠️ **RTS1을 써라. RTS2는 이 계정으로 401.** 마운트포인트도 RTS1은 `VRS-RTCM31`
> (RTS2의 VRS-RTCM32 아님). 2026-07-24 직접 접속 검증: `200 OK`, RTCM ~370 B/s.

| host | port | mountpoint | id | pw |
|---|---|---|---|---|
| **RTS1.ngii.go.kr** | 2101 | **VRS-RTCM31** | `shchon11` | `ngii` |

- `shchon11 // ngii` = 실시간 측위보정(RTS1/RTS2) 계정. (`@geodesy // gnss`는 데이터센터용, 별개)
- VRS라 GGA 업링크 필요 → `ntrip_client`가 `/sbg/ekf_nav` 위치로 자동 전송(`send_gga:=true` 기본).
- **차 컴퓨터에 인터넷 필수**(폰 테더링/LTE). 실서버 접속은 계정 등록돼야 열림.
- 드라이버가 `ntrip_client/rtcm`을 장치로 전달(`rtcm.subscribe: true`, config에 설정됨).

RTK fixed는 **안테나 + 하늘 시야(밖)** 필요. 실내/안테나 없으면 RTCM 흘러도 fix 안 뜸(정상).

NTRIP만 단독 실행(진단):
```bash
ros2 run sbg_driver ntrip_client --ros-args \
  -p host:=RTS1.ngii.go.kr -p port:=2101 -p mountpoint:=VRS-RTCM31 \
  -p username:=shchon11 -p password:=ngii -p send_gga:=true
# 성공 로그: "NTRIP stream open, receiving RTCM" / "RTCM ### B/s -> device"
```

---

## 4. 진단 / 검증

```bash
# 실시간 대시보드 (mode·GNSS·헤딩·baseline·레이트·aiding)
ros2 run hyu_localization status_monitor

# 토픽 레이트 — discovery 함정 때문에 아래 3개 필수
ros2 topic hz /sbg/ekf_nav --no-daemon --qos-reliability best_effort

# 장치가 실제로 스트리밍하는지 (ROS 무관, 원시 ECOM). 드라이버 정지 후:
stty -F /dev/ttyUSB0 115200 raw -echo
timeout 3 head -c 9000 /dev/ttyUSB0 | xxd | head   # ff5a … 프레임 보이면 정상
```

---

## 5. 함정 / 교훈 (세션에서 실제로 겪음)

- **ROS discovery**: 이 워크스페이스는 `ROS_LOCALHOST_ONLY=1`. 새 셸에서 토픽 안 보이면
  `--no-daemon` + `--qos-reliability best_effort`. 안 그러면 데이터 있어도 `NO DATA` 오판.
- **장치 config**: REST POST는 스테이징만 → `SAVE_SETTINGS` 안 하면 리부트 시 날아감.
  `REBOOT_ONLY`는 스테이징 폐기. GPS 메시지는 주기 아니라 `onChange`.
- **`confWithRos:true` 금지**(배포 yaml lever arm이 0) — 장치 실측 커미셔닝을 덮어씀.
- **NTRIP**: RTS1(RTS2 아님) / VRS-RTCM31 / 계정당 1세션 / 웹 테스터는 서버측 접속이라
  현장 직접접속을 보장 안 함.
- **pkill 자기매칭**: `pkill -f sbg_device` 같은 건 자기 셸까지 죽임 → PID로 kill.
