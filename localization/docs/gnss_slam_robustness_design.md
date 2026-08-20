# 복합 GNSS 열화에 강건한 SLAM — 설계 문서 (v0.4, 시니어 전문가 리뷰 반영)

> v0.1(앵커 전체 배선)→v0.2(매장)→v0.3(포렌식 균형)→**v0.4(시니어 리뷰 반영).**
> v0.4 핵심 변경(§0.6): **절대 헤딩 데이텀을 위치 앵커와 분리해 앞당김**(SBG 최고·최다·
> 최저위험 신호로 지배적 미해결 실패=대칭 aliasing을 침), 위치 앵커는 "약한 종속"이 아닌
> **A/B 실측가중 로버스트 factor**, A/B 승격은 **적대적 believed≠realized 스위트**로,
> 매핑기 데이텀 보호 유지, 역할3 재명명.

> **삭제된 앵커 코드(cea7cdc0^)와 실패 이력을 포렌식 정밀분석**한 결과: 앵커는 좁은 역할의
> **최소·올바른 느슨결합 데이텀**으로만 부활, CSM이 주권.
> 근거: RTK bag `rosbag2_2026_07_24-20_06_15`, PSRDIFF bag `..._07_23-21_23_30`,
> `graph_slam_node.cpp`, `sbg_odometry_bridge.py`, `sim_ellipse_d.py`,
> 삭제 커밋 `cea7cdc0` 및 그 이전 방어스택 이력(7fbee7d5→35581d09).

---

## 0. 설계 원칙
1. **원인(fault)을 모델링, 관측치는 원인의 함수로 파생** → 조합 열거 없이 동시다발 재현.
2. **정직한 공분산 + 공분산 가중 융합 + 이상치 게이트 + 폴백 사다리** → by-construction.
3. **(신규) 실패를 반복하지 않되 과잉반응도 안 한다** — 증거로 판단(§0.5).

## 0.5 ★포렌식 결론 + 핵심 결정★

### 무슨 일이 있었나 (증거)
앵커+거대 방어스택은 2026-07-17~21에 쌓였다 삭제(`cea7cdc0`)됐고, 콘-only CSM(Olson 2009)이
**0.07~0.09m로 RTK-앵커 최고 0.20m를 이겼다.** 포렌식 2건:

- **근본원인 귀속**: 앵커 개념 ~0%. **상류 센서/퍼셉션 버그 ~50%**(휠오돔 dt-rewind=
  "280m 유령의 씨앗", 거짓 tight 공분산, launch 누수=2초 67.9m랩, 이중 디스큐) +
  **SLAM 코어 대칭-aliasing ~30%**(skidpad 미러플립·seam이 반대 직선에 aliasing) +
  **앵커 통합버그 ~20%.** 가드들은 **오진된 상류버그의 밴드에이드**였다. 결정적으로
  **앵커는 진짜 cm RTK가 생기기(07-23~24) 전에 삭제**돼 공정한 시험을 못 받았다.
- **구현 품질**: 삭제된 앵커는 **"잘못 넣은" 게 아니라 교과서적으로 정교**했다 — 명시적
  **SE2 gauge 정점**(map→ENU, heading 관측가능), **DCS 로버스트 커널**, 병진-only
  verified recovery. 하지만 **구체 결함 2 + 긴장 1**: ①시간동기(최근접 키프레임 결합,
  보간 없음 → lever-arm) ②이상치 게이트(고정 2.0m 유클리드, 통계적 S=R+HPHᵀ 아님)
  ③hard RTK `setFixed` 핀이 gauge를 우회(이중 고정). 그리고 **가드 20개 + 파라미터
  63%가 앵커/복구** = 앵커가 콘지도를 *보완이 아니라 싸우고* 있었다는 신호.

### 결정 (v0.3)
**결정 1 — CSM(콘-only correlative scan matching)이 주 등록권자로 남는다.** 폐곡 FS
트랙에서 루프클로저가 전역제약을 이미 주고, CSM이 0.07~0.09m로 이겼다. **건드리지 않는다.**

**결정 2 — 앵커는 좁고·최소·올바른 "느슨결합 전역 데이텀"으로만 부활 검토.** CSM이
*증명적으로 못 하는* 3역할에 한정, **주권자 아님**:
1. **전역 지오레퍼런싱** (CSM 첫 정점은 임의 gauge — 절대좌표 필요 시).
2. **비폐곡(non-loop) 트랙 seam 폐합** — autocross_kase가 *"닫을 seam이 없어"* 0.35m에
   갇힘. cm RTK 위치엣지가 바로 그 빠진 전역제약. ← **구체 갭, 증명됨.**
3. **SLAM 발산/aliasing 재시드** (SBG 정상·SLAM 플립 시 절대 재시드; INS 전멸은 앵커도
   같은 SBG라 못 살림 → 그건 GNSS-free 사다리 몫. C5).

**결정 3 — "올바르게 최소로"의 정의**(부록 A):
- **유지(과거의 좋은 부분)**: SE2 gauge 정점 하나 + DCS 커널 + 병진-only recovery 개념.
- **고침(과거 버그)**: 통계적 innovation 게이트 `S=R+HPHᵀ` + **CUSUM 연속거부→강제
  재앵커**(자기강화 실패 차단); fix를 `rawOdomAt` 보간으로 **fix-시각 결합**.
- **삭제(실패 증폭기)**: hard RTK `setFixed` 핀, per-landmark GNSS priors, coherence-only
  recovery, seam-anchor grid, 그리고 **20개 가드 스택 전체.** 데이텀 메커니즘은 **gauge
  하나만.**

**결정 4 — A/B 게이트로만 승격.** 실 cm RTK로 현 CSM 베이스라인 대비 **격리 A/B**.
폐곡 트랙서 CSM을 못 이기면 CSM 주권 유지 + 앵커는 지오레퍼런싱·복구에만.

**결정 5 — 로버스트 우선순위는 여전히 GNSS-free.** 대회 규정상 주행 중 NTRIP 금지 가능 +
GNSS 신뢰불가. 앵커는 늦은·게이트된 마일스톤; 먼저 GNSS-free 모션 사다리 + 퍼셉션.

## 0.6 ★v0.4 — 시니어 전문가 리뷰 반영★ (조건부 승인의 조건)

**C1 (최고 개선) — 절대 헤딩 데이텀을 위치 앵커에서 분리해 앞당긴다.**
위치-only 앵커는 SBG의 최고 신호인 **듀얼안테나 절대 헤딩(0.12°, 정지 시에도 77% 사용)**
을 버렸다. 헤딩은 (a) tier-무관(기하학적)이라 float/single 열화에도 살아남고 위치-멀티패스
believed≠realized 함정이 없어 **최저위험**, (b) **가장 자주 됨**(77% vs fixed 65%), (c)
**지배적 미해결 실패(대칭 aliasing/미러플립, 원래 실패의 30%)의 직접 치료제** — 절대 헤딩
factor는 180° 플립을 즉시 죽인다. gauge의 오차는 XY-only라 회전이 약하게만 관측됨 →
**gauge에 헤딩 항 추가**(또는 `g`에 약한 yaw 데이텀). 브리지는 이미 gnss_odom에 절대 yaw +
`covariance[35]` 발행 중. **위치 앵커 A/B와 분리, M2/M3로 당김.**

**C2 — 위치 앵커를 "약한 종속"이 아니라 "A/B 실측가중 로버스트 factor"로.** RTK 1.4~3.3cm
vs CSM 7~9cm → fixed(65%)면 올바로 융합한 RTK는 CSM을 이겨야 정상. 가중치를 사전에 "약하게"
못박지 말고 **A/B가 measured realized 오차로 결정**(gauge가 common-mode bias를 흡수해
안전은 확보). 단 believed≠realized라 하드결합은 금지.

**C3 — A/B 승격 게이트 = 적대적 believed≠realized 스위트**(nominal RTK 아님). 과거를 죽인 건
멀티패스/float-excursion/정지 헤딩점프지 nominal RTK가 아니다. sim이 이제 이걸 생성 → 승격은
**"confident-wrong fix가 유령/association 분할 안 만든다"** 를 통과해야. nominal RTK는 sanity만.

**C4 — 매핑기 데이텀 보호 유지(gauge만으론 부족).** χ²는 `S=R+HPHᵀ`라 초기매핑(P 큼)에 게이트가
넓어져 colored excursion을 admit → 랜드마크 변형/유령. **매핑 중 데이텀은 pose/g만 당기고
랜드마크 재형성 금지** 노브 하나 유지(χ²는 매핑 방어 아님).

**C5 — 역할3 재명명**: "INS전멸 복구"는 오명명(앵커=INS와 같은 SBG, 독립 아님). INS 죽으면
앵커도 죽음 → 진짜 백스톱은 휠+자이로 사다리뿐. 앵커가 실제로 복구하는 건 **발산/aliased CSM
pose(SBG 정상, SLAM 플립)**. 역할3 = **"SLAM 발산/aliasing 재시드"**(헤딩 필요=C1).

**맹점(반영)**:
- **정지-헤딩 가정 낡음**: 브리지·sim 정지모델이 `gps1_hdt_used ~1%`(PSRDIFF 07-23) 가정 →
  **RTK 07-24는 정지 시 77% 사용.** ZUPT **헤딩 동결이 좋은 헤딩 버릴 수 있음** → 07-24 bag
  재적합 + ZUPT는 dual-ant 미사용 시에만 헤딩 동결. (M1에 포함.)
- 안테나 **lever arm**(공간; A1 시간동기와 별개, 15m/s+요레이트서 cm-dm) → 부록 A에 명시(고정 extrinsic).
- **CUSUM↔플립**: 플립 후 옳은 RTK가 huge innovation으로 영구 거부 → 강제 재앵커는 **헤딩
  운반 gauge 재시드**여야(병진-only 재시드는 플립 못 품).
- **수치 승격 기준** 명시(예: nominal ≥X% ATE 감소 AND 적대 스위트 지도무결성 무회귀).
- CSM reaper(재추가됨) vs 앵커 재시드 상호작용(gate 콘 reap 안전) 확인.

## 1. Fault 분류 (관측을 원인축으로) — 리뷰 확인됨
solution_mode(SBG EKF 상태)와 correction_type(GNSS 등급)은 **독립 두 축**. RTK bag:
mode 4 유지한 채 tier가 fixed↔float↔psrdiff 붕괴(65/24/5/5/2). 원인축: 위성가시성·
보정링크·멀티패스·baseline·동역학·센서물리·퍼셉션 — 각 원인이 여러 관측 동시구동.

## 2. 시뮬레이션 — 최소실용 원인기반 (리뷰로 축소)
현행 `sim_ellipse_d.py`가 ~70% 구현(tier별 pos/vel σ, believed≠realized, rtk_auto,
정지레짐 **완성**). 채울 것:
- **L1**: `rtk_auto`를 RTK bag 점유율(65/24/5/5/2)로 **재적합**(single/NO_SOL 분기 추가).
- **L2**: `sigma_heading`를 **tier표로**(실측 0.12°↔0.34°, 유일 실제 갭·식별가능);
  `gps1_hdt_used` 파생; solution_mode는 NO_SOL 끝단만 결합; `sigma_pos_psrdiff` 0.50→0.72.
- **L3**: 두 bag 시퀀스를 회귀 fixture + seed MC.
- 출력계약: gps_pos/hdt/num_sv/diff_age는 **소비자 생길 때만**(YAGNI).
- **RTK가 이제 실측 있으니, sim이 앵커 A/B용 cm RTK 시나리오를 생성 가능.**

## 3. SLAM 아키텍처
**주권(현행 유지)**: 콘 CSM(연속 드리프트억제) + 오도 모션엣지 + robust kernel.
velocity σ = 모션 입력의 유일 SLAM-직결 레버.

**B-모션. GNSS-free 열화 사다리 (직교 튜플)**:
```
motion_src ∈ {ins_fused(mode≥3), raw_gnss(RTK 델타+Doppler, HDT/자이로 헤딩 — EKF 리셋 대비, 2026-08-17 구현),
              wheel_DR(휠+EKF/HDT/자이로 헤딩), dead(GNSS-free 모션B: 휠+자이로)}
perception ∈ {ok, sparse, lost}
정지: ZUPT 동결(브리지 구현; RTK bag서도 유령 1.8m 잔존→유효)
```
조합별 거동 정의(대부분 σ/모션소스/DR-bound만 변화). {dead, lost}=오도 DR bounded로 coast.

**B-앵커(부록 A, A/B 게이트 후에만)**: gauge 정점 + 통계적 게이트 + DCS의 **약한 전역
bound**, 3역할 한정. CSM 주권 아래 얹음.

**B-supervisor**: 튜플 상태 추적·표출(HUD), 명시 전이(hysteresis).

## 4. 검증 하네스
- **몬테카를로**(L3 fault seed 스윕) → ATE·지도 일관성·유령콘·랩 반복정밀도.
- **적대 케이스**: 터널(→모션B), 정지(ZUPT), 콘 희박, 저속 헤딩, 멀티패스(believed≠realized).
- **앵커 A/B**: CSM-only vs CSM+최소앵커, 폐곡 & 개활 트랙, 실 cm RTK. 승격 판정.
- real bag 재생 + control_harness 폐루프 + rung 전이 회귀.

## 5. 마일스톤 (v0.4 — 헤딩 데이텀 앞당김)
1. **M1 — sim 최소실용**(heading σ tier + rtk_auto 재적합 + psrdiff σ 0.72) + **정지모델
   07-24 재적합/ZUPT 헤딩동결 조건화**(dual-ant 미사용 시만). 저위험·식별가능.
2. **M2 — GNSS-free 모션 사다리 튜플화(B-모션) + supervisor** + **★절대 헤딩 데이텀(C1)★**
   (gauge 헤딩 항, 위치앵커와 분리, 통계게이트) — SBG 최고신호로 대칭 aliasing 침. 저위험·고가치.
3. **M3 — 퍼셉션/CSM 강건성**(association·CSM 신뢰) — 실패 지배원(포렌식: 상류 50%+코어 30%).
4. **M4 — L3 시나리오 + §4 몬테카를로 하네스**(+ 적대 believed≠realized 스위트, 앵커 승격게이트용).
5. **M5 — 최소 위치 앵커(부록 A) + 적대 A/B 게이트.** 매핑기 보호(C4)·수치기준·CUSUM 헤딩재시드
   포함. nominal서 CSM 못 이기면 georef/재시드 한정.

## 6. 리스크
- 앵커가 폐곡서 또 CSM에 질 수 있음 → A/B가 그걸 잡음(승격 안 하면 손해 0).
- 대칭-layout aliasing(코어)은 앵커와 별개로 실재 → CSM/association 쪽에서 다뤄야(M3).
- GNSS-free 전환 히스테리시스; 정지 유령(휠속 CAN 미배선 시 ZUPT 비활성)→휠오돔 확보.

---

## 부록 A — 최소·올바른 앵커 (부활 시 정확히 이것만)
과거 앵커는 정교했으나 3결함으로 폐곡서 졌고 20가드로 커졌다. 부활은 **아래 그대로**:

**유지 (과거의 옳은 설계)**
- **SE2 gauge 정점**(`edge_se2_gauge_xy` 개념): map→ENU 등록을 모델링, 약한 identity prior로
  soft-hold — 아웃티지에 floating, colored bias를 gauge `g`에 흡수(pose 안 찢음). heading 관측가능.
- **DCS 로버스트 커널**(switchable constraints) — Huber보다 우월.
- **verified recovery** 병진-only(yaw 스크류는 먼 pose를 shred).

**고침 (과거 버그)**
- **A1 시간동기**: fix를 `rawOdomAt` 보간으로 **fix-시각 키프레임에 결합**(최근접 결합 금지).
- **A2 통계적 게이트**: `S = R + H P Hᵀ`(pose 주변공분산, 단일정점 back-sub) 기반 χ² +
  **CUSUM 연속거부 시 강제 재앵커**(드리프트한 지도가 옳은 RTK를 영영 거부 차단).

**삭제 (실패 증폭기 — 절대 재도입 금지)**
- hard RTK `setFixed` 핀(gauge 우회·이중 고정) → **데이텀은 gauge 하나만.**
- per-landmark GNSS priors, coherence-only recovery, seam-anchor grid search.
- 20개 가드 스택 전체(founding gates·map admission·fling cull·map-trust scaling·auto-reloc…).

**역할 한정**: 주 등록은 CSM. 앵커는 **약한 전역 bound**로 (1)지오레퍼런싱 (2)비폐곡 seam
(3)INS전멸 복구에만. **A/B로 CSM 능가 증명 못 하면 폐곡엔 미적용.**
