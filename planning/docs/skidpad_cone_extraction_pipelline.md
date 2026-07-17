# Skidpad Cone Map 원형 구간 추출 파이프 라인

Cone Map CSV에서 skidpad의 좌/우 원형 구간을 구성하는 cone을 분리하고,
global path 생성의 입력이 되는 **원 4개(중심·반지름)** 와 **skidpad
좌표계**를 산출한다.

**핵심 설계 원칙 — 문서 전체를 관통하는 5가지**

1. **좌표계·규격 불가지론**: 입력 좌표계에 대한 가정이 없고(SE(2) 불변),
   룰북 수치를 하드코딩하지 않는다. 규격 지식(표준 배치·공식 색상)은
   기본 off인 flag로만 들어온다.
2. **가설은 검정 후에만 제약으로**: "좌우 대칭인 동심원 2쌍 + 직선
   코리도어"라는 위상 구조는 가설로 쓰되, 순차 체인 M₀→M₁a→M₁b의 증분
   검정을 통과한 만큼만 제약으로 적용한다 (§5 Step 3).
3. **색상은 가설 생성의 사전 분할일 뿐**: 최종 membership은 기하로만
   결정하고(§5 Step 5), 무색(color-agnostic) 가설 풀로 가설 공간의 색상
   독립성을 보장한다 (§5 Step 1).
4. **실패는 예외가 아니라 상태**: 모든 알고리즘 실패는 `ValidationStatus`로
   산출물에 남고, 출력 유효성은 geometry × route의 3-상태로 판정한다 (§6).
5. **모든 통계 임계값은 조정 대상**: SLAM CSV의 landmark별 marginal
   covariance는 landmark 간 cross-covariance를 담지 못하므로, 본 문서의
   χ²/z/F 값은 확률 게이트가 아니라 **명목 분포에서 출발한 적합도 점수**이며,
   임계값은 보편 상수가 아니라 데이터에 맞춰 조정할 기본값이다.

**문서 버전**

| 버전 | 요약 |
|---|---|
| v1 | 초안 |
| v2 | 전면 개정 — 좌표계 가정 제거, 검정 후 제약, 정량 완료 기준 |
| v2.1 | 1차 외부 리뷰 반영 — M₀/M₁ 2-모델 구조, capability flags 등 |
| v2.2 | 2차 외부 리뷰 반영 — 모델별 출력 정책, E(S) 재정의, F-검정 격하 등 |
| v2.3 | 3차 외부 리뷰 독립 재검증 + 저장소 실측 재검증 — 체인 증분 검정, 무색 가설 풀, 3-상태 출력 등 |

---

## 1. 목적과 범위

- **입력**: EUFS-style cone map CSV
  (`tag,x,y,direction,x_variance,y_variance,xy_covariance`)
- **출력**:
  1. 원 4개의 파라미터 (`detected_skidpad_circles.csv`)
  2. cone별 membership (`skidpad_circle_cone_members.csv`)
  3. skidpad 좌표계 (`skidpad_frame.json`)
- **출력 유효성**: centerline 관련 출력은 동심성 제약이 채택된 경우에만
  유효하다 (§6 출력 정책). M₀ 채택 시 원 4개는 진단 전용이며 global path
  입력으로 사용할 수 없다. global path 입력 가능 여부는 **geometry 유효성
  ∧ route 해소(L/R·방향)** 의 논리곱이다 — 동심성이 채택돼도 L/R 미해소면
  주행 route를 만들 수 없다.
- **규격(반지름·중심 간 거리)은 알 수 없다고 가정**한다. 룰북 수치를
  하드코딩하지 않는다. "좌우 대칭인 동심원 2쌍 + 직선 코리도어"라는
  **위상 구조**는 가설로 사용하되 **데이터로 검정한 뒤에만 제약으로
  적용**한다 (§5 Step 3). 표준 배치 관계(외측 원과 반대쪽 내측 원의 근접
  접함)는 위상이 아니므로 별도 flag로 분리한다 (§5 Step 6).
- global path 생성 자체는 다음 단계 (§9 제외 범위).

## 2. 입력 계약, capability flags, 선행 조건

- tag 종류: `blue`, `yellow`, `orange`, `big_orange`, `car_start`, (시뮬 한정)
  `midpoint`. `midpoint` 행은 cone이 아니므로 모든 처리에서 제외한다
  (§3의 실측 대조에만 참고).

**Capability flags** — 로드 직후 검사하고 이후 단계가 자동으로 degrade한다.
실차 SLAM 출력에는 midpoint가 없고 orange·covariance·heading 의미도 보장되지
않는다. "존재"와 "유효"와 "의미 검증"을 분리한다.

| flag | 판정 | 실패 시 fallback |
|---|---|---|
| `has_car_start_row` | `car_start` 행 존재 | L/R `unresolved`, frame 부호 모호성 기록 |
| `heading_value_finite` | direction이 유한값 | heading 무시 → car_start **위치** 기반 해소 시도 (§5 Step 2c) |
| `heading_semantics_verified` | **frame 도출 후 사후 검증** (§5 Step 2c): heading이 frame y'축과 ≤ 30° 정렬 ∧ car_start의 \|x'\| ≤ 3 m (코리도어 축 근방) | 실패 시 heading 무시 → car_start 위치 기반 해소 시도 (§5 Step 2c), 그마저 불가하면 L/R `unresolved` (placeholder 0.0을 유한성만으로 구별할 수 없으므로 교차 검증 필수) |
| `has_covariance_columns` | 7컬럼 존재 | σ²_pt 항 제거, σ_model만 |
| `covariance_values_valid` | **전 행**: σ_x², σ_y² ≥ 0 ∧ 유한 ∧ σ_xy² ≤ σ_x²σ_y²·(1+10⁻⁶) | 위반 행 존재 시 σ_model만 + warning |
| `covariance_all_zero` | 전 행의 3개 값이 모두 0 | placeholder로 간주, σ_model만 |
| `covariance_is_heteroscedastic` | 행 간 값이 상이 | **진단 정보일 뿐** — 전 행 동일 상수여도 유효하면 등분산 covariance로 사용한다 |
| `has_orange_candidates` | orange ≥ 6 | 직선 모델 비활성. **yellow/blue를 straight로 강제 배정 금지** |
| `corridor_fit_valid` | frame 확보 후 side별 ≥ 3개 ∧ side별 종방향 spread ≥ 3 m ∧ fit 검사 통과 (§5 Step 5) | 직선 모델 비활성 + status (orange ≥ 6이어도 4개가 한쪽에 몰리면 실패해야 한다) |

**선행 조건**: 실차 경로에서는 `hyu_localization`이 landmark marginal covariance
포함 EUFS 7-컬럼 CSV exporter가 필요하다. 현 저장소 확인 결과(2026-07,
`hyu_localization/src/graph_slam_node.cpp` grep) `save_graph` 서비스는 g2o 그래프
저장뿐으로 해당 exporter는 없다. exporter 확보 전까지 실맵 검증은 불가능하다.

**주의**: CSV의 landmark별 2×2 marginal covariance는 SLAM pose 불확실성으로
인한 **landmark 간 cross-covariance를 담지 못한다**. 따라서 본 문서의 모든
χ²/z/F 임계값은 정확한 확률 게이트가 아니라 **명목 분포에서 출발한 적합도
점수의 조정 대상 임계값**이다.

## 3. 실측 근거 (파라미터 기본값의 출처)

`eufs_sim/eufs_tracks/csv/skidpad.csv` 실측:

| 항목 | 값 |
|---|---|
| 우측 원 중심 / 좌측 원 중심 | (9.250, 0.000) / (−9.248, 0.000) |
| R_inner / R_outer (cone 중심 기준) | 7.612 / 10.612 m (폭 정확히 3.000 m) |
| 내측 원 cone 수 / 외측 원 cone 수 | 내측 17행 = **유니크 16개 + 완전 중복 행 1** (완전 원 — FSG 내측 원 16개 규격과 정합) / **13개 (부분 호, 교차부 갭)** |
| centerline 반지름 (원 위 midpoint 26행) | 9.111 m ≈ (R_in+R_out)/2 = 9.112 |
| 중심 간 거리 d | 18.498 m, d − (R_in+R_out) = +0.27 m |
| 직선 코리도어 half-width | 1.637 m (orange 20개 중 18개; **2개는 exit funnel (±0.55, 21)** — 코리도어 직선에서 1.09 m 이탈, §5 Step 5 robust fit의 실증 근거) |
| 내측 원의 교차부 쪽 cone | 정확히 x = ±1.637 (코리도어 모서리와 겹침) |
| car_start | (0, −14.4), direction = 1.5708 (+y) |
| tag 개수 | blue 30 / yellow 30 / orange 20 / big_orange 4 / midpoint 30 / car_start 1 |
| **완전 중복 행** | blue(−1.637, 0)·yellow(16.862, 0)·midpoint(0.00025, 0) 각 2회 — §5 Step 0의 `possible_duplicate` 마킹은 **기준 데이터에서도 발화**한다 |
| **원에 속하지 않는 midpoint** | 4행: (0, 0)×2(중복 포함)·(0, ±2.913) — 교차부/코리도어 축 위 점. centerline 반지름 산출 시 이 4행을 제외해야 하며, 미제외 시 fit이 R ≈ 9.19·최대 잔차 364 mm로 오염된다 |
| FSG 룰북과 비교 | 18.25/7.625/10.625와 **불일치** → 규격 하드코딩 금지 재확인 |
| FSG 관계식 검산 | FSG에서는 d − (R_in+R_out) = 18.25 − 18.25 = **0 (정확히 접함)** → §5 Step 6 layout prior의 근거 |


이 CSV의 fitting 잔차는 최대 2 mm로, robust 장치는 시뮬 원본만으로는 검증되지
않는다. §7의 threshold 기본값들은 이 단일 시뮬 데이터와 공학적 추정에서 나온
**조정 대상**이며, 보편값이 아니다.

## 4. 파이프라인 개요

```
Step 0  전처리·capability 검사 : tag 필터, 행별 covariance 유효성, 중복 마킹
Step 1  다중 원 검출           : 색상별 + 무색(color-agnostic) MSAC 가설 풀
                                 → 전 가설 inlier refit → 단일 전역 4원 조합
                                 선택 E(S) (sequential 해는 풀에 삽입)
Step 2  M₀ 확정·pairing·frame : 4원 독립 정밀화(Hyper→LM), loop pairing
                                 3-조합 전수(full Mahalanobis), 잠정 frame,
                                 heading 교차검증 + 위치 기반 L/R 해소
Step 3  제약 검정 → M₁ → 채택  : 순차 체인 M₀→M₁a→M₁b — 화살표마다 추가
                                 제약만 증분 검정(loop별 F / Δp=2 F),
                                 채택 모델로 frame 재도출
Step 4  residual 척도 정의     : 비교용 균일 z / 진단용 studentized /
                                 strict용 LOO predictive
Step 5  전역 재할당·분류        : (z ∧ 절대) 게이트, arc interval(반복 갱신),
                                 Δχ² ambiguity, 재할당↔refit 반복,
                                 수렴 후 제약 재검증 + 단방향 fallback
Step 6  검증·색 진단           : layout prior 분리, expected-color 진단
Step 7  출력                   : 모델 × route 3-상태 출력 정책 (§6)
```

| 단계 | 핵심 질문 | 핵심 산출 |
|---|---|---|
| Step 0 | 입력이 무엇을 보장하는가 | capability flags, 중복 마킹 |
| Step 1 | 원 4개는 어디에 있는가 | 4원 조합 (가설 풀 → 전역 선택) |
| Step 2 | 어느 원이 짝이고 좌표계는 무엇인가 | M₀, loop pairing, 잠정 frame |
| Step 3 | 위상 가설(동심·대칭)이 데이터로 지지되는가 | adopted_model ∈ {m0, m1_concentric, m1_shared_radii} |
| Step 4 | cone-모델 거리를 어떤 척도로 잴 것인가 | 균일 z / studentized / LOO z |
| Step 5 | 각 cone은 최종적으로 어디에 속하는가 | membership + 수렴 후 재검증 |
| Step 6 | 결과가 자기 일관적인가 | validation_status, 색 진단 |
| Step 7 | downstream이 써도 되는가 | 3-상태 (geometry × route) |

어느 단계에도 입력 좌표계에 대한 가정이 없다. 전체 파이프라인은 SE(2) 변환에
불변이어야 하며, 제약은 검정 통과 후에만, centerline 출력은 동심성
채택 후에만 허용된다.

---

## 5. 단계별 상세

각 단계는 **개념과 설계 근거 → 관련 문헌 → 수식·판정 규칙 → 구현**의
순서로 기술한다.

### Step 0. 전처리·capability 검사

**개념과 설계 근거.** 이후 모든 단계는 "입력이 무엇을 보장하는가"에 따라
자동으로 degrade해야 한다(§2 flags). 이 단계에서 tag를 필터하고 행별
covariance 유효성을 검사한다.

- `midpoint` 제외, `blue/yellow`만 원 검출 대상, `orange/big_orange/car_start`는
  frame·코리도어용으로 보존. §2의 capability flags 산출 (covariance는 행별
  유효성 검사 포함).
- **중복 처리**: 기본값 `dedup_enabled = false`. cone map이 SLAM data
  association의 결과라면 각 행은 독립 landmark여야 하며, 자동 병합은 (a) 실제
  다른 cone의 융합, (b) association 오류 은폐, (c) 독립이 아닌 추정치의
  inverse-covariance 평균이라는 3중 위험이 있다. 본 저장소의 graph SLAM은 이미
  upstream에서 반복 관측을 융합하므로 downstream 병합은 이중 처리다.
  거리 < `dup_flag_radius`(0.75 m)인 동색 쌍은 병합하지 않고
  `possible_duplicate`로 마킹만 한다.
- **주의**: 기준 데이터 자체에 완전 중복 행 3개가 존재하므로(§3) 이 마킹은
  오염 없는 정상 입력에서도 발화한다 — 마킹 발화 자체는 오류가 아니다.

### Step 1. 다중 원 검출 — 색상별+무색 가설 풀 → 단일 전역 조합 선택

**개념과 설계 근거.** 각 색은 정상 맵에서 원 2개를 담는다(한쪽 inner +
반대쪽 outer). 색상 분할은 **주 가설 생성기의 사전 분할**로만 쓰이고 최종
membership은 Step 5의 전역 재할당이 결정한다. 단, 사전 분할만으로는 가설
공간 자체가 색상에 종속된다 — 색이 심하게 손상된 원은 애초에 풀에 들어오지
못하고, Step 5의 재할당은 기존 원에 점을 옮길 뿐 **새 원 가설을 만들지
못한다**. 따라서 무색 풀 C로 가설 공간의 색상 독립성을 보장한다.

sequential greedy(색상별 "MSAC → inlier 제거 → MSAC")는 첫 가설이 서로 다른
원의 점을 섞어도 그럴듯한 4원을 낼 수 있다. 그래서 sequential 해를
특권화하지 않고 풀의 원소 중 하나로만 취급하며, **전역 조합 선택 하나**가
최종 결정한다 (sequential 해가 풀에 포함되면 전역 최적 조합의 비용은 그
이하임이 자명하다).

**관련 문헌.**

- RANSAC 프레임: Fischler & Bolles, CACM 1981.
  반복 수 `N ≥ log(1−p) / log(1−w³)`, p=0.99, 최악 inlier비 w=0.3 → N≈168.
  기본값 500 + 조기 종료.
- MSAC 스코어: Torr & Zisserman, CVIU 2000, 식 (4):
  `ρ(e²) = min(e², T²)`, 단일 모델 비용 `C = Σᵢ ρ(eᵢ²)` 최소화.
- 가설 사전 refit: Chum, Matas & Kittler, LO-RANSAC, DAGM 2003.
- 퇴화 가드의 배경: 원 추정량은 표준 오차모델에서 유한 모멘트를 갖지
  않는다 (N. Chernov, arXiv:0907.0429) — 반지름 bound는 heavy tail을
  "해결"하는 것이 아니라 비물리적 가설만 걸러낸다.

**수식·판정 규칙.**

- 최소 샘플 모델(3점 원): 수직이등분선 연립
  ```
  | 2(x₂−x₁)  2(y₂−y₁) | |cx|   | x₂²+y₂²−x₁²−y₁² |
  | 2(x₃−x₁)  2(y₃−y₁) | |cy| = | x₃²+y₃²−x₁²−y₁² |
  ```
  행렬식 |det| < 1e−9·(스케일)² 이면 공선 → 샘플 폐기.
- 퇴화 가드: 반지름이 `[r_min, r_max]` 밖이면 가설 폐기. 준공선 최소표본이
  만드는 비현실적 가설 제거용 **configurable degeneracy guard**이며, 기본값
  [2, 25] m는 FS 차량 스케일의 프로젝트 기본값이다.

**가설 풀 구성** — 세 풀을 만들고 합친다:

- **풀 A/B (색상별)**: blue/yellow 각각 sequential MSAC ("MSAC → inlier
  제거 → MSAC"). 라운드별 **상위 K개(기본 10) 가설을 보관** → 최대 4K = 40개.
- **풀 C (무색)**: blue+yellow 전체에서 MSAC, 상위 K개. 전체 점집합에서
  단일 원의 inlier 비율은 w ≈ 13/60 ≈ 0.2로 떨어지므로 반복수는 전용
  `ransac_iters_agnostic`(1000; N(0.99, w=0.2) ≈ 574의 여유)를 쓴다.
  색 플립·색 누락·classifier 오류로 색상별 풀이 놓친 원을 회수하는 경로다.
- **sequential 해 삽입**: 색상별 sequential의 최종 4원도 풀에 명시적으로
  넣는다 — 전역 선택이 sequential 이상의 해를 자동으로 보장하게 된다.
- **사전 refit**: 조합 평가 전에 풀의 **모든 가설을 자기 inlier로 LM 1회
  refit**한다 (LO-RANSAC 논거). 3점 최소표본 파라미터로 E(S)를 매기면
  파라미터 노이즈 때문에 올바른 조합이 과대 비용을 받아 밀릴 수 있다.
  refit 후 중복 가설을 pruning한다.

**4원 조합 전역 선택.** 병합·refit·pruning된 풀에서 4개 부분집합 S를 고르는
문제. 목적함수는 **point-wise 최소 할당**이다. (naive한 정의
`Σ_hyp Σ_pts ρ(e²)`는 각 점이 모든 가설에 비용을 내거나(할당 제약과 모순),
할당된 점만 세면 공집합 할당이 최적이 되는 붕괴식이다 — v2.2에서 교체.)

```
E(S) = Σ_i min[ T²,  min_{h∈S} e_ih² ]        (i = 모든 yellow/blue cone)
```

- 어느 가설에도 맞지 않는 점은 자동으로 outlier 페널티 T²를 낸다.
- 부가 제약 (모두 만족해야 유효한 S):
  1. **중복 가설 배제**: ‖c_a−c_b‖ < `dup_hyp_center`(1.0 m) ∧
     |R_a−R_b| < `dup_hyp_radius`(1.0 m)인 쌍은 동일 물리 원으로 간주,
     동시 선택 금지 (풀 단계에서 사전 pruning).
  2. **support**: 각 h ∈ S에 대해 `argmin이 h이고 e² ≤ T²`인 점 ≥
     `min_inlier_count`.
  3. **pairing 가능성**: S의 4원이 Step 2b의 2-2 분할 중 하나 이상에서
     유효한 loop 구조(반지름 분리 조건 포함)를 이룰 것.
  4. **arc coverage**: 각 h ∈ S의 관측 호 coverage(Step 5·6과 공용 함수)
     ≥ `arc_min_hypothesis`(90°). inner/outer 역할은 pairing 후에야
     정해지므로 여기서는 **공통 하한**만 요구한다 (role별 기준 240°/120°는
     Step 6). support 하한(2번)이 5–7점짜리 가짜 호는 이미 막지만,
     min_inlier_count를 넘는 8점 이상의 짧은 호는 coverage로만 걸러진다.
- 규모: 풀 ≤ 54개(색상별 40 + 무색 10 + sequential 4)면 C(54,4) ≈ 3.2×10⁵
  — 전수 평가로 충분. 외부 multi-model 라이브러리 불필요 (문제가 커지면
  PEARL류가 상위 대안, §10).

**선택과 실패 처리.** 부가 제약을 만족하는 조합 중 E(S) 최소를 채택한다.
유효한 조합이 하나도 없으면 검출 실패 status.

**구현** (C++17 / Eigen)

```cpp
struct CircleHypothesis {
  Eigen::Vector2d center;
  double radius;
  std::vector<std::size_t> inlier_indices;
  double msac_score;
  bool refitted;                            // 조합 평가 전 inlier LM refit 완료 여부
};

// 원 1개만 검출되는 실패를 표현할 수 있어야 한다
struct ColorDetection {
  std::vector<CircleHypothesis> detected;   // 0..2개 (sequential 해 — 풀에 삽입됨)
  std::vector<CircleHypothesis> top_k;      // 조합 선택용 가설 풀
};

ColorDetection detectTwoCirclesPerColor(
  const std::vector<Cone> & cones_of_one_color, const MsacParams & params,
  std::mt19937 & rng);
// 무색 풀 C — blue+yellow 전체에서 가설 생성 (ransac_iters_agnostic)
std::vector<CircleHypothesis> detectAgnosticPool(
  const std::vector<Cone> & blue_and_yellow, const MsacParams & params,
  std::mt19937 & rng);

// 단일 전역 선택 — 풀 = A ∪ B ∪ C ∪ sequential, 전 가설 refit 후 E(S)
struct FourCircleSelection {
  std::array<CircleHypothesis, 4> circles;
  double cost;                              // E(S)
  SelectionSource source;                   // 진단용: 선택된 4원의 출신 풀
};
std::optional<FourCircleSelection> selectFourCircles(
  const ColorDetection & blue, const ColorDetection & yellow,
  const std::vector<CircleHypothesis> & agnostic_pool,
  const std::vector<Cone> & all_cones, const MsacParams & params);
```
- 난수는 `std::mt19937` + 파라미터로 받은 고정 seed (재현성 확보).

### Step 2. M₀ 확정, loop pairing, 잠정 frame

#### 2a. 개별 원 정밀화 → 비제약 모델 M₀

**개념과 설계 근거.** 원 4개를 **서로 독립적으로** 정밀화한 것이 비제약
모델 M₀다:

```
θ₀ = (c₁, R₁, c₂, R₂, c₃, R₃, c₄, R₄)   — 12 파라미터
```

외측 원이 13개·부분 호이므로 bias가 가장 작은 **Hyper fit**을 대수적
초기화로, geometric fit(LM)을 최종으로 쓴다.

**관련 문헌.**

- Chernov & Lesort, "Least Squares Fitting of Circles and Lines"
  (arXiv:cs/0301001) — Kåsa/Pratt/Taubin 통일 정리와 geometric fit의 수렴
  논의.
- Al-Sharadqah & Chernov, "Error Analysis for Circle Fitting Algorithms"
  (arXiv:0907.0421) — 부분 호 bias 분석과 **Hyper fit**, centering·SVD 절차.
- Abdul-Rahman & Chernov, arXiv:1505.03795 — 대반경에서도 기계 정밀도로
  안정한 원 fit 대안.

**수식·판정 규칙.**

- Hyper fit: 원을 `A(x²+y²) + Bx + Cy + D = 0`, `u = (A,B,C,D)`로 두고
  `z_i = x_i²+y_i²`, `w_i = (z_i, x_i, y_i, 1)ᵀ`, 모멘트 행렬
  `M = (1/n)Σ w_i w_iᵀ`. 일반화 고유문제 `M u = η N_H u`에서 가장 작은 양의
  η의 고유벡터. Hyper 제약 행렬 (0907.0421):
  ```
        | 8z̄  4x̄  4ȳ  2 |
  N_H = | 4x̄   1   0  0 |
        | 4ȳ   0   1  0 |
        | 2    0   0  0 |
  ```
  복원: `c = −(B, C)/(2A)`, `R = sqrt(B²+C²−4AD)/(2|A|)`.
  (remark: A→0 극한은 직선 `Bx+Cy+D=0`으로 퇴화한다. **단 이는 대수적
  성질일 뿐, noisy data에서 |A|의 크기는 정규화 스케일에 의존하므로 직선
  판정기로 쓰지 않는다** — 직선 판별은 Step 5의 전용 TLS fit.)
- **수치 구현 (필수 규정 — 출처 구분)**:
  1. centroid 평행이동 (`x̄ = ȳ = 0` → N_H 단순화) — **문헌 절차**
  2. SVD 기반 고유문제 풀이 (`Eigen::JacobiSVD` /
     `SelfAdjointEigenSolver`) — **문헌 절차**
  3. RMS 스케일 정규화 + 역정규화 — **프로젝트 추가 안정화** (원 논문의
     필수 단계는 아니나 대형 평행이동·대반경 입력과 양립하기 위한
     공학적 보강)
  명시적 `inverse()` 사용 금지.
- **geometric refine은 damped GN / Levenberg–Marquardt 의무** — 부분
  호·나쁜 초기값에서 순수 GN은 step 과대·radius 발산·비용 증가가 가능하다:
  ```
  (JᵀWJ + λ·diag(JᵀWJ)) Δθ = −JᵀWe
  - 수락 규칙: cost(θ+Δθ) < cost(θ)일 때만 적용, λ ← λ/3; 아니면 λ ← 10λ
  - W 갱신 (IRLS): accepted step마다 현재 중심으로 nᵢ = (pᵢ−c)/‖pᵢ−c‖
    → σ²_pt,i = nᵢᵀΣᵢnᵢ → W 재계산. nᵢ가 중심에 의존하므로 갱신하지
    않으면 초기 원 방향에서 계산한 가중치가 끝까지 유지된다 (Σᵢ가 등방에
    가까우면 영향 미미, 비등방 SLAM covariance에서 유효)
  - 가드: R ∉ [r_min, r_max] 또는 비유한 → step 반려
  - 종료: ‖Δθ‖ < 1e−10 또는 max_iter(50) 또는 λ > 1e8
  - 전체 실패 시 대수해(Hyper) 반환 + fit_degraded status
  ```

**가중치 의미.** `covariance_values_valid ∧ ¬covariance_all_zero`이면
W = diag(1/(σ²_pt,i + σ²_model))를 **absolute**로 취급해
Cov(θ̂) = (JᵀWJ)⁻¹ (σ̂² 추가 곱 없음). covariance 미사용 시 W = I를
**relative**로 취급해 Cov(θ̂) = σ̂²(JᵀJ)⁻¹. 두 경로를 `weights_are_absolute`
플래그로 코드에서 분기한다. **단**: σ_model(0.15 m)은 알려진 확률 분포의
표준편차가 아니라 calibration된 공학적 오차 흡수항이므로, absolute 경로의
(JᵀWJ)⁻¹도 정확한 유한표본 확률 공분산이 아닌 **working covariance**다 —
코드 명명(`working_cov`)과 주석에 이 지위를 남기고, 여기서 파생되는 모든
Mahalanobis/χ²/F 값은 §2 주의대로 "적합도 점수 + calibration 대상
임계값"으로 해석한다. (absolute/relative 구분 자체는 σ̂² 재스케일 여부에
관한 표준 용법이므로 유지한다.)

**구현**

```cpp
struct CircleFit {
  Eigen::Vector2d center;   // is_line == true 이면 NaN (사용 금지)
  double radius, rmse;      //         〃
  Eigen::Matrix3d cov;      // (c,R) working covariance (§2 주의).
                            // absolute: (JᵀWJ)⁻¹, relative: σ̂²(JᵀWJ)⁻¹,
                            // σ̂² = RSS_w/(n−3). LDLT.
  bool is_line;             // 대수해가 직선으로 퇴화한 경우의 표식일 뿐,
                            // 직선 파라미터는 별도 LineFit으로 관리한다
};
struct LineFit { Eigen::Vector2d n; double d; double rmse; };  // nᵀp + d = 0

CircleFit hyperFit(const std::vector<Eigen::Vector2d> & pts);  // 내부 center/scale/SVD
CircleFit lmRefine(const CircleFit & init,
                   const std::vector<Eigen::Vector2d> & pts,
                   const WeightModel & w, const LmParams & lm);
```

#### 2b. loop pairing — 3-조합 전수 평가 (full Mahalanobis)

**개념과 설계 근거.** 원 4개의 2-2 분할은 3가지뿐이므로 전수 평가한다.
부분 호에서 중심 공분산은 방향 의존적 2×2 행렬이므로 스칼라 분산 근사를
쓰지 않는다.

**수식·판정 규칙.**

```
D²(a,b)   = (c_a−c_b)ᵀ (Σ_c,a + Σ_c,b)⁻¹ (c_a−c_b)     (Σ_c = cov의 2×2 블록)
J_pair(분할) = Σ_loop D²(inner, outer)
```

- 비용 최소 분할 채택. **1위·2위 비용 차** < `pairing_margin_delta`(6.0,
  Mahalanobis 스케일 — χ²₂ 95% ≈ 5.99에서 출발한 calibration 대상) 이면
  `validation_status = pairing_ambiguous`.
- loop 내 역할: 반지름 작은 쪽 = inner (색상 불사용).
- **loop 내 반지름 분리 검사**:
  `(R_out−R_in)/√(σ²_R,in+σ²_R,out) ≥ z_radius_sep`(3.0)를 요구.
  미달이면 같은 물리 원의 중복 가설이 한 loop로 묶였을 가능성 —
  `duplicate_suspected` status.
- 동심성의 형식 판정은 Step 3에서 하고, 여기서는 D² ≤ `chi2_pairing_gate`
  (5.99에서 출발, calibration 대상)를 **분할 유효성 게이트**로만 쓴다.

#### 2c. 잠정 frame 도출과 L/R 해소

**개념과 설계 근거.** frame은 loop 중심에서 도출한다 — heading은 부호
해소에만 쓰이며, 그마저 placeholder일 수 있으므로 기하와 교차 검증한 뒤에만
믿는다. heading이 실패해도 car_start **위치**만으로 부호를 풀 수 있다
(차는 코리도어 진입측에서 출발한다는 사실만 사용).

**절차.**

1. M₀에서 loop 대표 중심 (frame 용도 한정):
   `c_loop = (n_in·c_in + n_out·c_out)/(n_in+n_out)` (support 가중 평균 —
   M₀에서는 loop당 중심이 2개이므로 대표값 정의가 필요하다).
2. 원점 `O = (c_loopA + c_loopB)/2`, `ex = (c_loopB − c_loopA)/‖·‖`,
   `ey = perp(ex)` — 여기까지는 heading 불필요 (부호·L/R만 모호).
3. **heading 의미 교차 검증** — `has_car_start_row ∧
   heading_value_finite`일 때, heading h에 대해:
   - `|ey·h| ≥ cos(30°)` (skidpad 진입 방향은 코리도어 축과 정렬), 그리고
   - car_start의 frame 좌표 `|x'| ≤ 3 m` (차는 중앙 코리도어 연장선에서 출발)
   둘 다 만족하면 `heading_semantics_verified = true` → `ey·h > 0`으로 부호
   고정, `cross(h, c_loop − p₀) < 0`인 loop가 right, `lr_resolved_by =
   heading`. 하나라도 실패하면 heading을 **무시**하고 4의 위치 기반 해소로
   넘어간다. (0.0 같은 placeholder heading은 유한성 검사를 통과하므로, 교차
   검증 없이 채택하면 잘못된 frame을 조용히 만든다.)
4. **car_start 위치 기반 L/R 해소** — heading이 없거나 교차 검증에
   실패해도, 차는 코리도어 **진입측**에서 출발하므로 car_start의 위치만으로
   부호를 풀 수 있다: `|y'(car_start)| ≥ carstart_y_min`(5 m, 교차부에서
   충분히 이격) ∧ `|x'| ≤ 3 m`이면 `y'(car_start) < 0`이 되도록 ey 부호를
   고정하고 (ex, ey)를 우수계로 맞춰 ex를 결정 — 주행자 기준 `x' > 0`인
   loop가 right. 성공 시 `lr_resolved = true, lr_resolved_by = position`.
   3의 heading 해소가 성립한 경우에도 이 위치 조건과 상충하면 `unresolved`
   + `frame_sign_conflict` 경고 (데이터 모순 — 조용히 한쪽을 믿지 않는다).
   둘 다 불가하면 `lr_resolved = false, lr_resolved_by = none` — frame은
   좌우 반전·180° 회전 모호성을 가지며 JSON에 기록한다.
   route 미해소의 출력 영향은 §6.
5. purity·색 진단은 여기서 하지 않는다 (Step 6).

**구현**

```cpp
std::optional<LoopPair> pairLoops(
  const std::array<CircleFit, 4> & m0, const PairingParams & params,
  ValidationStatus & status);
Frame deriveFrame(const LoopPair & loops,
                  const std::optional<CarStartPose> & car_start,
                  const FrameParams & params);   // heading 교차검증
                                                 // + 위치 기반 L/R 해소
```

### Step 3. 제약 검정 → M₁ refit → 채택 (순차 체인 증분 비교)

**개념과 설계 근거.** 부분 제약 채택은 없다. 후보 모델은 체인 위의
3종뿐이다 — shared-radii 모델은 loop당 중심이 1개이므로 구조상 동심성을
전제하며, 따라서 격자(lattice)가 아니라 체인이다:

```
M₀ (p=12) ──[동심성, Δp=4]──▶ M₁a (p=8) ──[좌우 반지름 공유, Δp=2]──▶ M₁b (p=6)
```

**화살표마다 추가되는 제약만 증분 검정한다.** M₁b를 M₀와 직접 비교(Δp=6)
하면 두 제약의 비용이 하나로 섞인다: 동심성이 완벽하고 반지름 공유만
위반이어도 위반 신호가 (6/2 =) 3배 희석되며, m1_concentric이 F-기각된
상태에서 m1_shared_radii가 채택되는 비일관이 생길 수 있다. 상위 화살표가
실패하면 하위 모델은 **후보 자격 자체가 없다**.

**3a. 1단계 — M₀ → M₁a (동심성)**

- **사전 게이트 (M₀ 기반)**: loop별 H_conc:
  `D² = Δcᵀ(Σ_in+Σ_out)⁻¹Δc ≤ 9.21` (명목 χ²₂(0.99) 출발, calibration
  대상). **양쪽 loop 모두** 통과해야 진행.
- **M₁a fit** — loop별 concentric fit (θ ∈ R⁴/loop):
  ```
  r_i = ‖p_i − c‖,   e_i = r_i − R_g(i),   g(i) ∈ {in, out}
  ∂e_i/∂c = −n_iᵀ,   ∂e_i/∂R_k = −δ_{g(i),k},   n_i = (p_i − c)/r_i
  ```
  최적화는 Step 2a와 동일한 **LM 규정** (수락 규칙·W 갱신·가드·fallback
  동일). 변수 소거 팁: c 고정 시 최적 `R_k = mean_{g(i)=k}(r_i)` (등가중 시).
- **증분 근사 F (loop별)**: 두 loop는 M₀·M₁a 어디서도 파라미터를 공유하지
  않으므로 loop별 분해가 유효하고 위반 loop을 국소화한다. loop별 p₀=6,
  p₁=4, df (2, N_loop−6):
  ```
  F_conc = ((RSS₁ₐ − RSS₀)/2) / (RSS₀/(N_loop − 6))        (loop별)
  ```
  **양쪽 loop 모두** 비기각이어야 M₁a 채택. 실패 시 체인 종료 → m0.

**3b. 2단계 — M₁a → M₁b (좌우 반지름 공유) — M₁a 채택 시에만 진입**

- **사전 게이트 (M₁a 기반)**: H_sym을 **M₁a의 반지름·공분산**으로 검정 —
  `|R_L,k − R_R,k| / √(σ²_L,k + σ²_R,k) ≤ 3.0` (k ∈ {in, out} 각각,
  **둘 다** 통과). 부분 호에서 중심과 반지름은 강하게 결합되어 M₀의 독립
  반지름 추정은 분산이 부풀고(특히 외측 13개 호) 검정력이 낮다 — 동심
  제약으로 조건이 좋아진 M₁a 추정이 일관된 기준이다.
- **M₁b fit** — shared-radii bilateral joint fit (θ ∈ R⁶):
  ```
  θ = (c_L, c_R, R_in, R_out),   e_i = ‖p_i − c_{l(i)}‖ − R_{g(i)}
  ```
  명칭 주의: 이 모델이 강제하는 것은 **좌우 반지름 공유뿐**, 중심 간 거리·
  배치는 강제하지 않는다.
- **증분 근사 F**:
  ```
  F_share = ((RSS₁ᵦ − RSS₁ₐ)/2) / (RSS₁ₐ/(N − 8))          (Δp = 8−6 = 2)
  ```
  분모를 RSS₀/(N−12)로 두는 변형도 통계적으로 가능하나 어차피 calibration
  대상이므로 M₁a-MSE 형으로 고정한다. 실패 시 m1_concentric에서 정지.

**해석 (검정의 지위).** 위 게이트·F 값은 모두 **적합도 점수**다. 임계값
9.21/3.0/f_crit은 명목 분포(χ²₂(0.99)/3σ/F 분위수)에서 출발하지만,
(i) landmark 간 cross-covariance 소실(두 원이 공통 SLAM pose 불확실성으로
상관될 수 있음), (ii) RANSAC-선택된 표본, (iii) 부분 호의 비대칭 추정 분포,
(iv) small-noise 선형화, (v) errors-in-variables, (vi) 사전 게이트와 동일
데이터의 이중 선택 때문에 정확한 확률 검정이 아니며 **p-value로 해석하지
않는다**. `has_point_covariance` 유효 시 RSS는 σ-정규화 잔차 제곱합. F 값
전체는 진단으로 출력에 기록하며, 임계값(f_crit)은 데이터에 맞춰 조정한다.

**3c. strict 모드 — parametric bootstrap.** 화살표별로 수행한다
(1단계: M₀ vs M₁a, 2단계: M₁a vs M₁b):

```
1. 검정 대상 제약 모델(1단계 M₁a / 2단계 M₁b)의 파라미터를 참으로 가정
2. 각 관측점을 자기 membership 원 위로 radial projection → latent point
   (관측 각도 분포·membership 고정)
3. latent에 2D noise ~ N(0, Σᵢ) 추가 (covariance 유효 시; 아니면 생략)
4. σ_model은 radial 1D noise N(0, σ²_model)로 추가 — 모델 불일치를 반경
   방향 오차로 흡수하는 항이므로 (2D isotropic 대안도 가능하나 한 방식으로
   고정해야 calibration이 일관된다)
5. 두 모델(1단계 M₀·M₁a / 2단계 M₁a·M₁b)을 동일 membership으로 refit
   → ΔRSS = RSS_제약 − RSS_상위 기록
6. B(2000)회 반복 → 관측 ΔRSS가 경험 분포 상위 bootstrap_alpha(0.01) 분위
   초과 시 해당 제약 기각
```

B = 2000의 근거: α=0.01에서 B=500이면 tail 기대 표본이 약 5개뿐이라 임계
분위수·p-value가 불안정하다. 비용: N≈60·LM refit 마이크로초 단위 × 2모델
× 2000 ≈ 수십 ms (오프라인 CLI에서 무시 가능). 대안: leave-one-out 예측
잔차 비교.

**채택 규칙 (체인 semantics).** 체인을 따라 전진하며 각 화살표의
(사전 게이트 성립 ∧ 증분 F 비기각 ∧ strict 시 bootstrap 비기각)을 검사하고
**처음 실패한 화살표에서 멈춘다**:
`adopted_model ∈ {m0, m1_concentric, m1_shared_radii}`. loop별
`concentric_ok[2]`와 개별 통계량은 진단 기록 전용이다. 검정 통계량 전체를
출력에 기록하며, 어떤 경우에도 조용한 강제는 없다. (재할당이 membership을
바꾼 뒤 이 채택이 여전히 유효한지는 §5 Step 5의 수렴 후 재검증이 1회
재확인한다.)

**3d. frame 재도출.** M₁ 채택 시 중심 추정이 갱신되므로 Step 2c의 frame을
**채택 모델의 중심으로 재계산**한다 (m1이면 loop당 중심이 유일해 2c의 가중
평균이 불필요해진다). heading 교차검증도 갱신된 frame으로 재확인.

**구현**

```cpp
struct ConstraintTestResult {
  bool concentric_ok[2];        // 진단 전용 (채택은 체인 전진 여부로 결정)
  bool symmetry_ok_inner, symmetry_ok_outer;   // M₁a 반지름 기반
  double stats[4];              // D²_L, D²_R, z_sym_in, z_sym_out (sym은 M₁a 기준)
};
ConstraintTestResult testConcentricGate(const std::array<CircleFit,4> & m0,
                                        const LoopPair & pairs);       // 3a 게이트
SymmetryGate testSymmetryGate(const AdoptedModel & m1a);               // 3b 게이트

struct AdoptedModel {
  ModelKind kind;                       // kM0 | kM1Concentric | kM1SharedRadii
  std::array<CircleFit, 4> circles;     // 채택 모델의 원 4개 (공유 파라미터 전개)
  Eigen::MatrixXd cov;                  // 채택 모델 파라미터 working covariance
  double f_stats[3];                    // F_conc_L, F_conc_R, F_share (진단 기록)
};
AdoptedModel selectModel(...);          // 체인 전진: 3a → 3b (→ strict 3c) → 3d
```

(참고: Cov(θ̂)의 absolute/relative 분기는 Step 2a와 동일. cs/0303015의 CRB
결과는 small-noise 점근 결과이므로 절대 신뢰 구간이 아니라 상대 비교용이다.)

### Step 4. residual 척도 — 역할별 3종 분리

**개념과 설계 근거.** 원에 대한 radial residual
`e_ih = ‖p_i − c_h‖ − R_h`는 근사가 아니라 **정확한 부호 있는 유클리드
최단거리**다. 1차 근사는 분산 투영(delta method) 쪽이다:

```
n_i = (p_i − c)/‖p_i − c‖          (‖p_i − c‖ < 0.5 m 이면 예외 처리)
σ²_pt,i = n_iᵀ Σ_i n_i             (delta-method; 보조 문헌은 §10)
```

in-sample studentized와 out-of-sample predictive를 **모델 간 비교에
혼용하면 안 된다** — 두 정의는 분모 구조가 달라(전자는 (1−h_ii)로 축소,
후자는 파라미터 공분산으로 확대) 같은 cone에 대해 모델별 z가 서로 다른
척도를 갖고, min-z 선택과 Δχ²가 불공정해진다. 역할별로 3종을 분리한다.

**(a) 비교/재할당용 균일 z (기본 모드, Step 5의 유일한 비교 척도)** —
모든 (점, 모델) 쌍에 동일 정의:

```
z_ih = |e_ih| / √(σ²_pt,i + σ²_model)
```

파라미터 불확실성과 leverage를 의도적으로 제외한다 — 척도 공정성이 우선이며,
제외분은 σ_model과 calibration된 z_max가 흡수한다.

**(b) 진단용 in-sample studentized (최종 membership의 outlier 플래깅 전용,
모델 간 비교에 사용 금지)**:

```
h_ii = J_i (JᵀWJ)⁻¹ J_iᵀ · w_i        (leverage, 채택 모델의 최종 Jacobian,
                                       W는 (a)와 동일한 분산 모델로 구성)
z̃_i = |e_i| / √( (σ²_pt,i + σ²_model) · (1 − min(h_ii, h_max)) )
```

h_max = 0.99 클램프 (h_ii → 1 발산 가드). W의 분산 모델과 분모의 분산 모델이
동일해야 studentization이 성립한다.

**(c) strict 모드: LOO predictive (선택 편향까지 보정하는 균일 척도)** —
점 i가 모델 h의 fitting에 사용됐다면 i를 제외하고 h를 LM refit한 뒤:

```
z^loo_ih = |e_ih⁽⁻ⁱ⁾| / √( σ²_pt,i + g_i Cov(θ̂⁽⁻ⁱ⁾) g_iᵀ + σ²_model ),
g_i = [−n_iᵀ, −1]
```

미사용 점은 전체 fit으로 동일 식 적용 — 전 모델 공통 척도가 확보된다.
N ≈ 60, 원 4개, LM 1회가 마이크로초 단위이므로 오프라인 CLI에서 실행 가능.

**구현**

```cpp
Eigen::VectorXd radialResiduals(
  const std::vector<Eigen::Vector2d> & pts, const Eigen::Vector2d & c, double R);
Eigen::VectorXd uniformZ(const std::vector<Cone> & cones,
                         const AdoptedModel & model, double sigma_model);   // (a)
Eigen::VectorXd studentizedInSample(const FitContext & fit);                // (b)
Eigen::VectorXd looPredictiveZ(const std::vector<Cone> & cones,
                               const AdoptedModel & model,
                               const LmParams & lm, double sigma_model);    // (c)
```

### Step 5. 전역 재할당과 분류

**개념과 설계 근거.** 모든 yellow/blue cone을 채택 모델의 원 4개(+ 직선,
활성 시)에 대해 **균일 z (Step 4(a); strict 모드에서는 (c))**로 재채점한다.
색 플립된 cone은 자기 색 검출에서 빠졌더라도 여기서 올바른 원으로 회수된다
— **색상은 Step 1의 가설 생성 사전 분할에만 쓰이고(가설 공간의 색상
독립성은 무색 풀 C가 보장) 최종 membership은 기하만으로 정해진다.**

**게이트** — 상대(z)와 절대(e) 게이트를 모두 판정식에 포함한다:

```
valid_ih = (z_ih ≤ z_max) ∧ (|e_ih| ≤ e_max)
```

z_max(3.0)는 실용적 3σ 게이트(§2 주의 참조), e_max(0.8 m)는 covariance 오염
안전장치.

**Circular arc interval** — 원 모델 h의 각도 membership 게이트는 in-sample
inlier들의 관측 호로 정의하되, naive [θ_min, θ_max]는 호가 ±π 경계를 넘으면
역전된다 (예: 150°→−150°를 지나는 60° 호에서 min/max는 [−150°,150°]이 되어
실제 호 위의 175° 점을 배제하고 비관측 반대편을 통과시킨다). 올바른 절차 —
Step 6의 coverage 계산과 **동일 함수**를 공유한다:

```
1. inlier 각도 정렬 → 인접 각도 갭(circular, 마지막→첫 wrap 포함) 계산
2. 최대 갭의 보완 구간 = 관측 호 [arc_start, arc_end] (wraps 플래그 포함)
3. 게이트 구간 = 호 ± arc_margin(15°), circular 포함 판정
4. coverage = 2π − 최대 갭 ≥ arc_full_disable(330°) 이면 각도 게이트 비활성
   (full circle에 가까우면 각도 제한이 무의미)
```

**arc interval의 반복 갱신**: arc_start/arc_end/wraps/coverage와 게이트 활성
여부는 아래 재할당↔refit **매 반복에서 현재 membership으로 재계산**한다 —
초기 inlier 호로 고정하면 margin(15°) 밖의 정상 cone이 영구히 ambiguous로
남는다 (갱신해도 반복당 최대 15°씩만 회복되므로 상한 3회에 45°). 옵션
`arc_gate_first_pass_relaxed`(기본 false, calibration 대상): 1차 재할당에서
각도 게이트를 비활성해 큰 dropout에서 회수율을 높인다 — 단 비관측 호 위의
구조적 outlier 유입과 trade-off라 기본은 끈다.

**직선 모델** (`has_orange_candidates`이고 frame 확보 후):

- **side 분할 (부호가 아니라 clustering)**: orange의 x'를 1D
  2-cluster(2-means 또는 정렬 후 최대 갭 분할)로 나누고 **side label은
  사후 부여**한다 — "두 직선이 x' = 0을 사이에 둔다"는 좌표 가정을 다시
  들이지 않기 위해서다 (frame 원점 흔들림·코리도어 offset·중앙선을 넘는
  orange outlier에 강건). 분할 후 **side별 조건**: 개수 ≥
  `corridor_min_per_side`(3) ∧ 종방향 spread ≥ `corridor_min_spread`(3 m).
  (orange ≥ 6이어도 분포가 나쁘면 여기서 실패해야 한다 — 총수 조건만으로는
  2+2 정확 통과·spread 부족·편측 쏠림을 못 거른다.)
- **side별 robust 직선 (n-계층화)** — "3점"은 직선 계산 가능 조건일 뿐
  강건 fitting 가능 조건이 아니다:
  - **3–4점**: TLS(PCA 최소 고유벡터)만, `corridor_confidence = low`
    (n < 5에서 MAD trim은 무의미 — 3점 중 1점을 trim하면 잔차 0의 임의
    직선을 정의하는 2점이 남는다)
  - **5점 이상**: TLS + MAD 기반 outlier 1회 trim
  - **6점 이상**: RANSAC-line (orange에도 ghost/오분류가 있을 수 있어
    순수 TLS는 비강건)
  실증 근거 (§3): skidpad.csv의 orange 20개 중 2개는 exit funnel (±0.55, 21)로
  코리도어 직선에서 1.09 m 이탈한다 — robust fit은 오염 없는 시뮬
  원본에서도 실제로 필요하다.
- 검사: 두 직선 평행도 ≤ 5°, 간격 ∈ [0.5, 6] m, frame y'축 정렬 ≤ 10°
  → 통과 시 `corridor_fit_valid = true`. 실패 시 직선 모델 비활성 + status.
- cone의 직선 잔차는 점-직선 거리, z 정규화는 (a)와 동일 형식.

**판정 규칙**:

```
V_i = { h : valid_ih }                       (원 4 + 직선, 활성 시)
1) V_i = ∅                                   → outlier
2) best = argmin_{h∈V_i} z_ih ;  |V_i| ≥ 2 이면 Δχ² = z²_second − z²_best
   Δχ² < ambiguity_delta(2.0)                → ambiguous
3) best가 원이고 점의 각도가 circular arc 게이트 밖  → ambiguous
4) best가 원                                  → circle_inlier
5) best가 직선                                → straight
```

**재할당↔refit 반복**: 재할당으로 membership이 바뀌면 **채택 모델의 제약
구조를 반복 중에는 고정한 채** 파라미터만 LM refit → z·arc interval 재계산
→ 재할당. 최대 `reassign_max_iter`(3)회 또는 membership 고정점 도달 시 종료.
반복 중에는 Step 3을 재실행하지 않는다 (검정↔재할당 진동 방지).

**수렴 후 제약 재검증 + 단방향 fallback**: 재할당은 검정의
전제(membership)를 바꾼다 — 갱신된 데이터가 더 이상 제약을 지지하지 않는데
M₁을 유지하면 그 왜곡이 최종 출력(= global path 입력)에 그대로 남는다.
수렴 후 **1회**, 최종 membership으로 Step 3의 게이트·증분 F 통계량을
재계산한다:

```
통과                        → 결과 확정
m1_shared_radii에서 실패    → m1_concentric로 fallback
m1_concentric에서 실패      → m0로 fallback (§6 not_usable_for_path)
```

fallback은 **제약이 약해지는 방향으로만** 허용한다 (승격 없음 — 최대 2회로
종료 보장, 검정↔재할당 진동 불가). fallback 시 해당 모델로 재할당↔refit을
1회 재수렴하고, `constraint_fallback` status와 재검증 통계량을 기록한다.

**셋 단위 직선-vs-원 판정 (기본 = LOO, G-AIC는 strict 대안)**: 원에
안 맞은 yellow/blue가 `gaic_min_points`(6)개 이상이고 spread(최대 쌍거리)
≥ 3 m이면, 그 점집합에 대해 원 fit과 TLS 직선 fit을 수행하고:

- **기본**: leave-one-out 예측 잔차 제곱합 비교 (작은 쪽 채택).
- **strict**: raw-distance Geometric AIC (Kanatani, IJCV 26:171–189, 1998):
  ```
  G-AIC(model) = Σe_i² + 2(N·d + p)·ε̂²      d=1, p_line=2, p_circle=3
  ε̂ = 1.4826 · MAD(그 점집합에 대한 원 fit 잔차)    (원 = 더 일반 모델)
  ```
  **두 항 모두 raw 거리 스케일 [m²]** — Ĵ만 σ-정규화(무차원)하면 페널티와
  단위가 어긋난다. 이 비교에서는 점별 covariance를 사용하지 않는다
  (whitened 공식을 쓰려면 페널티도 whitened 스케일로 재유도해야 하며, 이는
  구현 이득이 없다). ε̂도 **같은 후보 점집합**에서 추정한다 (다른 점집합에서
  얻은 채택 모델의 MAD 재사용 금지).

n < 6이면 비교하지 않는다 (비공선 3점은 잔차 0인 완전한 원을 정의).

**구현**

```cpp
struct Classification {
  Assignment assignment;           // kCircleInlier, kStraight, kAmbiguous, kOutlier
  AssignmentReason reason;
  int best_model;                  // 원 index 0..3 또는 kLineModel
  double z_best, delta_chi2;
};
Classification classifyCone(
  const Cone & cone, const AdoptedModel & model,
  const std::optional<Corridor> & corridor, const ClassifyParams & params);

struct ArcInterval { double arc_start, arc_end; bool wraps; double coverage; };
ArcInterval observedArc(const std::vector<double> & inlier_angles);  // Step 6와 공용
double gaicRaw(double rss, int n_points, int n_params, double eps_hat);
```

### Step 6. 검증과 색 진단 (최종 membership 기준)

**개념과 설계 근거.** 여기서는 **채택 모델이 강제하지 않은 항목만**
검사한다. 실패는 예외가 아니라 `validation_status`로 산출물에 남긴다.

| 검사 | 조건 (기본값) | 비고 |
|---|---|---|
| 반지름 순서 | 각 loop에서 R_in < R_out | 채택 모델 기준 |
| **중심 거리 sanity — layout prior** | `standard_layout_prior = true`일 때만: \|d(C_L,C_R) − (R_in+R_out)\| ≤ 0.6 m | 이 관계는 위상("동심원 2쌍 + 코리도어")에서 나오지 않는다. **표준 skidpad 배치**(한쪽 outer가 반대쪽 inner에 근접 접함)의 속성이다 — FSG 공개 수치로 d−(R_in+R_out)=0 (정확 접함), 시뮬 +0.27. 일반 4원 검출기로 쓸 때는 flag off |
| inlier 수 | 원당 ≥ 8 | 최종 membership 기준 (실측 13~17) |
| arc coverage | inner ≥ 240°, outer ≥ 120° | `coverage = 2π − max circular gap`, Step 5와 공용 함수; 부분 호 병리는 N. Chernov, arXiv:0907.0429 |
| 코리도어 기하 | 평행도·간격·정렬 (Step 5 기준) | corridor_fit_valid |
| **색 진단** | 아래 상세 | 진단 전용, 기하 게이트 아님 |
| adopted_model·검정 통계량 기록 | m0 / m1_concentric / m1_shared_radii + 3a/3b/3c 통계 | Step 3 결과 |

**색 진단** — majority purity는 (i) 10% flip에서 기대값 0.9라 0.8 경고와
모순이고, (ii) 원 전체가 반대 색이어도 1이 된다. 대체 진단:

1. **color_pattern_ok**: skidpad의 색 위상 — "inner_L·outer_R이 같은 색,
   inner_R·outer_L이 같은 색, 두 그룹은 서로 다른 색" — 을 원별 다수결 색으로
   검사한다. 특정 색 배정을 하드코딩하지 않으므로 규격 불가지론과 양립하고,
   L/R 미확정이어도 검사 가능(구조가 반전 대칭).
2. **expected_color(circle)**: pattern이 성립하면 다수결로 원별 기대 색 확정.
3. **color_mismatch_count / expected_color_fraction**: 최종 membership에서
   기대 색과 다른 콘 수 / 일치 비율.
4. **경고 규칙**: `color_mismatch_count > 0` → `color_warning`;
   `color_pattern_ok = false` → `color_pattern_broken` (심각 —
   majority_purity < 0.8 상황을 포함해 흡수).
5. majority_purity는 참고 진단값으로 계속 출력한다.
6. **absolute_color_rule_check (기본 off)**: color_pattern_ok는 상대 패턴만
   보므로 **전역 blue↔yellow 스왑을 구조상 통과시킨다** — 규격
   불가지론에서는 의도된 동작이다. 대회 파이프라인(KSAE/FSG 모드)에서 이
   flag를 켜면 L/R 해소 시(`lr_resolved = true`)에 한해 공식 색 convention과의
   절대 일치를 추가 검사해 `absolute_color_ok`를 기록한다.
   `standard_layout_prior`와 동일한 원칙: 규격 지식은 기본 off인 flag로만
   들어오며, 이 검사도 진단 전용이고 기하 게이트가 아니다.

## 6. 출력 설계 — 모델 × route 3-상태 출력 정책

**centerline이 원이 되는 조건.** 동심성이 기각된 loop에서, **두 경계 원
사이의 띠 영역(내측 원의 외부 ∧ 외측 원의 내부 — centerline이 놓이는
영역)**에 한해 inner/outer의 등거리 궤적은
‖p−c_in‖+‖p−c_out‖ = R_in+R_out인 **타원 branch**(초점 c_in, c_out)다
(원까지의 unsigned distance는 |‖p−c‖−R|이므로 전 평면에서는 부호 조합에
따라 다른 branch가 생긴다). 어느 경우든 원이 아니므로 반지름 평균만으로
centerline circle을 정의할 수 없다.

**geometry 유효성과 route 해소는 별개다.** 동심성이 채택돼도 L/R·주행
방향이 미해소면(lr_resolved = false) 어느 loop를 먼저 어느 방향으로 돌지,
진입 호를 어디에 잇는지 정할 수 없다. 출력 상태를 3개로 분리한다:

- `centerline_geometry_valid` = adopted_model ∈ {m1_concentric,
  m1_shared_radii} (수렴 후 재검증·fallback이 반영된 **최종** 모델 기준)
- `mission_route_resolved` = lr_resolved (heading 또는 car_start 위치 기반,
  §5 Step 2c)
- `usable_for_global_path` = centerline_geometry_valid ∧
  mission_route_resolved

| adopted_model | 원 4개 CSV | centerline_radius / track_width | frame JSON | centerline_geometry_valid | usable_for_global_path |
|---|---|---|---|---|---|
| m1_shared_radii | ✓ | ✓ (좌우 공통 값) | ✓ (재도출됨) | true | lr_resolved일 때만 **true** |
| m1_concentric | ✓ | ✓ (loop별 값) | ✓ (재도출됨) | true | lr_resolved일 때만 **true** |
| m0 | ✓ **(진단 전용 표기)** | **미출력 (NaN)** + `not_usable_for_path` | ✓ (잠정, 진단 전용) | false | **false** |

m0 채택은 "실패"가 아니라 "이 cone map은 skidpad 위상 가설을 통계적으로
지지하지 않는다"는 결과다. m1 채택 ∧ L/R 미해소는 "geometry는 유효하나
route 미정" 상태다 — centerline 기하 출력은 유지하되 global path 입력으로는
금지한다. downstream은 개별 필드가 아니라 **`usable_for_global_path`
하나를 게이트**로 사용하고, 원인 분석에만 centerline_geometry_valid /
mission_route_resolved / adopted_model을 참조해야 한다.

**출력 파일 명세**

- `detected_skidpad_circles.csv` — circle_id, side, role, center_x/y, radius,
  inlier_count, rmse, arc_coverage_deg, arc_start/arc_end/wraps,
  majority_purity, expected_color, expected_color_fraction,
  color_mismatch_count, fit_method, adopted_model, constraint_test_stats,
  f_stats, validation_status, track_width†, centerline_radius†, notes.
  († = 출력 정책 적용 필드)
- `skidpad_circle_cone_members.csv` — source_row_index, tag, x, y, covariance
  3컬럼, x_frame, y_frame, circle_id/role, radial_residual, z_best
  (균일 z), z_studentized (in-sample 진단, 해당 시), best_model, delta_chi2,
  assignment, assignment_reason, possible_duplicate.
- `skidpad_frame.json` — origin, ex, ey, corridor(활성 시: 두 직선 파라미터·
  간격, corridor_confidence), car_start_in_frame, lr_resolved,
  lr_resolved_by (heading | position | none), heading_semantics_verified,
  capability flags 전체, adopted_model, centerline_geometry_valid /
  mission_route_resolved / usable_for_global_path,
  constraint_fallback(발생 시), standard_layout_prior, validation_status.

## 7. 파라미터 표 (기본값과 근거 — 전부 데이터에 맞춰 조정할 기본값)

| 파라미터 | 기본값 | 근거 |
|---|---|---|
| dedup_enabled / dup_flag_radius | false / 0.75 m | §5 Step 0 (마킹만) |
| msac_T | 0.5 m | SLAM 오차 스케일 추정 |
| r_min / r_max | 2 / 25 m | FS 스케일 degeneracy guard |
| ransac_iters / seed / top_k | 500 / 고정값 / 10 | 색상별 풀: N(0.99, w=0.3)≈168의 3배 |
| ransac_iters_agnostic | 1000 | 무색 풀 C: w≈13/60이면 N(0.99, w=0.2)≈574 — 여유 포함 |
| dup_hyp_center / dup_hyp_radius | 1.0 / 1.0 m | §5 Step 1 중복 가설 배제 |
| min_inlier_count | 8 | 실측 최소 13의 60% |
| arc_min_hypothesis | 90° | §5 Step 1 조합 validity 공통 coverage 하한 (role별 기준은 Step 6) |
| pairing_margin_delta | 6.0 | §5 Step 2b (χ²₂ 95% ≈ 5.99 출발점, **차** 기준) |
| chi2_pairing_gate | 5.99 | §5 Step 2b 분할 유효성 |
| z_radius_sep | 3.0 | §5 Step 2b loop 내 inner/outer 분리 |
| heading_align_max / carstart_x_max | 30° / 3 m | §5 Step 2c heading 교차검증 |
| carstart_y_min | 5 m | §5 Step 2c 위치 기반 L/R 해소 — 교차부 이격 조건 |
| chi2_concentric / z_symmetry | 9.21 / 3.0 | §5 Step 3a/3b 게이트 (명목 χ²₂(0.99)/3σ 출발점; z_symmetry는 M₁a 기반) |
| f_crit 기준 α / bootstrap_B / bootstrap_alpha | 0.01 / 2000 / 0.01 | §5 Step 3 (증분 F + strict bootstrap) |
| lm_lambda0 / lm_max_iter | 1e−3 / 50 | §5 Step 2a LM 규정 |
| z_max / e_max | 3.0 / 0.8 m | §5 Step 5 (둘 다 판정식에 포함) |
| sigma_model | 0.15 m | 설치 오차 + 트랙 왜곡 |
| h_max | 0.99 | §5 Step 4(b) leverage 클램프 |
| ambiguity_delta (Δχ²) | 2.0 | §5 Step 5 규칙 2 |
| arc_margin / arc_full_disable | 15° / 330° | §5 Step 5 circular interval (매 반복 재계산) |
| arc_gate_first_pass_relaxed | false | §5 Step 5 (옵션 — 1차 재할당 각도 게이트 완화) |
| reassign_max_iter | 3 | §5 Step 5 반복 상한 (+ 수렴 후 재검증 1회) |
| gaic_min_points / gaic_min_spread | 6 / 3 m | §5 Step 5 (3점 완전원 방지) |
| corridor_min_per_side / corridor_min_spread | 3 / 3 m | §5 Step 5 (side별 조건) |
| corridor 평행도/간격/정렬 | 5° / [0.5, 6] m / 10° | §5 Step 5 |
| arc_min_inner / outer | 240° / 120° | 실측 360°/≈280°에 dropout 여유 |
| standard_layout_prior / ε_center_dist | true / 0.6 m | §5 Step 6 (skidpad 전용 모드에서만) |
| absolute_color_rule_check | false | §5 Step 6 (대회 모드 전용 — 규격 prior는 flag로만) |
| strict_validation | false | LOO 척도·bootstrap·G-AIC는 strict 한정 |

## 8. 코드 구조 제안 (C++17, ament_cmake 패키지)

colcon 워크스페이스에서 빌드되는 독립 패키지. **core 라이브러리는 rclcpp
비의존** (오프라인 CLI, ROS node wrapper는 다음 단계 §9).

```
planning/skidpad_extract/
  package.xml                       # ament_cmake, depend: eigen
  CMakeLists.txt                    # 라이브러리 + CLI
  include/skidpad_extract/
    types.hpp          # Cone, CircleHypothesis, CircleFit, LineFit,
                       # AdoptedModel, Frame, CapabilityFlags, Params,
                       # ValidationStatus, Assignment, ArcInterval
    io_eufs.hpp        # CSV 로드/저장, tag 필터, capability + covariance
                       # 행별 유효성 검사, 중복 마킹
    ransac.hpp         # detectTwoCirclesPerColor, detectAgnosticPool(무색 풀),
                       # 풀 가설 inlier refit, selectFourCircles
                       # (E(S) + support·coverage·pairing validity)
    fits.hpp           # circleFrom3Points, hyperFit(centering+SVD),
                       # lmRefine(공용 LM), concentricFit, sharedRadiiFit,
                       # tlsLineFit(+MAD trim / RANSAC-line)
    model_select.hpp   # 체인 게이트(동심성 M₀ / 대칭성 M₁a 기반), 증분 F,
                       # parametricBootstrap(화살표별), selectModel(체인 전진),
                       # 수렴 후 재검증·단방향 fallback, frame 재도출 호출
    residuals.hpp      # radialResiduals, uniformZ, studentizedInSample,
                       # looPredictiveZ
    frame.hpp          # pairLoops(full Mahalanobis), deriveFrame
                       # (heading 교차검증 + 위치 기반 L/R 해소)
    classify.hpp       # valid 게이트(z ∧ abs), observedArc(공용, 반복 갱신),
                       # 판정 규칙(Δχ²), 재할당 반복, gaicRaw / LOO 비교
    validate.hpp       # geometry validation(layout prior flag),
                       # colorDiagnostics (pattern/expected/mismatch)
  src/
    *.cpp              # 헤더 1:1 대응
    cli_main.cpp       # 입력 CSV → 출력 3종 (skidpad_extract_cli)
```

구현 규약:

- 선형대수는 **Eigen3만** (ROS 2 Humble 기본 제공). `inverse()` 금지 —
  LDLT/SVD. 고유문제는 centering 후 4×4.
- LM은 `fits.hpp`의 공용 유틸 하나로 — Step 2a/3b가 동일 규정 공유.
- 난수는 `std::mt19937` + 고정 seed 파라미터. `std::random_device` 금지.
- CSV 파서는 `hyu_global_planner_debug_visualizer_node.cpp` 스타일을 따르고 공용화
  가능하면 재사용.
- `skidpad_frame.json`은 수동 직렬화 (신규 의존성 금지).
- 예외는 입력 계약 위반(파일 없음, 컬럼 불일치)에만. 알고리즘 실패는 전부
  `ValidationStatus`로 표현한다.

## 9. 제외 범위

- global path 생성, 속도 프로파일, stop zone 경로 — 다음 단계
  (단, **usable_for_global_path = false인 출력은 다음 단계의 입력이 될 수
  없다** — §6).
- ROS node화, RViz publisher — 다음 단계.
- strict 모드 구성 요소(LOO 척도, parametric bootstrap, G-AIC)는 초기 릴리스에서
  기본 off — 기본 경로가 검증된 뒤 활성화를 검토.
- 좌표계 자동 정렬은 범위에 포함 (Step 2c·3d).

## 10. 참고 문헌 (단계 매핑)

| 단계 | 문헌 |
|---|---|
| Step 1 | Fischler & Bolles, RANSAC, CACM 1981 · Torr & Zisserman, MLESAC/MSAC, CVIU 2000 · Chum, Matas & Kittler, LO-RANSAC, DAGM 2003 (풀 가설 사전 refit의 근거) |
| Step 1 가드 | N. Chernov, "Fitting circles to scattered data: parameter estimates have no moments", arXiv:0907.0429 (일반적 heavy-tail 결과) |
| Step 2a | Chernov & Lesort, arXiv:cs/0301001 (Kåsa/Pratt/Taubin/geometric fit 수렴) · Al-Sharadqah & Chernov, arXiv:0907.0421 (Hyper, 부분 호 bias, centering·SVD 절차) · Abdul-Rahman & Chernov, arXiv:1505.03795 (대반경 수치 안정 원 fit) |
| Step 2b·3 공분산 | Chernov & Lesort, arXiv:cs/0303015 (CRB — small-noise 점근, 상대 비교용) |
| Step 3 | nested 모델 비교(표준 선형모형론의 F — 본 파이프라인에서는 체인 증분 비교의 근사 기준) · Efron 1979 (bootstrap **기초 문헌** — 본 문서의 구체 구현 절차를 직접 보증하는 인용이 아님) |
| Step 4 | σ²_r = nᵀΣn은 signed-distance의 1차 선형화에 따른 **일반 delta-method** — 특정 문헌이 보증하는 공식이 아니다 (보조: Sampson, CGIP 1982 · Rydell et al., arXiv:2401.07114) · (가중 확장) Leedan & Meer, IJCV 2000 |
| Step 5 | Bar-Shalom & Fortmann, Tracking and Data Association, 1988 (gating) · Kanatani, Geometric AIC, IJCV 1998 · Huber 1964 / Hampel et al. 1986 (robust scale, MAD) |
| 대안 구조 | Isack & Boykov, PEARL, IJCV 2012 (규모 확대 시 multi-model 상위 대안) |
| 구현 사례 | papalotis/ft-fsd-path-planning (skidpad relocalization, 공개 코드) |
