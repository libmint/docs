---
layout: default
title: Newton cloth franka 예제
parent: 나의 공부 게시판
---

코드 원본을 직접 가져오지 못했지만, NVIDIA 공식 블로그와 GitHub 정보를 통해 이 예제의 전체 구조와 흐름을 충분히 파악했습니다. 초보자 눈높이에서 설명해 드릴게요!

---

## 🤖 `example_cloth_franka.py` — 쉽게 이해하기

### 📌 이 코드가 하는 일 한 줄 요약

> **"Franka 로봇 팔이 천(cloth)을 집고 움직이는 장면을 물리 시뮬레이션으로 구현한다"**

---

### 🏗️ 큰 그림 — 무엇을 시뮬레이션하나요?

이 예제는 두 가지 물체를 동시에 다룹니다.

| 물체 | 종류 | 사용하는 물리 방식 |
|---|---|---|
| Franka 로봇 팔 | **딱딱한 물체** (Rigid Body) | Featherstone Solver |
| 천 (Cloth) | **유연한 물체** (Deformable) | VBD (Vertex Block Descent) Solver |

이처럼 **서로 다른 물리 법칙이 필요한 두 물체를 동시에 시뮬레이션**하는 것이 이 코드의 핵심입니다. 이걸 "멀티피직스(multiphysics)"라고 부릅니다.

---

### 🔧 코드 구조 — 어떻게 동작하나요?

코드는 크게 아래 5단계로 흘러갑니다.

**1단계 — 준비 (Import & 설정)**
```
newton, warp 같은 라이브러리를 불러오고,
시뮬레이션에 쓸 숫자들(프레임 수, 시간 간격 등)을 정의
```
> 레고 조립 전에 부품을 꺼내 놓는 단계

**2단계 — 세계 만들기 (Model 구성)**
```
- 바닥(Ground plane) 추가
- Franka 로봇 URDF/USD 파일을 불러와서 장면에 배치
- 천(Cloth)을 격자 메쉬로 생성하고 재질(탄성, 마찰 등) 설정
```
> 시뮬레이션 무대와 배우를 세팅하는 단계

**3단계 — 두 개의 솔버 연결**
```
- rigid_solver: 로봇 팔의 관절과 움직임 계산
- vbd_solver: 천의 구겨짐, 늘어짐, 중력 반응 계산
- 두 솔버가 충돌/접촉 정보를 서로 공유 (one-way coupling)
```
> 로봇은 천에 힘을 줄 수 있지만, 천이 로봇에게 힘을 돌려주지는 않는 방식

**4단계 — 시뮬레이션 루프 (매 프레임 반복)**
```python
for frame in range(num_frames):
    # 로봇 관절에 목표 각도 명령 전달
    # rigid_solver로 로봇 움직임 계산
    # vbd_solver로 천 형태 변화 계산
    # 화면에 렌더링
```
> 게임처럼 매 프레임마다 물리 계산 → 그림 그리기를 반복

**5단계 — 실행 & 뷰어**
```
OpenGL 창(--viewer gl)이나 USD 파일로 결과를 출력
```

---

### 💡 핵심 개념 — 초보자가 꼭 알아야 할 것

**VBD (Vertex Block Descent) 솔버란?**
천의 각 꼭짓점(vertex)이 중력, 장력, 충돌에 의해 어디로 움직일지를 계산하는 알고리즘입니다. GPU 기반으로 동작하며 RTX 4090에서 약 30FPS를 내면서도 관통(penetration) 없이 접촉을 보장합니다. [NVIDIA Developer](https://developer.nvidia.com/blog/train-a-quadruped-locomotion-policy-and-simulate-cloth-manipulation-with-nvidia-isaac-lab-and-newton/)

**왜 솔버를 두 개 써야 하나요?**
로봇 팔은 금속처럼 형태가 변하지 않아서 관절 방정식으로 계산하면 되지만, 천은 수천 개의 꼭짓점이 각자 움직이는 "유연체"라 완전히 다른 수학이 필요합니다. 두 솔버를 각자 최적화된 방식으로 돌리고 접촉 결과만 공유하는 방식이 훨씬 빠르고 정확합니다.

---

### 🚀 실행하는 법

```bash
pip install "newton[examples]"
python -m newton.examples cloth_franka
```

---

### 🗺️ 전체 흐름 요약

```
라이브러리 로드
    ↓
바닥 + 로봇 + 천 배치 (Model 빌드)
    ↓
Featherstone 솔버 (로봇용) + VBD 솔버 (천용) 생성
    ↓
루프: 로봇 명령 → 물리 계산 → 화면 출력
    ↓
종료
```

궁금한 부분이 있으면 더 자세히 설명해 드릴게요! 예를 들어 "VBD 솔버가 정확히 어떻게 계산하나요?" 또는 "관절 제어는 어떻게 코드로 작성하나요?" 같은 질문도 환영합니다.



완벽합니다! 전체 코드를 보니 훨씬 정확하게 설명할 수 있겠네요. 처음부터 차근차근 설명해 드릴게요.

---

## 🧩 이 코드 전체 구조 한눈에 보기

```
코드 = 설정값들 + 5개의 주요 함수들 + 메인 실행부
```

크게 **두 주인공**이 있습니다:
- 🦾 **Franka 로봇 팔** — 딱딱한 금속 팔
- 👕 **T셔츠 (천)** — 물렁물렁한 천

---

## 📐 파일 맨 위 — GPU 커널 함수들 (`@wp.kernel`)

```python
@wp.kernel
def scale_positions(src, scale, dst):
    i = wp.tid()
    dst[i] = src[i] * scale
```

`@wp.kernel`은 **GPU에서 병렬로 실행되는 함수**입니다. `wp.tid()`는 "나는 몇 번째 데이터야?"라고 묻는 것과 같아요.

이 함수들이 하는 일:
- `scale_positions` — 천의 각 꼭짓점 위치를 cm → m로 변환 (시각화용)
- `scale_body_transforms` — 로봇의 각 관절 위치도 cm → m로 변환
- `compute_ee_delta` — 로봇 손끝이 목표 위치에서 얼마나 떨어져 있는지 계산

> 💡 **왜 cm로 시뮬레이션?** VBD(천 물리) 알고리즘이 너무 작은 숫자(m 단위)에서 수치 오류가 생기기 때문에 cm로 키워서 계산합니다.

---

## 🏗️ `__init__` — 시뮬레이션 세계 만들기

### 1) 기본 숫자 설정

```python
self.fps = 60
self.sim_substeps = 10
self.sim_dt = self.frame_dt / self.sim_substeps  # 매우 작은 시간 단위
self.viz_scale = 0.01  # cm → m 변환 비율
```

| 설정 | 의미 |
|---|---|
| `fps = 60` | 초당 60프레임으로 화면 출력 |
| `sim_substeps = 10` | 1프레임 안에서 물리 계산을 10번 반복 (더 정확해짐) |
| `viz_scale = 0.01` | 시뮬레이션은 cm, 화면은 m 단위 |

### 2) 물리 재질 설정

```python
self.tri_ke = 1e4      # 천의 늘어남에 저항하는 힘 (탄성)
self.bending_ke = 5    # 천이 구겨지는 정도
self.robot_contact_mu = 1.5  # 로봇-천 마찰력
```

이것들은 마치 게임 엔진에서 재질(material)을 설정하는 것과 같습니다.

### 3) 세계 구성요소 추가

```python
self.scene = ModelBuilder(gravity=-981.0)  # 중력 = -981 cm/s²
```

```python
# 테이블 추가
self.scene.add_shape_box(...)

# T셔츠 USD 파일 불러와서 천으로 추가
usd_stage = Usd.Stage.Open(newton.examples.get_asset("unisex_shirt.usd"))
self.scene.add_cloth_mesh(vertices=..., indices=..., pos=..., density=0.02, ...)

# 바닥 추가
self.scene.add_ground_plane()
```

> 💡 **USD**는 NVIDIA Omniverse/Pixar에서 만든 3D 파일 형식입니다. `unisex_shirt.usd`는 T셔츠 3D 모델 파일입니다.

---

## 🦾 `create_articulation` — 로봇 팔 만들기

```python
builder.add_urdf(
    "fr3_franka_hand.urdf",
    scale=100,   # URDF는 m 단위 → cm로 변환
    floating=False,  # 바닥에 고정
)
```

**URDF**는 로봇의 구조를 설명하는 파일입니다 (관절이 몇 개, 어떻게 연결됐는지 등).

### 핵심 — 로봇 동작 목표 좌표들

```python
self.robot_key_poses = np.array([
    # [시간, x, y, z, 회전(4개), 그리퍼 개폐]
    [2.5, 31.0, -60.0, 23.0, 1, 0.0, 0.0, 0.0, 0.8],  # 오픈
    [2,   31.0, -60.0, 23.0, 1, 0.0, 0.0, 0.0, 0.1],  # 닫기 (잡기)
    ...
])
```

이것은 로봇이 순서대로 따라가야 할 **웨이포인트(경유점) 목록**입니다. 마치 내비게이션 경로처럼, 로봇이 이 지점들을 순서대로 방문하며 T셔츠를 접습니다.

`0.8` = 그리퍼 열림, `0.1` = 그리퍼 닫힘 (잡기)

---

## 🧮 `generate_control_joint_qd` — 로봇 제어 핵심 로직

이게 가장 복잡한 부분입니다. 쉽게 설명하면:

```
목표 위치가 있다
    ↓
현재 손끝 위치에서 얼마나 떨어졌는지 계산 (ee_delta)
    ↓
야코비안(J) 계산 — "관절을 어떻게 움직여야 손끝이 원하는 방향으로 가나?"
    ↓
역야코비안(J_inv)으로 필요한 관절 속도 계산
    ↓
관절 명령 전달
```

**야코비안(Jacobian)**이란? 로봇의 관절 각도 변화 → 손끝 위치 변화의 관계를 나타내는 행렬입니다.

```python
J_inv = np.linalg.pinv(J)   # 역야코비안 (유사역행렬)
delta_q = J_inv @ delta_target + N @ delta_q_null
```

`N @ delta_q_null`는 **null space 제어**로, 손끝 위치는 유지하면서 팔 자세를 자연스럽게 만드는 추가 제어입니다.

---

## ⏱️ `step` + `simulate` — 매 프레임 반복되는 루프

```python
def step(self):
    self.generate_control_joint_qd(self.state_0)  # 1. 로봇 명령 계산
    self.simulate()                                 # 2. 물리 계산
    self.sim_time += self.frame_dt                  # 3. 시간 전진
```

```python
def simulate(self):
    for _step in range(self.sim_substeps):  # 10번 반복
        
        # 🦾 로봇 시뮬레이션
        self.model.particle_count = 0    # 천은 잠시 꺼두고
        self.robot_solver.step(...)      # 로봇만 계산
        self.model.particle_count = ...  # 천 다시 켜기
        
        # 👕 충돌 감지
        self.collision_pipeline.collide(self.state_0, self.contacts)
        
        # 👕 천 시뮬레이션
        self.cloth_solver.step(...)
        
        # state_0 ↔ state_1 교체 (더블 버퍼링)
        self.state_0, self.state_1 = self.state_1, self.state_0
```

> 💡 **왜 state_0과 state_1이 두 개?** 현재 상태(0)로 다음 상태(1)를 계산하고, 다음 프레임에선 둘을 바꿔씁니다. 마치 앞/뒤 버퍼처럼요. 이를 **더블 버퍼링**이라고 합니다.

---

## 🎨 `render` — 화면에 그리기

```python
def render(self):
    # cm → m 스케일 변환 (GPU에서)
    wp.launch(scale_positions, ...)
    wp.launch(scale_body_transforms, ...)
    
    # 화면에 그리기
    self.viewer.begin_frame(self.sim_time)
    self.viewer.log_state(self.viz_state)
    self.viewer.end_frame()
    
    # 다시 cm 스케일로 복원 (물리 계산용)
    self.model.shape_transform = self.sim_shape_transform
```

시뮬레이션은 cm, 화면은 m — 계속 왔다갔다 변환합니다.

---

## 🚀 전체 흐름 최종 요약

```
main 실행
    ↓
Example.__init__()
  → 로봇 + 테이블 + T셔츠 + 바닥 배치
  → 두 솔버 생성 (Featherstone, VBD)
    ↓
루프 (3850 프레임)
  ┌──────────────────────────────────┐
  │  step()                          │
  │    → 로봇 다음 목표 계산          │
  │    → simulate() × 10회           │
  │       → 로봇 관절 이동            │
  │       → 충돌 감지                 │
  │       → 천 물리 계산              │
  │    → render() — 화면 출력        │
  └──────────────────────────────────┘
```

---

궁금한 부분 있으시면 더 파고들어 드릴 수 있어요! 예를 들어 야코비안 계산, VBD 솔버 원리, 또는 더블 버퍼링에 대해 더 깊이 설명해 드릴 수 있습니다.
