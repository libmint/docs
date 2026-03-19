---
layout: default
title: Newton 기초
parent: 나의 공부 게시판
---

좋습니다. 이 코드는 그냥 “예제” 수준이 아니라, **NVIDIA Newton의 핵심 구조 (멀티-피직스 커플링)**를 그대로 드러낸 코드라서 그대로 실행하면 거의 100% 에러가 납니다.
→ 이유는: **숨겨진 전제 (robot_builder, shapes_on_body 등)**가 많기 때문입니다.

아래에서 순서대로 정리해드립니다:

---

# 1️⃣ NVIDIA Newton 블로그 핵심 내용 (핵심만이 아니라 구조까지 깊게)

해당 글의 핵심은 한 줄입니다:

👉 **Rigid-body (MuJoCo) + Deformable (Warp/VBD)를 하나의 시뮬레이션으로 결합**

---

## 🔥 기존 문제

* MuJoCo → 빠르고 안정적인 rigid body (로봇)
* Warp/VBD → soft body (케이블, 천, rope 등)

❌ 문제:

* 서로 다른 solver라서 직접 coupling이 어려움
* contact-rich manipulation (케이블 잡기, 감기 등) 구현이 어려움

---

## 🧠 Newton의 해결 방식

Newton은 **"멀티 유니버스 + 프록시 바디" 구조**를 사용합니다.

---

### 🪐 Universe A (Rigid)

* Solver: MuJoCo 기반
* 대상: 로봇 (articulated rigid body)

---

### 🪐 Universe B (Soft)

* Solver: VBD (Vertex Block Descent)
* 대상: 케이블 / deformable

---

## 🔗 핵심: Proxy Body

👉 이게 진짜 핵심입니다.

* 로봇의 각 링크를
* VBD world 안에 “proxy body”로 복제

즉:

```
Robot link (MuJoCo)  <->  Proxy body (VBD)
```

---

## 🔄 Coupling 방식 (중요)

시간 스텝마다:

### Step 1

👉 이전 step에서 받은 force를 robot에 적용

```
VBD → MuJoCo
```

---

### Step 2

👉 로봇 시뮬레이션 진행

---

### Step 3

👉 로봇 상태를 proxy에 복사

---

### Step 4

👉 proxy velocity에서 coupling force 제거
(이거 안 하면 double counting 발생)

---

### Step 5

👉 VBD 시뮬레이션 진행 (케이블 + proxy 충돌)

---

### Step 6

👉 proxy에서 발생한 contact force 추출

```
MuJoCo ← VBD
```

---

👉 이걸 반복하면:

✔ 로봇이 케이블을 잡고
✔ 케이블이 로봇을 밀고
✔ 실제 물리처럼 상호작용

---

# 2️⃣ 왜 지금 코드가 실행 안 되는가

아래 문제들 때문입니다:

---

## ❌ (1) robot_builder 없음

```python
robot_model = robot_builder.finalize()
```

👉 정의 안됨

---

## ❌ (2) shapes_on_body 없음

```python
for shape in shapes_on_body(...)
```

👉 유틸 함수 없음

---

## ❌ (3) cable_points / cable_quats 없음

👉 케이블 geometry 없음

---

## ❌ (4) harvest_proxy_wrenches 없음

👉 force 추출 함수 없음

---

## ❌ (5) Newton 자체가 pip 패키지가 아님

👉 내부 SDK or preview 상태

---

# 3️⃣ 실행 가능한 최소 버전으로 수정 (핵심)

👉 목표: **“구조 이해 + 실제 실행” 가능한 최소 코드**

---

## ✅ 0. 설치 (중요)

```bash
pip install warp-lang
pip install mujoco
```

👉 Newton은 공식 pip 없음 → mock 처리 필요

---

# 4️⃣ 실행 가능한 "간이 Newton 구조 코드"

👉 Newton 없이 구조만 재현

```python
import warp as wp
import numpy as np

wp.init()

# ---------------------------
# Dummy robot (rigid body)
# ---------------------------
body_count = 2

robot_body_q = wp.zeros(body_count, dtype=wp.transform)
robot_body_qd = wp.zeros(body_count, dtype=wp.spatial_vector)

# ---------------------------
# Dummy cable (soft body)
# ---------------------------
cable_body_q = wp.zeros(body_count, dtype=wp.transform)
cable_body_qd = wp.zeros(body_count, dtype=wp.spatial_vector)

# ---------------------------
# Proxy mapping
# ---------------------------
robot_ids = wp.array([0, 1], dtype=int)
proxy_ids = wp.array([0, 1], dtype=int)

proxy_forces = wp.zeros(body_count, dtype=wp.spatial_vector)

# ---------------------------
# Kernel (원본 핵심)
# ---------------------------
@wp.kernel
def sync_proxy_state(
    robot_ids: wp.array(dtype=int),
    proxy_ids: wp.array(dtype=int),
    src_body_q: wp.array(dtype=wp.transform),
    src_body_qd: wp.array(dtype=wp.spatial_vector),
    dst_body_q: wp.array(dtype=wp.transform),
    dst_body_qd: wp.array(dtype=wp.spatial_vector),
):
    i = wp.tid()

    rid = robot_ids[i]
    pid = proxy_ids[i]

    dst_body_q[pid] = src_body_q[rid]
    dst_body_qd[pid] = src_body_qd[rid]

# ---------------------------
# Step loop
# ---------------------------
for step in range(10):

    # Step 1: apply dummy force
    proxy_forces.fill_(0.1)

    # Step 2: "robot step" (mock)
    robot_body_qd += 0.01

    # Step 3: sync
    wp.launch(
        sync_proxy_state,
        dim=body_count,
        inputs=[
            robot_ids,
            proxy_ids,
            robot_body_q,
            robot_body_qd,
            cable_body_q,
            cable_body_qd,
        ],
    )

    # Step 4: "VBD step" (mock)
    cable_body_qd += 0.02

    print(f"step {step} done")
```

---

# 5️⃣ 원본 코드에서 "진짜로 추가해야 하는 것"

이걸 제대로 돌리려면 아래를 구현해야 합니다:

---

## ✅ 1. robot_builder

* URDF or MJCF 로딩
* articulated body 생성

👉 추천:

```python
import mujoco
```

---

## ✅ 2. shapes_on_body

```python
def shapes_on_body(model, body_id):
    # geom 정보 추출
    return [...]
```

---

## ✅ 3. cable_points 생성

```python
cable_points = np.array([
    [0,0,0],
    [0,0,0.1],
    [0,0,0.2],
])
```

---

## ✅ 4. harvest_proxy_wrenches

👉 핵심:

```python
def harvest_proxy_wrenches(solver, state, contacts, dt):
    return wp.zeros(...)
```

→ 실제로는 contact impulse → force 변환

---

# 6️⃣ 현실적인 실행 방법 (추천)

👉 지금 당장 목표에 따라 3단계로 나누세요:

---

## 🟢 Level 1 (추천)

👉 Warp + 간이 coupling

✔ 위 minimal 코드

---

## 🟡 Level 2

👉 MuJoCo + Warp 연결

✔ mujoco로 robot
✔ warp로 cable

---

## 🔴 Level 3 (진짜 Newton)

👉 NVIDIA Isaac Sim + Newton SDK 필요

---

# 7️⃣ 핵심 요약 (진짜 중요한 부분만)

이 코드는 단순한 예제가 아니라:

👉 **"멀티 물리 엔진 coupling 패턴" 레퍼런스**

핵심은 3개:

---

### 1️⃣ Proxy body

* 서로 다른 physics world 연결

---

### 2️⃣ Staggered coupling

* 한 step 지연 force 적용

---

### 3️⃣ Force undo

* double counting 방지

---

---

# 👉 다음 단계 (원하면)

지금 상황에서 가장 도움이 되는 건:

### ✔ “실제로 로봇 + 케이블 붙여서 돌리는 코드”

or

### ✔ Isaac Sim + Newton 환경 구축

입니다.

---

원하시면:

👉 “UR5 + rope 잡는 시뮬레이션 코드”
👉 “Isaac Sim에서 Newton 쓰는 방법”

까지 **실행 가능한 수준으로** 만들어드릴게요.
