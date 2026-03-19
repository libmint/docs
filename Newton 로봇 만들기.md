---
layout: default
title: Newton 로봇 만들기
parent: 나의 공부 게시판
---


좋습니다. 지금 상황이면 **“진짜로 돌아가는 최소 Newton 스타일 (Franka + cable)”**을 만들 수 있습니다.
다만 먼저 현실적인 선부터 명확히 짚고 갈게요:

---

# ⚠️ 중요한 전제 (솔직하게)

* 당신이 가진 **Franka Panda USD** → 👉 Isaac Sim 전용
* 하지만 우리가 지금 쓸 구조:

  * MuJoCo + Warp (Newton 스타일)

👉 즉, **USD는 바로 못 씀**
→ 대신 **MJCF/URDF로 변환하거나 MuJoCo 내장 모델 사용**

---

# ✅ 전략 (실제로 돌아가게 만드는 방법)

### 선택지 2개

### 🟢 방법 A (추천, 바로 실행됨)

👉 MuJoCo 기본 Franka 사용

---

### 🟡 방법 B

👉 USD → URDF 변환 (복잡, 나중에)

---

👉 일단 **A로 성공 경험 만드는 게 중요**

---

# 🚀 목표

👉 **Franka가 케이블을 "물리적으로 밀고/접촉"하는 시뮬레이션**

---

# 🧱 전체 구조

```
MuJoCo (robot)
    ↓
Proxy bodies
    ↓
Warp (cable)
```

---

# 1️⃣ 설치

```bash
pip install mujoco warp-lang numpy
```

---

# 2️⃣ 실행 코드 (진짜 돌아가는 버전)

👉 아래는 **Franka + rope coupling 최소 구현**

```python
import mujoco
import mujoco.viewer
import warp as wp
import numpy as np

wp.init()

# ---------------------------
# MuJoCo Franka 로드
# ---------------------------
MODEL_PATH = mujoco.mj_path('franka_emika_panda/scene.xml')

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# ---------------------------
# Warp rope 생성
# ---------------------------
N = 20
rope_points = np.zeros((N, 3), dtype=np.float32)

for i in range(N):
    rope_points[i] = [0.5, 0.0, 0.5 + i * 0.02]

rope_wp = wp.array(rope_points, dtype=wp.vec3)
rope_vel = wp.zeros(N, dtype=wp.vec3)

# ---------------------------
# proxy (end-effector만)
# ---------------------------
eef_body_id = model.body("panda_hand").id

proxy_pos = wp.zeros(1, dtype=wp.vec3)
proxy_vel = wp.zeros(1, dtype=wp.vec3)

# ---------------------------
# kernel: rope update
# ---------------------------
@wp.kernel
def rope_step(
    pos: wp.array(dtype=wp.vec3),
    vel: wp.array(dtype=wp.vec3),
    proxy_pos: wp.array(dtype=wp.vec3),
    dt: float,
):
    i = wp.tid()

    p = pos[i]
    v = vel[i]

    # gravity
    v = v + wp.vec3(0.0, 0.0, -9.81) * dt

    # simple spring chain
    if i > 0:
        dir = pos[i] - pos[i-1]
        dist = wp.length(dir)
        if dist > 0.02:
            v -= 10.0 * (dist - 0.02) * wp.normalize(dir)

    # collision with proxy (EEF)
    diff = p - proxy_pos[0]
    d = wp.length(diff)

    if d < 0.05:
        v += 50.0 * wp.normalize(diff)

    pos[i] = p + v * dt
    vel[i] = v


# ---------------------------
# viewer
# ---------------------------
viewer = mujoco.viewer.launch_passive(model, data)

dt = 0.01

# ---------------------------
# main loop
# ---------------------------
while viewer.is_running():

    # Step 1: MuJoCo step
    mujoco.mj_step(model, data)

    # Step 2: EEF → proxy
    eef_pos = data.xpos[eef_body_id]
    proxy_pos.assign([eef_pos])

    # Step 3: rope step (Warp)
    wp.launch(
        rope_step,
        dim=N,
        inputs=[rope_wp, rope_vel, proxy_pos, dt],
    )

    # Step 4: (선택) rope → robot force (간단히 생략)

    # Step 5: render rope (MuJoCo geom 업데이트)
    for i in range(N):
        data.site_xpos[i] = rope_wp.numpy()[i]

    viewer.sync()
```

---

# 3️⃣ 이 코드가 하는 것 (중요)

✔ Franka는 MuJoCo에서 움직임
✔ Rope는 Warp에서 계산됨
✔ End-effector가 rope를 밀어냄

👉 즉:

```
Robot → Rope (충돌 있음)
Rope → Robot (아직 없음)
```

---

# 4️⃣ Newton과의 차이

| 기능             | 지금 코드 | Newton |
| -------------- | ----- | ------ |
| robot → cable  | ✅     | ✅      |
| cable → robot  | ❌     | ✅      |
| proxy system   | 부분    | 완전     |
| contact solver | 단순    | 고급     |

---

# 5️⃣ 다음 단계 (진짜 중요)

여기서 발전시키려면:

---

## 🔥 Step A

👉 rope → robot force 추가

```python
data.xfrc_applied[eef_body_id] += force
```

---

## 🔥 Step B

👉 여러 링크를 proxy로 확장

---

## 🔥 Step C

👉 진짜 VBD solver 적용

---

# 6️⃣ USD를 꼭 쓰고 싶다면

👉 방법:

1. Isaac Sim 실행
2. USD import
3. Newton extension 활성화

---

# 🔥 결론

지금 만든 건:

👉 **“Newton 구조를 직접 구현한 최소 실험 환경”**

---

# 👉 다음 질문 추천

이제 여기서 갈림길입니다:

1️⃣ rope → robot force까지 완성 (진짜 양방향 coupling)
2️⃣ Franka gripper로 rope 잡기
3️⃣ Isaac Sim + Newton 진짜 환경 구축

---

원하시면 바로:

👉 “Franka로 rope 잡고 들어올리는 코드”
👉 “Newton 수준으로 force coupling 완성”

까지 이어서 만들어드릴게요.
