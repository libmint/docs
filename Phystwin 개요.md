---
layout: default
title: PhysTwin 개요
parent: 나의 공부 게시판
--

다음 프로젝트는 **PhysTwin** 입니다.
GitHub: [https://github.com/Jianghanxiao/PhysTwin](https://github.com/Jianghanxiao/PhysTwin)

이 프로젝트는 **“영상에서 변형 가능한 물체(deformable object)의 물리적 디지털 트윈을 자동으로 생성하는 시스템”**입니다. 쉽게 말하면:

> **실제 물체를 영상으로 촬영 → AI가 3D 모델 + 물리 파라미터를 추정 → 실제처럼 움직이는 시뮬레이션 객체 생성**

즉 **Vision + Physics + Simulation + Rendering**을 결합한 **Real-to-Sim Digital Twin 시스템**입니다. ([Contact-Rich@ICRA2025][1])

아래에서 **기술 구조, 알고리즘, 구현 방식, 코드 구조** 중심으로 자세히 설명합니다.

---

# 1. PhysTwin의 핵심 목표

기존 문제:

영상에서 물체를 복원하는 기술들은 보통

* 3D shape reconstruction만 가능
* 물리적 특성(탄성, 강성, 질량 등)은 없음
* 시뮬레이션 불가능

반대로 물리 시뮬레이터는

* 실제 물체 파라미터가 필요
* 실제 영상에서 자동 추정이 어려움

그래서 PhysTwin의 목표는

**video → physically simulatable digital twin**

즉

영상만으로 다음을 모두 복원

1️⃣ 3D geometry
2️⃣ appearance
3️⃣ 물리 파라미터
4️⃣ 물체의 동역학

그리고 이를 **실시간 시뮬레이션 가능한 모델로 생성**합니다. ([Contact-Rich@ICRA2025][1])

---

# 2. 전체 시스템 파이프라인

PhysTwin 전체 구조:

```
Video input
   │
   ▼
Visual Perception
(Tracking + Segmentation + Geometry)
   │
   ▼
Physics Modeling
(Spring-mass system)
   │
   ▼
Inverse Physics Optimization
(physical parameters estimation)
   │
   ▼
Rendering Model
(Gaussian Splatting)
   │
   ▼
Interactive Digital Twin
```

구성 요소는 크게 4개입니다.

1️⃣ Visual perception
2️⃣ Physics model
3️⃣ Inverse physics optimization
4️⃣ Neural rendering

---

# 3. 핵심 기술 1 — Physics-informed representation

PhysTwin은 객체를 **3가지 모델로 동시에 표현**합니다.

### (1) Geometry model

물체 shape 표현

논문에서 사용:

* **Generative shape model**

목적

* 가려진 부분 보완
* sparse video에서도 3D shape 복원

---

### (2) Physics model

핵심은 **spring-mass model**

물체를

```
particles (mass points)
+ springs
```

구조로 표현합니다.

예

```
o----o----o
|    |    |
o----o----o
|    |    |
o----o----o
```

각 스프링에는

* stiffness
* damping
* rest length

등이 있습니다.

이것으로

* cloth
* rope
* stuffed toy

같은 soft body를 시뮬레이션합니다.

---

### (3) Appearance model

렌더링은 **Gaussian Splatting** 사용

코드에서도 확인 가능:

```
gaussian_splatting/
gs_train.py
gs_render.py
```

3D Gaussian Splatting은

```
3D points → Gaussian blobs → rasterization
```

장점

* NeRF보다 빠름
* 실시간 렌더링 가능

---

# 4. 핵심 기술 2 — Differentiable physics simulator

PhysTwin은 **미분 가능한 물리 시뮬레이터**를 사용합니다.

사용 기술

* **NVIDIA Warp**

코드:

```
warp simulator
```

특징

* GPU accelerated
* differentiable physics

그래서

```
physics parameters
   ↓
simulation
   ↓
loss
   ↓
gradient
   ↓
parameter update
```

이게 가능합니다.

논문에서는 이를

**inverse physics optimization**

이라고 합니다.

---

# 5. 핵심 기술 3 — Inverse Physics (물리 파라미터 추정)

영상에서 물리 파라미터를 직접 측정할 수 없기 때문에

**inverse modeling**을 합니다.

아이디어

```
실제 영상 motion
vs
simulation motion
```

차이를 최소화.

즉

```
argmin θ

|| simulated motion - observed motion ||
```

θ 는

* spring stiffness
* damping
* mass
* control forces

---

# 6. Multi-stage Optimization

PhysTwin은 **2단계 optimization**을 합니다.

### Stage 1 — Zero-order optimization

CMA-ES 사용

코드

```
optimize_cma.py
script_optimize.py
```

특징

gradient 필요 없음

목적

```
rough physics parameter estimation
```

시간

약 **12분**

---

### Stage 2 — First-order optimization

gradient 기반

```
train_warp.py
script_train.py
```

사용

* differentiable simulator

목적

```
fine tuning
```

시간

약 **5분**

---

# 7. Visual Perception Pipeline

영상에서 얻는 데이터

```
RGB
Depth
Camera calibration
```

코드

```
process_data.py
script_process_data.py
```

여기서 수행

1️⃣ segmentation
2️⃣ point tracking
3️⃣ mesh initialization

---

## 사용 모델들

GitHub에 따르면 optional dependencies:

* Grounded-SAM
* Grounding-DINO
* TRELLIS
* SDXL

목적

```
object segmentation
shape prior generation
```

---

# 8. Motion Tracking

영상에서

```
surface points tracking
```

을 합니다.

예

```
frame t
p_i

frame t+1
p_i'
```

이를 이용해

```
observed deformation field
```

구성.

이것이 physics optimization target이 됩니다.

---

# 9. Rendering — Gaussian Splatting

appearance reconstruction 단계.

코드

```
gs_train.py
gs_render.py
```

과정

```
first frame
↓
Gaussian point cloud
↓
training
↓
photorealistic rendering
```

장점

* real-time rendering
* differentiable

---

# 10. Interactive Simulation

생성된 PhysTwin은

```
interactive_playground.py
```

에서 사용됩니다.

예:

```
python interactive_playground.py
```

기능

* keyboard control
* object manipulation
* real-time simulation

RTX 4090 기준

```
~37 FPS
```

---

# 11. Force Inference

흥미로운 기능

```
visualize_force.py
```

영상만 보고

**사람이 가한 힘을 추정**

원리

```
physics equation
F = ma
```

simulation과 맞도록

force 추정.

---

# 12. Material Estimation

```
visualize_material.py
```

추정 가능

* stiffness
* elasticity
* damping

즉

```
cloth vs rubber vs plush toy
```

같은 material property.

---

# 13. 코드 구조 설명

GitHub 구조 핵심

```
PhysTwin
│
├── data_process
│   ├── tracking
│   ├── segmentation
│
├── gaussian_splatting
│   ├── gs_train.py
│   ├── gs_render.py
│
├── optimization
│   ├── optimize_cma.py
│   ├── train_warp.py
│
├── simulation
│   ├── warp physics
│
├── interactive
│   ├── interactive_playground.py
```

---

# 14. 실제 구현 핵심 기술 요약

PhysTwin은 사실 **3개 연구 분야 결합**입니다.

### Computer Vision

* segmentation
* tracking
* multi-view reconstruction

---

### Physics Simulation

* spring-mass model
* differentiable physics
* inverse dynamics

---

### Neural Rendering

* Gaussian splatting
* real-time rendering

---

# 15. 기술적으로 가장 중요한 포인트

PhysTwin이 혁신적인 이유

### 1️⃣ Video → physics model

기존

```
video → mesh
```

PhysTwin

```
video → mesh + physics
```

---

### 2️⃣ Differentiable physics

물리 파라미터 학습 가능

---

### 3️⃣ Real-time simulation

digital twin 생성

---

# 16. 실제 응용 분야

논문에서 제시한 활용

### robotics

robot manipulation simulation

---

### XR / VR

interactive virtual objects

---

### digital twin

현실 객체 복제

---

### dataset generation

robot learning

---

# 17. 한줄 핵심 정리

PhysTwin은

> **영상에서 deformable object의 geometry + appearance + physics를 동시에 복원하여 실제처럼 시뮬레이션 가능한 digital twin을 만드는 시스템**

입니다.

---

원하시면 제가 추가로 다음도 설명해 드리겠습니다.

1️⃣ **PhysTwin 아키텍처 논문 수준으로 상세 분석**
2️⃣ **코드 단위 (gs_train.py / optimize_cma.py) 구현 설명**
3️⃣ **Spring-mass + Warp simulator 실제 코드 동작 원리**
4️⃣ **PhysTwin vs NeRF vs Soft-body simulator 차이**

특히 **“이 프로젝트에서 가장 어려운 기술 3개”**도 설명해 드릴 수 있습니다.

[1]: https://contact-rich.github.io/assets/pdf/papers/7_PhysTwin_Physics_Informed_Reconstruction.pdf?utm_source=chatgpt.com "PhysTwin: Physics-Informed Reconstruction and Simulation of"
