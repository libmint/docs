---
layout: default
title: IsaacSIm 6 + Newton 설치
parent: 나의 공부 게시판
---

GTC 2026 발표 내용과 Newton deformable 관련 최신 정보를 찾아볼게요.최신 GTC 2026 정보를 바탕으로 핵심 차이점과 Newton deformable 사용법을 정리했습니다.

---

## Isaac Sim 5.1 vs 6.0 + Newton 비교

### 핵심 차이 한눈에 보기---

## 결론: Newton Deformable 실험이 목적이라면 반드시 Isaac Sim 6.0

**Isaac Sim 5.1의 deformable**은 PhysX 백엔드 전용으로만 동작하며, Newton 백엔드에서 deformable objects, surface gripper, material randomization은 지원되지 않습니다. 즉, 5.1에서 Newton deformable은 아직 불완전합니다.

반면 GTC 2026에서 발표된 Newton 1.0 GA는 VBD(Vertex Block Descent) 솔버로 선형 변형체(케이블), 얇은 변형체(천), 부피 변형체(고무 부품)를 처리하며, iMPM(implicit Material Point Method)으로 모래 등 입자 시뮬레이션도 지원합니다. VBD와 MPM 솔버는 MuJoCo Warp와 명시적으로 결합되어, 로봇이 변형체를 조작하는 시나리오를 지원합니다.

---

## Isaac Sim 6.0 + Newton 1.0 설치 (Ubuntu, 소스 빌드)

### 1단계 — Isaac Sim 6.0 소스 클론

```bash
git clone https://github.com/isaac-sim/IsaacSim.git -b develop
cd IsaacSim
# README의 빌드 지침 참고 (cmake 기반)
```

### 2단계 — Isaac Lab 3.0 Early Access 설치

```bash
git clone -b develop https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install

# Newton 전용 확장 설치
pip install newton mujoco-warp
```

### 3단계 — Newton Deformable 코드 예시

아래는 MuJoCo rigid-body 로봇과 VBD 케이블 deformable을 두 개의 독립적인 "Universe"로 결합하는 패턴입니다:

```python
import newton
from newton.solvers import SolverMuJoCo, SolverVBD

# Universe A: MuJoCo rigid-body 로봇
robot_model = robot_builder.finalize()
mj_solver = SolverMuJoCo(
    robot_model,
    solver="newton",
    integrator="implicitfast",
    cone="elliptic",
    iterations=20,
)

# Universe B: VBD 케이블 deformable
cable_builder = newton.ModelBuilder()
cable_builder.add_rod(
    positions=cable_points,
    quaternions=cable_quats,
    radius=0.003,                # 케이블 반경 [m]
    stretch_stiffness=1e12,      # EA [N]
    bend_stiffness=3.0,          # EI [N·m²]
    stretch_damping=1e-3,
    bend_damping=1.0,
)
```

### 4단계 — Franka + Cloth 데모 바로 실행 (가장 빠른 확인법)

Newton 리포에는 Franka 로봇 팔이 천을 조작하는 데모가 포함되어 있으며, RTX 4090 기준 약 30 FPS로 실행됩니다. GPU-IPC 대비 300배 이상 빠릅니다:

```bash
git clone https://github.com/newton-physics/newton.git
cd newton
# Franka + cloth 데모 실행
python examples/franka_cloth.py
```

---

## ⚠️ 현실적 주의사항

Isaac Sim 6.0은 현재 Early Access로, 여러 physics backend 지원과 성능 개선, 로봇 리깅 워크플로우를 확장 중입니다. 즉 아직 소스 빌드만 가능하고 API 변경이 있을 수 있습니다. **Newton standalone repo**(`newton-physics/newton`)는 Isaac Sim 없이도 VBD deformable을 바로 테스트할 수 있어서, 먼저 standalone으로 검증 후 Isaac Sim 6.0으로 이전하는 것을 추천합니다.
