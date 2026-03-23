---
layout: default
title: IsaacSIm 6 + Newton 설치
parent: 나의 공부 게시판
---

GTC 2026 발표 내용과 Newton deformable 관련 최신 정보를 찾아볼게요.최신 GTC 2026 정보를 바탕으로 핵심 차이점과 Newton deformable 사용법을 정리했습니다.

---

## Isaac Sim 5.1 vs 6.0 + Newton 비교

### 핵심 차이 한눈에 보기---
![Screenshot_20260323_133020_Chrome](https://github.com/user-attachments/assets/953b490a-122e-4025-883b-96d8112e2df3)

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


빌드 방법을 정확히 찾아볼게요.공식 문서 기준으로 처음부터 끝까지 순서대로 알려드릴게요.

---

## Isaac Sim 6.0 소스 빌드 완전 가이드 (Ubuntu 22.04)

전체 흐름은 이렇습니다:

**사전 요건 설치 → Isaac Sim 클론 & 빌드 → 환경변수 설정 → Isaac Lab 설치 → Newton 추가 → 검증**

---

### 0단계 — 사전 요건 확인 및 설치

Ubuntu 24.04는 GCC/G++ 11이 필요하며 12+는 지원하지 않습니다. **Ubuntu 22.04 LTS를 강력 권장합니다.** 또한 인터넷 연결이 필요합니다 — Omniverse Kit SDK, 익스텐션, 툴을 빌드 중에 다운로드하기 때문입니다.

```bash
# OS 확인
lsb_release -a        # Ubuntu 22.04여야 함

# GPU 드라이버 확인 (525 이상 필요)
nvidia-smi

# git, git-lfs 설치
sudo apt-get update
sudo apt-get install -y git git-lfs

# git-lfs 초기화
git lfs install
```

---

### 1단계 — Isaac Sim 소스 클론

Isaac Sim 저장소를 클론하고, Git LFS로 대용량 파일을 받습니다:

```bash
# 홈 디렉토리 아래에 작업 폴더 만들기
mkdir -p ~/robot_ws && cd ~/robot_ws

# 클론 (용량 큼 — 수 GB, 시간 걸림)
git clone https://github.com/isaac-sim/IsaacSim.git
cd IsaacSim

# LFS 파일 받기 (빠뜨리면 빌드 중간에 에러 남)
git lfs pull
```

---

### 2단계 — 빌드

Isaac Sim 빌드는 `./build.sh` 한 줄로 실행됩니다. 처음 빌드 시 Omniverse 라이선스 동의 프롬프트가 뜹니다.

```bash
# IsaacSim 폴더 안에서 실행
cd ~/robot_ws/IsaacSim
./build.sh
```

빌드 중에 일어나는 일:
- Omniverse Kit SDK 자동 다운로드 (인터넷 필요)
- CUDA 커널 컴파일
- 익스텐션 패키징

> ⏱️ **첫 빌드는 30분~1시간 이상 걸립니다.** 커피 드시면서 기다리시면 됩니다.

빌드가 끝나면 결과물이 여기 생깁니다:
```
~/robot_ws/IsaacSim/_build/linux-x86_64/release/
```

---

### 3단계 — 환경변수 설정

매번 터미널 열 때마다 설정해야 하므로 `~/.bashrc`에 넣어두는 게 편합니다:

```bash
# ~/.bashrc 에 추가
echo 'export ISAACSIM_PATH="$HOME/robot_ws/IsaacSim/_build/linux-x86_64/release"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON_EXE="$ISAACSIM_PATH/python.sh"'                      >> ~/.bashrc
source ~/.bashrc
```

빌드 확인:

```bash
# Python 환경 확인
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim OK')"

# 실제 실행 확인 (처음엔 셰이더 캐싱으로 수 분 걸림)
${ISAACSIM_PATH}/isaac-sim.sh
```

Isaac Sim을 처음 실행할 때는 익스텐션과 셰이더가 로드·캐싱되느라 수 분이 걸릴 수 있습니다. 이후 실행은 10~30초 정도입니다.

---

### 4단계 — Isaac Lab 설치

```bash
cd ~/robot_ws

# develop 브랜치 클론 (Isaac Sim 6.0 대응 버전)
git clone -b develop https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Isaac Sim 위치를 Isaac Lab에 심볼릭 링크로 연결
ln -s ${ISAACSIM_PATH} _isaac_sim

# Isaac Lab 설치
./isaaclab.sh --install
```

---

### 5단계 — Newton 패키지 추가 설치

```bash
# Isaac Lab 내장 Python으로 설치해야 함 (일반 pip 아님)
${ISAACSIM_PYTHON_EXE} -m pip install newton mujoco-warp
```

---

### 6단계 — 동작 검증

```bash
# 기본 환경 테스트
./isaaclab.sh -p scripts/environments/zero_agent.py \
  --task Isaac-Cartpole-Direct-v0 --num_envs 64

# Newton 백엔드 테스트
./isaaclab.sh -p scripts/environments/zero_agent.py \
  --task Isaac-Cartpole-Newton-Direct-v0 --num_envs 64
```

---

### 요약 — 명령어 순서 한눈에 보기

```
① git clone IsaacSim  →  git lfs pull
② ./build.sh          (30분~1시간)
③ 환경변수 ~/.bashrc 에 추가
④ git clone IsaacLab -b develop
⑤ ln -s + ./isaaclab.sh --install
⑥ pip install newton mujoco-warp
⑦ ./isaaclab.sh -p ... 으로 검증
```

빌드하다가 에러 메시지 뜨면 그대로 붙여넣어 주세요, 같이 해결해 드릴게요!
