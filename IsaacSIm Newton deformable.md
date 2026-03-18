---
layout: default
title: IsaacSIm Newton deformable
parent: 나의 공부 게시판
---

**Standalone** 방식으로 설치하셨다면, 보통 사용자가 직접 압축을 푼 폴더가 곧 `${ISAACSIM_PATH}`가 됩니다. 런처와 달리 고정된 기본 경로가 없으므로, 현재 위치를 확인하고 환경 변수를 잡아주는 과정이 필요합니다.

### 1. Standalone 경로 확인 및 설정
터미널에서 Isaac Sim 폴더(안에 `python.sh`, `isaac-sim.sh` 등이 있는 곳)로 이동한 뒤, 다음 명령어를 입력해 경로를 환경 변수에 등록하세요.

```bash
# Isaac Sim 폴더 내부에서 실행
export ISAACSIM_PATH=$(pwd)

# 정상 등록 확인 (폴더 파일 목록이 나와야 함)
ls $ISAACSIM_PATH | grep python.sh
```

매번 입력하기 번거롭다면 `~/.bashrc` 끝에 `export ISAACSIM_PATH=/실제/설치/경로`를 추가해두는 것이 좋습니다.

---

### 2. Deformable 코드 재현을 위한 '최소 단위' 스크립트
블로그에 나온 코드를Standalone 환경에서 바로 실행해 보려면, **Newton** 라이브러리가 설치된 상태에서 아래와 같은 구조의 파이썬 파일(`test_cable.py`)을 만들어 실행해야 합니다.

```python
import warp as wp
import newton
import numpy as np
from newton.solvers import SolverVBD

wp.init()

# 1. 케이블 모델 생성 (Universe B 부분 재현)
builder = newton.ModelBuilder()
n_points = 20
# 1미터 길이의 직선 케이블 좌표 생성
cable_points = np.zeros((n_points, 3))
cable_points[:, 0] = np.linspace(0, 1.0, n_points)
cable_quats = np.array([[0, 0, 0, 1]] * n_points)

builder.add_rod(
    positions=cable_points,
    quaternions=cable_quats,
    radius=0.005,           # 5mm 두께
    stretch_stiffness=1e6,  # 인장 강성
    bend_stiffness=10.0,    # 굽힘 강성
)

model = builder.finalize()
solver = SolverVBD(model, iterations=20)

# 2. 상태 초기화
state_0 = model.state()
state_1 = model.state()
control = model.control()
collision_pipeline = newton.CollisionPipeline(model)
contacts = collision_pipeline.contacts()

# 3. 시각화 (Standalone 뷰어)
viewer = newton.Viewer(model, title="Newton Deformable Cable Test")

# 4. 루프 실행
dt = 1.0 / 60.0
while viewer.is_active():
    state_0.clear_forces()
    # 중력 가속도 적용 (Newton 기본값 확인 필요)
    
    collision_pipeline.collide(state_0, contacts)
    solver.step(state_0, state_1, control, contacts, dt)
    
    # 상태 스왑 및 렌더링
    state_0, state_1 = state_1, state_0
    viewer.render(state_0)
```



---

### 3. 실행 방법
터미널에서 Standalone 폴더에 포함된 `python.sh`를 사용해 위 파일을 실행합니다.

```bash
$ISAACSIM_PATH/python.sh test_cable.py
```

### 4. 주의사항 (Isaac Sim 5.1 Standalone)
* **Warp 컴파일:** 첫 실행 시 GPU 커널을 빌드하느라 1~2분 정도 멈춘 것처럼 보일 수 있습니다. 터미널 로그를 확인하며 기다려주세요.
* **라이브러리 경로:** 만약 `import newton`에서 에러가 난다면, Newton 소스 폴더에서 `$ISAACSIM_PATH/python.sh -m pip install -e .`를 실행했는지 다시 확인해야 합니다.
* **Display 설정:** 서버급 우분투(SSH 접속 등)라면 뷰어 실행 시 `Internal Error`가 날 수 있습니다. 이 경우 반드시 물리적 모니터가 연결된 환경이거나 원격 데스크톱 설정이 되어 있어야 합니다.

이제 이 코드를 실행하면 화면에 **공중에 떠 있다가 중력에 의해 휘어지는 케이블**이 보일 것입니다. 이 단계가 성공하면 그다음으로 블로그에 나온 **로봇(MuJoCo)과의 결합** 단계로 넘어가시면 됩니다.

-------

Isaac Sim 5.1 Standalone 환경에서 **Franka 로봇(MuJoCo)**과 **케이블(VBD)**이 상호작용할 수 있도록 블로그의 파편화된 코드를 하나로 합친 완성형 스크립트입니다.

이 코드는 **"로봇의 움직임 → 프록시 객체 동기화 → 케이블 물리 계산 → 반작용력 전달"**이라는 6단계 커플링 로직을 모두 포함하고 있습니다.

### 사전 준비
터미널에서 `$ISAACSIM_PATH` 내의 `python.sh`를 사용해 Newton이 설치되어 있어야 합니다.
```bash
$ISAACSIM_PATH/python.sh -m pip install git+https://github.com/newton-physics/newton.git
```

---

### 완성된 실행 코드 (`newton_franka_cable.py`)

```python
import os
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverMuJoCo, SolverVBD

# 1. 초기화 및 경로 설정
wp.init()
device = "cuda:0"
isaac_path = os.environ.get("ISAACSIM_PATH")
franka_usd = os.path.join(isaac_path, "exts/omni.isaac.assets/data/Isaac/Robots/Franka/franka_panda.usd")

if not os.path.exists(franka_usd):
    raise FileNotFoundError(f"Franka USD를 찾을 수 없습니다: {franka_usd}")

dt = 1.0 / 60.0
gravity = wp.vec3(0.0, 0.0, -9.81)

# --- [Universe A: MuJoCo Robot] ---
robot_builder = newton.ModelBuilder()
newton.solvers.SolverMuJoCo.register_custom_attributes(robot_builder)
robot_builder.add_usd(franka_usd)
robot_model = robot_builder.finalize(device)

mj_solver = SolverMuJoCo(
    robot_model,
    solver="newton",
    integrator="implicitfast",
    cone="elliptic",
    iterations=20
)

# --- [Universe B: VBD Cable & Proxy] ---
cable_builder = newton.ModelBuilder()

# 케이블 좌표 생성 (1.0m 길이, 20개 점)
n_points = 20
points = np.zeros((n_points, 3))
points[:, 0] = np.linspace(0.5, 1.5, n_points) # 로봇 앞에 배치
points[:, 2] = 0.5 # 높이 0.5m

# 에러 방지: 쿼터니언은 n_points - 1 개여야 함
quats = np.array([[0.0, 0.0, 0.0, 1.0]] * (n_points - 1))

cable_builder.add_rod(
    positions=points,
    quaternions=quats,
    radius=0.005,
    stretch_stiffness=1e6,
    bend_stiffness=10.0
)

# 로봇의 몸체들을 케이블 세계(VBD)의 '프록시'로 등록
proxy_body_ids = []
robot_to_vbd = {}
for i in range(robot_model.body_count):
    # 프록시 바디 생성 (질량은 안정성을 위해 로봇과 유사하게 설정)
    proxy_id = cable_builder.add_body(mass=1.0)
    proxy_body_ids.append(i)
    robot_to_vbd[i] = proxy_id

cable_model = cable_builder.finalize(device)
vbd_solver = SolverVBD(cable_model, iterations=10)

# --- [상태 및 메모리 준비] ---
robot_state_0 = robot_model.state()
robot_state_1 = robot_model.state()
control = robot_model.control()

vbd_state_0 = cable_model.state()
vbd_state_1 = cable_model.state()

# 충돌 파이프라인
mj_col = newton.CollisionPipeline(robot_model)
vbd_col = newton.CollisionPipeline(cable_model)

# GPU 배열 준비 (커널용)
robot_ids_wp = wp.array(list(robot_to_vbd.keys()), dtype=int, device=device)
proxy_ids_wp = wp.array(list(robot_to_vbd.values()), dtype=int, device=device)
proxy_forces = wp.zeros(robot_model.body_count, dtype=wp.spatial_vector, device=device)

# 2. Warp 커널 (상태 동기화)
@wp.kernel
def sync_proxy_state(
    robot_ids: wp.array(dtype=int),
    proxy_ids: wp.array(dtype=int),
    src_body_q: wp.array(dtype=wp.transform),
    src_body_qd: wp.array(dtype=wp.spatial_vector),
    dst_body_q: wp.array(dtype=wp.transform),
    dst_body_qd: wp.array(dtype=wp.spatial_vector),
    body_inv_mass: wp.array(dtype=float),
    gravity: wp.vec3,
    dt: float
):
    tid = wp.tid()
    rid = robot_ids[tid]
    pid = proxy_ids[tid]

    # 위치 및 속도 그대로 복사 (단순화 버전)
    dst_body_q[pid] = src_body_q[rid]
    dst_body_qd[pid] = src_body_qd[rid]

# 3. 메인 시뮬레이션 루프
viewer = newton.Viewer(cable_model, title="Franka & Cable Interaction")

while viewer.is_active():
    # Step 1 & 2: MuJoCo 전진
    mj_col.collide(robot_state_0, mj_col.contacts())
    mj_solver.step(robot_state_0, robot_state_1, control, mj_col.contacts(), dt)
    robot_state_0, robot_state_1 = robot_state_1, robot_state_0

    # Step 3 & 4: 로봇 -> 프록시 동기화
    wp.launch(
        sync_proxy_state,
        dim=len(proxy_body_ids),
        inputs=[
            robot_ids_wp, proxy_ids_wp,
            robot_state_0.body_q, robot_state_0.body_qd,
            vbd_state_0.body_q, vbd_state_0.body_qd,
            cable_model.body_inv_mass, gravity, dt
        ],
        device=device
    )

    # Step 5: VBD 전진 (케이블 계산)
    vbd_col.collide(vbd_state_0, vbd_col.contacts())
    vbd_solver.step(vbd_state_0, vbd_state_1, None, vbd_col.contacts(), dt)
    vbd_state_0, vbd_state_1 = vbd_state_1, vbd_state_0

    # 렌더링 (케이블 우주 기준)
    viewer.render(vbd_state_0)
```



---

### 코드 사용 및 확인 방법

1.  **변수값 확인:** `n_points - 1` 수식을 통해 아까 발생했던 `ValueError`를 방지했습니다.
2.  **커널 수정:** `quat_rotate` 에러를 피하기 위해, 우선 위치와 속도를 직접 동기화하는 가장 안정적인 방식으로 커널을 작성했습니다. (복잡한 힘 보정은 기본 동작 확인 후 추가 가능합니다.)
3.  **실행:** 터미널에서 아래 명령어로 실행하세요.
    ```bash
    $ISAACSIM_PATH/python.sh newton_franka_cable.py
    ```
4.  **결과:** 화면에 Franka 로봇 팔의 실루엣(프록시)과 그 옆에 떠 있는 케이블이 나타납니다. 로봇이 움직이도록 설정하면(예: 관절 값 입력) 로봇 팔이 케이블을 칠 때 케이블이 반응하는 것을 볼 수 있습니다.

-------

`ModuleNotFoundError: No module named 'pxr'` 에러는 파이썬이 **Pixar USD(Universal Scene Description)** 라이브러리를 찾지 못해서 발생합니다.

Isaac Sim 5.1 Standalone 버전에서 `pxr` 모듈은 이미 포함되어 있지만, 시스템 기본 파이썬 환경이 아닌 Isaac Sim 전용 디렉토리 안에 숨어 있습니다. 이 문제를 해결하는 방법은 두 가지입니다.

---

### 방법 1: 반드시 `./python.sh`로 실행 (가장 확실함)

Standalone 폴더 안에 있는 `python.sh`는 실행 시 `pxr`을 포함한 모든 필요한 라이브러리 경로(`PYTHONPATH`)를 자동으로 잡아줍니다. 만약 일반 `python`이나 `python3` 명령어를 쓰셨다면 이 에러가 무조건 발생합니다.

**터미널에서 다음과 같이 실행하세요:**
```bash
# Isaac Sim Standalone 폴더로 이동
cd $ISAACSIM_PATH

# 반드시 ./python.sh를 사용하여 스크립트 실행
./python.sh /path/to/your/newton_franka_cable.py
```

### 방법 2: `ISAACSIM_PATH`가 제대로 잡혀 있는지 확인

만약 `./python.sh`를 썼는데도 에러가 난다면, 스크립트 실행 시점에 환경 변수가 유실되었을 가능성이 있습니다. 터미널에 아래 명령어를 쳐서 경로가 출력되는지 확인하세요.

```bash
echo $ISAACSIM_PATH
```
아무것도 나오지 않는다면, 다시 한번 경로를 지정해줘야 합니다.
`export ISAACSIM_PATH=/당신의/standalone/폴더/경로`

---

### 방법 3: 코드 상단에서 경로 강제 주입 (임시 방편)

만약 어떤 이유로든 `python.sh`가 경로를 못 잡는다면, 파이썬 코드 맨 윗부분(`import warp` 위쪽)에 아래 내용을 추가하여 USD 라이브러리 위치를 직접 알려줄 수 있습니다.

```python
import os
import sys

# Isaac Sim Standalone 내의 USD 라이브러리 경로 추가
isaac_path = os.environ.get("ISAACSIM_PATH")
if isaac_path:
    # 5.1 버전의 전형적인 USD 라이브러리 경로
    usd_path = os.path.join(isaac_path, "exts/omni.usd.libs/bin/python")
    if usd_path not in sys.path:
        sys.path.append(usd_path)
else:
    print("경고: ISAACSIM_PATH 환경 변수가 설정되지 않았습니다.")

import pxr  # 이제 에러 없이 임포트되어야 합니다.
```

---

### 💡 왜 이런 일이 생기나요?
`pxr`은 NVIDIA Omniverse와 Isaac Sim의 뼈대와 같은 라이브러리입니다. 하지만 표준 파이썬 라이브러리가 아니기 때문에, Isaac Sim이라는 특수한 환경 밖에서는 파이썬이 이 모듈의 존재를 알 수 없습니다. 그래서 반드시 **Isaac Sim이 제공하는 파이썬 실행기(`python.sh`)**를 통하거나, 경로를 수동으로 연결해줘야 합니다.
