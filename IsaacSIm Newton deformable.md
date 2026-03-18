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

**혹시 위 코드를 실행했을 때 발생하는 구체적인 에러 메시지가 있나요?** 있다면 바로 해결해 드릴 수 있습니다.
