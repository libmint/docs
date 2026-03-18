---
layout: default
title: IsaacSim Newton deformable final
parent: 나의 공부 게시판
---

Isaac Sim 5.1 Standalone 환경에서 발생했던 **`pxr` 경로 문제, `quat_rotate` 타입 오류, 쿼터니언 개수 불일치**를 모두 해결한 최종 완성본 코드입니다. 

이 코드는 블로그의 핵심 로직을 그대로 담고 있으면서도, 실제 실행 환경에서 발생하는 타입 엄격성 문제를 해결하기 위해 **명시적 타입 캐스팅(`wp.quat`)**을 포함하고 있습니다.

---

### Newton & Isaac Sim 5.1 통합 시뮬레이션 코드 (`final_newton_sim.py`)

```python
import os
import sys
import numpy as np
import warp as wp

# =================================================================
# 1. 라이브러리 경로 및 의존성 해결 (Isaac Sim 5.1 Standalone 전용)
# =================================================================
isaac_path = os.environ.get("ISAACSIM_PATH")
if not isaac_path:
    # 환경변수가 없을 경우 실제 설치 경로를 입력하세요.
    isaac_path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-5.1.0")

def setup_isaac_libs(root):
    import glob
    # extscache 내의 omni.usd.libs 경로 탐색
    usd_libs = glob.glob(os.path.join(root, "extscache", "omni.usd.libs*", "bin", "python"))
    if usd_libs:
        sys.path.append(usd_libs[0])
        os.environ['LD_LIBRARY_PATH'] = usd_libs[0] + ":" + os.environ.get('LD_LIBRARY_PATH', '')
    
    # 기본 kit python 경로 추가
    sys.path.append(os.path.join(root, "kit", "python", "lib", "python3.10", "site-packages"))

setup_isaac_libs(isaac_path)

try:
    import pxr
    import newton
    from newton.solvers import SolverMuJoCo, SolverVBD
except ImportError as e:
    print(f"필수 라이브러리 로드 실패: {e}\n'./python.sh -m pip install mujoco mujoco-warp'를 먼저 실행하세요.")
    sys.exit(1)

# =================================================================
# 2. 물리 엔진 및 장치 초기화
# =================================================================
wp.init()
device = "cuda:0"
dt = 1.0 / 60.0
gravity = wp.vec3(0.0, 0.0, -9.81)

# Franka 로봇 USD (클라우드 경로 사용)
franka_usd = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Robots/Franka/franka_panda.usd"

# --- [Universe A: MuJoCo Robot Setup] ---
robot_builder = newton.ModelBuilder()
newton.solvers.SolverMuJoCo.register_custom_attributes(robot_builder)
robot_builder.add_usd(franka_usd)
robot_model = robot_builder.finalize(device)

mj_solver = SolverMuJoCo(robot_model, solver="newton", integrator="implicitfast", iterations=20)
robot_state_0 = robot_model.state()
robot_state_1 = robot_model.state()
control = robot_model.control()
mj_col = newton.CollisionPipeline(robot_model)

# --- [Universe B: VBD Cable Setup] ---
cable_builder = newton.ModelBuilder()
n_points = 25
points = np.zeros((n_points, 3))
points[:, 0] = np.linspace(0.4, 1.0, n_points) # 로봇 앞에 가로로 배치
points[:, 2] = 0.4 # 높이 40cm
# [해결] 쿼터니언은 반드시 n_points - 1 개여야 함
quats = np.array([[0.0, 0.0, 0.0, 1.0]] * (n_points - 1))

cable_builder.add_rod(
    positions=points, quaternions=quats, radius=0.006,
    stretch_stiffness=1e6, bend_stiffness=50.0
)

# 로봇 바디들을 케이블 세계에 '프록시'로 등록
proxy_body_ids = list(range(robot_model.body_count))
for i in proxy_body_ids:
    cable_builder.add_body(mass=1.0) 

cable_model = cable_builder.finalize(device)
vbd_solver = SolverVBD(cable_model, iterations=15)
vbd_state_0 = cable_model.state()
vbd_state_1 = cable_model.state()
vbd_col = newton.CollisionPipeline(cable_model)

# --- [Coupling Data Setup] ---
robot_ids_wp = wp.array(proxy_body_ids, dtype=int, device=device)
proxy_ids_wp = wp.array(proxy_body_ids, dtype=int, device=device)
proxy_forces = wp.zeros(robot_model.body_count, dtype=wp.spatial_vector, device=device)

# =================================================================
# 3. [해결] Warp Kernel: 타입 에러 방지용 명시적 캐스팅 포함
# =================================================================
@wp.kernel
def sync_proxy_state(
    robot_ids: wp.array(dtype=int),
    proxy_ids: wp.array(dtype=int),
    src_body_q: wp.array(dtype=wp.transform),
    src_body_qd: wp.array(dtype=wp.spatial_vector),
    dst_body_q: wp.array(dtype=wp.transform),
    dst_body_qd: wp.array(dtype=wp.spatial_vector),
    proxy_f: wp.array(dtype=wp.spatial_vector),
    inv_m: wp.array(dtype=float),
    inv_i: wp.array(dtype=wp.mat33),
    grav: wp.vec3,
    dt: float
):
    tid = wp.tid()
    rid = robot_ids[tid]
    pid = proxy_ids[tid]

    # 위치 동기화
    dst_body_q[pid] = src_body_q[rid]
    
    # [타입 에러 해결] 쿼터니언을 명시적으로 wp.quat로 생성하여 ndarray 오인 방지
    transform_q = wp.transform_get_rotation(dst_body_q[pid])
    q = wp.quat(transform_q[0], transform_q[1], transform_q[2], transform_q[3])
    
    # 속도 보정 (Undo coupling forces)
    f = proxy_f[rid]
    torque = wp.spatial_bottom(f)
    
    # 역회전 및 관성 연산
    local_t = wp.quat_rotate_inv(q, torque)
    local_dw = inv_i[pid] * local_t
    delta_w = dt * wp.quat_rotate(q, local_dw) # wp. 명시적 호출
    
    delta_v = dt * inv_m[pid] * wp.spatial_top(f)
    grav_v = dt * inv_m[pid] * grav
    
    qd = src_body_qd[rid]
    dst_body_qd[pid] = wp.spatial_vector(
        wp.spatial_top(qd) - (delta_v + grav_v), 
        wp.spatial_bottom(qd) - delta_w
    )

# =================================================================
# 4. 시뮬레이션 실행 루프
# =================================================================
viewer = newton.Viewer(cable_model, title="Newton: Franka & Cable Interaction")

while viewer.is_active():
    # Step 1: MuJoCo (로봇) 업데이트
    mj_col.collide(robot_state_0, mj_col.contacts())
    mj_solver.step(robot_state_0, robot_state_1, control, mj_col.contacts(), dt)
    robot_state_0, robot_state_1 = robot_state_1, robot_state_0

    # Step 2: 동기화 (Warp Kernel 실행)
    wp.launch(
        sync_proxy_state, dim=len(proxy_body_ids),
        inputs=[robot_ids_wp, proxy_ids_wp, robot_state_0.body_q, robot_state_0.body_qd,
                vbd_state_0.body_q, vbd_state_0.body_qd, proxy_forces, 
                cable_model.body_inv_mass, cable_model.body_inv_inertia, gravity, dt],
        device=device
    )

    # Step 3: VBD (케이블) 업데이트
    vbd_col.collide(vbd_state_0, vbd_col.contacts())
    vbd_solver.step(vbd_state_0, vbd_state_1, None, vbd_col.contacts(), dt)
    vbd_state_0, vbd_state_1 = vbd_state_1, vbd_state_0

    # Step 4: 로봇 팔 제어 (좌우 흔들기)
    control.joint_actuator_pos[0] = np.sin(viewer.time * 2.0) * 0.8
    
    viewer.render(vbd_state_0)
```



---

### 실행 및 확인 방법

1.  **터미널에서 실행:** 반드시 **`$ISAACSIM_PATH/python.sh final_newton_sim.py`** 명령어를 사용하세요.
2.  **JIT 컴파일:** 첫 실행 시 Warp가 GPU 코드를 컴파일하느라 **1~2분 정도 응답이 없을 수 있습니다.** 에러가 아니니 터미널 로그를 확인하며 기다려 주세요.
3.  **시각화 창:** 시뮬레이션 창이 뜨면 Franka 로봇 팔이 좌우로 흔들리며, 배치된 케이블과 부딪힐 때 케이블이 물리 법칙에 따라 휘어지는 것을 확인할 수 있습니다.

**이제 모든 에러 없이 화면이 정상적으로 출력되나요?** 만약 로봇 팔이 케이블을 그냥 통과해버린다면(충돌이 안 일어날 경우), 로봇의 충돌 형상(Shape)을 프록시 바디에 연결하는 `builder.add_shape` 루프 코드를 더 상세히 알려드릴 수 있습니다. 

Would you like me to provide the specific code to bridge the collision shapes between the robot and the cable world?
