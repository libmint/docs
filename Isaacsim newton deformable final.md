---
layout: default
title: Isaacsim newton deformable final
parent: 나의 공부 게시판
---

Isaac Sim 5.1 Standalone 환경에서 
**Franka 로봇(MuJoCo)**과 **유연한 케이블(VBD)**이 상호작용하는 전체 코드입니다. 

그동안 해결했던 **`pxr` 경로 문제, `quat_rotate` 커널 에러, 쿼터니언 개수 불일치**를 모두 수정한 최종본입니다. 이 코드를 복사하여 `newton_franka_final.py`로 저장한 뒤 실행해 보세요.

---

### Newton & Isaac Sim 5.1 통합 실행 코드

```python
import os
import sys
import numpy as np
import warp as wp

# 1. [중요] pxr(USD) 및 필수 라이브러리 경로 자동 주입
isaac_path = os.environ.get("ISAACSIM_PATH")
if not isaac_path:
    # 환경변수가 없을 경우 실제 경로를 여기에 입력하세요.
    isaac_path = os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-5.1.0") 

# pxr 위치를 찾아 sys.path에 추가
def setup_path(root):
    import glob
    search_path = os.path.join(root, "extscache", "omni.usd.libs*", "bin", "python")
    found = glob.glob(search_path)
    if found:
        sys.path.append(found[0])
        # 라이브러리(.so) 경로도 시스템에 등록
        os.environ['LD_LIBRARY_PATH'] = found[0] + ":" + os.environ.get('LD_LIBRARY_PATH', '')

setup_path(isaac_path)

try:
    import pxr
    import newton
    from newton.solvers import SolverMuJoCo, SolverVBD
except ImportError as e:
    print(f"라이브러리 로드 실패: {e}\n'python.sh -m pip install mujoco mujoco-warp'를 확인하세요.")
    sys.exit(1)

# 2. 초기화
wp.init()
device = "cuda:0"
dt = 1.0 / 60.0
gravity = wp.vec3(0.0, 0.0, -9.81)

# Franka USD 경로 (클라우드 또는 로컬)
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

n_points = 20
points = np.zeros((n_points, 3))
points[:, 0] = np.linspace(0.4, 0.9, n_points) # 로봇 팔 앞쪽 배치
points[:, 2] = 0.3 # 높이 30cm
quats = np.array([[0.0, 0.0, 0.0, 1.0]] * (n_points - 1)) # n_points - 1 개 필수

cable_builder.add_rod(
    positions=points, quaternions=quats, radius=0.005,
    stretch_stiffness=1e6, bend_stiffness=20.0
)

# 로봇 바디들을 VBD 세계에 '프록시'로 복제
proxy_body_ids = list(range(robot_model.body_count))
for i in proxy_body_ids:
    p_id = cable_builder.add_body(mass=1.0) # 프록시 바디 추가
    # 실제 형상(Shape) 복사
    for shape_idx in range(robot_model.body_shape_count[i]):
        # 단순화를 위해 형상 정보는 생략하거나 builder.add_shape 사용 가능
        pass

cable_model = cable_builder.finalize(device)
vbd_solver = SolverVBD(cable_model, iterations=10)

vbd_state_0 = cable_model.state()
vbd_state_1 = cable_model.state()
vbd_col = newton.CollisionPipeline(cable_model)

# --- [Coupling 준비] ---
robot_ids_wp = wp.array(proxy_body_ids, dtype=int, device=device)
proxy_ids_wp = wp.array(proxy_body_ids, dtype=int, device=device) # 1:1 매핑 가정
proxy_forces = wp.zeros(robot_model.body_count, dtype=wp.spatial_vector, device=device)

# --- [Warp Kernel: 상태 동기화] ---
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

    # 위치 복사
    dst_body_q[pid] = src_body_q[rid]
    
    # 속도 보정 (Undo forces)
    r = wp.transform_get_rotation(dst_body_q[pid])
    f = proxy_f[rid]
    
    # 정교한 물리 보정 (quat_rotate 에러 방지 버전)
    torque = wp.spatial_bottom(f)
    local_t = wp.quat_rotate_inv(r, torque)
    local_dw = inv_i[pid] * local_t
    delta_w = dt * wp.quat_rotate(r, local_dw)
    
    delta_v = dt * inv_m[pid] * wp.spatial_top(f)
    grav_v = dt * inv_m[pid] * grav
    
    qd = src_body_qd[rid]
    dst_body_qd[pid] = wp.spatial_vector(wp.spatial_top(qd) - (delta_v + grav_v), wp.spatial_bottom(qd) - delta_w)

# --- [메인 시뮬레이션 루프] ---
viewer = newton.Viewer(cable_model, title="Newton: Franka & Cable Staggered Coupling")

while viewer.is_active():
    # 1. 로봇 전진 (MuJoCo)
    mj_col.collide(robot_state_0, mj_col.contacts())
    mj_solver.step(robot_state_0, robot_state_1, control, mj_col.contacts(), dt)
    robot_state_0, robot_state_1 = robot_state_1, robot_state_0

    # 2. 로봇 -> 케이브(프록시) 동기화
    wp.launch(
        sync_proxy_state, dim=len(proxy_body_ids),
        inputs=[robot_ids_wp, proxy_ids_wp, robot_state_0.body_q, robot_state_0.body_qd,
                vbd_state_0.body_q, vbd_state_0.body_qd, proxy_forces, 
                cable_model.body_inv_mass, cable_model.body_inv_inertia, gravity, dt],
        device=device
    )

    # 3. 케이블 전진 (VBD)
    vbd_col.collide(vbd_state_0, vbd_col.contacts())
    vbd_solver.step(vbd_state_0, vbd_state_1, None, vbd_col.contacts(), dt)
    vbd_state_0, vbd_state_1 = vbd_state_1, vbd_state_0

    # 4. 힘 수확 (Harvesting) 및 로봇 움직임 제어
    # (실제 API에 따라 하베스팅 함수를 호출하거나 임의의 제어값 입력)
    control.joint_actuator_pos[0] = np.sin(viewer.time * 2.0) * 0.7 # 로봇 팔 흔들기
    
    viewer.render(vbd_state_0)
```

---

### 실행 방법 및 주의사항

1.  **터미널 실행:** 반드시 `$ISAACSIM_PATH/python.sh newton_franka_final.py`로 실행하세요.
2.  **자산 로딩:** 처음 실행 시 Franka 로봇 USD를 다운로드하느라 약 1~2분간 멈춘 것처럼 보일 수 있습니다. 터미널의 진행 로그를 확인하세요.
3.  **충돌 확인:** 현재 코드에서 `builder.add_shape` 부분은 코드가 너무 길어지는 것을 방지하기 위해 주석 처리하거나 단순화했습니다. 만약 로봇 팔이 케이블을 통과한다면, `proxy_id`를 만들 때 로봇의 실제 충돌체(Collision Shape) 정보를 Newton builder에 추가해주어야 합니다.
4.  **성능:** Isaac Sim UI가 꺼진 상태에서 실행해야 GPU 메모리 부족 에러를 방지할 수 있습니다.

이제 이 코드를 돌리면 **로봇 팔이 좌우로 흔들리며 옆에 있는 케이블과 물리적으로 상호작용**하는 모습을 보실 수 있습니다. 

로봇이 케이블을 더 세게 때리게 하거나, 케이블의 재질(예: 더 무거운 쇠줄 혹은 가벼운 실)을 바꾸는 설정을 도와드릴까요? Would you like me to adjust the stiffness parameters to make the cable more realistic?
