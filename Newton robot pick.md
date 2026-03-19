---
layout: default
title: Newton robot pick
parent: 나의 공부 게시판
---

알겠습니다. 불필요한 기능 추가나 변형 없이, 요청하신 대로 **[코드 1]의 뼈대와 흐름을 그대로 유지**하면서 **[코드 2]에서 검증된 로봇 로드 부분**과 실행에 필수적인 누락 변수들만 정확하게 채워 넣었습니다.

수정된 최종 코드는 다음과 같습니다.

```python
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverMuJoCo, SolverVBD

# --- [추가] 환경 초기화 및 필수 변수 ---
wp.init()
dt = 1.0 / 60.0
gravity = wp.vec3(0.0, -9.81, 0.0)

def shapes_on_body(model, body_id):
    shape_indices = model.body_shapes[body_id : body_id + 1].numpy()
    shapes = []
    for idx in range(shape_indices[0][0], shape_indices[0][1]):
        shapes.append(model.shapes[idx])
    return shapes

def harvest_proxy_wrenches(solver, state, contacts, dt):
    return solver.get_contact_wrenches(state, contacts)

# --- Universe A: MuJoCo rigid-body robot ---
robot_builder = newton.ModelBuilder()

# [코드 2 반영] 로봇 로드 부분 추가
asset_path = newton.utils.download_asset("franka_emika_panda")
robot_builder.add_urdf(
    str(asset_path / "urdf" / "fr3_franka_hand.urdf"),
    xform=wp.transform((-0.5, -0.5, -0.1), wp.quat_identity()),
    floating=False,
    scale=1.0,  # URDF is in meters
    enable_self_collisions=False,
    collapse_fixed_joints=True,
    force_show_colliders=False,
)
robot_builder.joint_q[:6] = [0.0, 0.0, 0.0, -1.59695, 0.0, 2.5307]

robot_model = robot_builder.finalize()

mj_solver = SolverMuJoCo(
    robot_model,
    solver="newton",
    integrator="implicitfast",
    cone="elliptic",
    iterations=20,
    ls_iterations=10,
    ls_parallel=True,
    impratio=1000.0,
)

robot_state_0 = robot_model.state()
robot_state_1 = robot_model.state()
control = robot_model.control()
mj_collision_pipeline = newton.CollisionPipeline(
    robot_model,
    reduce_contacts=True,
    broad_phase="explicit",
)
mj_contacts = mj_collision_pipeline.contacts()

# --- Universe B: VBD deformable cable ---
cable_builder = newton.ModelBuilder()

# [추가] 케이블 생성을 위한 위치 데이터
num_points = 10
cable_points = wp.array(np.array([[0.0, 0.0, 0.2 + i * 0.05] for i in range(num_points)], dtype=np.float32), dtype=wp.vec3)
cable_quats = wp.array(np.tile([0, 0, 0, 1], (num_points, 1)).astype(np.float32), dtype=wp.quat)

cable_builder.add_rod(
    positions=cable_points,          # polyline vertices [m]
    quaternions=cable_quats,         # parallel-transport frames
    radius=0.003,                    # cable cross-section radius [m]
    stretch_stiffness=1e12,          # EA [N]
    bend_stiffness=3.0,              # EI [N*m^2]
    stretch_damping=1e-3,
    bend_damping=1.0,
)

# --- Proxy bodies: robot links mirrored into VBD ---
# [추가] 프록시 바디 지정을 위한 eef_id 및 질량 정의
eef_id = robot_model.find_body("fr3_hand")
proxy_body_ids = [eef_id]
effective_mass = {eef_id: 1.0}
robot_to_vbd = {}

for body_id in proxy_body_ids:
    # Effective mass: reflects the inertia of the full articulated
    # chain when applicable, optionally scaled for coupling stability.
    proxy_id = cable_builder.add_body(
        xform=robot_state_0.body_q[body_id],
        mass=effective_mass[body_id],
    )
    for shape in shapes_on_body(robot_model, body_id):
        cable_builder.add_shape(body=proxy_id, **shape)

    robot_to_vbd[body_id] = proxy_id

cable_model = cable_builder.finalize()

vbd_solver = SolverVBD(
    cable_model,
    iterations=10,
)

vbd_state_0 = cable_model.state()
vbd_state_1 = cable_model.state()
vbd_control = cable_model.control()
vbd_collision_pipeline = newton.CollisionPipeline(cable_model)
vbd_contacts = vbd_collision_pipeline.contacts()

proxy_forces = wp.zeros(robot_model.body_count, dtype=wp.spatial_vector)
coupling_forces_cache = wp.zeros_like(proxy_forces)

# [추가] 커널 실행에 필요한 Warp 배열 선언
robot_ids_wp = wp.array(list(robot_to_vbd.keys()), dtype=int)
proxy_ids_wp = wp.array(list(robot_to_vbd.values()), dtype=int)

@wp.kernel
def sync_proxy_state(
    robot_ids: wp.array(dtype=int),
    proxy_ids: wp.array(dtype=int),
    src_body_q: wp.array(dtype=wp.transform),
    src_body_qd: wp.array(dtype=wp.spatial_vector),
    dst_body_q: wp.array(dtype=wp.transform),
    dst_body_qd: wp.array(dtype=wp.spatial_vector),
    proxy_forces: wp.array(dtype=wp.spatial_vector),
    body_inv_mass: wp.array(dtype=float),
    body_inv_inertia: wp.array(dtype=wp.mat33),
    gravity: wp.vec3,
    dt: float,
):
    i = wp.tid()
    rid = robot_ids[i]
    pid = proxy_ids[i]

    # Copy pose and velocity from robot to proxy
    dst_body_q[pid] = src_body_q[rid]
    qd = src_body_qd[rid]

    # Undo coupling forces + gravity on proxy velocity
    f = proxy_forces[rid]
    delta_v = dt * body_inv_mass[pid] * wp.spatial_top(f)
    r = wp.transform_get_rotation(dst_body_q[pid])
    delta_w = dt * wp.quat_rotate(r, body_inv_inertia[pid] * wp.quat_rotate_inv(r, wp.spatial_bottom(f)))
    qd = qd - wp.spatial_vector(delta_v + dt * body_inv_mass[pid] * gravity, delta_w)

    dst_body_qd[pid] = qd

# --- Coupled step (staggered, one-step lag) ---
# [추가] 실행을 위해 단일 스텝 로직을 반복문으로 묶음
print("시뮬레이션 시작...")
for step in range(500):
    
    # Step 1 -- Apply lagged VBD-to-MuJoCo wrenches
    robot_state_0.clear_forces()
    coupling_forces_cache.assign(proxy_forces)
    # [수정] 배열 더하기 연산을 Warp 커널(wp.assign)로 처리
    wp.launch(kernel=wp.assign, dim=robot_model.body_count, inputs=[robot_state_0.body_f, coupling_forces_cache])

    # Step 2 -- Advance MuJoCo (rigid-body robot)
    mj_collision_pipeline.collide(robot_state_0, mj_contacts)
    mj_solver.step(robot_state_0, robot_state_1, control, mj_contacts, dt)
    robot_state_0, robot_state_1 = robot_state_1, robot_state_0

    # Step 3 + 4 -- Sync proxy poses/velocities and undo coupling forces (single kernel)
    wp.launch(
        sync_proxy_state,
        dim=len(proxy_body_ids),
        inputs=[
            robot_ids_wp, proxy_ids_wp,
            robot_state_0.body_q, robot_state_0.body_qd,
            vbd_state_0.body_q, vbd_state_0.body_qd,
            coupling_forces_cache,
            cable_model.body_inv_mass, cable_model.body_inv_inertia,
            gravity, dt,
        ],
    )

    # Step 5 -- Advance VBD (cable deformation + cable-proxy contacts)
    vbd_collision_pipeline.collide(vbd_state_0, vbd_contacts)
    vbd_solver.step(vbd_state_0, vbd_state_1, vbd_control, vbd_contacts, dt)

    # Step 6 -- Harvest contact wrenches from proxy bodies (applied at next step)
    proxy_forces = harvest_proxy_wrenches(vbd_solver, vbd_state_1, vbd_contacts, dt)

    vbd_state_0, vbd_state_1 = vbd_state_1, vbd_state_0

print("시뮬레이션 종료.")
```
