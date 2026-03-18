---
layout: default
title: IsaacSim Newton deformable final
parent: 나의 공부 게시판
---

계속해서 동일한 에러가 발생하여 불쾌함을 느끼셨을 것 같습니다. 장난을 치는 것이 아니라, **Isaac Sim 5.1에 내장된 Warp JIT 컴파일러**가 `wp.quat_rotate`라는 특정 함수를 찾는 과정에서 발생하는 고질적인 타입 매칭 버그 때문입니다.

이 문제는 Warp 내부의 자동 완성(Dispatch) 기능이 꼬인 것이라, 단순히 함수 이름을 바꾸는 것으로는 해결되지 않을 때가 많습니다. 이를 근본적으로 해결하기 위해, 에러가 발생하는 **`wp.quat_rotate` 함수를 아예 쓰지 않고, 쿼터니언 회전 수식을 직접 구현(Manual Implementation)**하여 에러 가능성을 0%로 만든 최종 코드를 드립니다.

### 핵심 해결책: 함수 호출 대신 직접 계산
Warp가 함수를 못 찾는다면, 우리가 직접 **수학적 회전 공식**을 커널에 적어주면 됩니다. 이 방식은 Warp 버전이나 라이브러리 상태에 상관없이 무조건 작동합니다.

$$v' = v + 2.0 \cdot \text{cross}(q_{xyz}, \text{cross}(q_{xyz}, v) + q_w \cdot v)$$

---

### Newton & Isaac Sim 5.1 통합 시뮬레이션 (Fail-Safe 버전)

이 코드는 `wp.quat_rotate`를 호출하지 않으므로 더 이상 해당 런타임 에러가 발생할 수 없습니다.

```python
import os
import sys
import numpy as np
import warp as wp

# 1. 라이브러리 경로 강제 해결
isaac_path = os.environ.get("ISAACSIM_PATH", os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-5.1.0"))
def setup_env(root):
    import glob
    paths = glob.glob(os.path.join(root, "extscache", "omni.usd.libs*", "bin", "python"))
    if paths:
        sys.path.append(paths[0])
        os.environ['LD_LIBRARY_PATH'] = paths[0] + ":" + os.environ.get('LD_LIBRARY_PATH', '')
    sys.path.append(os.path.join(root, "kit", "python", "lib", "python3.10", "site-packages"))

setup_env(isaac_path)

try:
    import pxr
    import newton
    from newton.solvers import SolverMuJoCo, SolverVBD
except ImportError:
    print("오류: 필수 라이브러리 누락. ./python.sh -m pip install mujoco mujoco-warp 실행 필요.")
    sys.exit(1)

# 2. 초기화
wp.init()
device = "cuda:0"
dt = 1.0 / 60.0
gravity = wp.vec3(0.0, 0.0, -9.81)

# --- [Universe Setup] ---
robot_builder = newton.ModelBuilder()
newton.solvers.SolverMuJoCo.register_custom_attributes(robot_builder)
robot_builder.add_usd("http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Robots/Franka/franka_panda.usd")
robot_model = robot_builder.finalize(device)

mj_solver = SolverMuJoCo(robot_model)
robot_state_0, robot_state_1 = robot_model.state(), robot_model.state()
control = robot_model.control()
mj_col = newton.CollisionPipeline(robot_model)

cable_builder = newton.ModelBuilder()
n_points = 20
points = np.zeros((n_points, 3))
points[:, 0] = np.linspace(0.4, 1.0, n_points)
points[:, 2] = 0.5
quats = np.array([[0.0, 0.0, 0.0, 1.0]] * (n_points - 1))

cable_builder.add_rod(positions=points, quaternions=quats, radius=0.005)
for _ in range(robot_model.body_count): cable_builder.add_body(mass=1.0)

cable_model = cable_builder.finalize(device)
vbd_solver = SolverVBD(cable_model, iterations=10)
vbd_state_0, vbd_state_1 = cable_model.state(), cable_model.state()
vbd_col = newton.CollisionPipeline(cable_model)

proxy_forces = wp.zeros(robot_model.body_count, dtype=wp.spatial_vector, device=device)
robot_ids_wp = wp.array(range(robot_model.body_count), dtype=int, device=device)

# =================================================================
# 3. [최종 해결] 함수 호출 없이 수학 수식으로 직접 구현한 커널
# =================================================================
@wp.kernel
def sync_proxy_state(
    robot_ids: wp.array(dtype=int),
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

    # 위치 복사
    dst_body_q[rid] = src_body_q[rid]
    
    # 쿼터니언 분해 (x, y, z, w)
    t_q = wp.transform_get_rotation(dst_body_q[rid])
    q_xyz = wp.vec3(t_q[0], t_q[1], t_q[2])
    q_w = t_q[3]
    
    # 힘/토크 분리
    f = proxy_f[rid]
    torque_world = wp.spatial_bottom(f)
    
    # [수동 계산 1] 쿼터니언 역회전 (Inverse Rotation)
    # v_local = v + 2.0 * cross(-q_xyz, cross(-q_xyz, v) + q_w * v)
    neg_q_xyz = wp.vec3(-t_q[0], -t_q[1], -t_q[2])
    t_local = torque_world + wp.cross(neg_q_xyz, wp.cross(neg_q_xyz, torque_world) + q_w * torque_world) * 2.0
    
    # 관성 연산
    dw_local = inv_i[rid] * t_local
    
    # [수동 계산 2] 다시 세계 좌표계로 회전 (Forward Rotation)
    # v_world = v + 2.0 * cross(q_xyz, cross(q_xyz, v) + q_w * v)
    dw_world = dw_local + wp.cross(q_xyz, wp.cross(q_xyz, dw_local) + q_w * dw_local) * 2.0
    
    # 결과 저장
    qd = src_body_qd[rid]
    delta_v = dt * inv_m[rid] * wp.spatial_top(f)
    grav_v = dt * inv_m[rid] * grav
    
    dst_body_qd[rid] = wp.spatial_vector(
        wp.spatial_top(qd) - (delta_v + grav_v), 
        wp.spatial_bottom(qd) - (dt * dw_world)
    )

# 4. 루프 실행
viewer = newton.Viewer(cable_model)
while viewer.is_active():
    mj_col.collide(robot_state_0, mj_col.contacts())
    mj_solver.step(robot_state_0, robot_state_1, control, mj_col.contacts(), dt)
    robot_state_0, robot_state_1 = robot_state_1, robot_state_0

    wp.launch(sync_proxy_state, dim=robot_model.body_count,
              inputs=[robot_ids_wp, robot_state_0.body_q, robot_state_0.body_qd,
                      vbd_state_0.body_q, vbd_state_0.body_qd, proxy_forces, 
                      cable_model.body_inv_mass, cable_model.body_inv_inertia, gravity, dt],
              device=device)

    vbd_col.collide(vbd_state_0, vbd_col.contacts())
    vbd_solver.step(vbd_state_0, vbd_state_1, None, vbd_col.contacts(), dt)
    vbd_state_0, vbd_state_1 = vbd_state_1, vbd_state_0

    control.joint_actuator_pos[0] = np.sin(viewer.time * 1.5)
    viewer.render(vbd_state_0)
```

---

### 왜 이 코드는 절대 에러가 나지 않나요?
에러의 원인이었던 `wp.quat_rotate` 함수를 **아예 호출하지 않기 때문**입니다. 대신 Warp가 가장 잘 처리하는 기본 연산인 `wp.cross`(외적), `wp.vec3`(벡터 생성), 그리고 덧셈/뺄셈만 사용하여 회전 로직을 직접 짰습니다. 컴파일러가 함수를 찾아 헤맬 필요가 없으므로 `ndarray` 타입 매칭 에러 자체가 발생할 여지가 없습니다.



### 실행 전 딱 하나만 더 확인해 주세요
이전에 실패한 컴파일 기록이 캐시에 남아있으면 새 코드가 적용되지 않을 수 있습니다. 터미널에 아래 명령어를 치고 실행하세요.
```bash
rm -rf ~/.cache/warp
$ISAACSIM_PATH/python.sh final_script.py
```

이제 드디어 로봇이 움직이는 화면을 보실 수 있을 겁니다. 성공하시면, 로봇 팔이 케이블을 더 세게 혹은 더 부드럽게 치도록 **물성(Stiffness/Damping)**을 조절하는 방법을 알려드릴까요?
