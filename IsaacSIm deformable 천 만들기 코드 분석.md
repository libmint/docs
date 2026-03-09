---
layout: default
title: IsaacSIm deformable 천 만들기 코드 분석
parent: 나의 공부 게시판
---

제시해주신 코드는 Isaac Sim 5.1.0 환경에서 PhysX 엔진의 연체(Deformable Body) 시뮬레이션 시 흔히 발생하는 `loadFromDesc Cooking` 에러를 우회하고, GPU 기반의 천(Cloth) 시뮬레이션을 구현하기 위해 고도로 최적화된 트러블슈팅 스크립트입니다.

코드의 각 섹션에 대한 기술적 분석과 동작 원리는 다음과 같습니다.

## 코드 상세 분석

### 1. Beta 활성화 (carb.settings)

```python
carb.settings.get_settings().set("/physics/deformableBodyEnableBeta", True)

```

* **분석:** Isaac Sim(Omniverse)의 내부 설정 관리자인 `carb.settings`를 사용하여 Deformable Body 베타 기능을 강제로 활성화합니다. 최신 버전에서도 연체 물리 엔진은 베타 기능으로 취급되는 경우가 많아, 이 설정이 없으면 시뮬레이션 자체가 동작하지 않을 수 있습니다.

### 2. Physics Scene 설정

* **분석:** 물리 시뮬레이션의 전역 환경을 설정합니다.
* **핵심 속성:**
* `EnableGPUDynamicsAttr().Set(True)`: 연체 시뮬레이션은 연산량이 방대하므로 GPU 가속을 필수로 켭니다.
* `BroadphaseTypeAttr().Set("GPU")`: 충돌 감지의 첫 단계(Broadphase)를 GPU로 처리하도록 강제합니다.
* `SolverTypeAttr().Set("TGS")`: Temporal Gauss-Seidel 솔버를 사용합니다. 일반적인 솔버보다 반복 연산에 강해 천이나 부드러운 물체가 찢어지거나 관통하는 현상을 줄이고 시뮬레이션 안정성을 극대화합니다.



### 3. 바닥 생성 (Ground Plane)

* **분석:** 생성된 천이 떨어져서 충돌할 Z축 기반의 무한 평면을 생성합니다.

### 4. 수동 메시 생성 (Topology 직접 주입)

* **분석:** 이 섹션은 물리 엔진이 메쉬 데이터를 파싱(Cooking)할 때 발생하는 에러를 막기 위한 사전 작업입니다.
* **동작 원리:** 외부 3D 모델을 불러오거나 단순한 평면 생성 API를 쓰는 대신, 중첩 반복문을 통해 버텍스(Points)와 삼각면 인덱스(Triangle Indices)를 배열(Array) 형태로 직접 계산합니다.
* **목적:** 총 800개의 삼각형(20x20 해상도) 메쉬 데이터를 명확하게 정의함으로써, PhysX가 충돌체를 계산할 때 데이터 누락이나 구조적 결함으로 인해 실패하는 것을 방지합니다. 생성된 메쉬는 Z축으로 300 높이에 배치됩니다.

### 5. 물성치(Material) 생성 및 연결

* **분석:** 연체 시뮬레이션에 필요한 물리적 특성을 정의합니다.
* **파라미터:**
* `youngs_modulus=10000.0`: 물체의 뻣뻣함(강성)을 결정합니다. 값이 높을수록 덜 늘어납니다.
* `poissons_ratio=0.4`: 물체가 한 방향으로 늘어날 때 다른 방향으로 수축하는 비율입니다. (고무와 같은 탄성체의 특징)



### 6. Raw Sdf 속성 주입 (에러 해결의 핵심 로직)

이 스크립트의 가장 중요한 부분입니다. Isaac Sim의 고수준 API(`PhysxDeformableSurfaceBodyAPI` 등)가 메쉬를 Deformable 객체로 변환하는 과정에서 발생하는 내부 버그를 피하기 위한 우회(Bypass) 기법입니다.

* **API 우회:** `ApplyAPI("OmniPhysicsDeformableBodyAPI")`와 같이 문자열 기반으로 기본 API만 적용합니다.
* **Rest Shape 강제 주입:**
```python
prim.CreateAttribute("omniphysics:restShapePoints", Sdf.ValueTypeNames.Point3fArray).Set(Vt.Vec3fArray(pts))
prim.CreateAttribute("omniphysics:restTriVtxIndices", Sdf.ValueTypeNames.Int3Array).Set(Vt.Vec3iArray(tri_indices))

```


* 물리 엔진이 변형 전의 기본 형태(Rest Shape)를 스스로 계산하게 두지 않고, 4번 단계에서 만든 기하학적 데이터를 `Sdf(Scene Description Foundation)` 레벨의 속성으로 강제 생성하여 주입합니다. 이로 인해 Cooking 에러가 원천 차단됩니다.


* **안정성 파라미터 강제 설정:** `force_attr` 함수를 만들어 반복 연산 횟수(`solverPositionIterationCount=20`), 자기 자신과의 충돌 허용(`selfCollision=True`), 중력 적용 등의 필수 파라미터를 USD 노드에 직접 써넣습니다.

---

**요약하자면**, 이 코드는 Isaac Sim 5.1.0에서 고수준 API를 사용할 때 발생하는 불안정성을 해결하기 위해, 메쉬의 기하학적 데이터와 물리 속성을 USD 하위 레벨(Sdf)에서 직접 조립하여 엔진에 밀어 넣는 방식으로 성공적인 천 시뮬레이션을 구현한 스크립트입니다.
