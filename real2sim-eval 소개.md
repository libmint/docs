---
layout: default
title: real2sim-eval 구조, 핵심기술, 구현 소개
parent: 나의 공부 게시판
---

`kywind/real2sim-eval` 프로젝트는 "Real-to-Sim Robot Policy Evaluation with Gaussian Splatting Simulation of Soft-Body Interactions" 논문의 공식 오픈소스 구현체입니다. 이 프로젝트는 로봇이 로프나 인형 같은 연체(Soft-Body)를 다루는 제어 정책(Policy)을 시뮬레이션 환경에서 정밀하게 평가하기 위해 개발된 프레임워크입니다.

프로젝트의 핵심 구조, 적용 기술, 동작 방식 및 실제 구현 방법은 다음과 같습니다.

### 1. 프로젝트 구조

리포지토리는 크게 자산 생성, 설정 관리, 시뮬레이션 및 평가 모듈로 나뉘어 있습니다.

* **`assets/`**: 가우시안 스플래팅(GS) 스캔 데이터의 시각화, 로봇 모델과의 위치 정렬(ICP 알고리즘 등 활용), 색상 정렬(Color Alignment)을 수행하는 스크립트가 포함되어 있습니다.
* **`cfg/`**: Hydra 라이브러리를 기반으로 시뮬레이션 환경(`env`), 가우시안 스플래팅(`gs`), 물리 엔진(`physics`), 로봇 정책(`policy`), 렌더러(`renderer`)에 대한 모든 하이퍼파라미터와 경로를 관리합니다.
* **`experiments/`**: 단일/다중 GPU 환경에서의 정책 평가(`eval_policy.py`), 키보드 원격 제어(`keyboard_teleop.py`), 모션 리플레이(`replay.py`) 및 성공률 계산 스크립트가 포함된 실행 모듈입니다.
* **`policy/`**: 서브모듈(Submodule) 형태로 연결되어 있으며, 로봇의 행동 정책을 학습(LeRobot 또는 OpenPI 스타일)하는 데 사용됩니다.
* **`sim/` & `third-party/**`: 시뮬레이션 엔진의 핵심 부분과 깊이(depth) 값을 처리할 수 있도록 수정된 가우시안 래스터라이저(`diff-gaussian-rasterization-w-depth`) 등 외부 종속성 코드가 위치합니다.

### 2. 적용된 핵심 기술

* **3D 가우시안 스플래팅 (3DGS, Gaussian Splatting)**: 실제 환경과 객체를 사진 기반으로 스캔하여 매우 사실적인 3D 렌더링 환경을 구축하는 기술입니다.  시뮬레이션과 현실 간의 시각적 격차(Sim-to-Real Gap)를 줄이는 데 사용됩니다.
* **PhysTwin**: 형태가 쉽게 변하는 연체(Deformable objects)의 물리적 상호작용을 계산하는 물리 엔진입니다. 가우시안 스플래팅으로 렌더링된 객체가 로봇의 물리적 조작에 따라 사실적으로 변형되도록 만듭니다.
* **uv**: 기존 Conda를 대체하여 파이썬 패키지 및 가상 환경을 매우 빠르게 구축할 수 있는 최신 프로젝트 매니저입니다.
* **PyTorch 2.7.1 & CUDA 12.8**: 딥러닝 기반의 로봇 정책 추론 및 시뮬레이션 연산을 처리하기 위한 프레임워크입니다.
* **Scaniverse & SuperSplat**: 현실 객체를 3D 스캔(Scaniverse)하고, 추출된 PLY 파일의 위치, 회전, 크기를 편집(SuperSplat)하여 로봇의 URDF 모델과 정렬하기 위해 사용된 도구입니다.

### 3. 동작 방식

이 프레임워크는 **현실 데이터 획득 -> 시뮬레이션 자산 변환 -> 물리 연동 -> 정책 평가**의 흐름으로 동작합니다.

1. **자산 스캔 및 정렬**: 스마트폰(Scaniverse 등)으로 로프, 인형, T자 블록 등을 스캔하여 가우시안 데이터(PLY)를 생성합니다. 이후 로봇 모델의 원점과 방향에 맞게 데이터를 정렬하고, 실제 카메라와 시뮬레이션 카메라 간의 색상 차이를 2차 RGB 변환을 통해 보정합니다.
2. **물리 트윈(PhysTwin) 매핑**: 스캔된 시각적 객체에 PhysTwin 물리 모델을 결합합니다. 연체의 경우 형태 변화를 계산하며, 강체(Rigid object)의 경우 메쉬 내부에 촘촘하고 강성(Stiffness)이 높은 스프링을 샘플링하여 단단한 물체처럼 동작하게 만듭니다.
3. **상호작용 및 렌더링**: 로봇이 물체를 조작할 때, PhysTwin 엔진이 물체의 변형을 실시간으로 계산하고, 변형된 좌표값을 바탕으로 가우시안 스플래팅 렌더러가 카메라 뷰를 새롭게 그려냅니다.
4. **정책 평가**: 현실에서 학습된 로봇 제어 정책이 시뮬레이션에서 제공하는 시각(렌더링) 데이터를 입력받아 행동을 결정합니다. 엔진은 이 행동에 따른 결과를 시뮬레이션하여 최종 작업 성공 여부를 판별합니다.

### 4. 실제 구현 및 실행 방법

프로젝트는 모듈화된 파이썬 스크립트와 터미널 명령어를 통해 구동되도록 구현되었습니다.

* **환경 구축**: `uv venv`를 통해 가상 환경을 생성하고, PyTorch와 하위 모듈(`policy`), 수정한 `urdfpy` 등을 설치합니다.
* **데이터 로드**: Hugging Face에서 사전 스캔된 GS 자산(`gs-scans`), 학습된 물리 모델(`phystwin-rope` 등), 그리고 제어 정책 가중치(Checkpoints)를 다운로드하여 `log/` 디렉토리에 배치합니다.
* **실행 (명령어 기반)**: Hydra 설정을 오버라이드하여 터미널에서 직접 실행합니다. 예를 들어, 로프 다루기 작업을 xarm_gripper 환경에서 평가하려면 다음과 같이 실행합니다.
```bash
CUDA_VISIBLE_DEVICES=0 python experiments/eval_policy.py \
  gs=rope \
  env=xarm_gripper \
  physics.ckpt_path=log/phystwin/rope \
  physics.case_name=rope_0001 \
  policy.inference_cfg_path=policy/configs/inference/insert_rope.json \
  policy.checkpoint_path=log/policy_checkpoints/...

```


* **분산 처리 기능**: `eval_policy_parallel.py`를 사용하여 단일 노드 내 여러 GPU에 평가 에피소드를 분산시켜 빠른 대규모 검증이 가능하도록 구현되어 있습니다.
* **수동 검증**: `keyboard_teleop.py` 스크립트를 통해 사용자가 직접 키보드로 로봇을 제어하며 가우시안 스플래팅 및 PhysTwin 연체 물리 엔진이 정상적으로 결합되어 동작하는지 실시간으로 확인할 수 있습니다.
