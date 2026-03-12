---
layout: default
title: PhysTwin 설치 방법 (uv)
parent: 나의 공부 게시판
---

**PhysTwin**은 NVIDIA Warp와 Gaussian Splatting 기술을 결합한 복잡한 프로젝트입니다. 앞서 설정한 **CUDA 12.1** 환경과 `uv`를 사용하여 빌드하는 가장 확실한 가이드를 정리해 드립니다.

---

## 1. 빌드 전 필수 체크리스트

빌드를 시작하기 전에 터미널 환경이 다음과 같은지 반드시 확인하세요.

| 항목 | 확인 명령 | 기대 결과 |
| --- | --- | --- |
| **CUDA 버전** | `nvcc --version` | `release 12.1, V12.1.XX` |
| **환경 변수** | `echo $CUDA_HOME` | `/usr/local/cuda-12.1` |
| **Python 버전** | `python --version` | `Python 3.10.X` |

---

## 2. PhysTwin 단계별 빌드 과정

### **Step 1: 저장소 복제 및 서브모듈 업데이트**

PhysTwin은 외부 라이브러리(Gaussian Splatting 등)를 서브모듈로 포함합니다. 반드시 `--recursive` 옵션을 써야 합니다.

```bash
git clone --recursive https://github.com/YuexinMa/PhysTwin.git
cd PhysTwin

```

### **Step 2: `uv` 가상환경 구축 및 기본 패키지 설치**

```bash
# 가상환경 생성 및 활성화
uv venv .venv --python 3.10
source .venv/bin/activate

# PyTorch 2.1.2 (CUDA 12.1용) 설치 - 안정성이 가장 높음
uv pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121

# 기타 일반 의존성 설치
uv pip install -r requirements.txt

```

### **Step 3: NVIDIA Warp 설치**

PhysTwin의 물리 시뮬레이션 핵심인 **Warp**를 설치합니다.

```bash
uv pip install warp-lang

```

### **Step 4: CUDA 확장 모듈 빌드 (핵심 단계)**

많은 사용자들이 "라이브러리가 없다"고 고생하는 구간입니다. 소스 코드를 직접 컴파일해야 합니다.

1. **diff-gaussian-rasterization 빌드:**
```bash
cd submodules/diff-gaussian-rasterization
uv pip install -e .
cd ../..

```


2. **simple-knn 빌드:**
```bash
cd submodules/simple-knn
uv pip install -e .
cd ../..

```



---

## 3. 빌드 중 발생할 수 있는 주요 에러 및 해결법

### **에러 1: `FileNotFoundError: [Errno 2] No such file or directory: 'nvcc'**`

* **원인:** 시스템이 `nvcc`를 어디서 찾을지 모르는 상태입니다.
* **해결:** 빌드 명령을 내리기 전 아래 명령어를 다시 입력하세요.
```bash
export PATH=/usr/local/cuda-12.1/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.1

```



### **에러 2: `RuntimeError: Error compiling objects for extension**`

* **원인:** 현재 그래픽카드 아키텍처와 컴파일 설정이 맞지 않음.
* **해결:** `setup.py` 파일 내부에 `nvcc_args`에 본인의 GPU 아키텍처(예: RTX 4090이면 `-gencode=arch=compute_89,code=sm_89`)를 수동으로 추가해야 할 수도 있습니다.

---

## 4. 설치 확인

모든 설치가 끝났다면 아래 명령어로 핵심 라이브러리들이 잘 로드되는지 확인하세요.

```bash
python -c "import torch; import warp; import diff_gaussian_rasterization; print('All systems GO!')"

```

**`All systems GO!`** 문구가 출력되면 성공입니다.

---

> **참고:** PhysTwin은 실행 시 상당한 양의 VRAM을 요구합니다. RTX 3090/4090 급의 환경을 권장하며, 만약 메모리 부족(OOM) 에러가 발생하면 데이터셋의 해상도를 낮춰서 테스트해 보세요.

이제 데이터셋을 준비해서 첫 시뮬레이션을 돌려보시겠어요? 아니면 **특정 단계에서 빨간색 에러 로그**가 발생했나요? 로그를 보여주시면 바로 디버깅해 드릴게요.
