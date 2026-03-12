---
layout: default
title: cuda 12.1 설치 방법
parent: 나의 공부 게시판
---

현재 시스템에 **CUDA 12.8**이 설치되어 있어도 **12.1**을 추가로 설치하여 병행해서 사용할 수 있습니다. 리눅스(Ubuntu/WSL2) 환경을 기준으로 가장 깔끔하고 충돌 없는 설치 방법을 단계별로 설명해 드릴게요.

핵심은 **"드라이버는 그대로 두고, 툴킷만 추가 설치하여 경로(Path)를 바꿔가며 쓰는 것"**입니다.

---

## 1. CUDA 12.1 설치 준비 (Ubuntu/WSL2 기준)

기존의 12.8을 지울 필요가 전혀 없습니다. `/usr/local/` 아래에 버전별로 폴더가 생성되기 때문입니다.

### **Step 1: NVIDIA 리포지토리 설정**

터미널을 열고 다음 명령어를 순서대로 입력하여 12.1을 설치할 수 있는 준비를 합니다.

```bash
# 기존 키링 제거 (안전성을 위해)
sudo apt-key del 7fa2af80

# CUDA 12.1용 키링 다운로드 및 설치
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

# 리포지토리 업데이트
sudo apt update

```

### **Step 2: CUDA Toolkit 12.1 설치**

여기서 **주의**할 점은 `sudo apt install cuda`가 아니라 **`cuda-toolkit-12-1`**을 설치해야 한다는 것입니다. 그래야 이미 설치된 최신 드라이버(12.8용)와 충돌하지 않습니다.

```bash
sudo apt install cuda-toolkit-12-1

```

---

## 2. 버전 스위칭 (12.8 → 12.1로 바꾸기)

설치가 끝나면 `/usr/local/cuda-12.1` 폴더가 생겼을 겁니다. 이제 터미널이 이 폴더를 바라보게 설정해야 합니다.

### **방법 A: 현재 터미널 세션에서만 임시로 바꾸기 (권장)**

프로젝트 빌드 시에만 잠깐 바꾸고 싶을 때 사용합니다.

```bash
export CUDA_HOME=/usr/local/cuda-12.1
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# 확인
nvcc --version
# 결과에 "release 12.1"이 뜨면 성공입니다!

```

### **방법 B: 영구적으로 바꾸기**

`~/.bashrc` 또는 `~/.zshrc` 파일 맨 밑에 아래 내용을 추가합니다.

```bash
# 기존의 /usr/local/cuda 관련 설정을 주석처리하거나 아래로 대체
export CUDA_HOME=/usr/local/cuda-12.1
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

```

---

## 3. `uv` 환경에서 PhysTwin/PyTorch3D 빌드하기

이제 `nvcc`가 12.1을 가리키고 있으니, `uv`를 사용해 프로젝트를 빌드하면 "라이브러리를 찾을 수 없다"는 에러가 해결됩니다.

### **Step 1: 깨끗한 가상환경 생성**

```bash
uv venv .venv --python 3.10
source .venv/bin/activate

```

### **Step 2: CUDA 12.1용 PyTorch 설치**

```bash
uv pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

```

### **Step 3: PhysTwin/PyTorch3D 빌드**

이제 소스 코드가 있는 디렉토리에서 빌드 명령을 내리면, `uv`가 현재 터미널의 `nvcc`(12.1)를 사용하여 컴파일을 진행합니다.

```bash
# 예: PyTorch3D 설치 시
uv pip install "git+https://github.com/facebookresearch/pytorch3d.git"

```

---

## 4. 왜 12.1인가요? (2026년 기준 팩트 체크)

* **호환성:** `PhysTwin`이나 `Gaussian Splatting` 계열 오픈소스들은 2023~2024년에 작성된 코드가 많습니다. 이 코드들은 CUDA 12.1의 메모리 레이아웃에 최적화되어 있어, 12.8 같은 최신 버전에서는 문법 에러나 "Architecture mismatch"가 자주 발생합니다.
* **안정성:** PyTorch 공식 바이너리 중 가장 검증된 버전이 12.1/12.4 기반입니다.

### **정리하자면**

1. **`cuda-toolkit-12-1`**만 추가로 설치한다.
2. **`export PATH`**를 통해 터미널이 12.1 폴더를 보게 만든다.
3. 그 상태에서 **`uv pip install`**을 실행한다.

이제 `nvcc --version`에서 12.1이 나오시나요? 설치 중에 막히는 명령어가 있다면 바로 말씀해 주세요! 구체적으로 어떤 에러가 뜨는지 알려주시면 더 확실히 잡아드릴 수 있습니다.

**다음으로 `PhysTwin`의 구체적인 빌드 에러를 함께 해결해 볼까요?**
