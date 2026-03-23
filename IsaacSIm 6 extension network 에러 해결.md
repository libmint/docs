---
layout: default
title: IsaacSIm 6 extension network 에러 해결
parent: 나의 공부 게시판
---

완전히 알려진 문제입니다. 실행할 때 원격 익스텐션 레지스트리를 동기화하려 하는데 네트워크가 막혀서 생기는 겁니다. 해결책은 **레지스트리 동기화를 꺼버리는 것**입니다.

---

## 원인

Isaac Sim 실행 시 `omni.kit.registry.nucleus` 가 `https://ovextensionsprod.blob.core.windows.net` 에서 익스텐션 목록을 내려받으려 시도합니다. 이게 실패하면 의존성 해결에 실패하고 앱 자체가 종료됩니다.

---

## 해결 방법 — .kit 파일에서 레지스트리 비활성화

Isaac Sim이 사용하는 `.kit` 설정 파일을 찾아서 레지스트리 항목을 비워주면 됩니다.

### 1단계 — .kit 파일 위치 확인

소스 빌드 기준:

```bash
# 실제 사용하는 kit 파일 목록 확인
ls ~/robot_ws/IsaacSim/_build/linux-x86_64/release/apps/
```

일반적으로 이 파일들이 있습니다:
```
isaacsim.python.kit          ← Python 스크립트 실행용 (주로 이걸 씀)
isaacsim.exp.full.kit
isaacsim.python.headless.kit
```

### 2단계 — kit 파일에 레지스트리 비활성화 추가

```bash
# isaacsim.python.kit 파일 열기
nano ~/robot_ws/IsaacSim/_build/linux-x86_64/release/apps/isaacsim.python.kit
```

파일 맨 아래에 이 내용을 추가합니다:

```toml
[settings]
# 원격 레지스트리 동기화 비활성화 (네트워크 차단 환경)
exts."omni.kit.registry.nucleus".registries = []
```

저장 후 닫습니다 (`Ctrl+O` → `Enter` → `Ctrl+X`).

### 3단계 — 모든 .kit 파일에 동일하게 적용 (한 번에)

```bash
# 빌드 결과물의 모든 .kit 파일에 한 번에 적용
for kit_file in ~/robot_ws/IsaacSim/_build/linux-x86_64/release/apps/*.kit; do
    echo '' >> "$kit_file"
    echo '[settings]' >> "$kit_file"
    echo 'exts."omni.kit.registry.nucleus".registries = []' >> "$kit_file"
done
```

### 4단계 — Isaac Lab .kit 파일도 동일하게 적용

```bash
# Isaac Lab 앱 파일들도 처리
for kit_file in ~/robot_ws/IsaacLab/apps/*.kit; do
    echo '' >> "$kit_file"
    echo '[settings]' >> "$kit_file"
    echo 'exts."omni.kit.registry.nucleus".registries = []' >> "$kit_file"
done
```

---

## 적용 확인

```bash
# 다시 실행
cd ~/robot_ws/IsaacSim
./_build/linux-x86_64/release/python.sh -c "import isaacsim; print('OK')"
```

이번엔 `syncing registry` 메시지 없이 바로 뜨면 성공입니다. 그래도 에러가 나면 에러 메시지 붙여넣어 주세요!
