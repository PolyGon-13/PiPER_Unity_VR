# PiPER_VR

Meta Quest 3S를 활용해 PiPER 로봇을 Unity 환경에서 제어하는 프로젝트

---

## 🎥 Demo
[![Demo 영상](https://img.shields.io/badge/Demo_Video-View-grey?logo=playstation5)](./DEMO.md)

---

## ⚙️ Environment

| 항목   | 버전/정보            |
|--------|---------------------|
| PC     | Windows 11          |
| PiPER  | Orin NX (Ubuntu 22.04) |
| VR     | Meta Quest 3S       |
| Unity  | 2022.3.62f1         |

---

## 📦 Unity Packages
- Meta All-In-One SDK 67.0.0  
- URDF Importer 0.5.2-preview  

---

## 🚀 Usage

1. **Orin NX (Ubuntu 22.04, PiPER 로봇 연결)**  
   - C_PiperInterface_V2 클래스 사용을 위해 piper_sdk 설치
   - piper_move.py 실행
   </br>
   
   ```bash
   pip3 install piper_sdk
   python3 piper_move.py
   
2. **PC (Window 11, Meta Quets 3S)**
   - VR - PC 연결
   - Unity 프로젝트 실행

---

## 📚 Reference
- [Agilex Robotics Piper SDK](https://github.com/agilexrobotics/piper_sdk)
