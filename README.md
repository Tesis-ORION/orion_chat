# Orion Chat 🤖💬

![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange) ![Python](https://img.shields.io/badge/Python-3.12-yellow)

## 🚀 Overview
**Orion Chat** is a ROS 2 Jazzy package that orchestrates speech‑to‑text (STT), chat engine, and text‑to‑speech (TTS) nodes using a single launch file. It runs on Ubuntu 24.04 with support for Ignition Harmonic simulation, integrating Whisper for STT, OLLAMA for language processing, and gTTS for TTS, enabling seamless human–robot interaction.

## ⚙️ Prerequisites
- **OS:** Ubuntu 24.04 LTS  
- **ROS 2:** Jazzy Jalisco (`ros-jazzy-desktop`)  
- **Ignition Gazebo:** Ignition Harmonic  
- **Tools:** `colcon`, `rosdep`  

## 🛠️ Installation

```bash
# Clone and prepare the workspace
mkdir -p ros2_ws/src && mv * ros2_ws/src  
git clone https://github.com/Tesis-ORION/orion_chat.git  
cd ros2_ws

# Install dependencies
rosdep update  
rosdep install --from-paths src --ignore-src -r -y  
cd orion_chat
./install_apt.sh
pip install -r requirements.txt --break-system-packages

# Build and source environment
colcon build --packages-select orion_chat  
source install/setup.bash
```

## 📂 Workspace Structure

```
ros2_ws/
└── src/
    ├── orion_chat/            # Main package
    │   ├── launch/
    │   │   └── orion_launch.py  # Single launch for STT, Chat, TTS
    │   ├── orion_chat/         # Chat logic & prompt loader
    │   ├── stt_node.py         # STT node (Whisper)
    │   ├── tts_node.py         # TTS node (gTTS + pygame)
    │   ├── resource/           # JSON prompts and user data
    │   └── sounds/             # Activation audio (MP3)
    ├── install/                # colcon install output
    ├── build/                  # colcon build output
    └── log/                    # Build logs
```

## 💬 Nodes

### 🎙️ Speech‑to‑Text (STT)
- **Executable:** `orion_stt`  
- **Description:** Uses OpenAI Whisper to transcribe microphone audio into text.

### 🤖 Chat Engine
- **Executable:** `orion_chat`  
- **Description:** Processes text with an LLM via the OLLAMA API using `resource/orion_prompt.json`.

### 🗣️ Text‑to‑Speech (TTS)
- **Executable:** `orion_tts`  
- **Description:** Converts text responses into speech using gTTS and plays back via pygame.

## 🔧 Usage

🔄 **Launch STT, chat engine, and TTS modules only**  
```bash
# in a separated terminal
ros2 run orion_chat audio_recorder
# in a separated terminal
ros2 launch orion_chat orion_launch.py
```

🔄 **Launch STT, chat engine, and TTS modules with web interface (requires orion_web_interface package)**  
```bash
ros2 launch orion_chat orion_launch.py
```

**Launch default simulation**  
```bash
ros2 launch orion_chat orion_gz_launch_def.py
```

**Launch with ORION simulation (requires orion_gz package)**  
```bash
ros2 launch orion_chat orion_gz_launch.py
```

## 📘 Configuration
- `resource/conversation_config.json` – Prompt template  
- `resource/movement_config.json` – Movement RAG settings  

## 🤝 Contributing
1. Fork the repository  
2. Create a branch (`feature/...`)  
3. Commit your changes and add tests  
4. Open a Pull Request against `main`

## 📜 License
**MIT License** – Permissive and ensures attribution. See [LICENSE](LICENSE).
