# Vocal Robotics — 2015 Thesis

> **A spoken-language dialogue platform for mobile robot control**, developed as a Master's thesis project (2015) by Oliver Kalmend.

The system lets a user speak natural-language commands to a mobile robot. Speech is captured from a microphone, recognised with a Kaldi-based acoustic model, interpreted by a Prolog dialogue engine, and executed as real robot actions (navigation, pick-and-place). The robot's responses are spoken back through a Festival TTS synthesiser. Dialogue rules were developed primarily in the **Estonian language**.

---

## Architecture

```
Microphone
    │
    ▼
Voice Activity Detection
  (Speex VAD  or  CMU PocketSphinx VAD)
    │
    ▼
Speech Recognition
  (Kaldi online decoder via GStreamer plugin)
    │
    ▼
Prolog Dialogue Server  ──►  Robot Action Server
  (SWI-Prolog + prolog_server)   (chatbot_core C++ node)
                                      │
                         ┌────────────┴────────────┐
                         ▼                         ▼
                   Navigation              Pick / Place
                (ROS move_base)            (actionlib)

Text-to-Speech  (Festival synthesiser)
    │
    ▼
Audio output
```

A **Qt/rqt GUI** (`chatbot_gui`) shows live status and conversation logs.  
A **Stage simulator** with a TurtleBot model is included for development without physical hardware.

---

## Technology Stack

| Layer | Technology |
|---|---|
| Robotics middleware | [ROS Indigo](http://wiki.ros.org/indigo) (catkin build system) |
| Primary language | C++ (ROS nodes), Python (GUI plugin) |
| Knowledge / Dialogue | [SWI-Prolog](https://www.swi-prolog.org/) |
| Speech recognition (ASR) | [Kaldi](https://kaldi-asr.org/) online decoder via `kaldinnet2onlinedecoder` GStreamer element |
| Audio pipeline | [GStreamer](https://gstreamer.freedesktop.org/) 1.0 |
| Voice activity detection | [Speex](https://www.speex.org/) VAD and [CMU PocketSphinx](https://cmusphinx.github.io/) |
| Text-to-speech (TTS) | [Festival](http://www.cstr.ed.ac.uk/projects/festival/) speech synthesis system |
| Audio compression | Speex codec |
| Navigation | ROS `move_base` + `actionlib` |
| Robot simulation | TurtleBot in ROS Stage |
| C++ utilities | Boost (asio, format) |
| Containerisation | Docker (base image: `ros:indigo`) |

---

## Repository Layout

```
src/
  chatbot/                   # Core chatbot node (C++) — orchestrates the whole pipeline
  chatbot_gui/               # rqt GUI plugin (Python) — status + conversation log
  chatbot_simulator/         # TurtleBot / Stage simulation launch files
  speech_recognition_simple/ # Kaldi + GStreamer ASR node (C++)
  speech_recognition_speex/  # Speex-based ASR node (C++)
  capture_audio/             # Raw audio capture node
  capture_vad_speex/         # Voice activity detection (Speex)
  capture_vad_sphinx/        # Voice activity detection (PocketSphinx)
  play_audio/                # Audio playback node
  play_audio_speex/          # Speex audio playback
  synth_festival/            # Festival TTS ROS node (C++)
  prolog_server/             # SWI-Prolog ROS server (nodelets)
  prolog_swi/                # SWI-Prolog C++ bindings
  prolog_common/             # Shared Prolog utilities
  prolog_msgs/               # Prolog ROS message definitions
  prolog_serialization/      # Prolog term serialization
  robot_prolog_connection/   # Bridge between chatbot and Prolog server
  roscpp_nodewrap/           # roscpp node wrapper library (3rd-party)
prolog_files/                # Prolog dialogue rules (Estonian language)
deps/                        # Pre-built Kaldi models + GStreamer plugin + config
docker/                      # Dockerfile (ros:indigo base)
```

---

## Getting Started

### Prerequisites

- Ubuntu 14.04 (Trusty)
- ROS Indigo (`ros-indigo-desktop-full`)
- SWI-Prolog (`swi-prolog`)
- Festival TTS (`festival`, `festival-dev`)
- GStreamer 1.0 development libraries
- Kaldi acoustic model files (placed in `deps/`)

### Build

```bash
# From the workspace root
catkin_make
source devel/setup.bash
```

### Prepare acoustic models

```bash
deps/prepare-models.sh
# Edit deps/conf/ivector_extractor.fixed.conf to set absolute paths
```

### Run

**Full system (with TurtleBot simulator):**
```bash
roslaunch chatbot chatbot.launch
```

**Speech recognition only (quick test):**
```bash
source devel/setup.bash
roslaunch chatbot test_sr_simple.launch
```

**Raw GStreamer voice recognition test (no ROS):**
```bash
./voice_recognition_test.sh
```

All available launch configurations live in `src/chatbot/launch/`.

---

## Notes

- Dialogue rules in `prolog_files/` are written in **Estonian**.
- Acoustic models in `deps/` target English speech (Kaldi English model).
- The Docker image (`docker/Dockerfile`) is a stub based on `ros:indigo` and is not fully configured.
- This is an archival research repository from 2015; dependencies (ROS Indigo, Ubuntu 14.04) are end-of-life.
