# MotionCraft

[![Status](https://img.shields.io/badge/MVP-Free%20to%20Use-00cc77?style=flat-square)]()
[![Unreal Engine](https://img.shields.io/badge/Demo-Unreal%20Engine%205-black?style=flat-square&logo=unrealengine)](https://www.unrealengine.com/)
[![Contact](https://img.shields.io/badge/Contact-motioncraft.gen%40gmail.com-red?style=flat-square&logo=gmail)](mailto:motioncraft.gen@gmail.com)

**AI model for real-time motion generation. No animations. No keyframes. No mocap. No hard work.**

MotionCraft is a motion AI — it learns how to move by trying, failing, and improving. Instead of pre-recorded animations, movement is generated live in response to player input and the environment's state. The model runs locally on your machine — no cloud, no latency. We train the model. It handles the rest.

This is an **early MVP**. It demonstrates one thing: a skeleton character generating locomotion in Unreal Engine 5, entirely from the model. No animation assets, no blend trees, just the network output driving the joints. A proof of concept that the technology works inside a real-time game engine.

---

## See it in action

https://github.com/user-attachments/assets/c1aa82f1-53d5-417d-9d5e-8978b8f5e95e

---

## Getting started

### Prerequisites

| | |
|---|---|
| Unreal Engine | 5.6 or later |
| C++ environment | Visual Studio or JetBrains Rider |
| GPU | NVIDIA (tested on GTX 1080) |
| Docker | Latest stable |
| OS | Windows 11 *(only tested platform)* |

### Run the demo

1. Open the project in UE5 and rebuild.

2. Start the project from the UE5 editor.

3. Start the backend in a terminal at the project root:
```bash
docker compose up
```

> The backend uses Kafka to communicate with UE5. Make sure no other Kafka instance is running on your machine.

4. Use **WASD** to move the character. Motion generation runs in the backend — you'll see the character respond in the viewport in real time.

---

## Current limitations

- Forward and turning motion only.
- The model is trained on a specific skeleton. Custom characters require retraining.

## Why we're building this

3D games demand animation — motion capture, dedicated animators, and hard work. For solo developers and small teams, that cost often means staying in 2D or shipping something that feels wrong. MotionCraft is built on the idea that character movement should be generated, not hand-authored.

The long-term goal: characters that handle complex terrain, react to their environment, and move naturally — all from a model, with no animation pipeline. A solo developer ships a 3D game without learning animation. An indie team iterates on movement without rebuilding state machines. A studio prototypes locomotion in hours instead of weeks. A content creator records a game teaser with believable characters and no hand-crafted animations.

This MVP is the first step.

## What's next

- More control (stopping, going sideways, jumping)
- Terrain-aware movement (going uphill, better collisions with obstacles)
- Easy application to other avatars

---

## Get in touch

If you have a project in mind that needs this or just feedback, reach out. 📬 [motioncraft.gen@gmail.com](mailto:motioncraft.gen@gmail.com)
