# SymphRÆlia

## About:

📺 [YouTubeチャンネル](https://www.youtube.com/@Symphraelia)  
📺 [Odyseeチャンネル](https://odysee.com/@nyqu:a) 

🎼 Creating a 512-key isomorphic 音ゲーム for deep musical expression.  
🎼 512鍵アイソモルフィック音ゲームを開発中。本物の音楽表現のために。

This project is an experimental next-generation 音ゲーム and musical controller system.  
The Push 2 is just being temporarily used as a proof of concept.  
Actual hardware design will take place later.

## Hardware Requirements

| Item                 | Tested model(s) | Notes                         |
| -------------------- | --------------- | ----------------------------- |
| Intel RealSense D415 | FR 5.16.0.1     | Stereo depth / IR projector   |
| Ableton Push 2       | N/A             | Pad dimensions hard-coded     |
| GPU (OpenGL 3.3+)    | RX 7900 XTX     | 4GB-VRAM / Linux / Wayland OK |

## Software Dependencies
```bash
6.14.4-artix1-1, gcc 15.1.1, CMake 4.0.1-dirty
raylib 5.0-dev  (system pkg)
OpenCV 4.11.0     (built with opencv_contrib)
Eigen3 3.4.0-2
librealsense 2.55.1 
```

## Building (Linux)
After installing the dependencies just run the build.sh script
```bash
git clone --recursive https://github.com/you/push-2-ar-composer
cd push-2-ar-composer
build.sh
```
## Development Philosophy

- Latency & feature velocity > pristine code.
- I have no pride as a programmer so, I'll use heavy LLM assistance, whatever gets the job done.
-  **functionality, accuracy, and joy** is the objective.

## Approach

- Code may be messy, rapidly evolving, and inconsistent.
- Priorities are working features, low latency, smooth player experience.
- Refactoring and polishing happen only when they unblock real progress.
- The focus is shipping a functional, joyful, playable system — not a perfect codebase.

# SymphRÆlia

## About:

📺 [YouTubeチャンネル](https://www.youtube.com/@Symphraelia)  
📺 [Odyseeチャンネル](https://odysee.com/@nyqu:a) 

🎼 Creating a 512-key isomorphic 音ゲーム for deep musical expression.  
🎼 512鍵アイソモルフィック音ゲームを開発中。本物の音楽表現のために。

This project is an experimental next-generation 音ゲーム and musical controller system.  
The Push 2 is just being temporarily used as a proof of concept.  
Actual hardware design will take place later.

## Hardware Requirements

| Item                 | Tested model(s) | Notes                         |
| -------------------- | --------------- | ----------------------------- |
| Intel RealSense D415 | FR 5.16.0.1     | Stereo depth / IR projector   |
| Ableton Push 2       | N/A             | Pad dimensions hard-coded     |
| GPU (OpenGL 3.3+)    | RX 7900 XTX     | 4GB-VRAM / Linux / Wayland OK |

## Software Dependencies
```bash
6.14.4-artix1-1, gcc 15.1.1, CMake 4.0.1-dirty
raylib 5.0-dev  (system pkg)
OpenCV 4.11.0     (built with opencv_contrib)
Eigen3 3.4.0-2
librealsense 2.55.1 
```

## Building (Linux)
After installing the dependencies just run the build.sh script
```bash
git clone --recursive https://github.com/you/push-2-ar-composer
cd push-2-ar-composer
build.sh
```
## Development Philosophy

- Latency & feature velocity > pristine code.
- I have no pride as a programmer so, I'll use heavy LLM assistance, whatever gets the job done.
-  **functionality, accuracy, and joy** is the objective.

## Approach

- Code may be messy, rapidly evolving, and inconsistent.
- Priorities are working features, low latency, smooth player experience.
- Refactoring and polishing happen only when they unblock real progress.
- The focus is shipping a functional, joyful, playable system — not a perfect codebase.
