# CS6010-Project-Group4

## 🏠 Floorplan to 3D Converter

A lightweight web application that converts 2D floorplan images (JPG/PNG) into interactive 3D OBJ models instantly.

---

## ✨ Features

- **Automatic Conversion**: Uses a C algorithm to detect walls and doors
- **3D Visualization**: Built-in interactive 3D viewer powered by Three.js
- **Cross-Platform**: Works on both macOS and Windows

---

## 🚀 Setup Guide

### 1. Prerequisites

- **Python 3.x** installed
- **GCC Compiler** (pre-installed on macOS/Linux; use MinGW on Windows)

### 2. Compile the C Program (One-time setup)

You must compile the C code before running the app. Open your terminal in the project folder:

**For macOS/Linux:**

```bash
gcc newextrusion_with_doors1.c -o converter -lm
chmod +x converter
```

**For Windows:**

```bash
gcc newextrusion_with_doors1.c -o converter.exe
```

### 3. Install Dependencies

Install the required Python libraries:

```bash
pip install flask pillow
```

---

## 🏃 How to Run

1. **Start the Server:**

```bash
   python app.py
```

_(Note: Use `python3 app.py` on macOS if `python` doesn't work)_

2. **Open Browser:**  
   Navigate to `http://127.0.0.1:5000`

3. **Generate Your 3D Model:**
   - Select a floorplan image
   - Click **Generate 3D Model**
   - Wait for the model to render

---

## 🎮 Controls

- **Left Click + Drag**: Rotate view
- **Right Click + Drag**: Pan camera
- **Scroll**: Zoom in/out

---

## 🛠 Troubleshooting

| Issue                       | Solution                                                                                      |
| --------------------------- | --------------------------------------------------------------------------------------------- |
| "Executable not found"      | Ensure `converter` (macOS) or `converter.exe` (Windows) exists in the same folder as `app.py` |
| "Permission denied" (macOS) | Run `chmod +x converter` in the terminal                                                      |
| Models appear black         | Ensure scene lighting is active in the code (default uses white material)                     |

---

## 🔧 Built With

- **Python Flask** - Backend server
- **Three.js** - 3D visualization
- **C** - Floorplan processing algorithm

---
