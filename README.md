# 3D Building Reconstruction from Floor Plans

A web application that converts 2D floor plan images into interactive 3D models using classical computer vision algorithms and C programming.

## Features

- Upload floor plan images (PNG, JPG, PGM, PPM)
- Automatic wall, room, door, and window detection
- Generate 3D models with and without roofs
- Interactive 3D visualization with Three.js
- Sample floor plans for quick testing
- Georgia Tech themed interface

## Requirements

- **Python 3.7+**
- **GCC compiler** (for C code)
- **Flask** and **Pillow**

## Installation

1. Clone the repository:

```bash
git clone https://github.com/MengpingZhang/CS6010-Project-Group4.git
cd CS6010-Project-Group4/Code
```

2. Install Python dependencies:

```bash
pip install flask pillow
```

3. Compile the C program:

**Mac/Linux:**

```bash
gcc -o 3D_Reconstruction 3D_Reconstruction_CSE6010_1.c -lm -fopenmp -O2
```

**Windows:**

```bash
gcc -o 3D_Reconstruction.exe 3D_Reconstruction_CSE6010_1.c -lm -fopenmp -O2
```

## Usage

1. Start the Flask server:

```bash
python app.py
```

2. Open your browser and go to:

```
http://127.0.0.1:8000
```

3. Upload a floor plan image or click a sample image to generate a 3D model

4. Use the toggle buttons to switch between "No Roof" and "With Roof" views

5. Interact with the 3D model:
   - **Left Click + Drag**: Rotate
   - **Right Click + Drag**: Pan
   - **Scroll**: Zoom

## Project Structure

```
Code/
├── 3D_Reconstruction_CSE6010_1.c    # Main C program
├── app.py                            # Flask web server
├── static/
│   ├── app.js                        # Frontend JavaScript
│   ├── style.css                     # Styling
│   └── sample_images/                # Sample floor plans
└── templates/
    └── index.html                    # Main HTML page
```

## Algorithm Overview

1. **Image Preprocessing**: Otsu thresholding for wall detection
2. **Segmentation**: BFS flood fill and connected component labeling
3. **Door Detection**:
   - Red pixel detection for door markers
   - RANSAC-based arc detection for curved doors
4. **3D Extrusion**: Wall and floor mesh generation
5. **OBJ Export**: Standard 3D model format

## Technical Details

- **Language**: C (core algorithm), Python (web server), JavaScript (visualization)
- **Parallelization**: OpenMP for performance optimization
- **Complexity**: O(N) where N = number of pixels
- **Output Format**: Wavefront OBJ files

## Team

- Zhang, Mengping
- Ma, Hsu, Chieh
- Gao, Tianxiang
- Nation, Ryan T
- Imran Aziz

## Course

CSE 6010 - Computational Problem Solving  
Georgia Institute of Technology

## License

This project is developed for academic purposes.
