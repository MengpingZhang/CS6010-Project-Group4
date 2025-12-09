import os
import subprocess
import uuid
import time
import platform
from flask import Flask, request, jsonify, render_template, send_from_directory
from PIL import Image

app = Flask(__name__)

# --- Configuration ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_FOLDER = os.path.join(BASE_DIR, 'static')
SAMPLE_IMAGES_FOLDER = os.path.join(STATIC_FOLDER, 'sample_images')

# --- Logic: Detect OS to choose the correct executable ---
if platform.system() == 'Windows':
    executable_name = '3D_Reconstruction.exe'
else:
    executable_name = '3D_Reconstruction'

C_EXECUTABLE = os.path.join(BASE_DIR, executable_name)

# Ensure folders exist
os.makedirs(STATIC_FOLDER, exist_ok=True)
os.makedirs(SAMPLE_IMAGES_FOLDER, exist_ok=True)

# --- Function: Clean up old files ---
def cleanup_old_files():
    now = time.time()
    expiration_seconds = 300  # 5 minutes
    
    if not os.path.exists(STATIC_FOLDER):
        return
    
    for filename in os.listdir(STATIC_FOLDER):
        file_path = os.path.join(STATIC_FOLDER, filename)
        
        if not os.path.isfile(file_path):
            continue

        if filename.endswith(('.obj', '.pgm', '.ppm', '.mtl')):
            try:
                file_age = now - os.path.getmtime(file_path)
                if file_age > expiration_seconds:
                    os.remove(file_path)
                    print(f"[CLEANUP] Removed old file: {filename}")
            except Exception as e:
                print(f"[ERROR] Cleanup error: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/sample-images')
def get_sample_images():
    """Return list of available sample images"""
    try:
        if not os.path.exists(SAMPLE_IMAGES_FOLDER):
            return jsonify({'samples': []})
        
        sample_files = []
        for filename in os.listdir(SAMPLE_IMAGES_FOLDER):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.ppm', '.pgm')):
                sample_files.append({
                    'name': filename,
                    'url': f'/static/sample_images/{filename}',
                    'display_name': filename.replace('_', ' ').replace('.png', '').replace('.jpg', '').title()
                })
        
        return jsonify({'samples': sample_files})
    except Exception as e:
        print(f"Error loading sample images: {e}")
        return jsonify({'samples': []})

@app.route('/upload', methods=['POST'])
def upload_file():
    # Run cleanup before processing new request
    cleanup_old_files()

    if 'file' not in request.files:
        return jsonify({'error': 'No file part in request'}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No file selected'}), 400

    unique_id = str(uuid.uuid4())
    
    # 1. Convert Image to PGM or PPM
    try:
        img = Image.open(file)
        
        # Save as PPM if color, PGM if grayscale
        if img.mode == 'RGB':
            image_filename = f"{unique_id}.ppm"
            image_path = os.path.join(STATIC_FOLDER, image_filename)
            img.save(image_path, format='PPM')
        else:
            image_filename = f"{unique_id}.pgm"
            image_path = os.path.join(STATIC_FOLDER, image_filename)
            img.convert('L').save(image_path, format='PPM')
    except Exception as e:
        return jsonify({'error': f'Image conversion failed: {str(e)}'}), 500

    # 2. Run C Program
    if not os.path.exists(C_EXECUTABLE):
        return jsonify({'error': f'Executable not found: {executable_name}. Please compile the C code.'}), 500

    # Grant execution permissions (For Mac/Linux)
    if platform.system() != 'Windows':
        try:
            os.chmod(C_EXECUTABLE, 0o755)
        except Exception:
            pass

    obj_basename = os.path.join(STATIC_FOLDER, unique_id)
    
    try:
        subprocess.run([C_EXECUTABLE, image_path, obj_basename], check=True)
    except subprocess.CalledProcessError as e:
        return jsonify({'error': f'C program failed to process the image: {str(e)}'}), 500

    obj_filename_noRoof = f"{unique_id}_noRoof.obj"
    obj_filename_withRoof = f"{unique_id}_withRoof.obj"
    
    noRoof_path = os.path.join(STATIC_FOLDER, obj_filename_noRoof)
    withRoof_path = os.path.join(STATIC_FOLDER, obj_filename_withRoof)
    
    if not os.path.exists(noRoof_path):
        return jsonify({'error': 'NoRoof model file was not generated'}), 500
    if not os.path.exists(withRoof_path):
        return jsonify({'error': 'WithRoof model file was not generated'}), 500

    return jsonify({
        'message': 'Success',
        'model_url_noRoof': f'/static/{obj_filename_noRoof}',
        'model_url_withRoof': f'/static/{obj_filename_withRoof}'
    })

@app.route('/upload-sample', methods=['POST'])
def upload_sample():
    """Handle sample image selection"""
    cleanup_old_files()
    
    data = request.get_json()
    if 'sample_url' not in data:
        return jsonify({'error': 'No sample URL provided'}), 400
    
    sample_url = data['sample_url']
    # Extract filename from URL
    sample_filename = sample_url.split('/')[-1]
    sample_path = os.path.join(SAMPLE_IMAGES_FOLDER, sample_filename)
    
    if not os.path.exists(sample_path):
        return jsonify({'error': 'Sample image not found'}), 404
    
    unique_id = str(uuid.uuid4())
    
    try:
        img = Image.open(sample_path)
        
        # Save as PPM if color, PGM if grayscale
        if img.mode == 'RGB':
            image_filename = f"{unique_id}.ppm"
            image_path = os.path.join(STATIC_FOLDER, image_filename)
            img.save(image_path, format='PPM')
        else:
            image_filename = f"{unique_id}.pgm"
            image_path = os.path.join(STATIC_FOLDER, image_filename)
            img.convert('L').save(image_path, format='PPM')
    except Exception as e:
        return jsonify({'error': f'Image conversion failed: {str(e)}'}), 500
    
    if not os.path.exists(C_EXECUTABLE):
        return jsonify({'error': f'Executable not found: {executable_name}'}), 500
    
    if platform.system() != 'Windows':
        try:
            os.chmod(C_EXECUTABLE, 0o755)
        except Exception:
            pass
    
    obj_basename = os.path.join(STATIC_FOLDER, unique_id)
    
    try:
        subprocess.run([C_EXECUTABLE, image_path, obj_basename], check=True)
    except subprocess.CalledProcessError as e:
        return jsonify({'error': f'C program failed: {str(e)}'}), 500
    
    obj_filename_noRoof = f"{unique_id}_noRoof.obj"
    obj_filename_withRoof = f"{unique_id}_withRoof.obj"
    
    noRoof_path = os.path.join(STATIC_FOLDER, obj_filename_noRoof)
    withRoof_path = os.path.join(STATIC_FOLDER, obj_filename_withRoof)
    
    if not os.path.exists(noRoof_path) or not os.path.exists(withRoof_path):
        return jsonify({'error': 'Model files were not generated'}), 500
    
    return jsonify({
        'message': 'Success',
        'model_url_noRoof': f'/static/{obj_filename_noRoof}',
        'model_url_withRoof': f'/static/{obj_filename_withRoof}'
    })

if __name__ == '__main__':
    print(f"Detected System: {platform.system()}")
    print(f"Looking for C executable: {C_EXECUTABLE}")
    print("Starting server... Go to http://127.0.0.1:8000")
    app.run(debug=True, host='0.0.0.0', port=8000)