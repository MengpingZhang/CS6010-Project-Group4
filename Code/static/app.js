import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

let modelUrls = { noRoof: null, withRoof: null };
let currentModel = "noRoof";
let scene, camera, renderer, controls, gridHelper;

function initScene() {
  const container = document.getElementById("container");

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x2a2a3e);
  scene.fog = new THREE.Fog(0x2a2a3e, 500, 2000);

  camera = new THREE.PerspectiveCamera(
    45,
    container.clientWidth / container.clientHeight,
    0.1,
    10000
  );
  camera.position.set(200, 300, 300);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.minDistance = 50;
  controls.maxDistance = 1500;

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
  scene.add(ambientLight);

  const mainLight = new THREE.DirectionalLight(0xffffff, 0.8);
  mainLight.position.set(200, 400, 200);
  mainLight.castShadow = true;
  mainLight.shadow.camera.left = -500;
  mainLight.shadow.camera.right = 500;
  mainLight.shadow.camera.top = 500;
  mainLight.shadow.camera.bottom = -500;
  mainLight.shadow.mapSize.width = 2048;
  mainLight.shadow.mapSize.height = 2048;
  scene.add(mainLight);

  const fillLight = new THREE.DirectionalLight(0xaaccff, 0.3);
  fillLight.position.set(-200, 100, -200);
  scene.add(fillLight);

  const backLight = new THREE.DirectionalLight(0xffddaa, 0.4);
  backLight.position.set(-100, 150, -300);
  scene.add(backLight);

  const hemisphereLight = new THREE.HemisphereLight(0x8899ff, 0x443322, 0.3);
  scene.add(hemisphereLight);

  gridHelper = new THREE.GridHelper(1200, 60, 0x00ff88, 0x444466);
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);

  const axesHelper = new THREE.AxesHelper(150);
  axesHelper.position.y = 0.1;
  scene.add(axesHelper);

  const groundGeometry = new THREE.PlaneGeometry(1500, 1500);
  const groundMaterial = new THREE.ShadowMaterial({
    opacity: 0.3,
    color: 0x000000,
  });
  const ground = new THREE.Mesh(groundGeometry, groundMaterial);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -1;
  ground.receiveShadow = true;
  scene.add(ground);

  animate();
  window.addEventListener("resize", onWindowResize);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

function onWindowResize() {
  const container = document.getElementById("container");
  const width = container.clientWidth;
  const height = container.clientHeight;
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function setupFileInput() {
  document.getElementById("fileInput").addEventListener("change", function (e) {
    const fileName = e.target.files[0]?.name;
    const display = document.getElementById("fileNameDisplay");
    if (fileName) {
      display.innerText = "✅ Selected: " + fileName;
      display.style.color = "#4f46e5";
    } else {
      display.innerText = "📂 Click to Select Floorplan Image";
      display.style.color = "#6b7280";
    }
  });
}

function removeOldModels() {
  for (let i = scene.children.length - 1; i >= 0; i--) {
    const obj = scene.children[i];
    if (
      obj.type === "Group" ||
      (obj.type === "Mesh" && obj.geometry.type !== "PlaneGeometry")
    ) {
      scene.remove(obj);
    }
  }
}

function createModelMaterial() {
  return new THREE.MeshStandardMaterial({
    color: 0xe8f4f8,
    roughness: 0.5,
    metalness: 0.05,
    side: THREE.DoubleSide,
    flatShading: false,
  });
}

function addEdgeLines(object) {
  object.traverse(function (child) {
    if (child.isMesh) {
      const edges = new THREE.EdgesGeometry(child.geometry, 15);
      const lineMaterial = new THREE.LineBasicMaterial({
        color: 0x333333,
        linewidth: 1,
        opacity: 0.6,
        transparent: true,
      });
      const lineSegments = new THREE.LineSegments(edges, lineMaterial);
      child.add(lineSegments);
    }
  });
}

function displayModel(url, modelType) {
  const spinner = document.getElementById("loadingSpinner");
  const statusText = document.getElementById("statusText");

  spinner.style.display = "block";
  statusText.innerText = `Loading ${
    modelType === "noRoof" ? "No Roof" : "With Roof"
  } model...`;
  statusText.style.color = "#4f46e5";

  removeOldModels();

  const loader = new OBJLoader();
  loader.load(
    url,
    function (object) {
      spinner.style.display = "none";
      statusText.innerText = `✨ ${
        modelType === "noRoof" ? "No Roof" : "With Roof"
      } Model Loaded!`;
      statusText.style.color = "#10b981";

      const box = new THREE.Box3().setFromObject(object);
      const center = box.getCenter(new THREE.Vector3());
      object.position.sub(center);

      const material = createModelMaterial();

      object.traverse(function (child) {
        if (child.isMesh) {
          child.material = material;
          child.geometry.computeVertexNormals();
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });

      addEdgeLines(object);
      scene.add(object);
      currentModel = modelType;
      adjustCameraToModel(object);
    },
    undefined,
    function (error) {
      console.error(error);
      statusText.innerText = "❌ Failed to load model.";
      statusText.style.color = "#ef4444";
      spinner.style.display = "none";
    }
  );
}

function adjustCameraToModel(object) {
  const box = new THREE.Box3().setFromObject(object);
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z);
  const fov = camera.fov * (Math.PI / 180);
  let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
  cameraZ *= 1.5;

  camera.position.set(cameraZ * 0.7, cameraZ * 0.8, cameraZ * 0.7);
  camera.lookAt(0, 0, 0);

  controls.target.set(0, 0, 0);
  controls.update();
}

window.loadModel = function (modelType) {
  const url = modelUrls[modelType];
  if (!url) return;

  document
    .getElementById("btnNoRoof")
    .classList.toggle("active", modelType === "noRoof");
  document
    .getElementById("btnWithRoof")
    .classList.toggle("active", modelType === "withRoof");

  displayModel(url, modelType);
};

window.uploadAndRun = async function () {
  const fileInput = document.getElementById("fileInput");
  const statusText = document.getElementById("statusText");
  const spinner = document.getElementById("loadingSpinner");
  const runBtn = document.getElementById("runBtn");

  if (fileInput.files.length === 0) {
    statusText.innerText = "⚠️ Please select a file first!";
    statusText.style.color = "#ef4444";
    return;
  }

  removeOldModels();
  document.getElementById("modelControls").style.display = "none";

  statusText.innerText = "Processing image & generating 3D model...";
  statusText.style.color = "#4f46e5";
  spinner.style.display = "block";
  runBtn.disabled = true;
  runBtn.style.opacity = "0.7";

  const formData = new FormData();
  formData.append("file", fileInput.files[0]);

  try {
    const response = await fetch("/upload", {
      method: "POST",
      body: formData,
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || "Server Error");
    }

    modelUrls.noRoof = data.model_url_noRoof;
    modelUrls.withRoof = data.model_url_withRoof;

    document.getElementById("modelControls").style.display = "flex";
    document.getElementById("btnNoRoof").classList.add("active");
    document.getElementById("btnWithRoof").classList.remove("active");

    statusText.innerText = "Loading No Roof model...";

    const loader = new OBJLoader();
    loader.load(
      modelUrls.noRoof,
      function (object) {
        spinner.style.display = "none";
        statusText.innerText = "✨ Model Generated Successfully!";
        statusText.style.color = "#10b981";
        runBtn.disabled = false;
        runBtn.style.opacity = "1";

        const box = new THREE.Box3().setFromObject(object);
        const center = box.getCenter(new THREE.Vector3());
        object.position.sub(center);

        const material = createModelMaterial();

        object.traverse(function (child) {
          if (child.isMesh) {
            child.material = material;
            child.geometry.computeVertexNormals();
            child.castShadow = true;
            child.receiveShadow = true;
          }
        });

        addEdgeLines(object);
        scene.add(object);
        currentModel = "noRoof";
        adjustCameraToModel(object);
      },
      undefined,
      function (error) {
        console.error(error);
        statusText.innerText = "❌ Failed to load 3D model.";
        statusText.style.color = "#ef4444";
        spinner.style.display = "none";
        runBtn.disabled = false;
        runBtn.style.opacity = "1";
      }
    );
  } catch (err) {
    console.error(err);
    statusText.innerText = "❌ Error: " + err.message;
    statusText.style.color = "#ef4444";
    spinner.style.display = "none";
    runBtn.disabled = false;
    runBtn.style.opacity = "1";
  }
};

async function loadSampleImages() {
  const container = document.getElementById("sampleImagesContainer");

  try {
    const response = await fetch("/api/sample-images");
    const data = await response.json();

    if (data.samples && data.samples.length > 0) {
      container.innerHTML = "";
      data.samples.forEach((sample) => {
        const card = document.createElement("div");
        card.className = "sample-image-card";
        card.onclick = () => handleSampleClick(sample.url);

        card.innerHTML = `
          <img src="${sample.url}" alt="${sample.display_name}" loading="lazy">
          <div class="sample-image-name">${sample.display_name}</div>
        `;

        container.appendChild(card);
      });
    } else {
      container.innerHTML =
        '<div class="no-samples">No sample images available.<br>Add images to static/sample_images/</div>';
    }
  } catch (error) {
    console.error("Error loading sample images:", error);
    container.innerHTML =
      '<div class="no-samples">Failed to load samples</div>';
  }
}

async function handleSampleClick(sampleUrl) {
  const statusText = document.getElementById("statusText");
  const spinner = document.getElementById("loadingSpinner");
  const runBtn = document.getElementById("runBtn");
  const fileDisplay = document.getElementById("fileNameDisplay");

  fileDisplay.innerText = "✅ Sample image selected";
  fileDisplay.style.color = "#EAAA00";

  removeOldModels();
  document.getElementById("modelControls").style.display = "none";

  statusText.innerText = "Processing sample image & generating 3D model...";
  statusText.style.color = "#EAAA00";
  spinner.style.display = "block";
  runBtn.disabled = true;
  runBtn.style.opacity = "0.7";

  try {
    const response = await fetch("/upload-sample", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sample_url: sampleUrl }),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || "Server Error");
    }

    modelUrls.noRoof = data.model_url_noRoof;
    modelUrls.withRoof = data.model_url_withRoof;

    document.getElementById("modelControls").style.display = "flex";
    document.getElementById("btnNoRoof").classList.add("active");
    document.getElementById("btnWithRoof").classList.remove("active");

    statusText.innerText = "Loading No Roof model...";

    const loader = new OBJLoader();
    loader.load(
      modelUrls.noRoof,
      function (object) {
        spinner.style.display = "none";
        statusText.innerText = "✨ Sample Model Generated Successfully!";
        statusText.style.color = "#10b981";
        runBtn.disabled = false;
        runBtn.style.opacity = "1";

        const box = new THREE.Box3().setFromObject(object);
        const center = box.getCenter(new THREE.Vector3());
        object.position.sub(center);

        const material = createModelMaterial();

        object.traverse(function (child) {
          if (child.isMesh) {
            child.material = material;
            child.geometry.computeVertexNormals();
            child.castShadow = true;
            child.receiveShadow = true;
          }
        });

        addEdgeLines(object);
        scene.add(object);
        currentModel = "noRoof";
        adjustCameraToModel(object);
      },
      undefined,
      function (error) {
        console.error(error);
        statusText.innerText = "❌ Failed to load sample model.";
        statusText.style.color = "#ef4444";
        spinner.style.display = "none";
        runBtn.disabled = false;
        runBtn.style.opacity = "1";
      }
    );
  } catch (err) {
    console.error(err);
    statusText.innerText = "❌ Error: " + err.message;
    statusText.style.color = "#ef4444";
    spinner.style.display = "none";
    runBtn.disabled = false;
    runBtn.style.opacity = "1";
  }
}

document.addEventListener("DOMContentLoaded", function () {
  initScene();
  setupFileInput();
  loadSampleImages();
});
