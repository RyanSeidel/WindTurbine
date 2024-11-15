// Basic Three.js setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);

const renderer = new THREE.WebGLRenderer();
renderer.shadowMap.enabled = false; // Disable shadow map rendering
renderer.setClearColor(0xadd8e6); // Set background color to light blue
const modelContainer = document.getElementById("3d-model-container");
renderer.setSize(modelContainer.clientWidth, modelContainer.clientHeight);
modelContainer.appendChild(renderer.domElement);

// Update camera aspect ratio based on container size
camera.aspect = modelContainer.clientWidth / modelContainer.clientHeight;
camera.updateProjectionMatrix();

// Adjust renderer size on window resize
window.addEventListener("resize", () => {
    renderer.setSize(modelContainer.clientWidth, modelContainer.clientHeight);
    camera.aspect = modelContainer.clientWidth / modelContainer.clientHeight;
    camera.updateProjectionMatrix();
});

camera.position.z = 5;

// Lighting setup
const ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
scene.add(ambientLight);

const pointLight = new THREE.PointLight(0xffffff, 1);
pointLight.position.set(5, 5, 5);
scene.add(pointLight);

const pointLightBack = new THREE.PointLight(0xffffff, 0.7);
pointLightBack.position.set(-5, -5, -5);
scene.add(pointLightBack);

// Load GLB model
const loader = new THREE.GLTFLoader();
let turbine;
let blades; // Variable to hold the blades mesh

// RPM setup
let rpm = 33; // Example RPM
let angularVelocity = (rpm * 2 * Math.PI) / 60; // Radians per second

loader.load('/static/model/Turbine1.glb', (gltf) => {
    turbine = gltf.scene;

    // Adjust scale and position as needed
    turbine.scale.set(6, 6, 6);
    turbine.position.set(0, -2, 0);

    scene.add(turbine);

    console.log("Loaded turbine model:", turbine.children);

    // Access the "Empty" object
    const emptyObject = turbine.getObjectByName("Empty");
    if (emptyObject) {
        console.log("Empty object found:", emptyObject);

        // Access the blades ("remesh1") from the children of "Empty"
        blades = emptyObject.getObjectByName("remesh1");
        if (blades) {
            console.log("Blades object found:", blades);
        } else {
            console.warn("Blades object ('remesh1') not found!");
        }
    } else {
        console.warn("Empty object not found!");
    }

    // Start animation loop
    animate();
}, undefined, (error) => {
    console.error('Error loading GLB file:', error);
});

// Animation loop for smooth transitions and blade spinning
let lastFrameTime = performance.now(); // Keep track of the last frame time

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const deltaTime = (currentTime - lastFrameTime) / 1000; // Convert to seconds
    lastFrameTime = currentTime;

    if (turbine) {
        // Lerp the rotation towards the target rotation (optional)
        currentRotation.x += (targetRotation.x - currentRotation.x) * lerpSpeed;
        currentRotation.y += (targetRotation.y - currentRotation.y) * lerpSpeed;
        currentRotation.z += (targetRotation.z - currentRotation.z) * lerpSpeed;

        // Apply the interpolated rotation to the turbine base
        turbine.rotation.x = currentRotation.x;
        turbine.rotation.y = currentRotation.y;
        turbine.rotation.z = currentRotation.z;
    }

    // Spin the blades based on angular velocity
    if (blades) {
        blades.rotation.z += angularVelocity * deltaTime; // Rotate around the z-axis like a steering wheel
    }

    renderer.render(scene, camera);
}

// Variables to hold the current and target orientation
let targetRotation = { x: 0, y: 0, z: 0 };
let currentRotation = { x: 0, y: 0, z: 0 };
const lerpSpeed = 0.1; // Adjust this value to control the smoothness (0.1 is usually good)

// Function to update the target orientation based on real-time data
function updateOrientation(heading, roll, pitch) {
    targetRotation.x = THREE.MathUtils.degToRad(pitch);
    targetRotation.y = THREE.MathUtils.degToRad(heading);
    targetRotation.z = THREE.MathUtils.degToRad(roll);
}

// Initialize Socket.IO connection to receive orientation data
const socket = io();

// Listen for 'orientation_data' event and update target orientation
socket.on('orientation_data', (data) => {
    console.log("Orientation data received:", data); // Debugging log
    const [heading, roll, pitch] = data.orientation.split(',').map(Number); // Extract values as numbers
    updateOrientation(heading, roll, pitch); // Update target orientation
});

// Function to dynamically update RPM
function updateRPM(newRPM) {
    rpm = newRPM;
    angularVelocity = (rpm * 2 * Math.PI) / 60; // Recalculate angular velocity
    console.log("Updated RPM:", rpm, "Angular Velocity:", angularVelocity);
}

// Example of RPM update simulation (optional)
// socket.on('rpm_data', (data) => {
//     updateRPM(data.rpm);
// });
