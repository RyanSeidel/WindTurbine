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

// Ambient Light to provide a base level of brightness
const ambientLight = new THREE.AmbientLight(0xffffff, 0.3); // Lower intensity for softer ambient light
scene.add(ambientLight);

// PointLights surrounding the object
const lights = [];
const lightIntensity = 0.8; // Lowered intensity for surrounding lights

// Light positions around the object
const positions = [
    { x: 5, y: 5, z: 5 },
    { x: -5, y: 5, z: 5 },
    { x: 5, y: -5, z: 5 },
    { x: -5, y: -5, z: 5 },
    { x: 5, y: 5, z: -5 },
    { x: -5, y: 5, z: -5 },
    { x: 5, y: -5, z: -5 },
    { x: -5, y: -5, z: -5 }
];

// Create PointLights at each position and add them to the scene
positions.forEach(pos => {
    const pointLight = new THREE.PointLight(0xffffff, lightIntensity); // Use the lower intensity
    pointLight.position.set(pos.x, pos.y, pos.z);
    scene.add(pointLight);
    lights.push(pointLight); // Store reference if you want to adjust them later
});

// Load GLB model
const loader = new THREE.GLTFLoader();
let turbine;

loader.load('/static/model/turbine.glb', (gltf) => {
    turbine = gltf.scene;

    // Adjust scale and position as needed
    turbine.scale.set(6.5, 6.5, 6.5);
    turbine.position.set(0, -2.5, 0);

    // Ensure the model doesn't cast or receive shadows
    turbine.traverse((node) => {
        if (node.isMesh) {
            node.castShadow = false;
            node.receiveShadow = false;
        }
    });

    // Add the turbine model to the scene
    scene.add(turbine);

    // Start animation
    animate();
}, undefined, (error) => {
    console.error('Error loading GLB file:', error);
});

function animate() {
    requestAnimationFrame(animate);

    // Rotate the turbine model continuously
    if (turbine) {
        turbine.rotation.y += 0.01; // Adjust speed if needed
    }

    renderer.render(scene, camera);
}

// Function to update orientation based on real-time data
function updateOrientation(heading, roll, pitch) {
    if (turbine) {
        turbine.rotation.x = THREE.MathUtils.degToRad(pitch);
        turbine.rotation.y = THREE.MathUtils.degToRad(heading);
        turbine.rotation.z = THREE.MathUtils.degToRad(roll);
    }
}
