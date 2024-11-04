// Basic Three.js setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000); // Temporary aspect ratio, will update later

const renderer = new THREE.WebGLRenderer();
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

// Lighting
const ambientLight = new THREE.AmbientLight(0x404040, 2);
scene.add(ambientLight);
const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(1, 1, 1).normalize();
scene.add(directionalLight);

// Load OBJ model
const objLoader = new THREE.OBJLoader();
let bunny;
objLoader.load('/static/model/bunny.obj', (obj) => {
    bunny = obj;

    bunny.scale.set(0.3, 0.3, 0.3); // Adjust the values as needed

    bunny.position.set(0, -1, 0);

    // Apply a basic material if the model lacks one
    bunny.traverse((child) => {
        if (child.isMesh) {
            child.material = new THREE.MeshStandardMaterial({ color: 0x808080 });
        }
    });

    // Add bunny to scene
    scene.add(bunny);

    // Start animation
    animate();
});

function animate() {
    requestAnimationFrame(animate);

    // Rotate the bunny model continuously
    if (bunny) {
        bunny.rotation.y += 0.01; // Adjust speed as needed (0.01 is a slow rotation)
    }

    renderer.render(scene, camera);
}


// Function to update orientation based on real-time data
function updateOrientation(heading, roll, pitch) {
    if (bunny) {
        bunny.rotation.x = THREE.MathUtils.degToRad(pitch);
        bunny.rotation.y = THREE.MathUtils.degToRad(heading);
        bunny.rotation.z = THREE.MathUtils.degToRad(roll);
    }
}
