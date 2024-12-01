//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//                           Wind Turbine Digital Twins                                                              //
//                        By Ryan Seidel and Zac Castaneda                                                           //
//                         Client: Dr. Jose Baca                                                                     //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Description:                                                                                                      //
// This script creates an interactive 3D visualization of a wind turbine model using Three.js, with real-time        // 
// updates driven by sensor data via Socket.IO. The turbine and its components, including blades, are loaded         // 
// from a GLB model and rendered with customizable lighting and camera settings.                                     //
//Key features include:                                                                                              //
//                                                                                                                   //
//   Real-Time Sensor Data Integration: Updates orientation, gyroscope, magnetometer, and accelerometer data         //
//   in real-time, dynamically visualized with arrow helpers for direction and magnitude.                            //
//   Rotating Turbine Blades: The turbine blades spin based on RPM data, with the angular velocity recalculated      //
// dynamically.                                                                                                      //
// Camera View Control: Users can switch between default, isometric, and top-down camera views for versatile         //
// visualization.                                                                                                    //
// Dynamic 3D Arrows: Visual indicators for gyroscope, magnetometer, and accelerometer data adjust in direction      //
// and length based on incoming sensor data.                                                                         //
// Responsive Rendering: The scene adjusts to window resizing, maintaining proper aspect ratio and resolution.       //
//  Extensibility: The code supports additional features like debugging tools (axes helper) and advanced             //
// animations for smoother interactions.                                                                             //
//                                                                                                                   //
//This visualization combines real-time physics and telemetry to provide a compelling way to monitor wind turbine    //
//performance and sensor outputs.                                                                                    //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// Basic Three.js setup
const scene = new THREE.Scene();

// Camera setup
const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
camera.position.z = 5;

// Renderer setup
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.shadowMap.enabled = false; // Disable shadows for performance
renderer.setClearColor(0xadd8e6); // Light blue background color

// Get the 3D model container
const modelContainer = document.getElementById("3d-model-container");
renderer.setSize(modelContainer.clientWidth, modelContainer.clientHeight);
modelContainer.appendChild(renderer.domElement);

// Update camera and renderer on window resize
window.addEventListener("resize", () => {
    const width = modelContainer.clientWidth;
    const height = modelContainer.clientHeight;

    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
});

// Lighting setup
const ambientLight = new THREE.AmbientLight(0xffffff, 0.3); // Soft light
scene.add(ambientLight);

const pointLightFront = new THREE.PointLight(0xffffff, 1);
pointLightFront.position.set(5, 5, 5);
scene.add(pointLightFront);

const pointLightBack = new THREE.PointLight(0xffffff, 0.7);
pointLightBack.position.set(-5, -5, -5);
scene.add(pointLightBack);

// Load GLB model
const loader = new THREE.GLTFLoader();
let turbine = null;
let blades = null; // To hold the blades mesh

// RPM setup
let rpm = 33; // Initial RPM
let angularVelocity = (rpm * 2 * Math.PI) / 60; // Convert to radians per second

loader.load(
    '/static/model/Turbine1.glb',
    (gltf) => {
        turbine = gltf.scene;

        // Scale and position the model
        turbine.scale.set(6, 6, 6);
        turbine.position.set(0, -2, 0);
        scene.add(turbine);

        console.log("Loaded turbine model:", turbine.children);

        // Access the "Empty" object and its children
        const emptyObject = turbine.getObjectByName("Empty");
        if (emptyObject) {
            console.log("Empty object found:", emptyObject);

            // Find blades object (e.g., "remesh1")
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
    },
    undefined,
    (error) => {
        console.error("Error loading GLB file:", error);
    }
);

// Add Axes Helper (optional for debugging orientation)
// const axesHelper = new THREE.AxesHelper(5);
// scene.add(axesHelper);

// Create arrow helpers for gyroscope and magnetometer
const gyroArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), // Initial direction
    new THREE.Vector3(0, 0, 0), // Origin
    2,                          // Length
    0xff0000                    // Color (red for gyroscope)
);
scene.add(gyroArrow);

const magnetoArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0), // Initial direction
    new THREE.Vector3(0, 0, 0), // Origin
    2,                          // Length
    0x0000ff                    // Color (blue for magnetometer)
);
scene.add(magnetoArrow);

const accelerometerArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1), // Initial direction
    new THREE.Vector3(0, 0, 0), // Origin
    2,                          // Length
    0x00ff00                    // Color (green for accelerometer)
);
scene.add(accelerometerArrow);

// Animation loop
let lastFrameTime = performance.now(); // Track the last frame time

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const deltaTime = (currentTime - lastFrameTime) / 1000; // Convert to seconds
    lastFrameTime = currentTime;

    // Smooth rotation interpolation for the turbine
    if (turbine) {
        currentRotation.x += (targetRotation.x - currentRotation.x) * lerpSpeed;
        currentRotation.y += (targetRotation.y - currentRotation.y) * lerpSpeed;
        currentRotation.z += (targetRotation.z - currentRotation.z) * lerpSpeed;

        turbine.rotation.set(
            currentRotation.x,
            currentRotation.y,
            currentRotation.z
        );
    }

    // Rotate blades based on angular velocity
    if (blades) {
        blades.rotation.z += angularVelocity * deltaTime; // Spin around the z-axis
    }

    renderer.render(scene, camera);
}

// View mode state
let currentView = 'default'; // Possible values: 'default', 'isometric', 'top-down'

// Function to switch views
function switchCameraView() {
    if (currentView === 'default') {
        // Switch to isometric view
        const isoAngle = Math.PI / 4; // 45 degrees
        const isoHeightAngle = Math.atan(Math.sqrt(2)); // ~35.26 degrees
        const distance = 10;

        camera.position.set(
            distance * Math.cos(isoAngle),
            distance * Math.sin(isoAngle),
            distance * Math.sin(isoHeightAngle)
        );
        camera.lookAt(new THREE.Vector3(0, 0, 0));
        currentView = 'isometric';
        document.getElementById("switch-view-btn").innerText = "Switch to Top-Down View";
    } else if (currentView === 'isometric') {
        // Switch to top-down view
        camera.position.set(0, 10, 0); // Directly above the scene
        camera.lookAt(new THREE.Vector3(0, 0, 0));
        currentView = 'top-down';
        document.getElementById("switch-view-btn").innerText = "Switch to Default View";
    } else if (currentView === 'top-down') {
        // Switch back to default view
        camera.position.set(0, 0, 5); // Default perspective
        camera.lookAt(new THREE.Vector3(0, 0, 0));
        currentView = 'default';
        document.getElementById("switch-view-btn").innerText = "Switch to Isometric View";
    }
}

// Add event listener to button
document.getElementById("switch-view-btn").addEventListener("click", switchCameraView);

// Variables to store current and target rotation
let targetRotation = { x: 0, y: 0, z: 0 };
let currentRotation = { x: 0, y: 0, z: 0 };
const lerpSpeed = 0.1; // Smoothing factor for rotation

// Update orientation based on real-time data
function updateOrientation(heading, roll, pitch) {
    targetRotation.x = THREE.MathUtils.degToRad(pitch);
    targetRotation.y = THREE.MathUtils.degToRad(heading);
    targetRotation.z = THREE.MathUtils.degToRad(roll);
}

// Update gyroscope display
function updateGyroscopeDisplay(gx, gy, gz) {
    //console.log(`Gyroscope Data: gx=${gx}, gy=${gy}, gz=${gz}`);
    // Apply gyroscope data to dynamically adjust rotation (example)
    if (turbine) {
        turbine.rotation.x += gx * 0.01;
        turbine.rotation.y += gy * 0.01;
        turbine.rotation.z += gz * 0.01;
    }
}

// Update magnetometer display
function updateMagnetometerDisplay(mx, my, mz) {
    //console.log(`Magnetometer Data: mx=${mx}, my=${my}, mz=${mz}`);
    // Example of applying magnetometer data to an arrow helper
    const magnetoDirection = new THREE.Vector3(mx, my, mz).normalize();
    const magnetoLength = new THREE.Vector3(mx, my, mz).length();
    magnetoArrow.setDirection(magnetoDirection);
    magnetoArrow.setLength(magnetoLength);
}

// Update accelerometer display
function updateAccelerometerDisplay(ax, ay, az) {
    const accelDirection = new THREE.Vector3(ax, ay, az).normalize();
    const accelMagnitude = new THREE.Vector3(ax, ay, az).length();

    accelerometerArrow.setDirection(accelDirection);
    accelerometerArrow.setLength(accelMagnitude);

}


// Socket.IO for real-time data
const socket = io();

// Listen for orientation data updates
socket.on('orientation_data', (data) => {
    //console.log("Orientation data received:", data);
    const [heading, roll, pitch] = data.orientation.split(',').map(Number); // Parse data
    updateOrientation(heading, roll, pitch);
});

// Listen for magnetometer data updates
socket.on('magnetometer_data', (data) => {
    //console.log("Magnetometer data received: ", data);  // Debugging log
    const [mx, my, mz] = data.magnetometer.split(',').map(Number); // Parse data
    updateMagnetometerDisplay(mx, my, mz);
});

// Listen for gyroscope data updates
socket.on('gyroscope_data', (data) => {
    //console.log("Gyroscope data received: ", data);  // Debugging log
    const [gx, gy, gz] = data.gyroscope.split(',').map(Number); // Parse data
    updateGyroscopeDisplay(gx, gy, gz);
});

socket.on('accelerometer_data', (data) => {
    //console.log("Accelerometer data received: ", data);  // Debugging log
    const [ax, ay, az] = data.accelerometer.split(',').map(Number); // Split and convert to numbers
    updateAccelerometerDisplay(ax, ay, az); // Update the display
});

// Dynamically update RPM and recalculate angular velocity
function updateRPM(newRPM) {
    rpm = newRPM;
    angularVelocity = (rpm * 2 * Math.PI) / 60; // Recalculate angular velocity
    console.log("Updated RPM:", rpm, "Angular Velocity:", angularVelocity);
}

// Listen for RPM updates
socket.on('rpm_data', (data) => {
    console.log("RPM data received:", data);

    // Ensure data contains the RPM value
    if (data && typeof data === 'object' && data.hasOwnProperty('latest_rpm')) {
        const rpm = parseFloat(data.latest_rpm); // Convert RPM value to float
        if (!isNaN(rpm)) {
            updateRPM(rpm); // Update RPM with the parsed value
        } else {
            console.error("Invalid RPM value received:", data.latest_rpm);
        }
    } else {
        console.error("Invalid RPM data format:", data);
    }
});
    
    

