//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//                           Wind Turbine Digital Twins                                                     //
//                        By Ryan Seidel and Zac Castaneda                                                  //
//                         Client: Dr. Jose Baca                                                            //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Description:                                                                                             //
//This code creates a dynamic data table to display and update wind turbine metrics in real time, using     //
//Socket.IO for seamless communication between the backend and frontend. The table shows real-time and      //
//predicted values for key metrics such as orientation, wind speed, air pressure, RPM, and voltage. Built   //
//with D3.js, the table allows efficient updates to specific rows based on incoming data streams. Real-time //
//updates are handled through Socket.IO events, ensuring the table reflects the latest sensor readings and  //
//predictions. The system is extensible, supporting additional metrics and data sources as needed.          //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

const socket = io(); // Initialize Socket.IO

// Data for real-time and prediction
let data = [
  { metric: "Blades", realTime: "60", prediction: "60" },
  { metric: "Wind Direction", realTime: "N", prediction: "NE" },
  { metric: "Wind Speed", realTime: "0 m/s", prediction: "1 m/s" },
  { metric: "RPM", realTime: "0", prediction: "0" },
  { metric: "Voltage", realTime: "0", prediction: "0" },
];

// Select the container
const tableContainer = d3.select("#data-table");

// Create a single table
const createTable = (data) => {
  const table = tableContainer.append("table").attr("class", "styled-table");

  // Table header
  table
    .append("thead")
    .append("tr")
    .selectAll("th")
    .data(["Metric", "Real-Time", "Prediction"])
    .enter()
    .append("th")
    .text((d) => d);

  // Table body
  const tbody = table.append("tbody");

  // Populate rows
  data.forEach((row, index) => {
    const tr = tbody.append("tr").attr("data-index", index); // Attach data-index for updating rows
    tr.append("td").text(row.metric);
    tr.append("td").attr("class", "real-time").text(row.realTime); // Assign a class for real-time updates
    tr.append("td").text(row.prediction);
  });
};

// Create the table
createTable(data);

// Function to update a specific metric in the table
const updateTableMetric = (metric, newValue) => {
  const index = data.findIndex((row) => row.metric === metric);
  if (index >= 0) {
    data[index].realTime = newValue;
    // Update the corresponding table cell
    d3.select(`tr[data-index="${index}"] .real-time`).text(newValue);
  }
};

const updateTablePrediction = (metric, newPrediction) => {
  const index = data.findIndex((row) => row.metric === metric);
  if (index >= 0) {
    data[index].prediction = newPrediction; // Update prediction
    d3.select(`tr[data-index="${index}"] td:nth-child(3)`).text(newPrediction); // Update Prediction Column
  }
};

// Listener for predicted RPM from the predictedRPS MQTT topic

let predictedAngularVelocity = 0; // Angular velocity for turbine2

socket.on("predicted_rps", function (data) {
  try {
    const predictedRpm = parseFloat(data.predictedRPM) || 0.0; // Parse RPM
    const predictedVoltage = parseFloat(data.predictedVoltage) || 0.0; // Parse Voltage

    predictedAngularVelocity = (predictedRpm * 2 * Math.PI) / 60;

    // Format to 2 decimals
    const formattedRpm = `${predictedRpm.toFixed(2)} RPM`;
    const formattedVoltage = `${predictedVoltage.toFixed(2)} V`;

    // Update Prediction Section
    updateTablePrediction("RPM", formattedRpm);
    updateTablePrediction("Voltage", formattedVoltage);

    console.log(`Updated Predictions -> RPM: ${formattedRpm}, Voltage: ${formattedVoltage}`);
  } catch (error) {
    console.error("Error updating predictions:", error);
  }
});

// Socket.IO listeners to update table in real-time
socket.on("gravity_data", (data) => {
  const [grx, gry, grz] = data.gravity.split(",").map(Number);
  const gravityText = `grx: ${grx.toFixed(2)}, gry: ${gry.toFixed(2)}, grz: ${grz.toFixed(2)}`;
  updateTableMetric("Gravity", gravityText);
});

socket.on('rpm_data', function(data) {
  const rpmValue = parseFloat(data.latest_rpm).toFixed(2);
  updateTableMetric("RPM", rpmValue);
});




socket.on('voltage_data', function(data) {
  const voltValue = parseFloat(data.voltage).toFixed(2);
  updateTableMetric("Voltage", voltValue);
});

socket.on("linear_acceleration_data", (data) => {
  const [lx, ly, lz] = data.linear_acceleration.split(",").map(Number);
  const linearText = `lx: ${lx.toFixed(2)}, ly: ${ly.toFixed(2)}, lz: ${lz.toFixed(2)}`;
  updateTableMetric("Linear Acceleration", linearText);
});

socket.on("temperature_data", (data) => {
  const tempText = `${data.temperature.toFixed(2)} °C`;
  updateTableMetric("Temp", tempText);
});

socket.on("magnetometer_data", (data) => {
  const [mx, my, mz] = data.magnetometer.split(",").map(Number);
  const magnetText = `mx: ${mx.toFixed(2)}, my: ${my.toFixed(2)}, mz: ${mz.toFixed(2)}`;
  updateTableMetric("Magnetometer", magnetText);
});

socket.on("accelerometer_data", (data) => {
  const [ax, ay, az] = data.accelerometer.split(",").map(Number);
  const accelText = `ax: ${ax.toFixed(2)}, ay: ${ay.toFixed(2)}, az: ${az.toFixed(2)}`;
  updateTableMetric("Accelerometer", accelText);
});

socket.on("gyroscope_data", (data) => {
  const [gx, gy, gz] = data.gyroscope.split(",").map(Number);
  const gyroscopeText = `gx: ${gx.toFixed(2)}, gy: ${gy.toFixed(2)}, gz: ${gz.toFixed(2)}`;
  updateTableMetric("Gyroscope", gyroscopeText);
});

socket.on("orientation_data", (data) => {
  const [heading, roll, pitch] = data.orientation.split(",").map((v) => parseFloat(v).toFixed(2));
  const orientationText = `Heading: ${heading}, Roll: ${roll}, Pitch: ${pitch}`;
  updateTableMetric("Orientation", orientationText);
});

// Wind Speed Listener
socket.on("speed_data", (data) => {
  const windSpeedText = `${data.speed.toFixed(2)} m/s`;
  updateTableMetric("Wind Speed", windSpeedText);
});

// Wind Direction Listener
const reverseDirectionMap = {
  1: "N",
  2: "NE",
  3: "E",
  4: "SE",
  5: "S",
  6: "SW",
  7: "W",
  8: "NW",
};

socket.on("direction_data", (data) => {
  const directionNumeric = data.direction;
  const direction = reverseDirectionMap[directionNumeric] || "Unknown";
  updateTableMetric("Wind Direction", direction);
});

// Pressure Listener
socket.on("pressure_data", (data) => {
  const pressureText = `${data.pressure.toFixed(2)} hPa`;
  updateTableMetric("Wind Pressure", pressureText);
});

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

//            Wind Speed Slider Updater                     //

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// The Wind Speed Slider
const form = document.getElementById('predictionForm');
const windSpeedSlider = document.getElementById('windSpeed');
const windSpeedValue = document.getElementById('windSpeedValue');
const windSpeedInput = document.getElementById('windSpeedInput');
const updateSliderButton = document.getElementById('updateSliderButton');

// Function to update all elements (slider, text box, and display)
function updateWindSpeed(value) {
  const floatValue = parseFloat(value);

  windSpeedValue.textContent = floatValue.toFixed(1); // Update displayed value
  windSpeedSlider.value = floatValue; // Sync slider
  windSpeedInput.value = floatValue; // Sync text box
  updateTablePrediction("Wind Speed", `${floatValue.toFixed(1)} m/s`);
}

// Event listener for slider input
windSpeedSlider.addEventListener('input', () => {
  updateWindSpeed(windSpeedSlider.value); // Update based on slider movement
  submitForm(); // Automatically submit form
});

// Event listener for "Update Slider" button
updateSliderButton.addEventListener('click', () => {
  const value = windSpeedInput.value;
  if (!isNaN(value) && value >= 0 && value <= 300) {
    updateWindSpeed(value); // Update slider and text box
    submitForm(); // Submit the form after button press
  }
});

// Add change event listeners to dropdowns
const dropdowns = form.querySelectorAll('select');
dropdowns.forEach((dropdown) => {
  dropdown.addEventListener('change', () => {
    submitForm(); // Automatically submit when dropdown value changes
  });
});

function submitForm() {
  const formData = new FormData(form);
  const jsonData = {};

  // Convert form data to JSON
  formData.forEach((value, key) => {
    jsonData[key] = key === 'windSpeed' ? parseFloat(value) : value;
  });

  console.log('Submitting form data:', jsonData); // Debugging output

  // Send data via AJAX
  fetch('/submit_prediction', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(jsonData),
  })
    .then((response) => response.json())
    .then((data) => {
      console.log('Prediction submitted:', data);
    })
    .catch((error) => {
      console.error('Error submitting prediction:', error);
    });
}
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

// Animation Model from previous Dashboard but for analysis //

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
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
let blades2 = null;

// RPM setup
let rpm = 0; // Initial RPM
let angularVelocity = (rpm * 2 * Math.PI) / 60; // Convert to radians per second

loader.load(
  '/static/model/Turbine1.glb',
  (gltf) => {
      // Load the first turbine
      turbine = gltf.scene;
      turbine.scale.set(5, 5, 5);
      turbine.position.set(-1, -2, 0); // Position the first turbine to the left
      scene.add(turbine);

      console.log("Loaded turbine model:", turbine.children);

      // Access the "Empty" object and its children for the first turbine
      const emptyObject = turbine.getObjectByName("Empty");
      if (emptyObject) {
          console.log("Empty object found in first turbine:", emptyObject);

          blades = emptyObject.getObjectByName("remesh1");
          if (blades) {
              console.log("Blades object found in first turbine:", blades);
          } else {
              console.warn("Blades object ('remesh1') not found in first turbine!");
          }
      }

      // Clone the turbine for the second model
      turbine2 = turbine.clone();
      turbine2.position.set(2, -2, 0); // Position the second turbine to the right
      scene.add(turbine2);

      // Access the "Empty" object and its children for the second turbine
      const emptyObject2 = turbine2.getObjectByName("Empty");
      if (emptyObject2) {
          console.log("Empty object found in second turbine:", emptyObject2);

          blades2 = emptyObject2.getObjectByName("remesh1");
          if (blades2) {
              console.log("Blades object found in second turbine:", blades2);
          } else {
              console.warn("Blades object ('remesh1') not found in second turbine!");
          }
      }

      // Start animation loop
      animate();
  },
  undefined,
  (error) => {
      console.error("Error loading GLB file:", error);
  }
);

// Animation loop
let lastFrameTime = performance.now(); // Track the last frame time

function animate() {
  requestAnimationFrame(animate);

  const currentTime = performance.now();
  const deltaTime = (currentTime - lastFrameTime) / 1000; // Convert to seconds
  lastFrameTime = currentTime;

  // Update the first turbine
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

  if (blades) {
      blades.rotation.z += angularVelocity * deltaTime; // Rotate the first turbine's blades
  }

  // Update the second turbine
  if (turbine2) {
      currentRotation2.x += (targetRotation2.x - currentRotation2.x) * lerpSpeed;
      currentRotation2.y += (targetRotation2.y - currentRotation2.y) * lerpSpeed;
      currentRotation2.z += (targetRotation2.z - currentRotation2.z) * lerpSpeed;

      turbine2.rotation.set(
          currentRotation2.x,
          currentRotation2.y,
          currentRotation2.z
      );
  }

  if (blades2) {
      blades2.rotation.z += predictedAngularVelocity * deltaTime; // Rotate the second turbine's blades
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
let targetRotation2 = { x: 0, y: 0, z: 0 };
let currentRotation = { x: 0, y: 0, z: 0 };
let currentRotation2 = { x: 0, y: 0, z: 0 };
const lerpSpeed = 0.1; // Smoothing factor for rotation

// Update orientation based on real-time data
function updateOrientation(heading, roll, pitch) {
    targetRotation.x = THREE.MathUtils.degToRad(pitch);
    targetRotation.y = THREE.MathUtils.degToRad(heading);
    targetRotation.z = THREE.MathUtils.degToRad(roll);

    targetRotation2.x = THREE.MathUtils.degToRad(pitch);
    targetRotation2.y = THREE.MathUtils.degToRad(heading);
    targetRotation2.z = THREE.MathUtils.degToRad(roll);
}

// Update gyroscope display
function updateGyroscopeDisplay(gx, gy, gz) {
    //console.log(`Gyroscope Data: gx=${gx}, gy=${gy}, gz=${gz}`);
    // Apply gyroscope data to dynamically adjust rotation (example)
    if (turbine) {
        turbine.rotation.x += gx * 0.01;
        turbine.rotation.y += gy * 0.01;
        turbine.rotation.z += gz * 0.01;

        turbine2.rotation.x += gx * 0.01;
        turbine2.rotation.y += gy * 0.01;
        turbine2.rotation.z += gz * 0.01;
    }
}

// Update magnetometer display
function updateMagnetometerDisplay(mx, my, mz) {
    //console.log(`Magnetometer Data: mx=${mx}, my=${my}, mz=${mz}`);
    // Example of applying magnetometer data to an arrow helper
    const magnetoDirection = new THREE.Vector3(mx, my, mz).normalize();
    const magnetoLength = new THREE.Vector3(mx, my, mz).length();

}

// Update accelerometer display
function updateAccelerometerDisplay(ax, ay, az) {
    const accelDirection = new THREE.Vector3(ax, ay, az).normalize();
    const accelMagnitude = new THREE.Vector3(ax, ay, az).length();



}



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
    
    

