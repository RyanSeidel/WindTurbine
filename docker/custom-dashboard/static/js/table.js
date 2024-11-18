const socket = io(); // Initialize Socket.IO
// Data for real-time and prediction
let data = [
  { metric: "Orientation", realTime: "Heading: 0, Roll: 0, Pitch: 0", prediction: "Heading: 1, Roll: 0, Pitch: 1" },
  { metric: "Temp", realTime: "0", prediction: "0.2" },
  { metric: "Wind Direction", realTime: "0", prediction: "1" },
  { metric: "Wind Pressure", realTime: "0", prediction: "0.1" },
  { metric: "Wind Speed", realTime: "0 mph", prediction: "1 mph" },
  { metric: "Magnetometer", realTime: "mx: 0, my: 0, mz: 0", prediction: "mx: 1, my: 0.5, mz: 0.3" },
  { metric: "Gyroscope", realTime: "gx: 0, gy: 0, gz: 0", prediction: "gx: 1, gy: 1, gz: 1" },
  { metric: "Accelerometer", realTime: "ax: 0, ay: 0, az: 0", prediction: "ax: 1, ay: 0.5, az: 0.3" },
  { metric: "Linear Acceleration", realTime: "lx: 0, ly: 0, lz: 0", prediction: "lx: 1, ly: 0.5, lz: 0.3" },
  { metric: "Gravity", realTime: "grx: 0, gry: 0, grz: 0", prediction: "grx: 1, gry: 0.8, grz: 0.6" },
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

// Socket.IO listeners to update table in real-time
socket.on("gravity_data", (data) => {
  const [grx, gry, grz] = data.gravity.split(",").map(Number);
  const gravityText = `grx: ${grx.toFixed(2)}, gry: ${gry.toFixed(2)}, grz: ${grz.toFixed(2)}`;
  updateTableMetric("Gravity", gravityText);
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

// Selection with Graph
document.getElementById("start-button").addEventListener("click", () => {
  const realTimeMetric = document.getElementById("real-time-metric").value;
  const predictionMetric = document.getElementById("prediction-metric").value;

  if (realTimeMetric && predictionMetric) {
    console.log(`Comparing Real-Time: ${realTimeMetric} with Prediction: ${predictionMetric}`);
    // Add logic to update graph or display comparison
  } else {
    alert("Please select both metrics.");
  }
});
