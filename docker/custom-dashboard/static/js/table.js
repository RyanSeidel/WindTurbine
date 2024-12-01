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
  { metric: "Orientation", realTime: "Heading: 0, Roll: 0, Pitch: 0", prediction: "Heading: 1, Roll: 0, Pitch: 1" },
  { metric: "Blades", realTime: "60", prediction: "60" },
  { metric: "Wind Direction", realTime: "N", prediction: "NE" },
  { metric: "Wind Speed", realTime: "0 m/s", prediction: "1 m/s" },
  { metric: "Accelerometer", realTime: "ax: 0, ay: 0, az: 0", prediction: "ax: 1, ay: 0.5, az: 0.3" },
  { metric: "RPM", realTime: "0", prediction: "0"},
  { metric: "Voltage", realTime: "0", prediction: "0"},

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
    data[index].prediction = newPrediction;
    // Update the corresponding table cell
    d3.select(`tr[data-index="${index}"] td:nth-child(3)`).text(newPrediction);
  }
};

socket.on('predicted_rpm', function (data) {
  const predictedRpmValue = `${data.rpm.toFixed(2)} RPM`;
  updateTablePrediction("RPM", predictedRpmValue);
});

socket.on('predicted_voltage', function (data) {
  const predictedVoltValue = `${data.voltage.toFixed(2)} V`; // Correct the data reference and label
  updateTablePrediction("Voltage", predictedVoltValue); // Update the "Voltage" row in the table
});

// Socket.IO listeners to update table in real-time
socket.on("gravity_data", (data) => {
  const [grx, gry, grz] = data.gravity.split(",").map(Number);
  const gravityText = `grx: ${grx.toFixed(2)}, gry: ${gry.toFixed(2)}, grz: ${grz.toFixed(2)}`;
  updateTableMetric("Gravity", gravityText);
});

socket.on('rpm_data', function(data) {
  const rpmValue = data.latest_rpm;
  updateTableMetric("RPM", rpmValue);
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
