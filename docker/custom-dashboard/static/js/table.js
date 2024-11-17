// Data for real-time and prediction
const data = [
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
  data.forEach((row) => {
    const tr = tbody.append("tr");
    tr.append("td").text(row.metric);
    tr.append("td").text(row.realTime);
    tr.append("td").text(row.prediction);
  });
};

// Create the table
createTable(data);

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

