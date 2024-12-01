let predictedMagnitude = 0;
let actualMagnitude = 0;
let residual = 0;
let threshold = 0;
let anomaly = false;

// Initial Anomaly Data and Settings
let anomalyCurrentTime = new Date();
let anomalyData = []; // Start with an empty dataset
const anomalyMaxDataPoints = 6; // Maximum number of points before resetting
const anomalyInterval = 2000; // Interval for adding new points (2 seconds)

// Set dimensions and margins
const anomalyMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const anomalyContainerWidth = document.querySelector(".anomaly-chart").offsetWidth;
const anomalyWidth = anomalyContainerWidth - anomalyMargin.left - anomalyMargin.right;
const anomalyHeight = 200 - anomalyMargin.top - anomalyMargin.bottom;

// Create SVG container
const anomalySVG = d3
  .select(".anomaly-chart")
  .append("svg")
  .attr("width", anomalyWidth + anomalyMargin.left + anomalyMargin.right)
  .attr("height", anomalyHeight + anomalyMargin.top + anomalyMargin.bottom)
  .style("background", "#1a1a1a");

// Add a title to the chart
anomalySVG.append("text")
  .attr("x", anomalyMargin.left)
  .attr("y", 20)
  .attr("text-anchor", "start")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Dynamic Anomaly Detection Data");

// Add labels for Predicted, Actual, Residual, and Threshold
const anomalyLabels = [
  { label: "Predicted", color: "#ff7f0e", x: 200 },
  { label: "Actual", color: "#1f77b4", x: 270 },
  { label: "Residual", color: "#2ca02c", x: 340 },
  { label: "Threshold", color: "#d62728", x: 410 }
];

anomalyLabels.forEach(({ label, color, x }) => {
  anomalySVG.append("text")
    .attr("x", anomalyMargin.left + x)
    .attr("y", 20)
    .attr("text-anchor", "start")
    .attr("fill", color)
    .attr("font-size", "16px")
    .text(label);
});

// Append group for the main chart content
const anomalyChartGroup = anomalySVG.append("g")
  .attr("transform", `translate(${anomalyMargin.left},${anomalyMargin.top})`);

// Create scales
const anomalyX = d3
  .scaleTime()
  .domain([anomalyCurrentTime, new Date(anomalyCurrentTime.getTime() + anomalyMaxDataPoints * anomalyInterval)])
  .range([0, anomalyWidth]);

const anomalyY = d3
  .scaleLinear()
  .domain([0, 15]) // Example domain for magnitude and residual
  .range([anomalyHeight, 0]);

// Add X-axis
const anomalyXAxis = anomalyChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${anomalyHeight})`)
  .call(
    d3.axisBottom(anomalyX)
      .ticks(anomalyMaxDataPoints)
      .tickSize(-anomalyHeight)
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis
anomalyChartGroup.append("g")
  .attr("class", "y-axis")
  .call(d3.axisLeft(anomalyY).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis label
anomalyChartGroup.append("text")
  .attr("x", -anomalyHeight / 2)
  .attr("y", -50)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Magnitude");

// Line generators for Predicted, Actual, Residual, and Threshold
const anomalyLineGenerators = {
  predicted: d3.line().x(d => anomalyX(d.time)).y(d => anomalyY(d.predicted)),
  actual: d3.line().x(d => anomalyX(d.time)).y(d => anomalyY(d.actual)),
  residual: d3.line().x(d => anomalyX(d.time)).y(d => anomalyY(d.residual)),
  threshold: d3.line().x(d => anomalyX(d.time)).y(d => anomalyY(d.threshold))
};

// Initialize lines
const anomalyLines = {};
["predicted", "actual", "residual", "threshold"].forEach((key, index) => {
  const colors = ["#ff7f0e", "#1f77b4", "#2ca02c", "#d62728"];
  anomalyLines[key] = anomalyChartGroup.append("path")
    .attr("fill", "none")
    .attr("stroke", colors[index])
    .attr("stroke-width", 2);
});

// Tooltip setup
const anomalyTooltip = d3
  .select(".anomaly-chart")
  .append("div")
  .style("position", "absolute")
  .style("background", "#222")
  .style("color", "#fff")
  .style("padding", "5px 10px")
  .style("border-radius", "5px")
  .style("pointer-events", "none")
  .style("opacity", 0);

// Socket.IO listener for anomaly data
socket.on("anomaly_data", (data) => {
  console.log("Anomaly data received: ", data);

  // Parse anomaly data
  predictedMagnitude = data.Predicted_Magnitude;
  actualMagnitude = data.Actual_Magnitude;
  residual = data.Residual;
  threshold = data.Threshold;
  anomaly = data.Anomaly; // Boolean
});

function updateAnomalyChart() {
  const currentTime = new Date();
  anomalyData.push({
    time: currentTime,
    predicted: predictedMagnitude,
    actual: actualMagnitude,
    residual: residual,
    threshold: threshold,
    anomaly: anomaly
  });

  if (anomalyData.length > anomalyMaxDataPoints + 1) {
    anomalyData.shift();
  }

  const allExtents = ["predicted", "actual", "residual", "threshold"].flatMap(key => d3.extent(anomalyData, d => d[key]));
  const overallExtent = [Math.min(...allExtents) - 2, Math.max(...allExtents) + 2];
  anomalyY.domain(overallExtent);

  anomalyChartGroup.select(".y-axis")
    .transition()
    .duration(500)
    .call(d3.axisLeft(anomalyY).ticks(5));

  anomalyX.domain([anomalyData[0].time, new Date(anomalyData[0].time.getTime() + anomalyMaxDataPoints * anomalyInterval)]);

  anomalyChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(d3.axisBottom(anomalyX).ticks(anomalyMaxDataPoints));

  Object.keys(anomalyLineGenerators).forEach(key => {
    anomalyLines[key]
      .datum(anomalyData)
      .transition()
      .duration(500)
      .attr("d", anomalyLineGenerators[key]);
  });

  const circles = anomalyChartGroup.selectAll(".data-point").data(anomalyData);

  circles.enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 4)
    .merge(circles)
    .attr("cx", d => anomalyX(d.time))
    .attr("cy", d => anomalyY(d.residual)) // Use Residual for tooltip demonstration
    .attr("fill", d => (d.anomaly ? "#d62728" : "#2ca02c")) // Highlight anomalies in red
    .on("mouseover", (event, d) => {
      anomalyTooltip
        .style("opacity", 1)
        .html(
          `Time: ${d.time.toLocaleTimeString()}<br>
          Predicted: ${d.predicted.toFixed(2)}, Actual: ${d.actual.toFixed(2)}<br>
          Residual: ${d.residual.toFixed(2)}, Threshold: ${d.threshold.toFixed(2)}<br>
          Anomaly: ${d.anomaly ? "Yes" : "No"}`
        )
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      anomalyTooltip.style("opacity", 0);
    });

  circles.exit().remove();
}

// Update chart with new data at the specified interval
setInterval(updateAnomalyChart, anomalyInterval);
