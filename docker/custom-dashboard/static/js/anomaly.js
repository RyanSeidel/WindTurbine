//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Anomaly Detection Chart for RPM and Magnitude Residuals                                                       //
// By [Wind Turbine Digital Twins]                                                                               //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Description:                                                                                                  //
// This script creates a real-time D3.js chart for detecting and visualizing anomalies in RPM and Magnitude data.//
// The chart dynamically updates with incoming residual data, displays threshold lines, and highlights anomalies.//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// Initialize residuals and thresholds for Magnitude and RPM
let residual = 0; // Current residual for RPM
let threshold = 0.3; // Initial threshold value
let anomaly = false;

let rpm_residual = 0; // Current residual for RPM
let rpm_threshold = 0.3; // Initial RPM threshold
let rpm_anomaly = false;

// Initial Anomaly Data and Settings
let anomalyCurrentTime = new Date();
let anomalyData = []; // Start with an empty dataset
const anomalyMaxDataPoints = 6; // Maximum number of points before resetting
const anomalyInterval = 1000; // Interval for adding new points (1 second)

// Set dimensions and margins
const anomalyMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const anomalyContainerWidth = document.querySelector(".anomaly-chart").offsetWidth;
const anomalyWidth = anomalyContainerWidth - anomalyMargin.left - anomalyMargin.right;
const anomalyHeight = 285 - anomalyMargin.top - anomalyMargin.bottom;

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
  .html(`
    Anomaly Detection (<tspan fill="#ff7f0e">RPM</tspan> vs <tspan fill="#00FF00">Magnitude</tspan>)
  `);

// Append group for the main chart content
const anomalyChartGroup = anomalySVG.append("g")
  .attr("transform", `translate(${anomalyMargin.left},${anomalyMargin.top})`);

// Create scales
let anomalyX = d3
  .scaleTime()
  .domain([anomalyCurrentTime, new Date(anomalyCurrentTime.getTime() + anomalyMaxDataPoints * anomalyInterval)])
  .range([0, anomalyWidth]);

let anomalyY = d3
  .scaleLinear()
  .domain([0, 1]) // Adjusted initial domain for better visualization
  .range([anomalyHeight, 0]);

// Add X-axis
anomalyChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${anomalyHeight})`)
  .call(d3.axisBottom(anomalyX).ticks(anomalyMaxDataPoints))
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
  .text("Residual");

// Line generator for Magnitude Residual
const magnitudeLineGenerator = d3
  .line()
  .x(d => anomalyX(d.time))
  .y(d => anomalyY(d.residual));

// Initialize line for Magnitude Residual
const magnitudeLine = anomalyChartGroup.append("path")
  .attr("class", "magnitude-line")
  .attr("fill", "none")
  .attr("stroke", "#2ca02c") // Green for Magnitude
  .attr("stroke-width", 2);

// Line generator for RPM Residual
const rpmLineGenerator = d3
  .line()
  .x(d => anomalyX(d.time))
  .y(d => anomalyY(d.residual));

// Initialize line for RPM Residual
const rpmLine = anomalyChartGroup.append("path")
  .attr("class", "rpm-line")
  .attr("fill", "none")
  .attr("stroke", "#ff7f0e") // Orange for RPM
  .attr("stroke-width", 2);

// Add Threshold Line for Magnitude
const thresholdLine = anomalyChartGroup.append("line")
  .attr("class", "threshold-line")
  .attr("x1", 0)
  .attr("x2", anomalyWidth)
  .attr("y1", anomalyY(threshold))
  .attr("y2", anomalyY(threshold))
  .attr("stroke", "#d62728")
  .attr("stroke-width", 2)
  .attr("stroke-dasharray", "5,5");

// Add Threshold Line for RPM
const rpmThresholdLine = anomalyChartGroup.append("line")
  .attr("class", "rpm-threshold-line")
  .attr("x1", 0)
  .attr("x2", anomalyWidth)
  .attr("y1", anomalyY(rpm_threshold))
  .attr("y2", anomalyY(rpm_threshold))
  .attr("stroke", "#ff7f0e")
  .attr("stroke-width", 2)
  .attr("stroke-dasharray", "5,5");

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
  residual = data.Magnitude_Residual;
  threshold = data.Magnitude_Threshold; // Update threshold dynamically
  anomaly = residual > threshold;

  // RPM anomaly data
  rpm_residual = data.RPM_Residual / 100;
  rpm_threshold = data.RPM_Threshold / 100;
  rpm_anomaly = rpm_residual > rpm_threshold;
});

// Update function for the chart
function updateAnomalyChart() {
  const currentTime = new Date();

  // Add data for Magnitude anomalies
  anomalyData.push({
    time: currentTime,
    residual: residual,
    threshold: threshold,
    anomaly: anomaly,
    type: "Magnitude"
  });

  // Add data for RPM anomalies
  anomalyData.push({
    time: currentTime,
    residual: rpm_residual,
    threshold: rpm_threshold,
    anomaly: rpm_anomaly,
    type: "RPM"
  });

  if (anomalyData.length > anomalyMaxDataPoints * 2 + 2) {
    // Remove pairs of Magnitude and RPM data points
    anomalyData.shift(); // Removes the oldest "Magnitude" data point
    anomalyData.shift(); // Removes the oldest "RPM" data point
  }

  // Update the Y-axis domain dynamically
  const combinedExtent = [
    Math.min(...anomalyData.map(d => d.residual), ...anomalyData.map(d => d.threshold)) - 0.1,
    Math.max(...anomalyData.map(d => d.residual), ...anomalyData.map(d => d.threshold)) + 0.1
  ];

  anomalyY.domain(combinedExtent);

  // Update scales and redraw axes
  anomalyChartGroup.select(".y-axis")
    .transition()
    .duration(500)
    .style("color", "#fff")
    .call(d3.axisLeft(anomalyY).ticks(5))
    ;

  anomalyX.domain([anomalyData[0].time, new Date(anomalyData[0].time.getTime() + anomalyMaxDataPoints * anomalyInterval)]);

  anomalyChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .style("color", "#fff")
    .call(d3.axisBottom(anomalyX).ticks(anomalyMaxDataPoints));

  // Update the Magnitude Residual line
  magnitudeLine
    .datum(anomalyData.filter(d => d.type === "Magnitude"))
    .transition()
    .duration(500)
    .attr("d", magnitudeLineGenerator);

  // Update the RPM Residual line
  rpmLine
    .datum(anomalyData.filter(d => d.type === "RPM"))
    .transition()
    .duration(500)
    .attr("d", rpmLineGenerator);

  // Update the Magnitude Threshold line
  thresholdLine
    .transition()
    .duration(500)
    .attr("y1", anomalyY(threshold))
    .attr("y2", anomalyY(threshold));

  // Update the RPM Threshold line
  rpmThresholdLine
    .transition()
    .duration(500)
    .attr("y1", anomalyY(rpm_threshold))
    .attr("y2", anomalyY(rpm_threshold));

  const circles = anomalyChartGroup.selectAll(".data-point").data(anomalyData);

  circles.enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 5)
    .merge(circles)
    .attr("cx", d => anomalyX(d.time))
    .attr("cy", d => anomalyY(d.residual))
    .attr("fill", d => (d.anomaly ? "#d62728" : "#2ca02c")) // Highlight anomalies in red
    .on("mouseover", (event, d) => {
      anomalyTooltip
        .style("opacity", 1)
        .html(
          `<strong>${d.type} Anomaly</strong><br>
          Time: ${d.time.toLocaleTimeString()}<br>
          Residual: ${d.residual.toFixed(2)}<br>
          Threshold: ${d.threshold.toFixed(2)}<br>
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
