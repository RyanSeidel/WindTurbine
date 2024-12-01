//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~// 
// Real-Time Accelerometer and Gyroscope Chart // // By [Wind Turbine Digital Twins]                               //                         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~// 
// Description:                                                                                                    // 
// This script generates a dynamic and interactive chart using D3.js to visualize real-time accelerometer          // 
// (Ax, Ay, Az) and gyroscope (Gx, Gy, Gz) data received via Socket.IO. The chart updates periodically to          // 
// reflect new sensor readings, offering an intuitive view of motion and orientation data.                         // 
//                                                                                                                 // 
// Key Features:                                                                                                   // 
// Real-Time Data Visualization:                                                                                   // 
// - Continuously updates the chart with incoming accelerometer and gyroscope data every 2 seconds.                // 
// Dynamic Line Chart:                                                                                             // 
// - Plots six metrics (Ax, Ay, Az, Gx, Gy, Gz) using distinct color-coded lines for clear differentiation.        // 
// Adaptive Scaling:                                                                                               // 
// - Dynamically adjusts Y-axis scales to fit the latest data ranges, ensuring optimal display for all values.     // 
// Tooltips for Interaction:                                                                                       // 
// - Displays detailed data, including time and all six metrics, when hovering over data points.                   // 
// Responsive Design:                                                                                              // 
// - Automatically resizes to fit the container, ensuring compatibility with various screen sizes.                 //
 // Smooth Transitions:                                                                                            // 
 // - Animates chart updates, including axis scaling and line adjustments, for a seamless user experience.         //
  // Lightweight Data Handling:                                                                                    // 
  // - Maintains a fixed dataset size by discarding older data points as new ones are added.                       // 
  //                                                                                                               // 
  // This tool provides a comprehensive and visually engaging solution for analyzing motion and orientation data,  // 
  // suitable for use cases in robotics, IoT, and real-time telemetry monitoring.                                  // 
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
let accAx = 0;
let accAy = 0;
let accAz = 0;
let Lx = 0;
let Ly = 0;
let Lz = 0;

// Initial Accelerometer Data and Settings
let accCurrentTime = new Date();
let accData = []; // Start with an empty dataset
const accMaxDataPoints = 6; // Maximum number of points before resetting
const accInterval = 2000; // Interval for adding new points (2 seconds)

// Set dimensions and margins
const accMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const accContainerWidth = document.querySelector(".accelerometer-chart").offsetWidth;
const accWidth = accContainerWidth - accMargin.left - accMargin.right;
const accHeight = 200 - accMargin.top - accMargin.bottom;


// Create SVG container
const accSVG = d3
  .select(".accelerometer-chart")
  .append("svg")
  .attr("width", accWidth + accMargin.left + accMargin.right)
  .attr("height", accHeight + accMargin.top + accMargin.bottom)
  .style("background", "#1a1a1a");

// Add a title to the chart
accSVG.append("text")
  .attr("x", accMargin.left)
  .attr("y", 20)
  .attr("text-anchor", "start")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Accelerometer and Linear Accleration Data");

// Add label for Ax
// Labels for Ax, Ay, Az, Gx, Gy, Gz
const labels = [
  { label: "Ax", color: "#ff0000", x: 340 },
  { label: "Ay", color: "#00ff00", x: 365 },
  { label: "Az", color: "#0000ff", x: 390 },
  { label: "Lx", color: "#ffff00", x: 415 },
  { label: "Ly", color: "#ff00ff", x: 440 },
  { label: "Lz", color: "#00ffff", x: 465 }
];

labels.forEach(({ label, color, x }) => {
  accSVG.append("text")
    .attr("x", accMargin.left + x)
    .attr("y", 20)
    .attr("text-anchor", "start")
    .attr("fill", color)
    .attr("font-size", "16px")
    .text(label);
});

// Append group for the main chart content
const accChartGroup = accSVG.append("g")
  .attr("transform", `translate(${accMargin.left},${accMargin.top})`);

// Create scales
const accX = d3
  .scaleTime()
  .domain([accCurrentTime, new Date(accCurrentTime.getTime() + accMaxDataPoints * accInterval)])
  .range([0, accWidth]);

const accY = d3
  .scaleLinear()
  .domain([-1.5, 15]) // Example domain for ax, ay, and az
  .range([accHeight, 0]);

// Add X-axis
const accXAxis = accChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${accHeight})`)
  .call(
    d3.axisBottom(accX)
      .ticks(maxDataPoints) // Limit to 6 ticks
      .tickSize(-accHeight) // Extend tick lines as grid lines
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
  )
  .selectAll("text")
  .attr("fill", "#ddd") // Text color for tick labels
  .attr("font-size", "10px"); // Font size for tick labels

accChartGroup.append("text")
  .attr("x", width / 2)
  .attr("y", height + 40)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .text("Time");

// Add Y-axis
accChartGroup.append("g")
  .attr("class", "y-axis")
  .call(d3.axisLeft(accY).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis label
accChartGroup.append("text")
  .attr("x", -accHeight / 2)
  .attr("y", -50)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Acceleration (m/s²)");

const lineGenerators = {
    ax: d3.line().x(d => accX(d.time)).y(d => accY(d.ax)),
    ay: d3.line().x(d => accX(d.time)).y(d => accY(d.ay)),
    az: d3.line().x(d => accX(d.time)).y(d => accY(d.az)),
    gx: d3.line().x(d => accX(d.time)).y(d => accY(d.gx)),
    gy: d3.line().x(d => accX(d.time)).y(d => accY(d.gy)),
    gz: d3.line().x(d => accX(d.time)).y(d => accY(d.gz))
  };

// Line generators for ax, ay, and az
const lines = {};
["ax", "ay", "az", "gx", "gy", "gz"].forEach((key, index) => {
  const colors = ["#ff0000", "#00ff00", "#0000ff", "#ffff00", "#ff00ff", "#00ffff"];
  lines[key] = accChartGroup.append("path")
    .attr("fill", "none")
    .attr("stroke", colors[index])
    .attr("stroke-width", 2);
});




// Tooltip setup
const acctooltip = d3
  .select(".accelerometer-chart")
  .append("div")
  .style("position", "absolute")
  .style("background", "#222")
  .style("color", "#fff")
  .style("padding", "5px 10px")
  .style("border-radius", "5px")
  .style("pointer-events", "none")
  .style("opacity", 0);

// Socket.IO listener for accelerometer data
socket.on("accelerometer_data", (data) => {
  console.log("Accelerometer data received: ", data);

  // Parse accelerometer data
  const [ax, ay, az] = data.accelerometer.split(",").map(Number);

  // Update global variables for Ax, Ay, and Az
  accAx = ax;
  accAy = ay;
  accAz = az;

});

socket.on('linear_acceleration_data', (data) => {
  //console.log("linear_acceleration data received: ", data);  // Debugging log
  const [lx, ly, lz] = data.linear_acceleration.split(',').map(Number); // Split and convert to numbers
  Lx = lx;
  Ly = ly;
  Lz = lz;
});

function updateAccChart() {
  // Generate new data point with the latest accelerometer and linear acceleration values
  const currentTime = new Date();
  accData.push({ time: currentTime, ax: accAx, ay: accAy, az: accAz, lx: Lx, ly: Ly, lz: Lz });

  // Remove oldest data point if exceeding accMaxDataPoints
  if (accData.length > accMaxDataPoints + 1) {
    accData.shift();
  }

  // Calculate the extent (min and max) of all data points for accelerometer and linear acceleration
  const allExtents = ["ax", "ay", "az", "lx", "ly", "lz"].flatMap(key => d3.extent(accData, d => d[key]));
  const overallExtent = [Math.min(...allExtents), Math.max(...allExtents)];

  // Update the Y-axis domain dynamically
  accY.domain([overallExtent[0] - 2, overallExtent[1] + 2]);

  // Update the Y-axis
  accChartGroup.select(".y-axis")
    .transition()
    .duration(500)
    .call(d3.axisLeft(accY).ticks(5))
    .selectAll("text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Update the X-axis domain
  accX.domain([accData[0].time, new Date(accData[0].time.getTime() + accMaxDataPoints * accInterval)]);

  // Update the X-axis
  accChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(
      d3.axisBottom(accX)
        .ticks(accMaxDataPoints)
        .tickSize(-accHeight)
        .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
    )
    .selectAll("text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Style grid lines for the X-axis
  accChartGroup.selectAll(".x-axis .tick line")
    .attr("stroke", "#444")
    .attr("stroke-dasharray", "2,2");

  // Update lines for Ax, Ay, Az, Lx, Ly, and Lz
  Object.keys(lineGenerators).forEach(key => {
    lines[key]
      .datum(accData)
      .transition()
      .duration(500)
      .attr("d", lineGenerators[key]);
  });

  // Update circles for data points (optional, for accelerometer only)
  const circles = accChartGroup.selectAll(".data-point").data(accData);

  circles
    .enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 4)
    .merge(circles)
    .attr("cx", d => accX(d.time))
    .attr("cy", d => accY(d.ax)) // Use Ax for tooltip demonstration
    .attr("fill", "#ddd")
    .on("mouseover", (event, d) => {
      acctooltip
        .style("opacity", 1)
        .html(
          `Time: ${d.time.toLocaleTimeString()}<br>
          Ax: ${d.ax.toFixed(2)}, Ay: ${d.ay.toFixed(2)}, Az: ${d.az.toFixed(2)}<br>
          Lx: ${d.lx.toFixed(2)}, Ly: ${d.ly.toFixed(2)}, Lz: ${d.lz.toFixed(2)}`
        )
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      acctooltip.style("opacity", 0);
    });

  circles.exit().remove();
}

// Update chart with new data at the specified interval
setInterval(updateAccChart, accInterval);
