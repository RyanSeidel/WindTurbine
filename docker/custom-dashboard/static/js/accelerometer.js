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

let accCurrentTime = new Date();
let accData = [];
const accMaxDataPoints = 6;
const accInterval = 2000;

const accMargin = { top: 30, right: 100, bottom: 50, left: 70 };
const accContainerWidth = document.querySelector(".accelerometer-chart").offsetWidth;
const accWidth = accContainerWidth - accMargin.left - accMargin.right;
const accHeight = 225 - accMargin.top - accMargin.bottom;

const accSVG = d3
  .select(".accelerometer-chart")
  .append("svg")
  .attr("width", accWidth + accMargin.left + accMargin.right)
  .attr("height", accHeight + accMargin.top + accMargin.bottom)
  .style("background", "#1a1a1a");

accSVG.append("text")
  .attr("x", accMargin.left)
  .attr("y", 20)
  .attr("text-anchor", "start")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Accelerometer and Linear Acceleration Data");

// Labels for the primary Y-axis (Lx, Ly, Lz, Ax, Ay)
const primaryLabels = [
  { label: "Lx", color: "#ffff00", offset: 0 },
  { label: "Ly", color: "#ff00ff", offset: 20 },
  { label: "Lz", color: "#00ffff", offset: 40 },
  { label: "Ax", color: "#ff0000", offset: 60 },
  { label: "Ay", color: "#00ff00", offset: 80 }
];

primaryLabels.forEach(({ label, color, offset }) => {
  accSVG.append("text")
    .attr("x", accMargin.left - 50) // Position near primary Y-axis
    .attr("y", accMargin.top + offset)
    .attr("text-anchor", "center")
    .attr("fill", color)
    .attr("font-size", "14px")
    .text(label);
});

// Label for the secondary Y-axis (Az)
accSVG.append("text")
  .attr("x", accMargin.left + accWidth + 40) // Position near secondary Y-axis
  .attr("y", accMargin.top)
  .attr("text-anchor", "start")
  .attr("fill", "#0000ff") // Color for Az
  .attr("font-size", "16px")
  .text("Az");

const accChartGroup = accSVG.append("g")
  .attr("transform", `translate(${accMargin.left},${accMargin.top})`);

const accX = d3
  .scaleTime()
  .domain([accCurrentTime, new Date(accCurrentTime.getTime() + accMaxDataPoints * accInterval)])
  .range([0, accWidth]);

const accYPrimary = d3
  .scaleLinear()
  .domain([-1.5, 1.5])
  .range([accHeight, 0]);

const accYSecondary = d3
  .scaleLinear()
  .domain([0, 15])
  .range([accHeight, 0]);

const accXAxis = accChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${accHeight})`)
  .call(d3.axisBottom(accX).ticks(accMaxDataPoints))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

accChartGroup.append("g")
  .attr("class", "y-axis-primary")
  .call(d3.axisLeft(accYPrimary).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

accChartGroup.append("g")
  .attr("class", "y-axis-secondary")
  .attr("transform", `translate(${accWidth}, 0)`)
  .call(d3.axisRight(accYSecondary).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

const lineGenerators = {
  ax: d3.line().x(d => accX(d.time)).y(d => accYPrimary(d.ax)),
  ay: d3.line().x(d => accX(d.time)).y(d => accYPrimary(d.ay)),
  az: d3.line().x(d => accX(d.time)).y(d => accYSecondary(d.az)),
  lx: d3.line().x(d => accX(d.time)).y(d => accYPrimary(d.lx)),
  ly: d3.line().x(d => accX(d.time)).y(d => accYPrimary(d.ly)),
  lz: d3.line().x(d => accX(d.time)).y(d => accYPrimary(d.lz))
};

const lines = {};
["ax", "ay", "az", "lx", "ly", "lz"].forEach((key, index) => {
  const colors = ["#ff0000", "#00ff00", "#0000ff", "#ffff00", "#ff00ff", "#00ffff"];
  lines[key] = accChartGroup.append("path")
    .attr("fill", "none")
    .attr("stroke", colors[index])
    .attr("stroke-width", 2);
});

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

socket.on("accelerometer_data", (data) => {
  const [ax, ay, az] = data.accelerometer.split(",").map(Number);
  accAx = ax;
  accAy = ay;
  accAz = az;
});

socket.on('linear_acceleration_data', (data) => {
  const [lx, ly, lz] = data.linear_acceleration.split(',').map(Number);
  Lx = lx;
  Ly = ly;
  Lz = lz;
});

function updateAccChart() {
  const currentTime = new Date();
  accData.push({ time: currentTime, ax: accAx, ay: accAy, az: accAz, lx: Lx, ly: Ly, lz: Lz });
  if (accData.length > accMaxDataPoints + 1) accData.shift();

  const primaryExtent = ["ax", "ay", "lx", "ly", "lz"]
    .flatMap(key => d3.extent(accData, d => d[key]));
  accYPrimary.domain([Math.min(...primaryExtent) - 0.5, Math.max(...primaryExtent) + 0.5]);

  const secondaryExtent = d3.extent(accData, d => d.az);
  accYSecondary.domain([secondaryExtent[0] - 2, secondaryExtent[1] + 2]);

  accX.domain([accData[0].time, new Date(accData[0].time.getTime() + accMaxDataPoints * accInterval)]);

  accChartGroup.select(".y-axis-primary")
    .transition().duration(500)
    .style("color", "#fff")
    .call(d3.axisLeft(accYPrimary));

  accChartGroup.select(".y-axis-secondary")
    .transition().duration(500)
    .style("color", "#fff")
    .call(d3.axisRight(accYSecondary));

  accChartGroup.select(".x-axis")
    .transition().duration(500)
    .style("color", "#fff")
    .call(d3.axisBottom(accX).ticks(accMaxDataPoints));

  Object.keys(lineGenerators).forEach(key => {
    lines[key]
      .datum(accData)
      .transition()
      .duration(500)
      .attr("d", lineGenerators[key]);
  });

  // Tooltip circles
  const circles = accChartGroup.selectAll(".data-point").data(accData);

  circles.enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 4)
    .merge(circles)
    .attr("cx", d => accX(d.time))
    .attr("cy", d => accYPrimary(d.ax)) // Use Ax as example
    .attr("fill", "#ddd")
    .on("mouseover", (event, d) => {
      acctooltip
        .style("opacity", 1)
        .html(`
          Time: ${d.time.toLocaleTimeString()}<br>
          Ax: ${d.ax.toFixed(2)}, Ay: ${d.ay.toFixed(2)}, Az: ${d.az.toFixed(2)}<br>
          Lx: ${d.lx.toFixed(2)}, Ly: ${d.ly.toFixed(2)}, Lz: ${d.lz.toFixed(2)}
        `)
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      acctooltip.style("opacity", 0);
    });

  circles.exit().remove();
}

setInterval(updateAccChart, accInterval);
