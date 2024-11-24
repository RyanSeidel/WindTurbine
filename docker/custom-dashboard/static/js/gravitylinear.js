let oriHeading = 0;
let oriRoll = 0;
let oriPitch = 0;

// Initial Orientation Data and Settings
let oriCurrentTime = new Date();
let oriData = []; // Start with an empty dataset
const oriMaxDataPoints = 6; // Maximum number of points before resetting
const oriInterval = 2000; // Interval for adding new points (2 seconds)

// Set dimensions and margins
const oriMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const oriContainerWidth = document.querySelector(".orientation-chart").offsetWidth;
const oriWidth = oriContainerWidth - oriMargin.left - oriMargin.right;
const oriHeight = 200 - oriMargin.top - oriMargin.bottom;

// Create SVG container
const oriSVG = d3
  .select(".orientation-chart")
  .append("svg")
  .attr("width", oriWidth + oriMargin.left + oriMargin.right)
  .attr("height", oriHeight + oriMargin.top + oriMargin.bottom)
  .style("background", "#1a1a1a");

// Add a title to the chart
oriSVG.append("text")
  .attr("x", oriMargin.left)
  .attr("y", 20)
  .attr("text-anchor", "start")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Dynamic Orientation Data");

// Add labels for Heading, Roll, Pitch
const oriLabels = [
  { label: "Heading", color: "#ff7f0e", x: 200 },
  { label: "Roll", color: "#1f77b4", x: 270 },
  { label: "Pitch", color: "#2ca02c", x: 305 }
];

oriLabels.forEach(({ label, color, x }) => {
  oriSVG.append("text")
    .attr("x", oriMargin.left + x)
    .attr("y", 20)
    .attr("text-anchor", "start")
    .attr("fill", color)
    .attr("font-size", "16px")
    .text(label);
});

// Append group for the main chart content
const oriChartGroup = oriSVG.append("g")
  .attr("transform", `translate(${oriMargin.left},${oriMargin.top})`);

// Create scales
const oriX = d3
  .scaleTime()
  .domain([oriCurrentTime, new Date(oriCurrentTime.getTime() + oriMaxDataPoints * oriInterval)])
  .range([0, oriWidth]);

const oriY = d3
  .scaleLinear()
  .domain([0, 360]) // Orientation is typically in degrees, ranging from 0 to 360
  .range([oriHeight, 0]);

// Add X-axis
const oriXAxis = oriChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${oriHeight})`)
  .call(
    d3.axisBottom(oriX)
      .ticks(oriMaxDataPoints) // Limit to 6 ticks
      .tickSize(-oriHeight) // Extend tick lines as grid lines
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
  )
  .selectAll("text")
  .attr("fill", "#ddd") // Text color for tick labels
  .attr("font-size", "10px"); // Font size for tick labels

oriChartGroup.append("text")
  .attr("x", oriWidth / 2)
  .attr("y", oriHeight + 40)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .text("Time");

// Add Y-axis
oriChartGroup.append("g")
  .attr("class", "y-axis")
  .call(d3.axisLeft(oriY).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis label
oriChartGroup.append("text")
  .attr("x", -oriHeight / 2)
  .attr("y", -50)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Degrees (°)");

const oriLineGenerators = {
  heading: d3.line().x(d => oriX(d.time)).y(d => oriY(d.heading)),
  roll: d3.line().x(d => oriX(d.time)).y(d => oriY(d.roll)),
  pitch: d3.line().x(d => oriX(d.time)).y(d => oriY(d.pitch))
};

// Line generators for Heading, Roll, and Pitch
const oriLines = {};
["heading", "roll", "pitch"].forEach((key, index) => {
  const colors = ["#ff7f0e", "#1f77b4", "#2ca02c"];
  oriLines[key] = oriChartGroup.append("path")
    .attr("fill", "none")
    .attr("stroke", colors[index])
    .attr("stroke-width", 2);
});

// Tooltip setup
const oriTooltip = d3
  .select(".orientation-chart")
  .append("div")
  .style("position", "absolute")
  .style("background", "#222")
  .style("color", "#fff")
  .style("padding", "5px 10px")
  .style("border-radius", "5px")
  .style("pointer-events", "none")
  .style("opacity", 0);

// Socket.IO listener for orientation data
socket.on("orientation_data", (data) => {
  console.log("Orientation data received: ", data);

  // Parse orientation data
  const [heading, roll, pitch] = data.orientation.split(',');

  // Update global variables for Heading, Roll, and Pitch
  oriHeading = heading;
  oriRoll = roll;
  oriPitch = pitch;
});


function updateOriChart() {
  // Generate new data point with the latest orientation values
  const currentTime = new Date();
  oriData.push({ time: currentTime, heading: oriHeading, roll: oriRoll, pitch: oriPitch });

  // Remove oldest data point if exceeding oriMaxDataPoints
  if (oriData.length > oriMaxDataPoints + 1) {
    oriData.shift();
  }

  // Calculate the extent (min and max) of all data points for Heading, Roll, and Pitch
  const allExtents = ["heading", "roll", "pitch"].flatMap(key => d3.extent(oriData, d => d[key]));
  const overallExtent = [Math.min(...allExtents) - 10, Math.max(...allExtents) + 10];

  // Update the Y-axis domain dynamically
  oriY.domain(overallExtent);

  // Update the Y-axis
  oriChartGroup.select(".y-axis")
    .transition()
    .duration(500)
    .call(d3.axisLeft(oriY).ticks(5))
    .selectAll("text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Update the X-axis domain
  oriX.domain([oriData[0].time, new Date(oriData[0].time.getTime() + oriMaxDataPoints * oriInterval)]);

  // Update the X-axis
  oriChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(
      d3.axisBottom(oriX)
        .ticks(oriMaxDataPoints)
        .tickSize(-oriHeight)
        .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
    )
    .selectAll("text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Update lines for Heading, Roll, and Pitch
  Object.keys(oriLineGenerators).forEach(key => {
    oriLines[key]
      .datum(oriData)
      .transition()
      .duration(500)
      .attr("d", oriLineGenerators[key]);
  });

  // Tooltip (optional, for orientation data)
  const circles = oriChartGroup.selectAll(".data-point").data(oriData);

  circles
    .enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 4)
    .merge(circles)
    .attr("cx", d => oriX(d.time))
    .attr("cy", d => oriY(d.heading)) // Use Heading for tooltip demonstration
    .attr("fill", "#ddd")
    .on("mouseover", (event, d) => {
      oriTooltip
        .style("opacity", 1)
        .html(
          `Time: ${d.time.toLocaleTimeString()}<br>
          Heading: ${d.heading.toFixed(2)}°, Roll: ${d.roll.toFixed(2)}°, Pitch: ${d.pitch.toFixed(2)}°`
        )
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      oriTooltip.style("opacity", 0);
    });

  circles.exit().remove();
}

// Update chart with new data at the specified interval
setInterval(updateOriChart, oriInterval);
