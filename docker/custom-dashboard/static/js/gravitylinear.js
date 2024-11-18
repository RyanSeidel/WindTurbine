// Initial Weather Data and Settings
let gravlincurrentTime = new Date();
let gravlinData = []; // Start with an empty dataset
const gravlinMaxDataPoints = 7; // Maximum number of points before resetting
const gravlinInterval = 2000; // Interval for adding new points (2 seconds)

// Set dimensions and margins
const gravlinMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const gravlinContainerWidth = document.querySelector(".gravlinear-chart").offsetWidth;
const gravlinWidth = gravlinContainerWidth - gravlinMargin.left - gravlinMargin.right;
const gravlinHeight = 200 - gravlinMargin.top - gravlinMargin.bottom;

// Create SVG container
const gravlinSVG = d3
  .select(".gravlinear-chart")
  .append("svg")
  .attr("width", gravlinWidth + gravlinMargin.left + gravlinMargin.right)
  .attr("height", gravlinHeight + gravlinMargin.top + gravlinMargin.bottom)
  .style("background", "#1a1a1a");

// Add a title to the chart
gravlinSVG.append("text")
  .attr("x", (gravlinWidth + gravlinMargin.left + gravlinMargin.right) / 2)
  .attr("y", 20)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Dynamic Gravity and Linear Acceleration Over Time");

// Append group for the main chart content
const gravlinChartGroup = gravlinSVG.append("g")
  .attr("transform", `translate(${gravlinMargin.left},${gravlinMargin.top})`);

// Create scales
const gravlinX = d3
  .scaleTime()
  .domain([gravlincurrentTime, new Date(gravlincurrentTime.getTime() + gravlinMaxDataPoints * gravlinInterval)])
  .range([0, gravlinWidth]);

const gravlinYWindSpeed = d3
  .scaleLinear()
  .domain([0, 50]) // Example domain for Wind Speed
  .range([gravlinHeight, 0]);

const gravlinYPressure = d3
  .scaleLinear()
  .domain([1000, 1020]) // Example domain for Pressure
  .range([gravlinHeight, 0]);

// Add X-axis
const gravlinXAxis = gravlinChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${gravlinHeight})`)
  .call(
    d3.axisBottom(gravlinX)
      .ticks(gravlinMaxDataPoints) // Show one tick for each data point
      .tickSize(-gravlinHeight) // Extend tick lines as grid lines
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : "")) // Show every alternate tick
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Style grid lines for X-axis
gravlinChartGroup.selectAll(".x-axis .tick line")
  .attr("stroke", "#444")
  .attr("stroke-dasharray", "2,2"); // Dashed grid lines

// Add X-axis label
gravlinChartGroup.append("text")
  .attr("x", gravlinWidth / 2)
  .attr("y", gravlinHeight + 40)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .text("Time");

// Add Y-axis for Wind Speed (left)
gravlinChartGroup.append("g")
  .attr("class", "y-axis-wind")
  .call(
    d3.axisLeft(gravlinYWindSpeed)
      .ticks(5)
      .tickSize(-gravlinWidth)
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Style Y-axis tick labels
gravlinChartGroup.selectAll(".y-axis-wind .tick text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis for Pressure (right)
gravlinChartGroup.append("g")
  .attr("class", "y-axis-pressure")
  .attr("transform", `translate(${gravlinWidth}, 0)`)
  .call(d3.axisRight(gravlinYPressure).ticks(5))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Line generators
const gravlinLineWindSpeed = d3
  .line()
  .x(d => gravlinX(d.time))
  .y(d => gravlinYWindSpeed(d.windSpeed))
  .curve(d3.curveMonotoneX);

const gravlinLinePressure = d3
  .line()
  .x(d => gravlinX(d.time))
  .y(d => gravlinYPressure(d.pressure))
  .curve(d3.curveMonotoneX);

// Add lines to the chart
const gravlinWindSpeedLine = gravlinChartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "#00FF9F")
  .attr("stroke-width", 2);

// Add Y-axis label for Wind Speed
gravlinChartGroup.append("text")
  .attr("x", -gravlinHeight / 2)
  .attr("y", -40)
  .attr("text-anchor", "middle")
  .attr("fill", "#00FF9F")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Gravity");

// Add Y-axis label for Pressure
gravlinChartGroup.append("text")
  .attr("x", -gravlinHeight / 2)
  .attr("y", gravlinWidth + 60)
  .attr("text-anchor", "middle")
  .attr("fill", "orange")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Linear Acceleration ");

const gravlinPressureLine = gravlinChartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "orange")
  .attr("stroke-width", 2);

// Add Tooltip
const gravlinTooltip = d3
  .select("body")
  .append("div")
  .attr("class", "tooltip")
  .style("position", "absolute")
  .style("background", "#1a1a1a")
  .style("color", "#00FF9F")
  .style("border", "1px solid #444")
  .style("padding", "10px")
  .style("border-radius", "5px")
  .style("display", "none")
  .style("pointer-events", "none")
  .style("font-size", "12px");

// Update Chart Function
function updateGravlinChart() {
  // Generate new data point
  const newTime = new Date();
  const newWindSpeed = Math.random() * 50; // Random wind speed
  const newPressure = 1000 + Math.random() * 20; // Random pressure
  gravlinData.push({ time: newTime, windSpeed: newWindSpeed, pressure: newPressure });

  // Remove oldest point if exceeding gravlinMaxDataPoints
  if (gravlinData.length > gravlinMaxDataPoints + 1) {
    gravlinData.shift();
  }

  // Update scales
  gravlinX.domain([gravlinData[0].time, new Date(gravlinData[0].time.getTime() + gravlinMaxDataPoints * gravlinInterval)]);

  // Update X-axis dynamically
  gravlinChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(
      d3.axisBottom(gravlinX)
        .ticks(gravlinMaxDataPoints)
        .tickSize(-gravlinHeight)
        .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
    );

  // Style grid lines for X-axis
  gravlinChartGroup.selectAll(".x-axis .tick line")
    .attr("stroke", "#444")
    .attr("stroke-dasharray", "2,2");

  gravlinChartGroup.selectAll(".x-axis .tick text")
    .attr("fill", "#ddd") // Keep consistent white color
    .attr("font-size", "10px");

  // Update lines
  gravlinWindSpeedLine
    .datum(gravlinData)
    .transition()
    .duration(500)
    .attr("d", gravlinLineWindSpeed);

  gravlinPressureLine
    .datum(gravlinData)
    .transition()
    .duration(500)
    .attr("d", gravlinLinePressure);
}

// Start real-time updates
setInterval(updateGravlinChart, gravlinInterval); // Update every 2 seconds
