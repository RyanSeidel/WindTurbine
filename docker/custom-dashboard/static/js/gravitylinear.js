// Initialize Gravity and Linear Acceleration Data
let gravlinData = []; // Start with an empty dataset
const gravlinMaxDataPoints = 10; // Maximum number of points before resetting
const gravlinInterval = 1000; // Interval for updating the chart (2 seconds)

// Set dimensions and margins
const gravlinMargin = { top: 30, right: 70, bottom: 50, left: 70 };
const gravlinContainerWidth = document.querySelector(".gravlinear-chart").offsetWidth;
const gravlinWidth = gravlinContainerWidth - gravlinMargin.left - gravlinMargin.right;
const gravlinHeight = 250 - gravlinMargin.top - gravlinMargin.bottom;

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
  .text("Dynamic Gravity vs. Linear Acceleration Over Time");

// Append group for the main chart content
const gravlinChartGroup = gravlinSVG.append("g")
  .attr("transform", `translate(${gravlinMargin.left},${gravlinMargin.top})`);

// Create scales
const gravlinX = d3
  .scaleTime()
  .domain([new Date(), new Date(Date.now() + gravlinMaxDataPoints * gravlinInterval)])
  .range([0, gravlinWidth]);

const gravlinYGravity = d3
  .scaleLinear()
  .domain([-15, 15]) // Example domain for Gravity
  .range([gravlinHeight, 0]);

const gravlinYLinear = d3
  .scaleLinear()
  .domain([-10, 15]) // Example domain for Linear Acceleration
  .range([gravlinHeight, 0]);

// Add X-axis
const gravlinXAxis = gravlinChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${gravlinHeight})`)
  .call(d3.axisBottom(gravlinX).ticks(gravlinMaxDataPoints).tickSize(-gravlinHeight))
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

gravlinChartGroup.selectAll(".x-axis .tick text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

gravlinChartGroup.append("text")
  .attr("x", -gravlinHeight / 2)
  .attr("y", -50)
  .attr("text-anchor", "middle")
  .attr("fill", "#00FF9F")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Gravity");

  gravlinChartGroup.append("text")
  .attr("x", -gravlinHeight / 2)
  .attr("y", gravlinWidth + 60)
  .attr("text-anchor", "middle")
  .attr("fill", "orange")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Linear Acceleration");



// Add Y-axis for Gravity (left)
gravlinChartGroup.append("g")
  .attr("class", "y-axis-gravity")
  .call(d3.axisLeft(gravlinYGravity).ticks(5))
  .selectAll("text")
  .attr("fill", "#00FF9F")
  .attr("font-size", "10px");

// Add Y-axis for Linear Acceleration (right)
gravlinChartGroup.append("g")
  .attr("class", "y-axis-linear")
  .attr("transform", `translate(${gravlinWidth}, 0)`)
  .call(d3.axisRight(gravlinYLinear).ticks(5))
  .selectAll("text")
  .attr("fill", "orange")
  .attr("font-size", "10px");

// Line generators
const gravlinLineGravity = d3
  .line()
  .x((d) => gravlinX(d.time))
  .y((d) => gravlinYGravity(d.gravity))
  .curve(d3.curveMonotoneX);

const gravlinLineLinear = d3
  .line()
  .x((d) => gravlinX(d.time))
  .y((d) => gravlinYLinear(d.linear))
  .curve(d3.curveMonotoneX);

// Add lines to the chart
const gravlinGravityLine = gravlinChartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "#00FF9F")
  .attr("stroke-width", 2);

const gravlinLinearLine = gravlinChartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "orange")
  .attr("stroke-width", 2);

// Add Tooltip
const gravlinTooltip = d3
  .select("body")
  .append("div")
  .attr("class", "tooltip")
  .style("position", "absolute")
  .style("background", "#222")
  .style("color", "#fff")
  .style("padding", "5px 10px")
  .style("border-radius", "5px")
  .style("pointer-events", "none")
  .style("opacity", 0);


// Update Chart Function
function updateGravlinChart() {
  // Update scales
  gravlinX.domain([gravlinData[0].time, new Date(gravlinData[0].time.getTime() + gravlinMaxDataPoints * gravlinInterval)]);

  // Update X-axis dynamically
  gravlinChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(d3.axisBottom(gravlinX).ticks(gravlinMaxDataPoints));

  gravlinChartGroup.selectAll(".x-axis .tick text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Update lines
  gravlinGravityLine
    .datum(gravlinData)
    .transition()
    .duration(500)
    .attr("d", gravlinLineGravity);

  gravlinLinearLine
    .datum(gravlinData)
    .transition()
    .duration(500)
    .attr("d", gravlinLineLinear);

  // Add data points as circles for gravity
  const gravityCircles = gravlinChartGroup.selectAll(".gravity-point").data(gravlinData);

  gravityCircles
    .enter()
    .append("circle")
    .attr("class", "gravity-point")
    .attr("r", 4)
    .attr("fill", "#00FF9F")
    .merge(gravityCircles)
    .attr("cx", (d) => gravlinX(d.time))
    .attr("cy", (d) => gravlinYGravity(d.gravity))
    .on("mouseover", (event, d) => {
      gravlinTooltip
        .style("opacity", 1)
        .html(
          `Gravity Magnitude: ${d.gravity.toFixed(2)}<br>Time: ${d3.timeFormat("%H:%M:%S")(d.time)}`
        )
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      gravlinTooltip.style("opacity", 0);
    });

  gravityCircles.exit().remove();

  // Add data points as circles for linear acceleration
  const linearCircles = gravlinChartGroup.selectAll(".linear-point").data(gravlinData);

  linearCircles
    .enter()
    .append("circle")
    .attr("class", "linear-point")
    .attr("r", 4)
    .attr("fill", "orange")
    .merge(linearCircles)
    .attr("cx", (d) => gravlinX(d.time))
    .attr("cy", (d) => gravlinYLinear(d.linear))
    .on("mouseover", (event, d) => {
      gravlinTooltip
        .style("opacity", 1)
        .html(
          `Linear Acceleration Magnitude: ${d.linear.toFixed(2)}<br>Time: ${d3.timeFormat("%H:%M:%S")(d.time)}`
        )
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      gravlinTooltip.style("opacity", 0);
    });

  linearCircles.exit().remove();
}

// Socket.IO listeners
socket.on("linear_acceleration_data", (data) => {
  const [lx, ly, lz] = data.linear_acceleration.split(",").map(Number);
  const linearMagnitude = Math.sqrt(lx ** 2 + ly ** 2 + lz ** 2);

  // Add new data point for linear acceleration
  const newTime = new Date();
  gravlinData.push({ time: newTime, linear: linearMagnitude, gravity: 0 });

  if (gravlinData.length > gravlinMaxDataPoints + 11) {
    gravlinData.shift();
  }

  updateGravlinChart();
});

socket.on("gravity_data", (data) => {
  const [grx, gry, grz] = data.gravity.split(",").map(Number);
  const gravityMagnitude = Math.sqrt(grx ** 2 + gry ** 2 + grz ** 2);

  // Add new data point for gravity
  const newTime = new Date();
  gravlinData.push({ time: newTime, gravity: gravityMagnitude, linear: 0 });

  if (gravlinData.length > gravlinMaxDataPoints + 1) {
    gravlinData.shift();
  }

  updateGravlinChart();
});


