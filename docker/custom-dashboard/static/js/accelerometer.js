// Initial Accelerometer Data and Settings
let accData = []; // Start with an empty dataset
const accMaxDataPoints = 6; // Maximum number of points before resetting
const accInterval = 2000; // Interval for adding new points (1 seconds)

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
  .attr("x", (accWidth + accMargin.left + accMargin.right) / 2)
  .attr("y", 20)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Dynamic Accelerometer Magnitude Over Time");

// Append group for the main chart content
const accChartGroup = accSVG.append("g")
  .attr("transform", `translate(${accMargin.left},${accMargin.top})`);

// Create scales
const accX = d3
  .scaleTime()
  .domain([new Date(), new Date(new Date().getTime() + accMaxDataPoints * accInterval)])
  .range([0, accWidth]);

const accY = d3
  .scaleLinear()
  .domain([0, 50]) // Example domain for magnitude
  .range([accHeight, 0]);

// Add X-axis
const accXAxis = accChartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${accHeight})`)
  .call(
    d3.axisBottom(accX)
      .ticks(accMaxDataPoints)
      .tickSize(-accHeight)
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Style grid lines for X-axis
accChartGroup.selectAll(".x-axis .tick line")
  .attr("stroke", "#444")
  .attr("stroke-dasharray", "2,2");

// Add X-axis label
accChartGroup.append("text")
  .attr("x", accWidth / 2)
  .attr("y", accHeight + 40)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .text("Time");

// Add Y-axis
accChartGroup.append("g")
  .attr("class", "y-axis")
  .call(
    d3.axisLeft(accY)
      .ticks(5)
      .tickSize(-accWidth)
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis label
accChartGroup.append("text")
  .attr("x", -accHeight / 2)
  .attr("y", -50)
  .attr("text-anchor", "middle")
  .attr("fill", "#00FF9F")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Magnitude");

// Line generator for magnitude
const accLine = d3
  .line()
  .x((d) => accX(d.time))
  .y((d) => accY(d.magnitude))
  .curve(d3.curveMonotoneX);

// Add line to the chart
const accMagnitudeLine = accChartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "#00FF9F")
  .attr("stroke-width", 2);

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

// Update chart function
function updateAccChart() {
  // Update scales
  accX.domain([accData[0].time, new Date(accData[0].time.getTime() + accMaxDataPoints * accInterval)]);


  // Update X-axis dynamically
  accChartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .call(
      d3.axisBottom(accX)
        .ticks(accMaxDataPoints)
        .tickSize(-accHeight)
        .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : ""))
    );

  // Style grid lines for X-axis
  accChartGroup.selectAll(".x-axis .tick line")
    .attr("stroke", "#444")
    .attr("stroke-dasharray", "2,2");

  accChartGroup.selectAll(".x-axis .tick text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Update Y-axis
  accChartGroup.select(".y-axis")
    .call(d3.axisLeft(accY).ticks(5).tickSize(-accWidth));

  // Update the line
  accMagnitudeLine
    .datum(accData)
    .transition()
    .duration(500)
    .attr("d", accLine);

  // Add circles for data points
  const circles = accChartGroup.selectAll(".data-point").data(accData);

  circles
    .enter()
    .append("circle")
    .attr("class", "data-point")
    .attr("r", 4)
    .attr("fill", "#00FF9F")
    .merge(circles)
    .attr("cx", (d) => accX(d.time))
    .attr("cy", (d) => accY(d.magnitude))
    .on("mouseover", (event, d) => {
      acctooltip
        .style("opacity", 1)
        .html(`Time: ${d.time.toLocaleTimeString()} <br>Magnitude: ${d.magnitude.toFixed(2)}`)
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      acctooltip.style("opacity", 0);
    });

  circles.exit().remove();
}

// Socket.IO listener for accelerometer data
socket.on("accelerometer_data", (data) => {
  console.log("Accelerometer data received: ", data); // Debugging log

  // Parse accelerometer data
  const [ax, ay, az] = data.accelerometer.split(",").map(Number); // Split and convert to numbers

  // Calculate magnitude
  const magnitude = Math.sqrt(ax ** 2 + ay ** 2 + az ** 2);

  // Add new data point
  const newTime = new Date();
  accData.push({ time: newTime, magnitude });

  // Remove oldest point if exceeding accMaxDataPoints
  if (accData.length > accMaxDataPoints + 7) {
    accData.shift();
  }

  // Update chart with new data
  updateAccChart();
});
