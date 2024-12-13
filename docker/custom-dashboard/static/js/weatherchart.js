//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//                           Wind Turbine Digital Twins                                                     //
//                        By Ryan Seidel and Zac Castaneda                                                  //
//                         Client: Dr. Jose Baca                                                            //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Description:                                                                                             //
//This project implements a real-time chart to display the relationship between wind speed and air pressure. //
//Using D3.js, the system dynamically updates a dual-axis line chart as new data is received, ensuring smooth //
//transitions and responsive visuals. The chart features interactive tooltips to show precise values for wind //
//speed (mph) and air pressure (hPa) at any given time. With adjustable axes and a sleek, modern design, the //
//visualization provides a clear and engaging way to monitor environmental data in real-time.                 //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//


// Initial Weather Data and Settings
let currentTime = new Date(); // Current time as the starting point
let weatherData = []; // Start with an empty dataset
const maxDataPoints = 6; // Maximum number of points before resetting
const interval = 2000; // Interval for adding new points (2 seconds)

// Set dimensions and margins
const margin = { top: 40, right: 70, bottom: 50, left: 70 };
const containerWidth = document.querySelector(".weather-chart").offsetWidth;
const width = containerWidth - margin.left - margin.right;
const height = 225 - margin.top - margin.bottom;

let latestWindSpeed = 0; // Placeholder for the latest wind speed
let latestPressure = 0; // Placeholder for the latest pressure

// Create SVG container
const svg = d3
  .select(".weather-chart")
  .append("svg")
  .attr("width", width + margin.left + margin.right)
  .attr("height", height + margin.top + margin.bottom)
  .style("background", "#1a1a1a");

// Add a title to the chart
svg.append("text")
  .attr("x", (width + margin.left + margin.right) / 2)
  .attr("y", 20)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "16px")
  .text("Dynamic Wind Speed and Pressure Over Time");

// Append group for the main chart content
const chartGroup = svg.append("g")
  .attr("transform", `translate(${margin.left},${margin.top})`);

// Create scales
const x = d3
  .scaleTime()
  .domain([currentTime, new Date(currentTime.getTime() + maxDataPoints * interval)])
  .range([0, width]);

const yWindSpeed = d3
  .scaleLinear()
  .domain([0, 3]) // Example domain for Wind Speed
  .range([height, 0]);

const yPressure = d3
  .scaleLinear()
  .domain([1020, 1024]) // Example domain for Pressure
  .range([height, 0]);

// Add X-axis
const xAxis = chartGroup.append("g")
  .attr("class", "x-axis")
  .attr("transform", `translate(0,${height})`)
  .call(
    d3.axisBottom(x)
      .ticks(maxDataPoints) // Show one tick for each data point
      .tickSize(-height) // Extend tick lines as grid lines
      .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : "")) // Show every alternate tick
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Style grid lines for X-axis
chartGroup.selectAll(".x-axis .tick line")
.attr("stroke", "#444")
.attr("stroke-dasharray", "2,2"); // Dashed grid lines


// Add X-axis label
chartGroup.append("text")
  .attr("x", width / 2)
  .attr("y", height + 40)
  .attr("text-anchor", "middle")
  .attr("fill", "#ddd")
  .attr("font-size", "12px")
  .text("Time");

// Add Y-axis for Wind Speed (left)
chartGroup.append("g")
  .attr("class", "y-axis-wind")
  .call(
    d3.axisLeft(yWindSpeed)
      .tickValues(d3.range(0, 3, 0.3)) // Option 1: Explicit ticks at 0.5 intervals
      //.ticks(10) // Option 2: Auto-generate ticks with finer granularity
      .tickSize(-width)
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Style Y-axis tick labels
chartGroup.selectAll(".y-axis-wind .tick text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Add Y-axis for Pressure (right)
chartGroup.append("g")
  .attr("class", "y-axis-pressure")
  .attr("transform", `translate(${width}, 0)`)
  .call(
    d3.axisRight(yPressure)
      .tickValues(d3.range(1020, 1024, 0.5)) // Generate ticks at intervals of 0.5
  )
  .selectAll("text")
  .attr("fill", "#ddd")
  .attr("font-size", "10px");

// Line generators
const lineWindSpeed = d3
  .line()
  .x(d => x(d.time))
  .y(d => yWindSpeed(d.windSpeed))
  .curve(d3.curveMonotoneX);

const linePressure = d3
  .line()
  .x(d => x(d.time))
  .y(d => yPressure(d.pressure))
  .curve(d3.curveMonotoneX);

// Add lines to the chart
const windSpeedLine = chartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "#00FF9F")
  .attr("stroke-width", 2);

// Add Y-axis label for Wind Speed
chartGroup.append("text")
  .attr("x", -height / 2)
  .attr("y", -40)
  .attr("text-anchor", "middle")
  .attr("fill", "#00FF9F")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Wind Speed (mph)");

// Add Y-axis label for Pressure
chartGroup.append("text")
  .attr("x", -height / 2)
  .attr("y", width + 60)
  .attr("text-anchor", "middle")
  .attr("fill", "orange")
  .attr("font-size", "12px")
  .attr("transform", "rotate(-90)")
  .text("Air Pressure (hPa)");

const pressureLine = chartGroup.append("path")
  .attr("fill", "none")
  .attr("stroke", "orange")
  .attr("stroke-width", 2);

// Add Tooltip
const tooltip = d3
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

// Debugging: Log latest wind speed and pressure updates
socket.on('speed_data', (data) => {
  latestWindSpeed = parseFloat(data.speed);

});

socket.on('pressure_data', (data) => {
  latestPressure = parseFloat(data.pressure);

});


// Update Chart Function
// Update Chart Function
function updateChart() {
  // Generate new data point using the latest wind speed and pressure
  const newTime = new Date();
  const newWindSpeed = latestWindSpeed; // Latest wind speed from socket
  const newPressure = latestPressure;  // Latest pressure from socket

  // Add the new data point to the dataset
  weatherData.push({ time: newTime, windSpeed: newWindSpeed, pressure: newPressure });

  // Remove the oldest data point if exceeding maxDataPoints
  if (weatherData.length > maxDataPoints + 1) {
    weatherData.shift();
  }

  // Dynamically adjust Y-axis domains
  const windSpeedExtent = d3.extent(weatherData, d => d.windSpeed);
  const pressureExtent = d3.extent(weatherData, d => d.pressure);

  yWindSpeed.domain([
    Math.floor(windSpeedExtent[0] - 1), // 2 units below the min wind speed
    Math.ceil(windSpeedExtent[1] + 1),  // 2 units above the max wind speed
  ]);

  yPressure.domain([
    Math.floor(pressureExtent[0] - 1),  // 2 units below the min pressure
    Math.ceil(pressureExtent[1] + 1),  // 2 units above the max pressure
  ]);

  // Update X-axis scale dynamically based on new data
  x.domain([weatherData[0].time, new Date(weatherData[0].time.getTime() + maxDataPoints * interval)]);

  // Update Y-axes
  chartGroup.select(".y-axis-wind")
    .transition()
    .duration(500)
    .call(
      d3.axisLeft(yWindSpeed)
        .tickValues(d3.range(Math.floor(windSpeedExtent[0] - 1), Math.ceil(windSpeedExtent[1] + 1), 0.3)) // Ticks every 0.5
    )
    .selectAll("text")
    .style("color", "#fff") // Force tick text color during updates
    .attr("font-size", "10px");

  chartGroup.select(".y-axis-pressure")
    .transition()
    .duration(500)
    .call(
      d3.axisRight(yPressure)
        .tickValues(d3.range(Math.floor(pressureExtent[0] - 1), Math.ceil(pressureExtent[1] + 1), 0.3)) // Ticks every 0.5
    )
    .selectAll("text")
    .style("color", "#fff")
    .attr("font-size", "10px");

  

  // Update X-axis dynamically
  chartGroup.select(".x-axis")
    .transition()
    .duration(500)
    .style("color", "#fff")
    .call(
      d3.axisBottom(x)
        .ticks(maxDataPoints)
        .tickSize(-height)
        .tickFormat((d, i) => (i % 2 === 0 ? d3.timeFormat("%H:%M:%S")(d) : "")) // Alternate tick format
    );

  chartGroup.selectAll(".y-axis-wind y-axis-pressure .tick text")
    .attr("fill", "#ddd")
    .attr("font-size", "10px");

  // Style X-axis grid lines
  chartGroup.selectAll(".x-axis .tick line")
    .attr("stroke", "#444")
    .attr("stroke-dasharray", "2,2");

  // Update wind speed line
  windSpeedLine
    .datum(weatherData)
    .transition()
    .duration(500)
    .attr("d", lineWindSpeed);

  // Update pressure line
  pressureLine
    .datum(weatherData)
    .transition()
    .duration(500)
    .attr("d", linePressure);

  // Update wind speed dots
  const windDots = chartGroup.selectAll(".dot-wind").data(weatherData);
  windDots
    .enter()
    .append("circle")
    .attr("class", "dot-wind")
    .attr("r", 4)
    .attr("fill", "red")
    .merge(windDots)
    .transition()
    .duration(500)
    .attr("cx", d => x(d.time))
    .attr("cy", d => yWindSpeed(d.windSpeed));
  windDots.exit().remove();

  // Update pressure dots
  const pressureDots = chartGroup.selectAll(".dot-pressure").data(weatherData);
  pressureDots
    .enter()
    .append("circle")
    .attr("class", "dot-pressure")
    .attr("r", 4)
    .attr("fill", "orange")
    .merge(pressureDots)
    .transition()
    .duration(500)
    .attr("cx", d => x(d.time))
    .attr("cy", d => yPressure(d.pressure));
  pressureDots.exit().remove();

  // Update Tooltip for wind speed
  chartGroup.selectAll(".dot-wind")
    .on("mouseover", (event, d) => {
      tooltip
        .style("display", "block")
        .html(`Time: ${d.time.toLocaleTimeString()}<br>Wind Speed: ${d.windSpeed.toFixed(2)} mph`);
    })
    .on("mousemove", event => {
      tooltip
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      tooltip.style("display", "none");
    });

  // Update Tooltip for pressure
  chartGroup.selectAll(".dot-pressure")
    .on("mouseover", (event, d) => {
      tooltip
        .style("display", "block")
        .html(`Time: ${d.time.toLocaleTimeString()}<br>Pressure: ${d.pressure.toFixed(2)} hPa`);
    })
    .on("mousemove", event => {
      tooltip
        .style("left", `${event.pageX + 10}px`)
        .style("top", `${event.pageY - 20}px`);
    })
    .on("mouseout", () => {
      tooltip.style("display", "none");
    });
}

// Start real-time updates
setInterval(updateChart, interval);