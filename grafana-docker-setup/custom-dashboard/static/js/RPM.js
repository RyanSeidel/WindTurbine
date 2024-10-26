const socket = io(); // Initialize Socket.IO
let chartData = []; // Array to hold the chart data

// Function to render or update the RPM data as a chart using D3.js
function renderChart() {
    // Clear the existing chart before rendering a new one
    d3.select("#chart").selectAll("*").remove();

    // Prepare the SVG canvas dimensions
    const svgWidth = 800, svgHeight = 400;
    const margin = { top: 20, right: 30, bottom: 30, left: 60 };
    const width = svgWidth - margin.left - margin.right;
    const height = svgHeight - margin.top - margin.bottom;

    // Create the SVG canvas
    const svg = d3.select("#chart")
        .append("svg")
        .attr("width", svgWidth)
        .attr("height", svgHeight)
        .append("g")
        .attr("transform", `translate(${margin.left},${margin.top})`);

    // Set up scales
    const x = d3.scaleTime()
        .domain(d3.extent(chartData, d => new Date(d._time)))
        .range([0, width]);

    const y = d3.scaleLinear()
        .domain([0, d3.max(chartData, d => d._value)])
        .range([height, 0]);

    // Line generator
    const line = d3.line()
        .x(d => x(new Date(d._time)))
        .y(d => y(d._value));

    // Add the dark gray background rectangle for the chart area
    svg.append("rect")
        .attr("width", width)
        .attr("height", height)
        .attr("fill", "#444");

    // Add the line path with green color
    svg.append("path")
        .datum(chartData)
        .attr("fill", "none")
        .attr("stroke", "green")
        .attr("stroke-width", 1.5)
        .attr("d", line);

    // Add the X Axis
    svg.append("g")
        .attr("transform", `translate(0,${height})`)
        .call(d3.axisBottom(x).ticks(5));

    // Add the Y Axis
    svg.append("g")
        .call(d3.axisLeft(y));
}

// Listen for 'rpm_data' events from the server
socket.on('rpm_data', function(data) {
    console.log('Received data:', data); // Log data to see if it's received correctly
    chartData = data.data; // Update the chart data
    renderChart(); // Update the chart with the new data

    const latestRpm = data.latest_rpm; // Access the latest RPM from the data object
    console.log(`Latest RPM: ${latestRpm}`);
    const formattedRpm = latestRpm.toFixed(4);
    document.getElementById('current-rpm').textContent = `CURRENT RPM: ${formattedRpm}`; // Update the displayed RPM
});

// Fetch initial data when the page loads
fetch('/api/data')
    .then(response => response.json())
    .then(data => {
        console.log('Initial data received:', data); // Log the initial data
        chartData = data; // Set the initial chart data
        renderChart(); // Render the chart with the initial data
    })
    .catch(error => console.error('Error fetching initial data:', error));
