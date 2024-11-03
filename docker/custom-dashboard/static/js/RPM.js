document.addEventListener('DOMContentLoaded', function () {
const socket = io(); // Initialize Socket.IO

// Function to render or update the RPM data as a responsive full-circle gauge using D3.js
function renderGauge(rpm) {
    // Clear the existing chart before rendering a new one
    d3.select("#chart").selectAll("*").remove();

    // Define a slightly larger viewBox for better label spacing
    const viewBoxSize = 250; // Adjusted size to fit tick labels and text better
    const margin = 25;
    const radius = viewBoxSize / 2 - margin;

    const svg = d3.select("#chart")
        .append("svg")
        .attr("viewBox", `0 0 ${viewBoxSize} ${viewBoxSize}`) // Responsive size
        .attr("preserveAspectRatio", "xMidYMid meet") // Keeps the gauge centered
        .append("g")
        .attr("transform", `translate(${viewBoxSize / 2}, ${viewBoxSize / 2})`);

    // Set up scale for RPM, starting from the bottom
    const maxRpm = 100;
    const arcScale = d3.scaleLinear()
        .domain([0, maxRpm])
        .range([Math.PI, Math.PI + 2 * Math.PI]); // Full circle starting from the bottom

    // Red arc for the remaining portion
    svg.append("path")
        .datum({ endAngle: Math.PI + 2 * Math.PI })
        .style("fill", "#ff4c4c")
        .attr("d", d3.arc()
            .innerRadius(radius - 10)
            .outerRadius(radius)
            .startAngle(Math.PI)
        );

    // Green arc for the covered portion based on the RPM
    const greenArc = svg.append("path")
        .datum({ endAngle: arcScale(rpm) })
        .style("fill", "#00bfae")
        .attr("d", d3.arc()
            .innerRadius(radius - 10)
            .outerRadius(radius)
            .startAngle(Math.PI)
        );

    // Add tick marks and labels for every 10 RPM
    const tickValues = d3.range(0, maxRpm, 10);
    tickValues.forEach(tick => {
        const angle = arcScale(tick) - Math.PI / 2; // Adjust starting point to the bottom
        const x1 = (radius - 5) * Math.cos(angle);
        const y1 = (radius - 5) * Math.sin(angle);
        const x2 = radius * Math.cos(angle);
        const y2 = radius * Math.sin(angle);

        svg.append("line")
            .attr("x1", x1)
            .attr("y1", y1)
            .attr("x2", x2)
            .attr("y2", y2)
            .attr("stroke", "black")
            .attr("stroke-width", 1);

        const labelX = (radius + 15) * Math.cos(angle);
        const labelY = (radius + 15) * Math.sin(angle);
        svg.append("text")
            .attr("x", labelX)
            .attr("y", labelY)
            .attr("text-anchor", "middle")
            .attr("alignment-baseline", "middle")
            .style("font-size", "8px") // Smaller font size
            .style("fill", "#333")
            .text(tick);
    });

    // Add text for the current RPM value below the gauge
    svg.append("text")
        .attr("text-anchor", "middle")
        .attr("dy", radius + 30) // Position text lower for visibility
        .attr("class", "rpm-text")
        .style("font-size", "14px") // Smaller font size for RPM text
        .style("fill", "#333");

    function updateGauge(newRpm) {
        greenArc.transition()
            .duration(500)
            .attrTween("d", function(d) {
                const interpolate = d3.interpolate(d.endAngle, arcScale(newRpm));
                return function(t) {
                    d.endAngle = interpolate(t);
                    return d3.arc()
                        .innerRadius(radius - 10)
                        .outerRadius(radius)
                        .startAngle(Math.PI)(d);
                };
            });

        // Update the HTML element with the new RPM value
        document.getElementById("rpm-data").textContent = `RPM: ${newRpm.toFixed(2)}`;
    }

    return updateGauge;
}

let updateGauge = renderGauge(0);

socket.on('rpm_data', function(data) {
    const latestRpm = data.latest_rpm;
    updateGauge(latestRpm);
});

fetch('/api/data')
    .then(response => response.json())
    .then(data => {
        const initialRpm = data.latest_rpm || 0;
        updateGauge(initialRpm);
    })
    .catch(error => console.error('Error fetching initial data:', error));

// BME Sensor
// Function to update the temperature display
// Function to update the temperature display
function updateTemperatureDisplay(temp) {
    document.getElementById('temp-value').textContent = temp; 
}
// Function to update the orientation display
function updateOrientationDisplay(heading, roll, pitch) {
    document.getElementById('heading').textContent = heading; 
    document.getElementById('roll').textContent = roll; 
    document.getElementById('pitch').textContent = pitch; 
}
// Socket.IO listener for temperature data
socket.on('temperature_data', (data) => {
    console.log("Temperature data received: ", data);  // Debugging log
    updateTemperatureDisplay(data.temperature); // Update the display
});

// Socket.IO listener for orientation data
socket.on('orientation_data', (data) => {
    console.log("Orientation data received: ", data);  // Debugging log
    const [heading, roll, pitch] = data.orientation.split(','); // Extract values
    updateOrientationDisplay(heading, roll, pitch); // Update the display
});




// Function to update the magnetometer display
function updateMagnetometerDisplay(mx, my, mz) {
    document.getElementById('mx').textContent = mx; 
    document.getElementById('my').textContent = my; 
    document.getElementById('mz').textContent = mz; 
}
// Socket.IO listener for magnetometer data
socket.on('magnetometer_data', (data) => {
    console.log("Magnetometer data received: ", data);  // Debugging log
    const [mx, my, mz] = data.magnetometer.split(',').map(Number); // Split and convert to numbers
    updateMagnetometerDisplay(mx, my, mz); // Update the display
});

// Function to update the accelerometer display
function updateAccelerometerDisplay(ax, ay, az) {
    document.getElementById('ax').textContent = ax; 
    document.getElementById('ay').textContent = ay; 
    document.getElementById('az').textContent = az; 
}

// Socket.IO listener for accelerometer data
socket.on('accelerometer_data', (data) => {
    console.log("Accelerometer data received: ", data);  // Debugging log
    const [ax, ay, az] = data.accelerometer.split(',').map(Number); // Split and convert to numbers
    updateAccelerometerDisplay(ax, ay, az); // Update the display
});


// Function to update the gyroscope display
function updateGyroscopeDisplay(gx, gy, gz) {
    document.getElementById('gx').textContent = gx; 
    document.getElementById('gy').textContent = gy; 
    document.getElementById('gz').textContent = gz; 
}

// Socket.IO listener for magnetometer data
socket.on('gyroscope_data', (data) => {
    console.log("gyroscope data received: ", data);  // Debugging log
    const [gx, gy, gz] = data.gyroscope.split(',').map(Number); // Split and convert to numbers
    updateGyroscopeDisplay(gx, gy, gz); // Update the display
});

// Function to update the linear acceleration display
function updateLinearAccelerationDisplay(lx, ly, lz) {
    document.getElementById('lx').textContent = lx; 
    document.getElementById('ly').textContent = ly; 
    document.getElementById('lz').textContent = lz; 
}

// Socket.IO listener for linear acceleration data
socket.on('linear_acceleration_data', (data) => {
    console.log("Linear acceleration data received: ", data);  // Debugging log
    const [lx, ly, lz] = data.linear_acceleration.split(',').map(Number); // Split and convert to numbers
    updateLinearAccelerationDisplay(lx, ly, lz); // Update the display
});

// Function to update the gravity display
function updateGravityDisplay(grx, gry, grz) {
    document.getElementById('gravity-values').textContent = 
        `grx: ${grx} gry: ${gry} grz: ${grz}`; 
}

// Socket.IO listener for gravity data
socket.on('gravity_data', (data) => {
    console.log("Gravity data received: ", data);  // Debugging log
    const [grx, gry, grz] = data.gravity.split(',').map(Number); // Split and convert to numbers
    updateGravityDisplay(grx, gry, grz); // Update the display
});





});