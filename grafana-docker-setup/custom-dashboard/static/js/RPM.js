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
