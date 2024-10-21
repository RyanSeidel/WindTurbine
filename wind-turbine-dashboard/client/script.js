const token = "FYLngrNxEhbCYLp0YChB7H9sj26WUrE0SSdXzLktzR515Gw5P778LyKlIKl1oBmhoAzi4-_H5Rtq9k97wYmLMQ==";  // Replace with your InfluxDB token
const org = "TAMUCC";      // Replace with your organization name
const bucket = "windturbine";     // Replace with your bucket name

fetch("http://localhost:8086/api/v2/query", {
  method: "POST",
  headers: {
    "Authorization": `Token ${token}`,
    "Content-Type": "application/vnd.flux"
  },
  body: JSON.stringify({
    query: `from(bucket: "${bucket}")
            |> range(start: -1h)  // Adjust range as needed
            |> filter(fn: (r) => r._measurement == "wind_turbine_data")`,
    type: "flux"
  })
})
  .then(response => response.json())
  .then(data => {
    console.log(data);  // Process and visualize data using D3.js
    // You can now use D3.js to visualize the data
  })
  .catch(error => {
    console.error("Error fetching data from InfluxDB:", error);
  });
