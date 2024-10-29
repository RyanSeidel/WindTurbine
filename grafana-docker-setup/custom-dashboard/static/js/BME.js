// Function to update only the temperature value in the "Temperature: x:0" format
function updateTemperatureData(temp) {
  // Get the temperature div
  const temperatureDiv = document.getElementById("temperature");

  // Set the content, keeping "Temperature: x:" constant, and replace the number
  temperatureDiv.textContent = `Temperature: x:${temp}`;
}

// Example usage
updateTemperatureData(27);
