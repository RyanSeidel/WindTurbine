document.getElementById("mqttConnectForm").addEventListener("submit", async function(event) {
  event.preventDefault();

  const brokerIp = document.getElementById("broker_ip").value;
  const blade1 = document.getElementById("blade1").value;
  const blade2 = document.getElementById("blade2").value;
  const blade3 = document.getElementById("blade3").value;

  try {
      const response = await fetch("/connect-mqtt", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
              broker_ip: brokerIp,
              blade1_orientation: blade1,
              blade2_orientation: blade2,
              blade3_orientation: blade3
          })
      });

      const result = await response.json();
      
      // Check if connection was successful
      if (result.status === "Connection started") {
          alert(JSON.stringify({ status: "Connected", message: `Connected to MQTT Broker at ${brokerIp}` }));
          document.getElementById("connection-status").textContent = "Status: Connected";
          document.getElementById("connection-status").style.color = "green";
          window.location.href = "/socket";  // Redirect to socket page
      } else {
          alert(JSON.stringify({ status: "Not Connected", message: "Connection failed. Please try again." }));
          document.getElementById("connection-status").textContent = "Status: Not Connected";
          document.getElementById("connection-status").style.color = "red";
      }
  } catch (error) {
      console.error("Error connecting to MQTT broker:", error);
      alert(JSON.stringify({ status: "Connection Error", message: "An error occurred. Please check the broker IP and try again." }));
      document.getElementById("connection-status").textContent = "Status: Connection Failed";
      document.getElementById("connection-status").style.color = "red";
  }
});
