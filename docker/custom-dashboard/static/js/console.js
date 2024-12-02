//For future work, I intended to develop a notifier alert module.

document.addEventListener("DOMContentLoaded", () => {
  const consoleToggle = document.getElementById("console-toggle");
  const consolePanel = document.getElementById("console-panel");
  const closeConsole = document.getElementById("close-console");
  const closeButtonIcon = closeConsole.querySelector("i");
  const bugIcon = document.querySelector("#console-toggle i");
  const consoleContainer = document.querySelector(".console-msg-container");

  // Add nav-link for error-active behavior
  const navLink = consoleToggle;

  // Function to add a message to the console panel
  const addConsoleMessage = (message, duration = 5000) => {
    const currentTime = new Date().toLocaleTimeString();
    const newMessage = document.createElement("div");
    newMessage.classList.add("console-msg");
    newMessage.innerHTML = `<p>${message} (${currentTime})</p>`;
    consoleContainer.appendChild(newMessage);
    consolePanel.scrollTop = consolePanel.scrollHeight; // Auto-scroll to the latest message

    // Remove the message after the specified duration
    setTimeout(() => {
      newMessage.remove();
    }, duration);
  };

  // Function to activate error state
  const triggerErrorState = () => {
    closeButtonIcon.classList.add("error-active");
    bugIcon.classList.add("error-active");
    navLink.classList.add("error-active");
  };

  // Function to deactivate error state
  const clearErrorState = () => {
    closeButtonIcon.classList.remove("error-active");
    bugIcon.classList.remove("error-active");
    navLink.classList.remove("error-active");
  };

  // Open the console panel
  consoleToggle.addEventListener("click", (e) => {
    e.preventDefault();
    consolePanel.classList.add("open");
    clearErrorState(); // Remove error state when opened
  });

  // Close the console panel
  closeConsole.addEventListener("click", (e) => {
    e.stopPropagation();
    consolePanel.classList.remove("open");
  });

  // Simulate an error (replace this with your actual error condition)
  setTimeout(() => {
    triggerErrorState();
    addConsoleMessage("Error: Something went wrong!", 7000); // Message will auto-clear in 7 seconds
  }, 3000); // Trigger error after 3 seconds
});
