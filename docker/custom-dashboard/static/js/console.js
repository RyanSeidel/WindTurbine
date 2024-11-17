document.addEventListener("DOMContentLoaded", () => {
  const consoleToggle = document.getElementById("console-toggle");
  const consolePanel = document.getElementById("console-panel");
  const closeConsole = document.getElementById("close-console");
  const closeButtonIcon = closeConsole.querySelector("i");
  const bugIcon = document.querySelector("#console-toggle i");

  // Add nav-link for error-active behavior
  const navLink = consoleToggle; // Target the entire nav-link

  // Function to activate error state
  const triggerErrorState = () => {
    closeButtonIcon.classList.add("error-active"); // Add red effect to close button
    bugIcon.classList.add("error-active"); // Add red effect to bug icon
    navLink.classList.add("error-active"); // Add error-active to the nav-link
  };

  // Function to deactivate error state
  const clearErrorState = () => {
    closeButtonIcon.classList.remove("error-active");
    bugIcon.classList.remove("error-active");
    navLink.classList.remove("error-active"); // Remove error-active from the nav-link
  };

  // Open the console panel
  consoleToggle.addEventListener("click", (e) => {
    e.preventDefault();
    consolePanel.classList.add("open");
    clearErrorState(); // Remove error state when opened
  });

  // Close the console panel
  closeConsole.addEventListener("click", (e) => {
    e.stopPropagation(); // Prevent any event bubbling
    consolePanel.classList.remove("open");
  });

  // Simulate an error (replace this with your actual error condition)
  setTimeout(triggerErrorState, 3000); // Trigger error after 3 seconds
});
