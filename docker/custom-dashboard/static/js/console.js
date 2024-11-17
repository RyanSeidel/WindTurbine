document.addEventListener("DOMContentLoaded", () => {
  const consoleToggle = document.getElementById("console-toggle");
  const consolePanel = document.getElementById("console-panel");
  const closeConsole = document.getElementById("close-console");

  // Open the console panel
  consoleToggle.addEventListener("click", (e) => {
    e.preventDefault();
    consolePanel.classList.add("open");
  });

  // Close the console panel
  closeConsole.addEventListener("click", (e) => {
    e.stopPropagation(); // Prevent any event bubbling
    consolePanel.classList.remove("open");
  });
});

