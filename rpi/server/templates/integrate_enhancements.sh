#!/bin/bash

# Insert the enhanced functions before the chart initialization
# Find line 239 (before chart setup) and insert the enhanced code

# First, let's add the plugin registration and enhanced functions
sed -i '239i\        // 🚀 FULL MONTY ENHANCEMENTS - Custom Annotation Plugin' monitor.html
sed -i '240r monitor_enhanced.js' monitor.html

# Now modify the chart initialization to include the plugin
# Find the chart creation and add the plugin
sed -i '/const chart = new Chart(ctx, {/a\            plugins: [annotationPlugin],' monitor.html

# Enhance the grid settings for better visibility
sed -i 's/grid: { color: .#333. }/grid: { color: "#444", lineWidth: 1, drawTicks: true }/g' monitor.html

# Add auto-zoom and smoothing to the loadWeldData function
# Find the line where chart data is updated and add smoothing + auto-zoom
sed -i '/chart.data.datasets\[0\].data = weld.data.timestamps.map/,/chart.update();/{
    /chart.update();/i\                // Apply smoothing\
                chart.data.datasets[0].data = smoothData(chart.data.datasets[0].data, 3);\
                // Auto-zoom to relevant data\
                autoZoomChart(chart);
}' monitor.html

echo "✅ Enhancements integrated into monitor.html!"
echo "Restarting server to apply changes..."
