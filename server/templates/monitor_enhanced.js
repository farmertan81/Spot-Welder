// 🚀 FULL MONTY CHART ENHANCEMENTS

// ==================== Annotation Plugin ====================

const annotationPlugin = {
    id: 'customAnnotations',
    afterDatasetsDraw(chart) {
        const ctx = chart.ctx;
        const chartArea = chart.chartArea;
        const datasets = chart.data.datasets;

        if (!datasets || datasets.length === 0) return;
        if (!datasets[0].data || datasets[0].data.length === 0) return;

        const currentData = datasets[0].data;

        // Calculate metrics using improved method
        const peakCurrent = Math.max(...currentData.map(d => d.y));
        const riseTimeData = calculateRiseTime(currentData);
        const weldStartIdx = currentData.findIndex(d => d.x >= 0);

        // Draw rise time annotation (10–90% of peak in main weld region)
        if (riseTimeData) {
            drawRiseTimeMarker(ctx, chart, riseTimeData);
        }

        // Draw peak current line
        drawPeakLine(ctx, chart, peakCurrent);

        // Draw weld start marker (t=0)
        if (weldStartIdx >= 0) {
            drawWeldStartMarker(ctx, chart);
        }

        // Draw metrics overlay box
        drawMetricsOverlay(ctx, chartArea, riseTimeData, peakCurrent);
    }
};

// ==================== Metrics & Drawing Helpers ====================

// Improved rise time: 10–90% of peak, but only in main weld window
function calculateRiseTime(data) {
    if (!data || data.length < 3) return null;

    const currents = data.map(d => d.y);
    const times = data.map(d => d.x);

    const peakCurrent = Math.max(...currents);
    if (peakCurrent <= 0) return null;

    const thresholdMain = 0.2 * peakCurrent;   // main weld region threshold
    const threshold10 = 0.10 * peakCurrent;
    const threshold90 = 0.90 * peakCurrent;

    // Find main weld window where current > 20% of peak
    let firstMainIdx = currents.findIndex(i => i >= thresholdMain);
    let lastMainIdx = currents.length - 1;
    for (let i = currents.length - 1; i >= 0; i--) {
        if (currents[i] >= thresholdMain) {
            lastMainIdx = i;
            break;
        }
    }

    if (firstMainIdx < 0 || lastMainIdx <= firstMainIdx) {
        return null;
    }

    // Search 10–90% only inside that window
    let t10 = null;
    let t90 = null;
    let i10 = null;
    let i90 = null;

    for (let i = firstMainIdx; i <= lastMainIdx; i++) {
        const t = times[i];
        const c = currents[i];

        if (t10 === null && c >= threshold10) {
            t10 = t;
            i10 = c;
        }
        if (t90 === null && c >= threshold90) {
            t90 = t;
            i90 = c;
            break;
        }
    }

    if (t10 === null || t90 === null || t90 <= t10) {
        return null;
    }

    return {
        startTime: t10,
        endTime: t90,
        riseTime: t90 - t10,
        startCurrent: i10,
        endCurrent: i90
    };
}

function drawRiseTimeMarker(ctx, chart, riseData) {
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;

    const x1 = xScale.getPixelForValue(riseData.startTime);
    const x2 = xScale.getPixelForValue(riseData.endTime);
    const y1 = yScale.getPixelForValue(riseData.startCurrent);
    const y2 = yScale.getPixelForValue(riseData.endCurrent);

    ctx.save();
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);

    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    // Label
    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(`Rise: ${riseData.riseTime.toFixed(2)}ms`, x1 + 10, y1 - 10);

    ctx.restore();
}

function drawPeakLine(ctx, chart, peakCurrent) {
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;
    const chartArea = chart.chartArea;

    const y = yScale.getPixelForValue(peakCurrent);

    ctx.save();
    ctx.strokeStyle = '#ff6b35';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);

    ctx.beginPath();
    ctx.moveTo(chartArea.left, y);
    ctx.lineTo(chartArea.right, y);
    ctx.stroke();

    // Label
    ctx.fillStyle = '#ff6b35';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`Peak: ${peakCurrent.toFixed(1)}A`, chartArea.right - 100, y - 5);

    ctx.restore();
}

function drawWeldStartMarker(ctx, chart) {
    const xScale = chart.scales.x;
    const chartArea = chart.chartArea;

    const x = xScale.getPixelForValue(0);

    ctx.save();
    ctx.strokeStyle = '#ffff00';
    ctx.lineWidth = 2;
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.moveTo(x, chartArea.top);
    ctx.lineTo(x, chartArea.bottom);
    ctx.stroke();

    // Label
    ctx.fillStyle = '#ffff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText('t=0 (Weld Start)', x + 5, chartArea.top + 20);

    ctx.restore();
}

function drawMetricsOverlay(ctx, chartArea, riseData, peakCurrent) {
    if (!riseData) return;

    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(chartArea.left + 10, chartArea.top + 10, 180, 80);

    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 13px monospace';
    ctx.fillText('📊 Weld Metrics', chartArea.left + 20, chartArea.top + 30);

    ctx.font = '12px monospace';
    ctx.fillStyle = '#fff';
    ctx.fillText(`Rise Time: ${riseData.riseTime.toFixed(2)} ms`, chartArea.left + 20, chartArea.top + 50);
    ctx.fillText(`Peak: ${peakCurrent.toFixed(1)} A`, chartArea.left + 20, chartArea.top + 68);

    ctx.restore();
}

// ==================== Data Processing Helpers ====================

// Simple moving average smoother
function smoothData(data, windowSize = 3) {
    if (!data || data.length < windowSize) return data;

    const smoothed = [];
    for (let i = 0; i < data.length; i++) {
        const start = Math.max(0, i - Math.floor(windowSize / 2));
        const end = Math.min(data.length, i + Math.ceil(windowSize / 2));
        const window = data.slice(start, end);
        const avg = window.reduce((sum, d) => sum + d.y, 0) / window.length;
        smoothed.push({ x: data[i].x, y: avg });
    }
    return smoothed;
}

// Auto-zoom X axis around weld data
function autoZoomChart(chart) {
    const data = chart.data.datasets[0]?.data || [];
    if (data.length === 0) return;

    const times = data.map(d => d.x);
    const minTime = Math.min(...times);
    const maxTime = Math.max(...times);

    const padding = (maxTime - minTime) * 0.1; // 10% padding

    chart.options.scales.x.min = minTime - padding;
    chart.options.scales.x.max = maxTime + padding;
}

// ==================== New: Idealized Pulse & Energy Trace ====================

// Build "cartoon" idealized weld pulse from measured current data
function buildCartoonPulse(data) {
    if (!data || data.length < 4) return null;

    const n = data.length;
    const startIdx = Math.floor(n * 0.25);
    const endIdx = Math.floor(n * 0.75);

    let sumI = 0;
    let count = 0;
    for (let i = startIdx; i < endIdx; i++) {
        sumI += data[i].y;
        count++;
    }
    if (count === 0) return null;

    const plateauCurrent = sumI / count;

    const t0 = data[0].x;
    const tEnd = data[n - 1].x;
    const tRiseEnd = data[startIdx].x;
    const tFallStart = data[endIdx - 1].x;

    return [
        { x: t0, y: 0.0 },
        { x: tRiseEnd, y: plateauCurrent },
        { x: tFallStart, y: plateauCurrent },
        { x: tEnd, y: 0.0 }
    ];
}

// Build cumulative energy trace from current data using I^2 * R
function buildEnergyTraceFromCurrent(data, resistance_milliohm = 2.96) {
    if (!data || data.length < 2) return null;

    const R = resistance_milliohm / 1000.0; // mΩ -> Ω
    const eData = [];
    let energyJ = 0.0;

    eData.push({ x: data[0].x, y: 0.0 });

    for (let i = 1; i < data.length; i++) {
        const t0 = data[i - 1].x / 1000.0; // ms -> s
        const t1 = data[i].x / 1000.0;
        const dt = t1 - t0;

        const i0 = data[i - 1].y;
        const i1 = data[i].y;
        const iAvg = 0.5 * (i0 + i1);

        const power = iAvg * iAvg * R; // Watts = I^2 R
        energyJ += power * dt;        // Joules

        eData.push({ x: data[i].x, y: energyJ });
    }

    return eData;
}

// ==================== Exports (if you use modules) ====================
// If you're not using modules, ignore this section.
// In a module system you might export these:
//
// export {
//   annotationPlugin,
//   smoothData,
//   autoZoomChart,
//   buildCartoonPulse,
//   buildEnergyTraceFromCurrent
// };