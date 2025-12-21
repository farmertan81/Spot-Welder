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

        // Draw rise time annotation
        if (riseTimeData) {
            drawRiseTimeMarker(ctx, chart, riseTimeData);
        }

        // Draw peak current line
        drawPeakLine(ctx, chart, peakCurrent);

        // Draw weld start marker (t=0 at start of main rise)
        if (riseTimeData) {
            drawWeldStartMarker(ctx, chart, riseTimeData);
        }

        // Draw metrics overlay
        drawMetricsOverlay(ctx, chartArea, riseTimeData, peakCurrent);
    }
};

// ==================== Metrics & Drawing Helpers ====================

// Improved 10–90% rise time, only in main weld region
function calculateRiseTime(data) {
    if (!data || data.length < 3) return null;

    const currents = data.map(d => d.y);
    const times = data.map(d => d.x);

    const peakCurrent = Math.max(...currents);
    if (peakCurrent <= 0) return null;

    const thresholdMain = 0.2 * peakCurrent;
    const threshold10 = 0.10 * peakCurrent;
    const threshold90 = 0.90 * peakCurrent;

    let firstMainIdx = currents.findIndex(i => i >= thresholdMain);
    let lastMainIdx = currents.length - 1;
    for (let i = currents.length - 1; i >= 0; i--) {
        if (currents[i] >= thresholdMain) {
            lastMainIdx = i;
            break;
        }
    }
    if (firstMainIdx < 0 || lastMainIdx <= firstMainIdx) return null;

    let t10 = null, t90 = null, i10 = null, i90 = null;

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

    if (t10 === null || t90 === null || t90 <= t10) return null;

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

    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(`Rise: ${riseData.riseTime.toFixed(2)}ms`, x1 + 10, y1 - 10);
    ctx.restore();
}

function drawPeakLine(ctx, chart, peakCurrent) {
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

    ctx.fillStyle = '#ff6b35';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`Peak: ${peakCurrent.toFixed(1)}A`, chartArea.right - 100, y - 5);
    ctx.restore();
}

function drawWeldStartMarker(ctx, chart, riseData) {
    const xScale = chart.scales.x;
    const chartArea = chart.chartArea;
    const weldStartTime = riseData ? riseData.startTime : 0;
    const x = xScale.getPixelForValue(weldStartTime);

    ctx.save();
    ctx.strokeStyle = '#ffff00';
    ctx.lineWidth = 2;
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.moveTo(x, chartArea.top);
    ctx.lineTo(x, chartArea.bottom);
    ctx.stroke();

    ctx.fillStyle = '#ffff00';
    ctx.font = 'bold 12px monospace';
    ctx.fillText('t=0', x + 5, chartArea.bottom - 10);
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

function autoZoomChart(chart) {
    const data = chart.data.datasets[0]?.data || [];
    if (data.length === 0) return;

    const times = data.map(d => d.x);
    const minTime = Math.min(...times);
    const maxTime = Math.max(...times);

    const padding = (maxTime - minTime) * 0.1;
    chart.options.scales.x.min = minTime - padding;
    chart.options.scales.x.max = maxTime + padding;
}

// ==================== Idealized Pulse & Energy ====================

function buildCartoonPulse(data) {
    if (!data || data.length < 4) return null;

    const n = data.length;
    const startIdx = Math.floor(n * 0.25);
    const endIdx = Math.floor(n * 0.75);

    let sumI = 0, count = 0;
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

function buildEnergyTraceFromCurrent(data, resistance_milliohm = 2.96) {
    if (!data || data.length < 2) return null;

    const R = resistance_milliohm / 1000.0;
    const eData = [];
    let energyJ = 0.0;

    eData.push({ x: data[0].x, y: 0.0 });

    for (let i = 1; i < data.length; i++) {
        const t0 = data[i - 1].x / 1000.0;
        const t1 = data[i].x / 1000.0;
        const dt = t1 - t0;

        const i0 = data[i - 1].y;
        const i1 = data[i].y;
        const iAvg = 0.5 * (i0 + i1);

        const power = iAvg * iAvg * R;
        energyJ += power * dt;

        eData.push({ x: data[i].x, y: energyJ });
    }

    return eData;
}

// ==================== Chart + Socket Wiring ====================

const socket = io();

const ctx = document.getElementById('waveform-chart').getContext('2d');
const chart = new Chart(ctx, {
    plugins: [annotationPlugin],
    type: 'line',
    data: {
        datasets: [
            {
                label: 'Current (A)',
                data: [],
                borderColor: '#ff6b35',
                backgroundColor: 'rgba(255, 107, 53, 0.1)',
                borderWidth: 2,
                pointRadius: 0,
                yAxisID: 'y'
            },
            {
                label: 'Idealized Pulse',
                data: [],
                borderColor: '#00ff00',
                backgroundColor: 'rgba(0, 255, 0, 0)',
                borderWidth: 3,
                pointRadius: 0,
                yAxisID: 'y',
                tension: 0
            },
            {
                label: 'Voltage (V)',
                data: [],
                borderColor: '#2196F3',
                backgroundColor: 'rgba(33, 150, 243, 0.1)',
                borderWidth: 2,
                pointRadius: 0,
                yAxisID: 'y1'
            },
            {
                label: 'Energy (J)',
                data: [],
                borderColor: '#00bcd4',
                backgroundColor: 'rgba(0, 188, 212, 0)',
                borderWidth: 2,
                pointRadius: 0,
                yAxisID: 'y2',
                borderDash: [4, 2]
            }
        ]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
            x: {
                type: 'linear',
                title: { display: true, text: 'Time (ms)', color: '#fff' },
                ticks: { color: '#aaa' },
                grid: { color: "#444", lineWidth: 1, drawTicks: true },
                min: 0,
                max: 20   // full weld window
            },
            y: {
                type: 'linear',
                position: 'left',
                title: { display: true, text: 'Current (A)', color: '#ff6b35' },
                ticks: { color: '#ff6b35' },
                grid: { color: "#444", lineWidth: 1, drawTicks: true },
                min: 0,
                max: 6000
            },
            y1: {
                type: 'linear',
                position: 'right',
                title: { display: true, text: 'Voltage (V)', color: '#2196F3' },
                ticks: { color: '#2196F3' },
                grid: { display: false },
                min: 0,
                max: 12
            },
            y2: {
                type: 'linear',
                position: 'right',
                title: { display: true, text: 'Energy (J)', color: '#00bcd4' },
                ticks: { color: '#00bcd4' },
                grid: { display: false },
                min: 0
            }
        },
        plugins: {
            legend: { labels: { color: '#fff' } }
        }
    }
});

let currentWeldData = { timestamps: [], voltage: [], current: [] };

// live weld points from ESP
socket.on('weld_data_point', (data) => {
    const t_ms = data.t * 1000;  // seconds -> ms
    currentWeldData.timestamps.push(t_ms);
    currentWeldData.voltage.push(data.v);
    currentWeldData.current.push(data.i);

    chart.data.datasets[0].data.push({ x: t_ms, y: data.i });
    chart.data.datasets[2].data.push({ x: t_ms, y: data.v });

    const currentData = chart.data.datasets[0].data;
    chart.data.datasets[1].data = buildCartoonPulse(currentData) || [];
    chart.data.datasets[3].data = buildEnergyTraceFromCurrent(currentData) || [];

    chart.update('none');
});

// weld complete summary from server
socket.on('weld_complete', async (data) => {
    document.getElementById('energy').textContent = data.energy_joules.toFixed(2) + ' J';
    document.getElementById('peak-current').textContent = data.peak_current_amps.toFixed(1) + ' A';
    document.getElementById('duration').textContent = data.duration_ms.toFixed(1) + ' ms';
    document.getElementById('weld-num').textContent = data.weld_number;

    // Refresh weld history list
    await loadWeldHistory();

    // Automatically load the most recent weld into the chart
    await loadMostRecentWeld();
});

socket.on('pedal_active', (data) => {
    if (data.active) {
        currentWeldData = { timestamps: [], voltage: [], current: [] };
        chart.data.datasets.forEach(ds => ds.data = []);
        chart.update();
    }
});
// -------- History + controls (adapted from old monitor.html) --------
async function loadMostRecentWeld() {
    const resp = await fetch('/api/weld_history');
    const data = await resp.json();

    if (data.status === 'ok' && data.welds.length > 0) {
        const newest = data.welds[0];
        await loadWeldData(newest.filename);
    }
}
async function loadWeldHistory() {
    const resp = await fetch('/api/weld_history');
    const data = await resp.json();

    if (data.status === 'ok' && data.welds.length > 0) {
        const container = document.getElementById('weld-history');
        container.innerHTML = '';

        data.welds.forEach(weld => {
            const item = document.createElement('div');
            item.className = 'weld-item';
            item.onclick = (event) => loadWeldData(weld.filename, event);

            const timestamp = new Date(weld.timestamp).toLocaleString();

            item.innerHTML = `
                <div class="weld-header">
                    <span class="weld-number">Weld #${weld.weld_number}</span>
                    <span style="color: #aaa; font-size: 0.9em;">${timestamp}</span>
                </div>
                <div class="weld-stats">
                    <div>⚡ ${weld.energy_joules.toFixed(2)} J</div>
                    <div>📈 ${weld.peak_current_amps.toFixed(1)} A</div>
                    <div>⏱️ ${weld.duration_ms.toFixed(1)} ms</div>
                </div>
            `;
            container.appendChild(item);
        });
    }
}

async function loadWeldData(filename, event) {
    const resp = await fetch(`/api/weld_data/${filename}`);
    const weld = await resp.json();

    if (weld.data) {
        document.getElementById('energy').textContent = weld.energy_joules.toFixed(2) + ' J';
        document.getElementById('peak-current').textContent = weld.peak_current_amps.toFixed(1) + ' A';
        document.getElementById('duration').textContent = weld.duration_ms.toFixed(1) + ' ms';
        document.getElementById('weld-num').textContent = weld.weld_number;

        let currentData, voltageData;
        if (Array.isArray(weld.data) && weld.data.length > 0 && 't' in weld.data[0]) {
            // new ESP format, t in µs
            currentData = weld.data.map(d => ({ x: d.t / 1000.0, y: d.i }));
            voltageData = weld.data.map(d => ({ x: d.t / 1000.0, y: d.v }));
        } else {
            // legacy format
            currentData = weld.data.timestamps.map((t, i) => ({
                x: t * 1000,
                y: weld.data.current[i]
            }));
            voltageData = weld.data.timestamps.map((t, i) => ({
                x: t * 1000,
                y: weld.data.voltage[i]
            }));
        }

        currentData = smoothData(currentData, 3);

        chart.data.datasets[0].data = currentData;
        chart.data.datasets[2].data = voltageData;
        chart.data.datasets[1].data = buildCartoonPulse(currentData) || [];
        chart.data.datasets[3].data = buildEnergyTraceFromCurrent(currentData) || [];

        // Fixed time window; no auto zoom
        chart.options.scales.x.min = 0;
        chart.options.scales.x.max = 20;   // show 0–20 ms
        chart.update();

        document.querySelectorAll('.weld-item').forEach(item => item.classList.remove('active'));
        if (event && event.target) event.target.closest('.weld-item').classList.add('active');
    }
}

function updateYAxis() {
    const yMax = parseFloat(document.getElementById('y-max').value);
    chart.options.scales.y.max = yMax;
    chart.update();
}

// Clear weld data button wiring
document.addEventListener('DOMContentLoaded', async () => {
    await loadWeldHistory();
    await loadMostRecentWeld();

    const btn = document.getElementById('clear-weld-data-btn');
    if (btn) {
        btn.addEventListener('click', function () {
            if (confirm('Clear all weld data and reset counter?')) {
                socket.emit('clear_weld_data');
            }
        });
    }

    socket.on('weld_data_cleared', function () {
        console.log('✅ Weld data cleared - forcing reload...');
        window.location.href = window.location.href;
    });
});