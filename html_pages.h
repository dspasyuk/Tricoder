#ifndef HTML_PAGES_H
#define HTML_PAGES_H

const char MAIN_PAGE_START[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Tricoder</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/echarts/5.4.3/echarts.min.js"></script>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #f0f2f5; margin: 0; }
    .navbar { background: #fff; padding: 0 20px; border-bottom: 1px solid #dcdfe6; display: flex; justify-content: space-between; align-items: center; min-height: 60px; flex-wrap: wrap; }
    .navbar-brand { font-size: 22px; font-weight: 600; color: #303133; }
    .navbar-menu { display: flex; align-items: center; gap: 15px; flex-wrap: wrap; }
    .nav-item { padding: 8px 12px; cursor: pointer; border-radius: 5px; transition: background 0.3s; font-size: 14px; border: 1px solid transparent; white-space: nowrap; }
    .nav-item.primary { background: #409eff; color: white; border-color: #409eff; }
    .nav-item.primary:hover { background: #66b1ff; border-color: #66b1ff; }
    .nav-item.success { background: #67c23a; color: white; border-color: #67c23a; }
    .nav-item.success:hover { background: #85ce61; border-color: #85ce61; }
    .nav-item.danger { background: #f56c6c; color: white; border-color: #f56c6c; }
    .nav-item.danger:hover { background: #f78989; border-color: #f78989; }
    .nav-item.action { border: 1px solid #dcdfe6; color: #606266; }
    .nav-item.action:hover { background: #ecf5ff; color: #409eff; }
    .content { padding: 5px; }
    #chart { width: 100%; height: calc(100vh - 100px); background: #fff; border-radius: 8px; box-shadow: 0 2px 12px 0 rgba(0,0,0,0.1); }
    .form-group { display: flex; align-items: center; gap: 8px; }
    .form-group label { font-size: 14px; color: #606266; }
    .form-group input { width: 50px; padding: 8px; border-radius: 4px; border: 1px solid #dcdfe6; font-size: 14px; }
    #status { padding: 10px; padding-top:5px; font-size: 14px; color: #709399; text-align: center; }
    #color-swatch { width: 38px; height: 38px; background: #eee; border: 1px solid #dcdfe6; border-radius: 5px; }
  </style>
</head>
<body>
  <nav class="navbar">
    <div class="navbar-brand">Tricoder</div>
    <div class="navbar-menu">
      <div id="color-swatch" title="Estimated Color"></div>
      <div class="form-group">
        <label for="accumTime">Accum (s):</label>
        <input type="number" id="accumTime" min="1" max="60" onchange="saveConfig()">
      </div>
      <div class="form-group">
        <label for="scanInterval">Interval (s):</label>
        <input type="number" id="scanInterval" min="1" max="3600" onchange="saveConfig()">
      </div>
      <div class="nav-item primary" id="startBtn" onclick="startCollection(false)">▶ Scan</div>
      <div class="nav-item success" id="contBtn" onclick="toggleContinuousScan()">∞ Continuous</div>
      <div class="nav-item action" onclick="clearChart()">■ Clear</div>
      <div class="nav-item action" onclick="saveData()">💾 Save</div>
      <div class="nav-item action" onclick="resetWifi()">⚙ Reset</div>
      <div class="form-group">
        <label for="light-checkbox">Light:</label>
        <input type="checkbox" id="light-checkbox" onchange="toggleLight()">
      </div>
      <div class="wifi-badge">📡 )rawliteral";

const char MAIN_PAGE_END[] PROGMEM = R"rawliteral(</div>
    </div>
  </nav>

  <div class="content">
    <div id="status">
      <span id="status-text">Idle</span>
      <span id="start-time"></span>
      <span id="elapsed-time"></span>
    </div>
    <div id="chart"></div>
  </div>

  <script>
    let chart = null;
    let initialOption = null;
    let continuousScanIntervalId = null;
    let scanHistory = [];
    let clientScanStartTime = 0;

    function toggleLight() {
      const checkbox = document.getElementById('light-checkbox');
      const state = checkbox.checked ? 'on' : 'off';
      fetch(`/light?state=${state}`);
    }

    function updateStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          const startTimeEl = document.getElementById('start-time');
          const elapsedTimeEl = document.getElementById('elapsed-time');
          if (clientScanStartTime > 0) {
            const startTime = new Date(clientScanStartTime).toLocaleTimeString();
            startTimeEl.textContent = `Start: ${startTime}`;
            elapsedTimeEl.textContent = `Elapsed: ${data.elapsedTime / 1000}s`;
          } else {
            startTimeEl.textContent = '';
            elapsedTimeEl.textContent = '';
          }
        });
    }

    const CMF = {
        wavelengths: [380, 385, 390, 395, 400, 405, 410, 415, 420, 425, 430, 435, 440, 445, 450, 455, 460, 465, 470, 475, 480, 485, 490, 495, 500, 505, 510, 515, 520, 525, 530, 535, 540, 545, 550, 555, 560, 565, 570, 575, 580, 585, 590, 595, 600, 605, 610, 615, 620, 625, 630, 635, 640, 645, 650, 655, 660, 665, 670, 675, 680, 685, 690, 695, 700, 705, 710, 715, 720, 725, 730, 735, 740, 745, 750, 755, 760, 765, 770, 775, 780],
        x: [0.0014, 0.0022, 0.0042, 0.0076, 0.0143, 0.0232, 0.0435, 0.0776, 0.1344, 0.2148, 0.2839, 0.3285, 0.3483, 0.3457, 0.3233, 0.2940, 0.2650, 0.2280, 0.1954, 0.1581, 0.1116, 0.0655, 0.0320, 0.0082, 0.0049, 0.0116, 0.0298, 0.0633, 0.1096, 0.1655, 0.2257, 0.2904, 0.3597, 0.4334, 0.5121, 0.5945, 0.6784, 0.7621, 0.8425, 0.9163, 0.9786, 1.0263, 1.0567, 1.0622, 1.0456, 1.0026, 0.9384, 0.8544, 0.7514, 0.6424, 0.5419, 0.4479, 0.3608, 0.2835, 0.2187, 0.1649, 0.1212, 0.0874, 0.0636, 0.0468, 0.0329, 0.0227, 0.0158, 0.0114, 0.0081, 0.0058, 0.0041, 0.0029, 0.0021, 0.0015, 0.0010, 0.0007, 0.0005, 0.0003, 0.0002, 0.0002, 0.0001, 0.0001, 0.0001, 0.0000],
        y: [0.0000, 0.0001, 0.0001, 0.0002, 0.0004, 0.0006, 0.0012, 0.0022, 0.0040, 0.0073, 0.0116, 0.0168, 0.0230, 0.0298, 0.0380, 0.0480, 0.0600, 0.0739, 0.0910, 0.1126, 0.1390, 0.1693, 0.2080, 0.2586, 0.3230, 0.4073, 0.5030, 0.6082, 0.7100, 0.7932, 0.8620, 0.9149, 0.9540, 0.9803, 0.9950, 1.0000, 0.9950, 0.9786, 0.9520, 0.9154, 0.8700, 0.8163, 0.7570, 0.6949, 0.6310, 0.5668, 0.5030, 0.4412, 0.3810, 0.3210, 0.2650, 0.2170, 0.1750, 0.1382, 0.1070, 0.0816, 0.0610, 0.0446, 0.0320, 0.0227, 0.0159, 0.0112, 0.0079, 0.0057, 0.0041, 0.0029, 0.0021, 0.0015, 0.0010, 0.0007, 0.0005, 0.0003, 0.0002, 0.0002, 0.0001, 0.0001, 0.0001, 0.0000, 0.0000],
        z: [0.0065, 0.0105, 0.0201, 0.0362, 0.0679, 0.1102, 0.2074, 0.3713, 0.6456, 1.0391, 1.3856, 1.6230, 1.7471, 1.7721, 1.7441, 1.6692, 1.5281, 1.3483, 1.1495, 0.9163, 0.6413, 0.4458, 0.2720, 0.1582, 0.0782, 0.0454, 0.0272, 0.0172, 0.0113, 0.0071, 0.0048, 0.0034, 0.0024, 0.0017, 0.0011, 0.0008, 0.0006, 0.0003, 0.0002, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
    };

    function initChart() {
      const chartDom = document.getElementById('chart');
      chart = echarts.init(chartDom, null, { renderer: 'svg' });
      initialOption = {
        title: { text: 'Spectrum Analysis', left: 'center' },
        tooltip: { trigger: 'axis', formatter: params => `Wavelength: ${params[0].value[0].toFixed(1)} nm<br/>Intensity: ${params[0].value[1].toFixed(4)}` },
        legend: { top: 30, type: 'scroll' },
        toolbox: {
          right: 20,
          feature: {
            dataZoom: { yAxisIndex: 'none', title: { zoom: 'Area Zoom', back: 'Restore Zoom' } },
            dataView: { readOnly: false, title: 'View Data', lang: ['Data View', 'Close', 'Refresh'] },
            saveAsImage: { name: 'tricoder_spectrum', title: 'Save as Image', type: 'svg' }
          }
        },
        xAxis: {
          type: 'value', name: 'Wavelength (nm)', nameLocation: 'middle', nameGap: 35, min: 400, max: 1000
        },
        yAxis: {
          type: 'value', name: 'Intensity', nameLocation: 'middle', nameGap: 50, min: 0
        },
        series: [],
        grid: { left: 90, right: 60, top: 80, bottom: 70 }
      };
      chart.setOption(initialOption);
    }

    function clearChart() {
        scanHistory = [];
        chart.setOption(initialOption, true);
        document.getElementById('status-text').textContent = 'Chart cleared. Ready for new scan.';
    }

    function toggleContinuousScan() {
        const contBtn = document.getElementById('contBtn');
        if (continuousScanIntervalId) {
            // Stop scanning
            clearInterval(continuousScanIntervalId);
            continuousScanIntervalId = null;
            contBtn.textContent = '∞ Continuous';
            contBtn.classList.remove('danger');
            contBtn.classList.add('success');
            document.getElementById('status-text').textContent = 'Continuous scan stopped.';
        } else {
            // Start scanning
            clearChart();
            const interval = document.getElementById('scanInterval').value * 1000;
            if (interval < 1000) {
                alert('Interval must be at least 1 second.');
                return;
            }
            contBtn.textContent = '■ Stop';
            contBtn.classList.remove('success');
            contBtn.classList.add('danger');
            startCollection(true); // Start first scan immediately
            continuousScanIntervalId = setInterval(() => startCollection(true), interval);
        }
    }

    function startCollection(isContinuous = false) {
      const startBtn = document.getElementById('startBtn');
      const status = document.getElementById('status-text');
      
      startBtn.disabled = true;
      if (!isContinuous) {
        chart.showLoading();
        status.textContent = 'Acquiring and processing data... please wait.';
      } else {
        status.textContent = `Scanning continuously... next scan in ${document.getElementById('scanInterval').value}s.`;
      }
      document.getElementById('start-time').textContent = '';
      document.getElementById('elapsed-time').textContent = '';
      clientScanStartTime = Date.now();

      fetch('/start')
        .then(response => {
          if (!response.ok) throw new Error('Failed to start collection.');
          return fetch('/data');
        })
        .then(response => response.json())
        .then(data => {
          if (!isContinuous) {
            scanHistory = []; // Clear history for single scan
            chart.setOption(initialOption, true);
          }
          scanHistory.push({timestamp: new Date(), data: data});

          const interpolatedLine = [...data.interpolated];
          const nirPoint = data.raw[8];
          if (nirPoint) {
            interpolatedLine.push([750, 0]);
            interpolatedLine.push([850, 0]);
            interpolatedLine.push(nirPoint);
            interpolatedLine.push([1000, 0]);
          }

          updateColorSwatch(interpolatedLine);

          const sigma = 8.5;
          function generateGaussian(mu, A) {
              const points = [];
              if (A <= 0) return points;
              for (let x = mu - (3*sigma); x <= mu + (3*sigma); x += (6*sigma)/40) {
                  points.push([x, A * Math.exp(-Math.pow(x - mu, 2) / (2 * Math.pow(sigma, 2)))]);
              }
              return points;
          }

          const rawChannelsSeries = data.raw.map(channelData => ({
              name: 'Raw Channels',
              type: 'line',
              data: generateGaussian(channelData[0], channelData[1]),
              smooth: true, showSymbol: false, lineStyle: { width: 2, opacity: 0.6 }, areaStyle: { opacity: 0.1 }, tooltip: { show: false }
          }));

          const newSeries = {
            name: new Date().toLocaleTimeString(),
            type: 'line',
            data: interpolatedLine,
            smooth: true,
            showSymbol: false
          };

          const existingSeries = isContinuous ? chart.getOption().series.filter(s => s.name !== 'Raw Channels') : [];

          chart.hideLoading();
          chart.setOption({
            series: [
              ...rawChannelsSeries,
              ...existingSeries,
              newSeries
            ]
          });
          status.textContent = `Scan complete. Total scans: ${scanHistory.length}`;
          updateStatus();
        })
        .catch(error => {
          console.error('Error:', error);
          chart.hideLoading();
          status.textContent = `Error: ${error.message}`;
          if(continuousScanIntervalId) { toggleContinuousScan(); } // Stop on error
        })
        .finally(() => {
          startBtn.disabled = false;
        });
    }

    function updateColorSwatch(spectrum) {
        function getSpectrumValue(wavelength) {
            if (!spectrum || spectrum.length < 2) return 0;
            for (let i = 0; i < spectrum.length - 1; i++) {
                if (wavelength >= spectrum[i][0] && wavelength <= spectrum[i+1][0]) {
                    const w1 = spectrum[i][0], v1 = spectrum[i][1];
                    const w2 = spectrum[i+1][0], v2 = spectrum[i+1][1];
                    if (w2 - w1 === 0) return v1;
                    return v1 + (v2 - v1) * (wavelength - w1) / (w2 - w1);
                }
            }
            return 0;
        }

        let X = 0, Y = 0, Z = 0;
         for (let i = 0; i < CMF.wavelengths.length; i++) {
            const lambda = CMF.wavelengths[i];
            const val = getSpectrumValue(lambda);

            if (!isFinite(val)) {
                console.warn(`Skipping invalid spectrum value at ${lambda} nm`);
                continue; // skip bad points instead of returning
            }

            const x = CMF.x[i], y = CMF.y[i], z = CMF.z[i];
            if (!isFinite(x) || !isFinite(y) || !isFinite(z)) {
                console.warn(`Skipping invalid CMF data at ${lambda} nm`);
                continue;
            }

            X += val * x;
            Y += val * y;
            Z += val * z;
        }

        if (Y === 0 || isNaN(X) || isNaN(Y) || isNaN(Z)) {
            document.getElementById('color-swatch').style.backgroundColor = 'rgb(0,0,0)';
            return;
        }

        let Rl =  3.2406 * X - 1.5372 * Y - 0.4986 * Z;
        let Gl = -0.9689 * X + 1.8758 * Y + 0.0415 * Z;
        let Bl =  0.0557 * X - 0.2040 * Y + 1.0570 * Z;

        const max = Math.max(Rl, Gl, Bl);
        if (max > 0) {
            Rl /= max; Gl /= max; Bl /= max;
        }

        Rl = Math.max(0, Rl); Gl = Math.max(0, Gl); Bl = Math.max(0, Bl);

        const gamma = c => (c <= 0.0031308) ? 12.92 * c : 1.055 * Math.pow(c, 1/2.4) - 0.055;
        
        let r = Math.round(gamma(Rl) * 255);
        let g = Math.round(gamma(Gl) * 255);
        let b = Math.round(gamma(Bl) * 255);

        if (isNaN(r) || isNaN(g) || isNaN(b)) return;

        document.getElementById('color-swatch').style.backgroundColor = `rgb(${r},${g},${b})`;
    }

    function saveData() {
      if (scanHistory.length === 0) {
        alert('No data to save. Please perform a scan first.');
        return;
      }

      let fileContent = '# Tricoder Spectrum Data\n';
      scanHistory.forEach((scan, index) => {
        fileContent += `\n# --- Scan ${index + 1} at ${scan.timestamp.toISOString()} ---\n`;
        fileContent += '# Raw Sensor Channels\n';
        fileContent += '# Wavelength (nm),Normalized Intensity\n';
        scan.data.raw.forEach(row => { fileContent += row.join(',') + '\n'; });
        fileContent += '\n# Interpolated Spectrum\n';
        fileContent += '# Wavelength (nm),Normalized Intensity\n';
        scan.data.interpolated.forEach(row => { fileContent += row.join(',') + '\n'; });
      });

      const blob = new Blob([fileContent], { type: 'text/plain;charset=utf-8,' });
      const link = document.createElement('a');
      link.href = URL.createObjectURL(blob);
      const timestamp = new Date().toISOString().replace(/[:.-]/g, '');
      link.download = `tricoder_multiscan_${timestamp}.txt`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(link.href);
    }

    function loadConfig() {
      fetch('/getconfig')
        .then(response => response.json())
        .then(data => {
          document.getElementById('accumTime').value = data.accumulationTime;
          document.getElementById('scanInterval').value = data.scanInterval;
        });
    }

    function saveConfig() {
      const time = document.getElementById('accumTime').value;
      const interval = document.getElementById('scanInterval').value;
      const status = document.getElementById('status-text');
      
      const formData = new FormData();
      formData.append('accumulationTime', time);
      formData.append('scanInterval', interval);

      fetch('/setconfig', { method: 'POST', body: new URLSearchParams(formData) })
        .then(response => {
            if(response.ok) {
                status.textContent = `Configuration saved.`;
            } else {
                status.textContent = 'Error saving configuration.';
            }
        });
    }

    function resetWifi() {
      if (confirm('This will reset WiFi settings and restart the device. Are you sure?')) {
        fetch('/reset-wifi').then(() => alert('Device is restarting in AP mode.'));
      }
    }

    window.onload = function() {
      initChart();
      loadConfig();
      setInterval(updateStatus, 5000);
    };

    window.onresize = () => chart && chart.resize();
  </script>
</body>
</html>
)rawliteral";

const char CONFIG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WiFi Configuration</title>
  <style>
    body { font-family: Arial, sans-serif; max-width: 500px; margin: 50px auto; padding: 20px; background: #f0f0f0; }
    .container { background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    h1 { color: #333; text-align: center; margin-bottom: 30px; }
    .form-group { margin-bottom: 20px; }
    label { display: block; margin-bottom: 5px; color: #555; font-weight: bold; }
    input { width: 100%; padding: 12px; border: 1px solid #ddd; border-radius: 5px; font-size: 16px; box-sizing: border-box; }
    button { width: 100%; padding: 15px; background: #4CAF50; color: white; border: none; border-radius: 5px; font-size: 18px; cursor: pointer; }
    button:hover { background: #45a049; }
  </style>
</head>
<body>
  <div class="container">
    <h1>WiFi Configuration</h1>
    <form onsubmit="saveWifi(event)">
      <div class="form-group">
        <label for="ssid">WiFi Network (SSID):</label>
        <input type="text" id="ssid" name="ssid" required>
      </div>
      <div class="form-group">
        <label for="password">Password:</label>
        <input type="password" id="password" name="password" required>
      </div>
      <button type="submit">Save and Restart</button>
    </form>
  </div>
  <script>
    function saveWifi(event) {
      event.preventDefault();
      const formData = new FormData(event.target);
      fetch('/save-wifi', { method: 'POST', body: new URLSearchParams(formData) })
        .then(() => alert('WiFi settings saved. The device will now restart.'));
    }
  </script>
</body>
</html>
)rawliteral";

#endif