// observer/app/static/js/main.js

let cameraWs = null;
let lidarWs = null;

// 카메라 WebSocket 초기화
function initCameraWebSocket() {
    cameraWs = new WebSocket(`ws://${window.location.host}/ws/camera`);
    cameraWs.onmessage = function (event) {
        if (event.data instanceof Blob) {
            const url = URL.createObjectURL(event.data);
            document.getElementById('camera-feed').src = url;
        }
    };
}

// LiDAR WebSocket 초기화
function initLidarWebSocket() {
    lidarWs = new WebSocket(`ws://${window.location.host}/ws/lidar`);
    lidarWs.onmessage = function (event) {
        const data = JSON.parse(event.data);

        // LiDAR 데이터 시각화
        const ranges = data.data.ranges;
        const angles = Array.from({ length: ranges.length }, (_, i) =>
            i * (data.data.angle_increment * 180 / Math.PI)
        );

        const plotData = [{
            type: 'scatterpolar',
            r: ranges,
            theta: angles,
            mode: 'markers',
            marker: {
                size: 3,
                color: ranges,
                colorscale: 'Viridis'
            }
        }];

        const layout = {
            //polar: { radialaxis: { visible: true, range: [0, data.data.range_max] } },
            polar: { radialaxis: { visible: true, range: [0, 10] } },
            showlegend: false,
            margin: { t: 0, b: 0, l: 0, r: 0 },
            paper_bgcolor: 'rgba(0,0,0,0)',
            plot_bgcolor: 'rgba(0,0,0,0)'
        };

        Plotly.newPlot('lidar-plot', plotData, layout);
    };
}

// Jetson 상태 관리를 위한 클래스
class JetsonStatusManager {
  constructor() {
    this.jetsonIds = new Set();
    this.statusContainer = document.getElementById('jetson-status-container');
  }

  // Jetson ID 추가 및 UI 생성
  addJetson(jetsonId) {
    if (this.jetsonIds.has(jetsonId)) return;
    this.jetsonIds.add(jetsonId);
    this.createJetsonStatusUI(jetsonId);
  }

  // Jetson 상태 UI 생성
  createJetsonStatusUI(jetsonId) {
    const statusHtml = `
      <div class="mb-4 last:mb-0">
        <h3 class="text-lg font-semibold text-gray-700 mb-2">Jetson ${jetsonId}</h3>
        <div class="grid grid-cols-2 md:grid-cols-5 gap-4">
          <div class="p-4 bg-gray-50 rounded-lg">
            <div class="text-sm text-gray-600">Status</div>
            <div class="text-2xl font-bold text-green-600" data-jetson-id="${jetsonId}" data-status-type="status">OK</div>
          </div>
          <div class="p-4 bg-gray-50 rounded-lg">
            <div class="text-sm text-gray-600">Latency</div>
            <div class="text-2xl font-bold" data-jetson-id="${jetsonId}" data-status-type="latency">0 ms</div>
          </div>
          <div class="p-4 bg-gray-50 rounded-lg">
            <div class="text-sm text-gray-600">Battery</div>
            <div class="text-2xl font-bold text-gray-500" data-jetson-id="${jetsonId}" data-status-type="battery">N/A</div>
          </div>
          <div class="p-4 bg-gray-50 rounded-lg"></div>
          <div class="p-4 bg-gray-50 rounded-lg"></div>
        </div>
      </div>
    `;
    this.statusContainer.insertAdjacentHTML('beforeend', statusHtml);
  }

  // 상태 업데이트
  updateStatus(jetsonId, type, value) {
    const element = document.querySelector(`[data-jetson-id="${jetsonId}"][data-status-type="${type}"]`);
    if (element) {
      element.textContent = value;
    }
  }
}

// 예: /api/status 응답 형태:
// {
//   "jetsons": [
//     {"id": "001", "status": "OK", "latency": 12, "battery": "N/A"},
//     {"id": "00A", "status": "OK", "latency": 34, "battery": "84%"}
//   ],
//   "connected_clients": { "camera": 1, "lidar": 1 }
// }

// observer/app/static/js/main.js

async function pollStatus() {
  setInterval(async () => {
    try {
      // 1) /api/status 엔드포인트 호출
      const response = await fetch('/api/status');
      const data = await response.json(); // { "jetsons": [...], "connected_clients": {...} }

      // 2) connected_clients 표시
      const cameraCount = data.connected_clients?.camera || 0;
      const lidarCount = data.connected_clients?.lidar || 0;
      document.getElementById('client-count').textContent 
        = `Clients: Camera (${cameraCount}), LiDAR (${lidarCount})`;

      // 3) System Status 섹션: Jetson 목록 갱신
      updateJetsonSystemStatus(data.jetsons || []);
    } catch (error) {
      console.error('Status update failed:', error);
    }
  }, 2000);
}

function updateJetsonSystemStatus(jetsonsArray) {
  const container = document.getElementById('system-status-container');
  if (!container) return;

  // (A) 기존 내용 초기화
  container.innerHTML = '';

  // (B) Jetsons 배열 순회
  jetsonsArray.forEach(jetson => {
    // jetson: { id: "001", status: "OK", latency: 12, battery: "N/A" }

    // (B1) 패널 컨테이너
    const panel = document.createElement('div');
    panel.className = 'p-4 bg-gray-50 rounded-lg';

    // (B2) 타이틀
    const title = document.createElement('div');
    title.className = 'text-sm text-gray-600 mb-1';
    title.textContent = `Jetson ${jetson.id}`;
    panel.appendChild(title);

    // (B3) Status
    const statStatus = document.createElement('div');
    statStatus.className = 'text-md font-bold text-green-600 mb-1';
    statStatus.textContent = `Status: ${jetson.status || 'N/A'}`;
    panel.appendChild(statStatus);

    // (B4) Latency
    const statLat = document.createElement('div');
    statLat.className = 'text-md font-bold mb-1';
    statLat.textContent = `Latency: ${jetson.latency || 0} ms`;
    panel.appendChild(statLat);

    // (B5) Battery
    const statBattery = document.createElement('div');
    statBattery.className = 'text-md font-bold text-gray-700';
    statBattery.textContent = `Battery: ${jetson.battery || 'N/A'}`;
    panel.appendChild(statBattery);

    // (B6) 실제 container에 추가
    container.appendChild(panel);
  });
}

// 초기화 함수
function initDashboard() {
  // 만약 카메라/라이다 WebSocket도 초기화한다면 여기서...
  initCameraWebSocket();
  initLidarWebSocket();

  // pollStatus로 /api/status 주기적 호출
  pollStatus();
}

// DOM 로드 후 init
window.addEventListener('DOMContentLoaded', initDashboard);

