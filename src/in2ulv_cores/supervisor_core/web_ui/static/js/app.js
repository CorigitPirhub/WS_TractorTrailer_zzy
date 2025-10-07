// 连接到WebSocket服务器
const socket = io.connect('http://' + document.domain + ':' + location.port);

// 初始化图表
const ctx = document.getElementById('topic-chart').getContext('2d');
const topicChart = new Chart(ctx, {
    type: 'bar',
    data: {
        labels: [],
        datasets: [{
            label: '发布频率 (Hz)',
            data: [],
            backgroundColor: 'rgba(54, 162, 235, 0.5)',
            borderColor: 'rgba(54, 162, 235, 1)',
            borderWidth: 1
        }]
    },
    options: {
        responsive: true,
        scales: {
            y: {
                beginAtZero: true,
                title: {
                    display: true,
                    text: '频率 (Hz)'
                }
            },
            x: {
                title: {
                    display: true,
                    text: '话题名称'
                }
            }
        }
    }
});

// 处理节点更新
socket.on('node_update', function(data) {
    updateNodeTable(data);
});

socket.on('metric_update', function(data) {
    if (data.metric_name === "node_status") {
        updateNodeStatus(data.node_name, data.value);
    } else if (data.metric_name.includes("publish_frequency")) {
        updateTopicFrequency(data.topic, data.value);
    }
});

// 处理话题更新
socket.on('topic_update', function(data) {
    updateTopicChart(data);
});

// 处理告警更新
socket.on('alert_update', function(data) {
    updateAlertList(data);
});

socket.on('new_alert', function(alert) {
    addNewAlert(alert);
});

// 更新节点表格
function updateNodeTable(nodeData) {
    const tableBody = document.querySelector('#node-table tbody');
    tableBody.innerHTML = '';
    
    for (const [node, info] of Object.entries(nodeData)) {
        const row = document.createElement('tr');
        
        const nodeCell = document.createElement('td');
        nodeCell.textContent = node;
        
        const statusCell = document.createElement('td');
        statusCell.textContent = getStatusText(info.status);
        statusCell.className = getStatusClass(info.status);
        
        const timeCell = document.createElement('td');
        timeCell.textContent = formatTime(info.timestamp);
        
        row.appendChild(nodeCell);
        row.appendChild(statusCell);
        row.appendChild(timeCell);
        tableBody.appendChild(row);
    }
}

// 更新节点状态
function updateNodeStatus(nodeName, status) {
    const tableBody = document.querySelector('#node-table tbody');
    let rowFound = false;
    
    // 查找现有行
    for (const row of tableBody.rows) {
        if (row.cells[0].textContent === nodeName) {
            row.cells[1].textContent = getStatusText(status);
            row.cells[1].className = getStatusClass(status);
            row.cells[2].textContent = formatTime(Date.now() / 1000);
            rowFound = true;
            break;
        }
    }
    
    // 如果没有找到，添加新行
    if (!rowFound) {
        const row = document.createElement('tr');
        
        const nodeCell = document.createElement('td');
        nodeCell.textContent = nodeName;
        
        const statusCell = document.createElement('td');
        statusCell.textContent = getStatusText(status);
        statusCell.className = getStatusClass(status);
        
        const timeCell = document.createElement('td');
        timeCell.textContent = formatTime(Date.now() / 1000);
        
        row.appendChild(nodeCell);
        row.appendChild(statusCell);
        row.appendChild(timeCell);
        tableBody.appendChild(row);
    }
}

// 更新话题图表
function updateTopicChart(topicData) {
    const topics = Object.keys(topicData);
    const frequencies = topics.map(topic => topicData[topic].frequency);
    
    topicChart.data.labels = topics;
    topicChart.data.datasets[0].data = frequencies;
    topicChart.update();
}

// 更新话题频率
function updateTopicFrequency(topic, frequency) {
    const topics = topicChart.data.labels;
    const index = topics.indexOf(topic);
    
    if (index !== -1) {
        topicChart.data.datasets[0].data[index] = frequency;
        topicChart.update();
    } else {
        // 新话题，添加到图表
        topics.push(topic);
        topicChart.data.datasets[0].data.push(frequency);
        topicChart.update();
    }
}

// 更新告警列表
function updateAlertList(alerts) {
    const alertList = document.getElementById('alert-list');
    alertList.innerHTML = '';
    
    for (const alert of alerts) {
        addAlertToDOM(alert);
    }
}

// 添加新告警
function addNewAlert(alert) {
    addAlertToDOM(alert);
}

// 添加告警到DOM
function addAlertToDOM(alert) {
    const alertList = document.getElementById('alert-list');
    const alertDiv = document.createElement('div');
    
    alertDiv.className = `alert-item alert-${alert.severity}`;
    
    const header = document.createElement('div');
    header.className = 'alert-header';
    
    const sourceSpan = document.createElement('span');
    sourceSpan.textContent = alert.source;
    
    const timeSpan = document.createElement('span');
    timeSpan.className = 'alert-time';
    timeSpan.textContent = formatTime(alert.timestamp);
    
    header.appendChild(sourceSpan);
    header.appendChild(timeSpan);
    
    const messageDiv = document.createElement('div');
    messageDiv.textContent = alert.message;
    
    alertDiv.appendChild(header);
    alertDiv.appendChild(messageDiv);
    
    // 添加到列表顶部
    alertList.insertBefore(alertDiv, alertList.firstChild);
}

// 获取状态文本
function getStatusText(status) {
    if (status === 0) return '正常';
    if (status === 1) return '警告';
    if (status === 2) return '错误';
    return '未知';
}

// 获取状态类名
function getStatusClass(status) {
    if (status === 0) return 'status-normal';
    if (status === 1) return 'status-warning';
    if (status === 2) return 'status-error';
    return '';
}

// 格式化时间
function formatTime(timestamp) {
    const date = new Date(timestamp * 1000);
    return date.toLocaleTimeString();
}