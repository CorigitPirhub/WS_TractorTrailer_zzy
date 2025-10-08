// 连接到WebSocket服务器
const socket = io.connect('http://' + document.domain + ':' + location.port);

// 存储数据
let nodeData = {};  // 节点状态数据
let publishData = {};  // 发布数据: { nodeName: { topic: { msg_type: type, frequency: value } } }
let subscribeData = {};  // 订阅数据: { nodeName: { topic: { msg_type: type, frequency: value } } }

// 处理节点更新
socket.on('node_update', function(data) {
    if (data.data) {
        // 初始化数据
        nodeData = data.data;
        updateNodeTable(nodeData);
    } else {
        // 更新单个节点
        updateNodeStatus(data.node_name, data.status, data.timestamp);
    }
});

// 处理发布更新
socket.on('publish_update', function(data) {
    if (data.data) {
        // 初始化数据
        publishData = data.data;
        renderPublishData();
    } else {
        // 更新单个发布项
        if (!publishData[data.node_name]) {
            publishData[data.node_name] = {};
        }
        publishData[data.node_name][data.topic] = {
            msg_type: data.msg_type,
            frequency: data.frequency
        };
        renderPublishData();
    }
});

// 处理订阅更新
socket.on('subscribe_update', function(data) {
    if (data.data) {
        // 初始化数据
        subscribeData = data.data;
        renderSubscribeData();
    } else {
        // 更新单个订阅项
        if (!subscribeData[data.node_name]) {
            subscribeData[data.node_name] = {};
        }
        subscribeData[data.node_name][data.topic] = {
            msg_type: data.msg_type,
            frequency: data.frequency
        };
        renderSubscribeData();
    }
});

// 处理告警
socket.on('alert_update', function(response) {
    updateAlertList(response.data);
});

socket.on('new_alert', function(alert) {
    addNewAlert(alert);
});

// 更新节点表格
function updateNodeStatus(nodeName, status, timestamp) {
    const tableBody = document.querySelector('#node-table tbody');
    let rowFound = false;
    
    // 查找现有行
    for (const row of tableBody.rows) {
        if (row.cells[0].textContent === nodeName) {
            row.cells[1].textContent = getStatusText(status);
            row.cells[1].className = getStatusClass(status);
            row.cells[2].textContent = formatTime(timestamp);
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
        timeCell.textContent = formatTime(timestamp);
        
        row.appendChild(nodeCell);
        row.appendChild(statusCell);
        row.appendChild(timeCell);
        tableBody.appendChild(row);
    }
}

// 更新节点表格（初始化）
function updateNodeTable(nodeData) {
    const tableBody = document.querySelector('#node-table tbody');
    tableBody.innerHTML = '';
    
    if (!nodeData || Object.keys(nodeData).length === 0) {
        const row = document.createElement('tr');
        row.innerHTML = '<td colspan="3" style="text-align: center; color: #999;">暂无节点数据</td>';
        tableBody.appendChild(row);
        return;
    }
    
    for (const [nodeName, info] of Object.entries(nodeData)) {
        const row = document.createElement('tr');
        
        const nodeCell = document.createElement('td');
        nodeCell.textContent = nodeName;
        
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

// 渲染发布数据
function renderPublishData() {
    const container = document.getElementById('publish-container');
    container.innerHTML = '';
    
    if (Object.keys(publishData).length === 0) {
        container.innerHTML = '<div class="no-data">暂无发布数据</div>';
        return;
    }
    
    for (const [nodeName, topics] of Object.entries(publishData)) {
        const groupDiv = document.createElement('div');
        groupDiv.className = 'node-group';
        
        // 节点标题
        const headerDiv = document.createElement('div');
        headerDiv.className = 'node-header';
        headerDiv.textContent = nodeName;
        groupDiv.appendChild(headerDiv);
        
        // 创建表格
        const table = document.createElement('table');
        table.className = 'topic-table';
        
        // 表头
        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const headers = ['话题', '消息类型', '频率'];
        headers.forEach(text => {
            const th = document.createElement('th');
            th.textContent = text;
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);
        
        // 表格内容
        const tbody = document.createElement('tbody');
        for (const [topic, info] of Object.entries(topics)) {
            const row = document.createElement('tr');
            
            // 话题名称
            const topicCell = document.createElement('td');
            topicCell.textContent = topic;
            row.appendChild(topicCell);
            
            // 消息类型
            const typeCell = document.createElement('td');
            typeCell.textContent = info.msg_type;
            row.appendChild(typeCell);
            
            // 频率
            const freqCell = document.createElement('td');
            freqCell.textContent = info.frequency.toFixed(2) + ' Hz';
            row.appendChild(freqCell);
            
            tbody.appendChild(row);
        }
        table.appendChild(tbody);
        groupDiv.appendChild(table);
        container.appendChild(groupDiv);
    }
}

// 渲染订阅数据
function renderSubscribeData() {
    const container = document.getElementById('subscribe-container');
    container.innerHTML = '';
    
    if (Object.keys(subscribeData).length === 0) {
        container.innerHTML = '<div class="no-data">暂无订阅数据</div>';
        return;
    }
    
    for (const [nodeName, topics] of Object.entries(subscribeData)) {
        const groupDiv = document.createElement('div');
        groupDiv.className = 'node-group';
        
        // 节点标题
        const headerDiv = document.createElement('div');
        headerDiv.className = 'node-header';
        headerDiv.textContent = nodeName;
        groupDiv.appendChild(headerDiv);
        
        // 创建表格
        const table = document.createElement('table');
        table.className = 'topic-table';
        
        // 表头
        const thead = document.createElement('thead');
        const headerRow = document.createElement('tr');
        const headers = ['话题', '消息类型', '频率'];
        headers.forEach(text => {
            const th = document.createElement('th');
            th.textContent = text;
            headerRow.appendChild(th);
        });
        thead.appendChild(headerRow);
        table.appendChild(thead);
        
        // 表格内容
        const tbody = document.createElement('tbody');
        for (const [topic, info] of Object.entries(topics)) {
            const row = document.createElement('tr');
            
            // 话题名称
            const topicCell = document.createElement('td');
            topicCell.textContent = topic;
            row.appendChild(topicCell);
            
            // 消息类型
            const typeCell = document.createElement('td');
            typeCell.textContent = info.msg_type;
            row.appendChild(typeCell);
            
            // 频率
            const freqCell = document.createElement('td');
            freqCell.textContent = info.frequency.toFixed(2) + ' Hz';
            row.appendChild(freqCell);
            
            tbody.appendChild(row);
        }
        table.appendChild(tbody);
        groupDiv.appendChild(table);
        container.appendChild(groupDiv);
    }
}

// 更新告警列表
function updateAlertList(alerts) {
    const alertList = document.getElementById('alert-list');
    alertList.innerHTML = '';
    
    if (!alerts || alerts.length === 0) {
        const noAlertItem = document.createElement('div');
        noAlertItem.className = 'alert-item alert-info';
        noAlertItem.innerHTML = '<div class="alert-header">系统状态</div><div>暂无告警信息</div>';
        alertList.appendChild(noAlertItem);
        return;
    }
    
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
    if (status === 0 || status === 0.0) return '正常';
    if (status === 1 || status === 1.0) return '警告';
    if (status === 2 || status === 2.0) return '错误';
    return '未知';
}

// 获取状态类名
function getStatusClass(status) {
    if (status === 0 || status === 0.0) return 'status-normal';
    if (status === 1 || status === 1.0) return 'status-warning';
    if (status === 2 || status === 2.0) return 'status-error';
    return '';
}

// 格式化时间
function formatTime(timestamp) {
    if (!timestamp || isNaN(timestamp)) {
        return "未更新";
    }
    
    try {
        // ROS时间戳是秒，需要乘以1000转换为毫秒
        const date = new Date(timestamp * 1000);
        if (isNaN(date.getTime())) {
            return "时间格式错误";
        }
        return date.toLocaleTimeString();
    } catch (e) {
        return "时间解析错误";
    }
}

// 初始化页面
document.addEventListener('DOMContentLoaded', function() {
    // 初始化空表格
    renderPublishData();
    renderSubscribeData();
    updateAlertList([]);
});