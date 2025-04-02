#pragma once

#include <Arduino.h>

const char html[] PROGMEM = R"rawliteral(
    <!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ESP32 Balance Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1.0, user-scalable=no">
    <style>
        body { 
            touch-action: pan-x pan-y;
            font-family: Arial; 
            margin: 20px;
            background-color: #f5f5f5;
        }
        .collapse-btn {
            background: #e0e0e0;
            border: 1px solid #ccc;
            padding: 15px;
            width: 100%;
            text-align: left;
            font-size: 24px;
            cursor: pointer;
            transition: 0.3s;
            border-radius: 5px;
            margin: 5px 0;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .collapse-content {
            display: none;
            overflow: hidden;
            background: #fff;
            border-radius: 0 0 5px 5px;
            padding: 10px;
        }
        .control-group {
            padding: 10px;
        }
        .sub-btn {
            display: block;
            width: 90%;
            padding: 12px;
            margin: 8px auto;
            background: #2196F3;
            color: white;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            transition: 0.2s;
            font-size: 20px;
        }
        .sub-btn:hover {
            background: #1976D2;
        }
        .slider-container {
            display: flex;
            align-items: center;
            margin: 15px 0;
            padding: 10px;
            background: #f8f9fa;
            border-radius: 8px;
        }
        .slider-label {
            width: 140px;
            margin-right: 15px;
            font-weight: bold;
            color: #333;
            font-size: 20px;
        }
        .slider-input {
            flex: 1;
            margin: 0 15px;
            height: 6px;
            background: #ddd;
            border-radius: 3px;
        }
        .number-input {
            width: 80px;
            padding: 8px;
            border: 1px solid #ccc;
            border-radius: 4px;
            text-align: center;
            font-size: 20px;
        }
        /* 新增样式 */
        .param-btns {
            display: flex;
            gap: 15px;
            justify-content: center;
            margin-top: 20px;
        }

        .get-btn, .save-btn {
            padding: 12px 25px;
            border: none;
            border-radius: 15px;
            cursor: pointer;
            font-weight: bold;
            transition: 0.2s;
            font-size: 20px;
        }

        .get-btn {
            background: #2196F3;
            color: white;
        }

        .save-btn {
            background: #4CAF50;
            color: white;
        }

        .get-btn:hover {
            background: #1976D2;
        }

        .save-btn:hover {
            background: #45a049;
        }
        .send-btn {
            /* padding: 12px 35px; */
            /* height: 8 px; */
            background: #4CAF50;
            color: white;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
            margin-left: 30px;
            font-size: 20px;
        }

        .send-btn:hover {
            background: #45a049;
        }
    </style>
</head>
<body>
    <h1 style="text-align: center; color: #2c3e50;">Cubli_Mini_S_Pro</h1>

    <!-- 控制模块 -->
    <button class="collapse-btn" onclick="toggleCollapse(1)">▼ 控制</button>
    <div class="collapse-content" id="group1">
        <div class="control-group">
            <button class="sub-btn" onclick="sendCommand('idle')">空闲</button>
            <button class="sub-btn" onclick="sendCommand('u_balance')">边平衡</button>
            <button class="sub-btn" onclick="sendCommand('point_balance')">点平衡</button>
            <button class="sub-btn" onclick="sendCommand('edge_jump')">边起跳</button>
            <button class="sub-btn" onclick="sendCommand('point_jump')">点起跳</button>
            <div class="slider-container" data-param="rotate_speed">
                <span class="slider-label">旋转速度</span>
                <input type="range" class="slider-input" min="-10" max="10" step="0.1" value="0">
                <input type="number" class="number-input" id="rotate-value" 
                    min="-10" max="10" step="0.1" value="0">
                <button class="send-btn">发送</button>
            </div>
        </div>
    </div>

    <!-- 校准模块 -->
    <button class="collapse-btn" onclick="toggleCollapse(2)">▶ 校准设置</button>
    <div class="collapse-content" id="group2">
        <div class="control-group">
            <button class="sub-btn" onclick="sendCommand('calibrate_edge')">边校准</button>
            <button class="sub-btn" onclick="sendCommand('calibrate_point')">点校准</button>
            <button class="sub-btn" onclick="sendCommand('autoCalibrate_edge')">自动边校准</button>
            <button class="sub-btn" onclick="sendCommand('autoCalibrate_point')">自动点校准</button>
        </div>
    </div>

    <!-- 起跳参数 -->
    <button class="collapse-btn" onclick="toggleCollapse(3)">▶ 起跳参数</button>
    <div class="collapse-content" id="group3">
        <div class="control-group">
            <div class="param-btns">
                <button class="get-btn" onclick="getParams('jump')">获取参数</button>
                <button class="save-btn" onclick="saveParams('jump')">保存参数</button>
            </div>
            <div class="slider-container" data-param="u_jump_speed">
                <span class="slider-label">边起跳速度</span>
                <input type="range" class="slider-input" min="-600" max="600" value="0" >
                <input type="number" class="number-input" id="edge-speed" 
                    min="-600" max="600" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_jump_speed_ch1">
                <span class="slider-label">点CH1</span>
                <input type="range" class="slider-input" min="-600" max="600" value="0">
                <input type="number" class="number-input" id="point-speed_ch1" 
                    min="-600" max="600" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_jump_speed_ch2">
                <span class="slider-label">点CH2</span>
                <input type="range" class="slider-input" min="-600" max="600" value="0">
                <input type="number" class="number-input" id="point-speed_ch2" 
                    min="-600" max="600" value="0">
                <button class="send-btn">发送</button>
            </div>
        </div>
    </div>

    <!-- 边缘平衡参数 -->
    <button class="collapse-btn" onclick="toggleCollapse(4)">▶ 边平衡参数</button>
    <div class="collapse-content" id="group4">
        <div class="control-group">
            <div class="param-btns">
                <button class="get-btn" onclick="getParams('u_balance')">获取参数</button>
                <button class="save-btn" onclick="saveParams('u_balance')">保存参数</button>
            </div>            
            <div class="slider-container" data-param="u_ka">
                <span class="slider-label">Ka</span>
                <input type="range" class="slider-input" min="-100" max="0" step="0.1" value="0" >
                <input type="number" class="number-input" id="ka-value" 
                    min="-100" max="100" step="0.1" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="u_kp">
                <span class="slider-label">Kp</span>
                <input type="range" class="slider-input" min="-5" max="5" step="0.05" value="0">
                <input type="number" class="number-input" id="kp-value" 
                    min="-5" max="5" step="0.05" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="u_kv">
                <span class="slider-label">Kv</span>
                <input type="range" class="slider-input" min="-3" max="3" step="0.005" value="0">
                <input type="number" class="number-input" id="kv-value" 
                    min="-3" max="3" step="0.005" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="u_ks">
                <span class="slider-label">Ks</span>
                <input type="range" class="slider-input" min="-1" max="1" step="0.002" value="0">
                <input type="number" class="number-input" id="ks-value" 
                    min="-1" max="1" step="0.002" value="0">
                <button class="send-btn">发送</button>
            </div>
        </div>
    </div>

    <!-- 空间参数 -->
    <button class="collapse-btn" onclick="toggleCollapse(5)">▶ 点平衡参数</button>
    <div class="collapse-content" id="group5">
        <div class="control-group">
            <!-- X轴参数 -->
            <div class="param-btns">
                <button class="get-btn" onclick="getParams('p_balance')">获取参数</button>
                <button class="save-btn" onclick="saveParams('p_balance')">保存参数</button>
            </div>
            <div class="slider-container" data-param="p_xa">
                <span class="slider-label">XA</span>
                <input type="range" class="slider-input" min="-100" max="100" step="0.1" value="0">
                <input type="number" class="number-input" id="xa-value" 
                    min="-100" max="100" step="0.1" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_ya">
                <span class="slider-label">YA</span>
                <input type="range" class="slider-input" min="-100" max="100" step="0.1" value="0">
                <input type="number" class="number-input" id="ya-value" 
                    min="-100" max="100" step="0.1" value="0">
                <button class="send-btn">发送</button>
            </div>
            
            <!-- X轴PID -->
            <div class="slider-container" data-param="p_xka">
                <span class="slider-label">XKP</span>
                <input type="range" class="slider-input" min="-3" max="3" step="0.05" value="0">
                <input type="number" class="number-input" id="xkp-value" 
                    min="-3" max="3" step="0.05" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_xkv">
                <span class="slider-label">XKV</span>
                <input type="range" class="slider-input" min="-1" max="1" step="0.01" value="0">
                <input type="number" class="number-input" id="xkv-value" 
                    min="-1" max="1" step="0.01" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_xks">
                <span class="slider-label">XKS</span>
                <input type="range" class="slider-input" min="-0.5" max="0.5" step="0.001" value="0">
                <input type="number" class="number-input" id="xks-value" 
                    min="-0.5" max="0.5" step="0.001" value="0">
                <button class="send-btn">发送</button>
            </div>

            <!-- Y轴PID -->
            <div class="slider-container" data-param="p_ykp">
                <span class="slider-label">YKP</span>
                <input type="range" class="slider-input" min="-3" max="3" step="0.05" value="0">
                <input type="number" class="number-input" id="ykp-value" 
                    min="-3" max="3" step="0.05" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_ykv">
                <span class="slider-label">YKV</span>
                <input type="range" class="slider-input" min="-1" max="1" step="0.01" value="0">
                <input type="number" class="number-input" id="ykv-value" 
                    min="-1" max="1" step="0.01" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_yks">
                <span class="slider-label">YKS</span>
                <input type="range" class="slider-input" min="-0.5" max="0.5" step="0.001" value="0">
                <input type="number" class="number-input" id="yks-value" 
                    min="-0.5" max="0.5" step="0.001" value="0">
                <button class="send-btn">发送</button>
            </div>

            <!-- Z轴PID -->
            <div class="slider-container" data-param="p_zkp">
                <span class="slider-label">ZKP</span>
                <input type="range" class="slider-input" min="-3" max="3" step="0.05" value="0">
                <input type="number" class="number-input" id="zkp-value" 
                    min="-3" max="3" step="0.05" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_zkv">
                <span class="slider-label">ZKV</span>
                <input type="range" class="slider-input" min="-1" max="1" step="0.01" value="0">
                <input type="number" class="number-input" id="zkv-value" 
                    min="-1" max="1" step="0.01" value="0">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="p_zks">
                <span class="slider-label">ZKS</span>
                <input type="range" class="slider-input" min="-0.5" max="0.5" step="0.001" value="0">
                <input type="number" class="number-input" id="zks-value" 
                    min="-0.5" max="0.5" step="0.001" value="0">
                <button class="send-btn">发送</button>
            </div>
        </div>
    </div>

    <!-- 保护 -->
    <button class="collapse-btn" onclick="toggleCollapse(6)">▶ 保护参数</button>
    <div class="collapse-content" id="group6">
        <div class="control-group">
            <div class="param-btns">
                <button class="get-btn" onclick="getParams('protected')">获取参数</button>
                <button class="save-btn" onclick="saveParams('protected')">保存参数</button>
            </div>
            <div class="slider-container" data-param="low_power">
                <span class="slider-label">低电压</span>
                <input type="range" class="slider-input" min="0" max="24" step="0.1" value="1000" >
                <input type="number" class="number-input" id="low_power_value" 
                    min="0" max="24" step="0.1" value="1000">
                <button class="send-btn">发送</button>
            </div>
            <div class="slider-container" data-param="opt">
                <span class="slider-label">高温</span>
                <input type="range" class="slider-input" min="0" max="120" step="0.1" value="0">
                <input type="number" class="number-input" id="opt_value" 
                    min="0" max="120" step="0.1" value="0">
                <button class="send-btn">发送</button>
            </div>
        </div>
    </div>

    <script>
        // 同步逻辑保持不变
        document.querySelectorAll('.slider-input').forEach(slider => {
            slider.addEventListener('input', function() {
                this.nextElementSibling.value = this.value;
            });
        });

        document.querySelectorAll('.value-input').forEach(input => {
            input.addEventListener('change', function() {
                const slider = this.previousElementSibling;
                const min = parseFloat(slider.min);
                const max = parseFloat(slider.max);
                let value = parseFloat(this.value);
                value = Math.max(min, Math.min(max, value));
                slider.value = value;
                this.value = value;
            });
        });

        document.querySelectorAll('.send-btn').forEach(btn => {
            btn.addEventListener('click', function() {
                // 获取参数名称和值
                const container = this.closest('.slider-container');
                const paramName = container.dataset.param; // 从data-param获取参数名
                const valueInput = container.querySelector('.number-input');
                const value = valueInput.value;

                // 参数验证
                if (!paramName || isNaN(value)) {
                    alert('参数配置错误');
                    return;
                }

                // 发送数据
                fetch('/setParam', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `param=${encodeURIComponent(paramName)}&value=${encodeURIComponent(value)}`
                }).then(response => {
                    if(!response.ok) alert('发送失败');
                });
            });
        });

        window.addEventListener('wheel', e => {
            if (e.ctrlKey) e.preventDefault();
        }, { passive: false });

        // 折叠面板控制
        function toggleCollapse(groupNum) {
            const content = document.getElementById(`group${groupNum}`);
            const btn = content.previousElementSibling;
            content.style.display = content.style.display === "block" ? "none" : "block";
            btn.innerHTML = content.style.display === "block" 
                ? `▼ ${btn.innerText.substr(2)}` 
                : `▶ ${btn.innerText.substr(2)}`;
        }

        // 通信功能
        function sendCommand(cmd) {
            fetch('/setCmd', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `param=${encodeURIComponent(cmd)}`
                }).then(response => {
                    if(!response.ok) alert('设置失败');
                });

        }

        // 初始化默认展开第一个面板
        document.getElementById('group1').style.display = 'block';

        // 参数映射表
        const paramMap = {
            jump: {
                'edge-speed': 'u_jump_speed',
                'point-speed_ch1': 'p_jump_speed_ch1',
                'point-speed_ch2': 'p_jump_speed_ch2'
            },
            // 新增旋转速度映射
            basic: {
                'rotate-value': 'rotate'
            },
            u_balance: {
                'ka-value': 'ka',
                'kp-value': 'kp',
                'kv-value': 'kv',
                'ks-value': 'ks'
            },
            p_balance: {
                'xa-value': 'xa',
                'ya-value': 'ya',
                'xkp-value': 'xkp',
                'xkv-value': 'xkv',
                'xks-value': 'xks',
                'ykp-value': 'ykp',
                'ykv-value': 'ykv',
                'yks-value': 'yks',
                'zkp-value': 'zkp',
                'zkv-value': 'zkv',
                'zks-value': 'zks'
            },
            protected: {
                'low_power_value': 'low_power',
                'opt_value': 'opt'
            }
        };
        // 获取参数
        async function getParams(type) {
            try {
                const response = await fetch(`/get_params?type=${type}`);
                const params = await response.json();
                
                Object.entries(params).forEach(([paramKey, value]) => {
                    const localKey = Object.entries(paramMap[type])
                        .find(([k, v]) => v === paramKey)[0];
                        
                    const slider = document.querySelector(`#${localKey}`)
                        .previousElementSibling;
                    const numInput = document.getElementById(localKey);
                    
                    slider.value = value;
                    numInput.value = value;
                });

            } catch (error) {
                alert('参数获取失败')
            }
        }

        // 保存参数
        async function saveParams(type) {
            const params = {};
            const containers = document.querySelectorAll(`#group${getGroupNum(type)} .slider-container`);
            
            containers.forEach(container => {
                const label = container.querySelector('.slider-label').textContent.trim();
                const value = container.querySelector('.slider-input').value;
                const paramKey = paramMap[type][container.querySelector('.number-input').id];
                
                params[paramKey] = parseFloat(value);
            });

            try {
                
                await fetch('/save_params', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({type, params})
                });
                alert('参数保存成功');
            } catch (error) {
                alert('参数保存失败')
            }
        }

        // 获取面板编号
        function getGroupNum(type) {
            return {
                jump: 3,
                u_balance: 4,
                p_balance: 5,
                protected: 6
            }[type];
        }
    </script>
</body>
</html>
    )rawliteral";