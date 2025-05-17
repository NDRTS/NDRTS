const rosnodejs = require('rosnodejs');
const WebSocket = require('ws');
const os = require('os');

const wss = new WebSocket.Server({ port: 8080 });

process.env.ROS_MASTER_URI = "http://localhost:11311";
process.env.ROS_IP = "192.168.0.103";

const getSystemUsage = () => ({
    cpu: os.loadavg()[0] * 10,
    ram: ((os.totalmem() - os.freemem()) / os.totalmem()) * 100,
});

let laneDetectionSub = null;
let rawImageSub = null;

async function startROSNode() {
    console.log(`🌍 Connecting to ROS Master at ${process.env.ROS_MASTER_URI}...`);

    await rosnodejs.initNode('/websocket_bridge');
    const nh = rosnodejs.nh;

    console.log("✅ Connected to ROS topics!");
    const std_msgs = rosnodejs.require('std_msgs').msg;
    const stopPub = nh.advertise('/stop_lanekeeping_cmd', std_msgs.Int32);
    const speedPub = nh.advertise('/speed', std_msgs.Float32);

    // Handle commands from frontend
    wss.on('connection', (ws) => {
        console.log("🔗 New WebSocket client connected!");
        ws.on('message', (message) => {
            const msg = JSON.parse(message);
            console.log(`📩 [WebSocket] Received: ${message}`);

            if (msg.type === 'toggle_lane_detection') {
                const value = msg.data; // msg.data is either 1 or 0
                console.log(`🛑 [WebSocket] Sending to /stop_lanekeeping: ${value}`);
                stopPub.publish({ data: value });
            }

            if (msg.type === 'adjust_speed') {
                const interval = setInterval(() => {
                    if (speedPub.getNumSubscribers() > 0) {
                        console.log(`[ROS] Publishing to /speed: ${msg.data}`);
                        speedPub.publish({ data: msg.data });
                        clearInterval(interval);
                    } else {
                        console.log(`[ROS] Waiting for /speed subscriber...`);
                    }
                }, 100);
            }

        });
    });


    // Subscribe to Direction Lane
    const directionLaneSub = nh.subscribe('/direction_lane', 'std_msgs/Float32', (msg) => {
        // console.log(`📡 [Direction Lane] Received: ${msg.data}`);
        sendData({ type: 'direction_lane', data: msg.data });
    });

    // Subscribe to Speed
    const speedSub = nh.subscribe('/speed', 'std_msgs/Float32', (msg) => {
        // console.log(`🚗 [Speed] Received: ${msg.data} km/h`);
        sendData({ type: 'speed', data: msg.data });
    });

    // Subscribe to Detected Traffic Sign
    const detectedClassSub = nh.subscribe('/detected_class', 'std_msgs/String', (msg) => {
        // console.log(`🚦 [Traffic Sign] Detected: ${msg.data}`);
        sendData({ type: 'detected_class', data: msg.data });
    });

    // Subscribe to Battery Voltage
    const batterySub = nh.subscribe('/battery', 'std_msgs/Float32', (msg) => {
        const percentage = Math.min(Math.max(msg.data * 100, 0), 100);
        sendData({ type: 'battery', data: Math.round(percentage) });
    });

    // ✅ Switch between base64 video feeds depending on stop_lanekeeping value
    nh.subscribe('/stop_lanekeeping', 'std_msgs/Int32', (msg) => {
        const value = msg.data;

        sendData({ type: 'lane_detection_state', data: value === 0 });

        // Unsubscribe current image stream
        if (laneDetectionSub) {
            laneDetectionSub.shutdown();
            laneDetectionSub = null;
        }
        if (rawImageSub) {
            rawImageSub.shutdown();
            rawImageSub = null;
        }

        if (value === 1) {
            console.log("🛑 LaneKeeping disabled. Subscribing to raw image...");
            rawImageSub = nh.subscribe('/raw_image_base64', 'std_msgs/String', (msg) => {
                const base64Image = `data:image/jpeg;base64,${msg.data}`;
                sendData({ type: 'camera_feed', data: base64Image });
            });
        } else {
            console.log("✅ LaneKeeping enabled. Subscribing to lane detection image...");
            laneDetectionSub = nh.subscribe('/lane_detection_viz_base64', 'std_msgs/String', (msg) => {
                const base64Image = `data:image/jpeg;base64,${msg.data}`;
                sendData({ type: 'camera_feed', data: base64Image });
            });
        }
    });

    nh.subscribe('/battery', 'std_msgs/Float32', (msg) => {
        const percent = Math.round(Math.min(Math.max(msg.data * 100, 0), 100));
        sendData({ type: 'battery', data: percent });
    });

    // Send system usage (CPU & RAM) every second
    setInterval(() => {
        const systemStats = getSystemUsage();
        sendData({ type: 'system', data: systemStats });
    }, 1000);

    function sendData(message) {
        wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify(message));
            }
        });
    }
}

startROSNode().catch(console.error);
console.log("🌍 WebSocket server running on ws://localhost:8080");