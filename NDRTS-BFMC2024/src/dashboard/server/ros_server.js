/* dashboard/server/ros_server.js  (no TypeScript) */
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const os = require('os');

//
// -- 1 · WebSocket -------------------------------------------------------------------
const wss = new WebSocket.Server({ port: 8080 });
console.log('🌍 WebSocket server on ws://localhost:8080 ');

function broadcast(obj) {
    const msg = JSON.stringify(obj);
    wss.clients.forEach(c => c.readyState === WebSocket.OPEN && c.send(msg));
}

//
// -- 2 · ROS 2 node -------------------------------------------------------------------
async function main() {
    await rclnodejs.init();
    const node = rclnodejs.createNode('websocket_bridge');

    // publishers we might use from the browser
    const stopPub = node.createPublisher('std_msgs/msg/Int32', '/stop_lanekeeping_cmd');
    const speedPub = node.createPublisher('std_msgs/msg/Int32', '/speed');

    // ↳ handle messages coming *from* the browser
    wss.on('connection', ws => {
        console.log('🔗 WS client connected');
        ws.send(JSON.stringify({ type: "test", data: "hello from ros_server" }));
        ws.on('message', d => {
            const msg = JSON.parse(d);
            if (msg.type === 'toggle_lane_detection')
                stopPub.publish({ data: msg.data });            // 0 / 1
            if (msg.type === 'adjust_speed')
                speedPub.publish({ data: msg.data });           // cm / s
        });
    });

    // -- helpers -----------------------------------------------------------------------
    const sysUsage = () => ({
        cpu: os.loadavg()[0] * 10,
        ram: (1 - os.freemem() / os.totalmem()) * 100,
    });

    setInterval(() => broadcast({ type: 'system', data: sysUsage() }), 1000);

    // -- subscriptions -----------------------------------------------------------------
    node.createSubscription('std_msgs/msg/Int32', '/direction_lane',
        m => broadcast({ type: 'direction_lane', data: m.data }));

    node.createSubscription('std_msgs/msg/Int32', '/speed',
        m => broadcast({ type: 'speed', data: m.data }));

    node.createSubscription('std_msgs/msg/String', '/detected_class',
        m => broadcast({ type: 'detected_class', data: m.data }));

    node.createSubscription('std_msgs/msg/Float32', '/battery',
        m => broadcast({ type: 'battery', data: Math.round(m.data * 100) }));

    node.createSubscription('std_msgs/msg/Int32', '/battery_instant',
        m => broadcast({ type: 'battery_instant', data: m.data }));

    node.createSubscription('std_msgs/msg/Float32', '/battery_power',
        m => broadcast({ type: 'battery_power', data: parseFloat(m.data.toFixed(2)) }));

    node.createSubscription('std_msgs/msg/Float32', '/battery_percent',
        m => broadcast({ type: 'battery_percent', data: parseFloat(m.data.toFixed(1)) }));

    node.createSubscription('std_msgs/msg/Int32', '/stop_lanekeeping',
        m => broadcast({ type: 'lane_detection_state', data: m.data === 0 }));

    node.createSubscription('std_msgs/msg/String', '/lane_detection_viz_base64',
        m => broadcast({ type: 'camera_feed', data: `data:image/jpeg;base64,${m.data}` }));

    node.createSubscription('std_msgs/msg/String', '/raw_image_base64',
        m => broadcast({ type: 'camera_feed', data: `data:image/jpeg;base64,${m.data}` }));

    node.createSubscription('nav_msgs/msg/Path', '/waypoints',
        m => {
            const pts = m.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
            const edges = m.header.frame_id.split('|')[1]?.split(';')
                .map(p => p.split(',').map(Number)) || [];
            broadcast({ type: 'waypoints', data: { points: pts, edges } });
        });

    node.createSubscription('geometry_msgs/msg/PointStamped', '/car_position',
        m => broadcast({
            type: 'car_position',
            data: { x: m.point.x, y: m.point.y }
        }));

    node.spin();
}

main().catch(err => console.error(err));
