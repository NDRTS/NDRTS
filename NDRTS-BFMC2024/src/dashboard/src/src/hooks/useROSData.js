import { useState, useEffect } from 'react';

export default function useROSData() {
    const [data, setData] = useState({
        directionLane: 0,
        speed: 0,
        camera_feed: null,
        detected_class: null,
        cpu: 0,
        ram: 0,
        battery: 0,
        laneDetectionOn: true,
        waypoints: [],
        car_position: null,
    });

    const [ws, setWs] = useState(null);

    useEffect(() => {
        const socket = new WebSocket("ws://localhost:8080");

        socket.onopen = () => {
            console.log("✅ WebSocket connected!");
        };

        setWs(socket);

        socket.onmessage = (event) => {
            const message = JSON.parse(event.data);
            // console.log("📩 WebSocket received:", message);

            setData((prev) => {
                if (message.type === "system") {
                    return {
                        ...prev,
                        cpu: message.data.cpu,
                        ram: message.data.ram,
                    };
                }
                else if (message.type === "waypoints") {
                    return { ...prev, waypoints: message.data };
                }
                else if (message.type === "car_position") {
                    return { ...prev, car_position: message.data };
                }
                else if (message.type === "detected_class") {
                    setTimeout(() => {
                        setData((prevData) => ({ ...prevData, detected_class: null }));
                        console.log("🕒 Cleared detected_class after 5s");
                    }, 10000); // 10 seconds
                    return {
                        ...prev,
                        detected_class: message.data,
                    };
                } else if (message.type === "battery") {
                    return { ...prev, battery: message.data };
                } else if (message.type === "lane_detection_state") {
                    return { ...prev, laneDetectionOn: message.data };
                }
                return {
                    ...prev,
                    [message.type]: message.data,
                };
            });
        };

        socket.onerror = (error) => {
            console.error("❌ WebSocket Error:", error);
        };

        socket.onclose = () => {
            console.warn("⚠️ WebSocket Disconnected!");
        };

        return () => socket.close();
    }, []);

    const sendToggleLaneDetection = () => {
        if (ws?.readyState === WebSocket.OPEN) {
            console.log("🔄 data laneDetectionOn", data.laneDetectionOn);
            const newState = data.laneDetectionOn ? 1 : 0;
            ws.send(JSON.stringify({ type: "toggle_lane_detection", data: newState }));
        }
    };


    const sendAdjustSpeed = (newSpeed) => {
        if (ws?.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: "adjust_speed", data: newSpeed }));
        }
    };

    return { data, sendToggleLaneDetection, sendAdjustSpeed };
}
