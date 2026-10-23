echo "🚀 Starting dashboard backend..."
cd /home/ndrts/workspaces/NDRTS-BFMC2024/src/dashboard/server

# Kill existing WebSocket server
PORT=8080
EXISTING_PID=$(lsof -ti :$PORT)
if [ -n "$EXISTING_PID" ]; then
    echo "🛑 Port $PORT is in use by PID $EXISTING_PID. Killing it..."
    kill -9 $EXISTING_PID
    sleep 1
else
    echo "✅ Port $PORT is free."
fi

# Run ros_server.js and show logs immediately
echo "🧠 Starting ros_server.js..."
node ros_server.js &
ROS_PID=$!
sleep 1

# Check if it crashed
if ! kill -0 $ROS_PID 2>/dev/null; then
    echo "❌ ros_server.js failed to start"
    exit 1
fi

echo "🌐 Starting static_server.js..."
node static_server.js &
STATIC_PID=$!

echo "ros_server.js started with PID $ROS_PID"
echo "static_server.js started with PID $STATIC_PID"

# Wait and keep logs visible
wait $ROS_PID $STATIC_PID
