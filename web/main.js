var ros = null;
var cmdVel = null;
var isConnected = false;

// UI Elements
const connectBtn = document.getElementById('connect-btn');
// const statusText = document.getElementById('status-text'); // Removed
const statusDot = document.getElementById('status-dot');
const cameraFeed = document.getElementById('camera-feed');
const cameraPlaceholder = document.getElementById('camera-placeholder');

connectBtn.addEventListener('click', function () {
    if (isConnected) {
        disconnect();
    } else {
        connect();
    }
});

function updateStatus(status, connected) {
    // statusText.innerText = status; // Element removed
    if (connected) {
        statusDot.classList.add('connected');
        connectBtn.innerText = "TERMINATE";
        connectBtn.classList.add('disconnect');
        cameraFeed.style.display = 'block';
        cameraPlaceholder.style.display = 'none';
    } else {
        statusDot.classList.remove('connected');
        connectBtn.innerText = "INITIATE";
        connectBtn.classList.remove('disconnect');
        cameraFeed.style.display = 'none';
        cameraPlaceholder.style.display = 'flex';
    }
}

function connect() {
    // statusText.innerText = "Connecting...";

    // Connect to ROS
    ros = new ROSLIB.Ros({
        url: 'ws://' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
        updateStatus("Connected", true);
        cameraFeed.src = "/video_feed";
        isConnected = true;
        console.log('Connected to websocket server.');

        // Setup Topic
        cmdVel = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    });

    ros.on('error', function (error) {
        updateStatus("Error", false);
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        updateStatus("Disconnected", false);
        cameraFeed.src = "";
        isConnected = false;
        console.log('Connection to websocket server closed.');
    });
}

function disconnect() {
    if (ros) {
        ros.close();
    }
}

// Joystick
var joystickManager = nipplejs.create({
    zone: document.getElementById('joystick-zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#00f3ff', // Neon Cyan
    size: 200
});

var linear_speed = 0.0;
var angular_speed = 0.0;
var max_linear = 0.5; // m/s
var max_angular = 1.0; // rad/s

joystickManager.on('move', function (evt, data) {
    if (data && data.vector && data.angle) {
        // Calculate speeds based on distance/force
        var force = Math.min(data.force, 1.0); // Cap force at 1.0

        // Simple mapping
        linear_speed = Math.sin(data.angle.radian) * force * max_linear;
        angular_speed = -Math.cos(data.angle.radian) * force * max_angular;

        publishCmdVel();
    }
});

joystickManager.on('end', function () {
    linear_speed = 0;
    angular_speed = 0;
    publishCmdVel();
});

function publishCmdVel() {
    if (!cmdVel) return; // Prevent publishing if not connected

    var twist = new ROSLIB.Message({
        linear: {
            x: linear_speed,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: angular_speed
        }
    });
    cmdVel.publish(twist);
}
