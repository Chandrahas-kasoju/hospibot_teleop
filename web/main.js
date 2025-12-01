var ros = null;
var cmdVel = null;
var isConnected = false;

// UI Elements
const connectBtn = document.getElementById('connect-btn');
const statusText = document.getElementById('status-text');
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
    statusText.innerText = status;
    if (connected) {
        statusDot.classList.add('connected');
        connectBtn.innerText = "DISCONNECT";
        connectBtn.classList.add('disconnect');
        cameraFeed.style.display = 'block';
        cameraPlaceholder.style.display = 'none';
    } else {
        statusDot.classList.remove('connected');
        connectBtn.innerText = "CONNECT";
        connectBtn.classList.remove('disconnect');
        cameraFeed.style.display = 'none';
        cameraPlaceholder.style.display = 'flex';
    }
}

function connect() {
    statusText.innerText = "Connecting...";

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
    color: 'blue'
});

var linear_speed = 0.0;
var angular_speed = 0.0;
var max_linear = 0.5; // m/s
var max_angular = 1.0; // rad/s

joystickManager.on('move', function (evt, data) {
    if (data && data.vector) {
        // Nipple.js returns vector.y (up is positive) and vector.x (right is positive)
        // We map y to linear.x and x to angular.z (negative because left turn is positive z)

        // Calculate speeds based on distance/force
        var force = Math.min(data.force, 1.0); // Cap force at 1.0

        // Simple mapping
        // linear x: forward/backward
        linear_speed = Math.sin(data.angle.radian) * force * max_linear;

        // angular z: left/right
        // In ROS, positive Z is left turn. 
        // In nipplejs, 0 is right, PI/2 is up, PI is left.
        // cos(0) = 1 (right) -> should be -angular
        // cos(PI) = -1 (left) -> should be +angular
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
