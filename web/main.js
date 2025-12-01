var ros = null;
var cmdVel = null;
var isConnected = false;

document.getElementById('connect-btn').addEventListener('click', function () {
    if (isConnected) {
        disconnect();
    } else {
        connect();
    }
});

function connect() {
    document.getElementById("status").innerHTML = "Connecting...";

    // Connect to ROS
    ros = new ROSLIB.Ros({
        url: 'ws://' + window.location.hostname + ':9090'
    });

    ros.on('connection', function () {
        document.getElementById("status").innerHTML = "Connected";
        document.getElementById("status").style.color = "#00ff00";
        document.getElementById("connect-btn").innerText = "Disconnect";
        document.getElementById("camera-feed").src = "/video_feed";
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
        document.getElementById("status").innerHTML = "Error";
        document.getElementById("status").style.color = "#ff0000";
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        document.getElementById("status").innerHTML = "Disconnected";
        document.getElementById("status").style.color = "#ffff00";
        document.getElementById("connect-btn").innerText = "Connect";
        document.getElementById("camera-feed").src = "";
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
