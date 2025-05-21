var twist;
var manager;
var ros;
var batterySub;
var batterySub1;
var cmdVelPub;
var twistIntervalID;
var robotHostname;

var maxLinearSpeed = 11.0;
var maxAngularSpeed = 11.0;

var namespaceSub;
var robotNamespace;

var publishersClient;
var topicsForTypeClient;

var popupOpen = false;
var robotState = "manual";
var ledPatternClient;
var stop_cmd = false;
var frontierPub;
var probeSub;
var probeThresh = 3;
var goalPub;
var goal;

var select;

var intervalFlag = false;
var initROSinterval;
var lastSelection;

var currentOptions = ["None"];

function initROS() {

    ros = new ROSLIB.Ros({
        url: "ws://localhost:9090"
    });

    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });

    goal = new ROSLIB.Message({
         header: {   
            frame_id: "map"
         },
         pose: {
            position: {
                  x: 0.0,
                  y: 0.0,
                  z: 0.0
            },
            orientation: {
                  x: 0.0,
                  y: 0.0,
                  z: 0.0,
                  w: 1.0
            }
         }
      });

    cmdVelPub = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist',
        queue_size: 10
    });
    cmdVelPub.advertise();

    systemRebootPub = new ROSLIB.Topic({
        ros: ros,
        name: 'system/reboot',
        messageType: 'std_msgs/Empty'
    });
    systemRebootPub.advertise();

    systemShutdownPub = new ROSLIB.Topic({
        ros: ros,
        name: 'system/shutdown',
        messageType: 'std_msgs/Empty'
    });
    systemShutdownPub.advertise();

    // batterySub1 = new ROSLIB.Topic({
    //     ros: ros,
    //     name: 'firmware/battery_averaged_cyclone',
    //     messageType: 'std_msgs/Float32',
    //     queue_length: 1
    // });
    // batterySub1.subscribe(batteryCallback);
    Sub1 = new ROSLIB.Topic({
        ros: ros,
        name: 'firmware/battery_averaged_cyclone',
        messageType: 'std_msgs/Float32',
        queue_length: 1
    });
    Sub1.subscribe(batteryCallback);
    Sub2 = new ROSLIB.Topic({
        ros: ros,
        name: 'firmware/battery_averaged',
        messageType: 'std_msgs/Float32',
        queue_length: 1
    });
    Sub2.subscribe(batteryCallback);

    namespaceSub = new ROSLIB.Topic({
        ros: ros,
        name: 'robot_namespace',
        messageType: 'std_msgs/String',
        queue_length: 1
    });
    namespaceSub.subscribe(namespaceCallback);

    publishersClient = new ROSLIB.Service({
        ros : ros,
        name : '/rosapi/publishers',
        serviceType : '/rosapi/Publishers'
    });
    
    topicsForTypeClient = new ROSLIB.Service({
        ros : ros,
        name : '/rosapi/topics_for_type',
        serviceType : '/rosapi/TopicsForType'
    });

    ledPatternClient = new ROSLIB.Service({
        ros : ros,
        name : 'change_led_pattern',
        serviceType : 'interfaces/srv/SetLedPattern'
    });

    frontierPub = new ROSLIB.Topic({
        ros: ros,
        name: 'trigger_exploration',
        messageType: 'std_msgs/Bool'
    });
    frontierPub.advertise();

    goalPub = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
    });
    goalPub.advertise();

    probeSub = new ROSLIB.Topic({
      ros: ros,
      name: '/filtered_probes',
      messageType: 'interfaces/msg/ProbeData',
      queue_length: 1
   });
   probeSub.subscribe(probeCallBack);

   poseSub = new ROSLIB.Topic({
      ros: ros,
      name: 'zed/zed_node/pose',
      messageType: 'geometry_msgs/PoseStamped',
      queue_length: 1
   });
   poseSub.subscribe(poseCallback);
    

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        getVideoTopics();
        
        if(intervalFlag) {
            clearInterval(initROSinterval)
            intervalFlag = false;
            retrieveVideoSrc();
        }

        if(typeof lastSelection == 'undefined') {
            const timeout = setTimeout(defaultVideoSrc, 2000);
        }
    });
    
    ros.on('error', function(error) {
        console.error('Error connecting to websocket server: ', error);
    });
    
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        if(intervalFlag == false) {
            initROSinterval = setInterval(initROS, 5000);
            intervalFlag = true;
            if(select.selectedIndex != -1) {
                lastSelection = select.options[select.selectedIndex].text;
            }
            select.innerHTML = '';
            currentOptions = ["None"];
        }
    });
}

function setProbeThresh(){
    var probeThresh = prompt('Enter probe threshold:', '3');
    if (!probeThresh) {
        return;
    }
    probeThresh = parseInt(probeThresh);
    if (isNaN(probeThresh) || probeThresh < 0) {
        alert('Invalid probe threshold. Please enter a positive integer.');
        return;
    }
}

function probeCallBack(message) {
    var probeCount = message.probe_count;
    document.getElementById('pinsID').innerHTML = probeCount;
    console.log("Probe count: ", probeCount);
   if (probeCount >= probeThresh && robotState == "frontier") {
      // return home
      goHome();
      confirm("Probe count is " + probeCount + ". Returning home.");
   }
}

function batteryCallback(message) {
    // console.log("Battery voltage: ", message.data);
    document.getElementById('batteryID').innerHTML = message.data.toPrecision(4) + 'V';
}

function poseCallback(message) {
    var pose = message.pose;
    var position = pose.position;
    document.getElementById('position-x').innerHTML = position.x.toPrecision(4) + 'm';
    document.getElementById('position-y').innerHTML = position.y.toPrecision(4) + 'm';
    document.getElementById('position-z').innerHTML = position.z.toPrecision(4) + 'm';
}


function namespaceCallback(message) {
    robotNamespace = message.data;
    if (typeof lastSelection == 'undefined') {
        video.src = "http://" + robotHostname + ":8080/stream?topic=" + robotNamespace + "camera/image_color&type=ros_compressed";
        const timeout = setTimeout(function () {selectCorrectOption(robotNamespace + "camera/image_color");}, 3000);
    } else {
        video.src = "http://" + robotHostname + ":8080/stream?topic=" + lastSelection + "&type=ros_compressed";
    }
}



function setGPIO() {
   var mode = prompt('Enter LED mode ("auto", "manual", or "off"):', 'manual');
   if (!mode) {
      return;
   }
    var req = new ROSLIB.ServiceRequest({ mode: mode });
    ledPatternClient.callService(req, function (resp) {
      if (!resp.success) {
         alert('ERROR: ' + resp.message);
      }else {
        document.getElementById('mode').innerHTML = mode;
      }
    });
}


function triggerEStop() {
   frontierPub.publish(new ROSLIB.Message({data: false}));

   var req = new ROSLIB.ServiceRequest({ mode: "off" });
   ledPatternClient.callService(req, function (resp) {
      if (resp.success) {
        alert('Robot stopped. Refresh the page to reconnect.');
        document.getElementById('mode').innerHTML = "off";
      } else {
         alert('ERROR: ' + resp.message);
      }
   });
   robotState = "e-stop";
   confirm("E-stop triggered. Please refresh the page to reconnect.");
}


function setFrontier() {
   if (robotState == "e-stop") {
      alert("Robot is in e-stop mode. Please refresh the page to reconnect.");
      return;
   }
    var req = new ROSLIB.ServiceRequest({ mode: "autonomous" });
    ledPatternClient.callService(req, function (resp) {
        if (!resp.success) {
            console.error("Error setting frontier mode:", resp.message);
            alert('ERROR: ' + resp.message);
        }else
            document.getElementById('mode').innerHTML = "autonomous";
    });
    console.log("beginning frontier exploration");
    robotState = "frontier-soon";
    const delay = robotState == "manual" ? 5000 : 0;
    
    setTimeout(() => {
        if (robotState == "frontier-soon") {
            robotState = "frontier";
            frontierPub.publish(new ROSLIB.Message({data: true}));
        }
    }, delay);
}


function goHome() {
    if (robotState == "e-stop") {
        alert("Robot is in e-stop mode. Please refresh the page to reconnect.");
        return;
    }
    var req = new ROSLIB.ServiceRequest({ mode: "autonomous" });

    ledPatternClient.callService(req, function (resp) {
        if (!resp.success) {
            console.error("Error setting frontier mode:", resp.message);
            alert('ERROR: ' + resp.message);
        }else
            document.getElementById('mode').innerHTML = "autonomous";
    });
    frontierPub.publish(new ROSLIB.Message({data: false}));
    console.log("Going home");
    robotState = "target-home-soon";
    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;

    const delay = robotState == "manual" ? 5000 : 0;
    
    setTimeout(() => {
        if (robotState == "target-home-soon") {
            robotState = "target-home";
            goalPub.publish(goal);
        }
    }, delay);
}


function goTarget() {
   if (robotState == "e-stop") {
      alert("Robot is in e-stop mode. Please refresh the page to reconnect.");
      return;
   }
   var res = prompt('Enter target coordinates (x,y):', '0,0');
   var x, y;
   try{
      var coords = res.split(',');
      x = parseFloat(coords[0]);
      y = parseFloat(coords[1]);
      if (isNaN(x) || isNaN(y)) {
         throw new Error('Invalid coordinates');
      }
   }catch (e) {
      console.error('Invalid input:', e);
      return;
   }
   
    var req = new ROSLIB.ServiceRequest({ mode: "autonomous" });
    ledPatternClient.callService(req, function (resp) {
        if (!resp.success) {
            console.error("Error setting frontier mode:", resp.message);
            alert('ERROR: ' + resp.message);
        }else
            document.getElementById('mode').innerHTML = "autonomous";
    });
    console.log("Going to target");
    robotState = "target-soon";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    const delay = robotState == "manual" ? 5000 : 0;
    setTimeout(() => {
    if (robotState == "target-soon") {
        robotState = "target";
        goalPub.publish(goal);
    }
    }, delay);
}


function publishTwist() {
    if ((robotState == "manual" && (twist.linear.x != 0 || twist.angular.z != 0))) {
        cmdVelPub.publish(twist);
        console.log("Twist: ", twist.linear.x, twist.angular.z);
    }
    if (stop_cmd) {
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmdVelPub.publish(twist);
        stop_cmd = false;
    }
}

function triggerManual() {
   if (robotState != "manual" && robotState != "manual-soon" && robotState != "e-stop") {
      if (popupOpen) {
         return;
      }
      popupOpen = true;
      if (confirm("Switch to manual mode?\n5 seconds delay")) {
        robotState = "manual-soon";
        frontierPub.publish(new ROSLIB.Message({data: false}));
        var req = new ROSLIB.ServiceRequest({ mode: "manual" });
        ledPatternClient.callService(req, function (resp) {
            if (!resp.success) {
                console.error("Error setting frontier mode:", resp.message);
                alert('ERROR: ' + resp.message);
            }else
            document.getElementById('mode').innerHTML = "manual";
        });

        const countdownOverlay = document.getElementById('countdown-overlay');
        countdownOverlay.textContent = '5';
        countdownOverlay.classList.remove('countdown-hidden');
        
        // Start the countdown from 5
        let count = 5;
        const countdownInterval = setInterval(() => {
            count--;
            if (count > 0) {
                countdownOverlay.textContent = count.toString();
            } else {
                // Clear the interval and hide the overlay when done
                clearInterval(countdownInterval);
                countdownOverlay.classList.add('countdown-hidden');
                if (robotState == "manual-soon") {
                    robotState = "manual";
                }
            }
        }, 1000);
        
      }
      popupOpen = false;
   }
}


function createJoystick() {

    joystickContainer = document.getElementById('joystick');

    manager = nipplejs.create({
        zone: joystickContainer,
        position: { right: 0 + 'px', bottom: 0 + 'px' },
        mode: 'static',
        size: 150,
        color: '#ffffff',
        restJoystick: true
    });

    manager.on('move', function (evt, nipple) {
        triggerManual();
        var lin = Math.sin(nipple.angle.radian) * nipple.distance * 0.0015;
        var ang = -Math.cos(nipple.angle.radian) * nipple.distance * 0.005;

        twist.linear.x = lin * maxLinearSpeed;
        twist.angular.z = ang * maxAngularSpeed;
    });

    manager.on('end', function () {
        stop_cmd = true;
        console.log("Joystick released");
    });
}


function initTeleopKeyboard() {
    const left_keys = ["ArrowLeft", "a", "A"];
    const right_keys = ["ArrowRight", "d", "D"];
    const up_keys = ["ArrowUp", "w", "W"];
    const down_keys = ["ArrowDown", "s", "S"];

    var body = document.getElementsByTagName('body')[0];
    body.addEventListener('keydown', function (e) {
        if (robotState !== "manual" && robotState !== "manual-soon") 
            triggerManual();
        else if (left_keys.includes(e.key)) 
            twist.angular.z = maxAngularSpeed;
        else if (right_keys.includes(e.key)) 
            twist.angular.z = -maxAngularSpeed;
        else if (up_keys.includes(e.key)) 
            twist.linear.x = maxLinearSpeed;
        else if (down_keys.includes(e.key)) 
            twist.linear.x = -maxLinearSpeed;
        
    });
    body.addEventListener('keyup', function (e) {
        if (left_keys.includes(e.key) || right_keys.includes(e.key))
            twist.angular.z = 0;
        else if (up_keys.includes(e.key) || down_keys.includes(e.key))
            twist.linear.x = 0;
        if(twist.linear.x == 0 && twist.angular.z == 0) {
            stop_cmd = true;
        }
    });
}


function systemReboot() {
    systemRebootPub.publish()
}

function turnOff() {
    systemShutdownPub.publish()
}

window.onblur = function () {
    twist.linear.x = 0;
    twist.angular.z = 0;
    publishTwist();
}

function shutdown() {
    clearInterval(twistIntervalID);
    cmdVelPub.unadvertise();
    systemRebootPub.unadvertise();
    systemShutdownPub.unadvertise();
    Sub1.unsubscribe();
    Sub2.unsubscribe();
    // batterySub.unsubscribe();
    ros.close();
}

function defaultVideoSrc() {
    namespaceSub.unsubscribe();
    
    if(typeof robotNamespace == 'undefined') {
        console.log("Unable to get the robot namespace. Assuming it's '/'.");
        video.src = "http://" + robotHostname + ":8080/stream?topic=/camera/image_color&type=ros_compressed";
        const timeout = setTimeout(function () {selectCorrectOption("/camera/image_color"); }, 3000);
    }
}

function checkPublishers(topicName) {
    var request = new ROSLIB.ServiceRequest({topic : topicName});

    publishersClient.callService(request, function(result) {
	    var publishers = result.publishers;

        if(publishers.length != 0 && topicName.endsWith("/compressed")) {
            var opt = document.createElement('option'); 
            var name = topicName.slice(0,-11);
            opt.innerHTML = name;
            if(!currentOptions.includes(name)) {
                select.appendChild(opt);
                currentOptions.push(name);
                if (name == lastSelection)
                    select.selectedIndex = currentOptions.length -1;
            }
        }
    });
}

function getVideoTopics() {
    var request = new ROSLIB.ServiceRequest({type : "sensor_msgs/msg/CompressedImage"});
    var empty_opt = document.createElement('option');
    empty_opt.innerHTML = "None";
    select.appendChild(empty_opt);

    topicsForTypeClient.callService(request, function(result) {
	    var topics = result.topics;

	    for(var i = 0; i < topics.length; i++) {
	        checkPublishers(topics[i]);
	    }
    });
}

function changeVideoSrc() {
    var selected = select.options[select.selectedIndex].text;
    if (selected == "None")
        video.src = "";
    else
        video.src = "http://" + robotHostname + ":8080/stream?topic=" + selected + "&type=ros_compressed";
}

function selectCorrectOption(name) {
    for(var i = 0; i < select.options.length; i++) {
        if(select.options[i].text == name) {
            select.selectedIndex = i;
            break;
        }
    }
}

function imgWidth() {
    var element = document.getElementById("video");
    element.classList.toggle("center-fit-full")
}

function retrieveVideoSrc() {
    if (lastSelection != "None") {
        video.src = ""
        video.src = "http://" + robotHostname + ":8080/stream?topic=" + lastSelection + "&type=ros_compressed";
    } else {
        select.selectedIndex = 0;
    }
}

window.onload = function () {

    robotHostname = "192.168.1.18";

    video = document.getElementById('video');
    select = document.getElementById('camera-select');

    initROS();
    initTeleopKeyboard();
    createJoystick();

    console.log("ROS connection: " + ros, ros.isConnected);

    twistIntervalID = setInterval(() => publishTwist(), 100); // 10 hz

    window.addEventListener("beforeunload", () => shutdown());
}


