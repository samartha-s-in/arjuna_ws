var ros = new ROSLIB.Ros({
  url: 'ws://172.17.0.1:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

var cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'
});

var Linear_Velocity_Input = document.getElementById("linear_velo_id");
var Linear_Velocity_Value_variable = document.getElementById("Linear_Velo");
var Angular_Velocity_Input = document.getElementById("angular_velo_id");
var Angular_Velocity_Value_variable = document.getElementById("Angular_velo");


Linear_Velocity_Input.addEventListener("input", function() 
{
Linear_Velocity_Value_variable.textContent = parseFloat(Linear_Velocity_Input.value * 0.1) ;
publishLTwist();  
});

Angular_Velocity_Input.addEventListener("input", function()
{
  Angular_Velocity_Value_variable.textContent = parseFloat(Angular_Velocity_Input.value);    
});


function publishLTwist() 
{
  var twist = new ROSLIB.Message({
    linear: {
      x: parseFloat(Linear_Velocity_Input.value) * 0.1 ,
      y: 0.0,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    }
  });
  cmdVel.publish(twist);
}


function move_forward()
{
  var cmdVel = new ROSLIB.Topic
  ({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message
  ({
  linear : 
  {
  x : parseFloat(Linear_Velocity_Input.value) * 0.1 ,
  y : 0.0,
  z : 0.0
  },
  angular : 
  {
  x : 0.0,
  y : 0.0,
  z : 0.0
  }
  });
  cmdVel.publish(twist);
}


function move_backward()
{
  var cmdVel = new ROSLIB.Topic
  ({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message
  ({
  linear : 
  {
  x : parseFloat(-Linear_Velocity_Input.value) * 0.1,
  y : 0.0,
  z : 0.0
  },
  angular : 
  {
  x : 0.0,
  y : 0.0,
  z : 0.0
  }
  });
  cmdVel.publish(twist);
}


function turn_left()
{
  var cmdVel = new ROSLIB.Topic
  ({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message
  ({
  linear : 
  {
  x : 0.0,
  y : 0.0,
  z : 0.0
  },
  angular : 
  {
  x : 0.0,
  y : 0.0,
  z : parseFloat(Angular_Velocity_Input.value),
  }
  });
  cmdVel.publish(twist);
}


function turn_right()
{
  var cmdVel = new ROSLIB.Topic
  ({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message
  ({
  linear : 
  {
  x : 0.0,
  y : 0.0,
  z : 0.0
  },
  angular : 
  {
  x : 0.0,
  y : 0.0,
  z : -parseFloat(Angular_Velocity_Input.value),
  } 
  });
  cmdVel.publish(twist);
}


function stop() {
  var twist = new ROSLIB.Message({
    linear: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    }
  });
  cmdVel.publish(twist);
}

var ros = new ROSLIB.Ros();
var imageTopic = '/frames';
var imageWidth = 480; // Modify according to your camera
var imageHeight = 350; // Modify according to your camera

ros.connect('ws://192.168.2.11:9090');

var imageListener = new ROSLIB.Topic
({
    ros: ros,
    name: imageTopic,
    messageType: 'sensor_msgs/CompressedImage'
});


var canvas = document.getElementById('imageCanvas');
var ctx = canvas.getContext('2d');
canvas.width = imageWidth;
canvas.height = imageHeight;

// Subscribe to image topic
imageListener.subscribe(function(message) 
{
    var imageData = 'data:image/jpeg;base64,' + message.data;
    var img = new Image();
    img.onload = function() {
        ctx.drawImage(img, 0, 0, imageWidth, imageHeight);
    };
    img.src = imageData;
});

// Unsubscribe from the image topic when the page is closed
window.addEventListener('beforeunload', function() {
    imageListener.unsubscribe();
});

var battery = new ROSLIB.Topic
({
    ros : ros,
    name : '/BATTERY_PERCENTAGE',
    messageType : 'std_msgs/Float64'
});
battery.subscribe(function(message) 
{
    document.getElementById("battery_data").innerHTML = message.data; 
});
