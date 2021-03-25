/*Carmen Hurtado and Samuel Sze 03-05-2021 
EC444 Quest 3: Hurricane Box 
Code adapted from udp server example from node js */

//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');

var app = require('express')();

var http = require('http').Server(app);

// initialize data structures to store sensor data
var voltage =[], temp = [], xaccel = [], yaccel = [], zaccel = [], roll = [], pitch = [];

//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('Acknowledged');

//When opening the udp socket. 
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

//when receiving a packet from udp_client (esp32)
var remote_address; //store remote client's address to send additional message data
var remote_port; //store remote client's port to send additional message data
server.on('message', function (message, remote) {
        console.log(remote.address + ':' + remote.port +' - ' + message.toString());
        remote_address = remote.address;
        remote_port = remote.port;
        server.send(msg, remote.port, remote.address);
        push_sensor_data(message);
});

// i is x-axis
var i = 0; 
function push_sensor_data(data)
{
    data = data.toString();
    data = data.split(",");
    i = i + 1;
    voltage = data[0];
    xaccel.push({
		x: parseInt(i),
		y: parseFloat(data[1]/9.81)
	});
	yaccel.push({
		x: parseInt(i),
		y: parseFloat(data[2]/9.81)
	});
	zaccel.push({
		x: parseInt(i),
		y: parseFloat(data[3]/9.81)
    });
    roll.push({
		x: parseInt(i),
		y: parseFloat(data[4])
	});
	pitch.push({
		x: parseInt(i),
		y: parseFloat(data[5])
    });
	temp = data[6]
	if (xaccel.length > 15) //number of data poitns visible at any time
    {
        xaccel.shift();
        yaccel.shift();
        zaccel.shift();
        roll.shift();
        pitch.shift();

    }
}
//Bind server to PI local address on router with port at 3333
server.bind(3333,'192.168.1.110');

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// socket io script connect server to client (web browser) to display canvasjs charts.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// declare socket io for http
var io = require('socket.io')(http);

//interface with index.html directory to plot chart
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});

// xaccel chart format
var chartOptions1 = {
	title:{
		text: "Acceleration in X",
		fontSize: 18,
	},
	axisX: {
		Prefix: "Seconds",
		title: "Time(sec)",
		interval: 1
	},
	axisY: {
		maximum: 2,
		title: "Gs",
		includeZero: true,
		suffix: "g",
	},
	legend: {
		cursor: "pointer",
		verticalAlign: "bottom",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
	},
	toolTip: {
		shared: true
	},
	data: [
	{
		name: "X acceleration",
		type: "spline",
		color: "#00FF00",
		showInLegend: true,
		dataPoints: xaccel,
	}]

};

//chart for y accel format
var chartOptions2 = {
	title:{
		text: "Acceleration in Y",
		fontSize: 18,
	},
	axisX: {
		Prefix: "Seconds",
		title: "Time(sec)",
		interval: 1
	},
	axisY:[
	{
		title: "Gs",
		maximum: 2,
		includeZero: true,
		suffix: "g"
	}],
	legend: {
		cursor: "pointer",
		verticalAlign: "bottom",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
	},
	toolTip: {
		shared: true
	},
	data: [
		{
			name: "Y acceleration",
			type: "spline",
			markerType: "cross",
			color: "#FF0000",
			showInLegend: true,
			dataPoints: yaccel,
		}]

};

//chart for zaccel format
var chartOptions3 = {
	title:{
		text: "Acceleration in Z",
		fontSize: 18,
	},
	axisX: {
		Prefix: "Seconds",
		title: "Time(sec)",
		interval: 1
	},
	axisY:[
	{
		title: "Gs",
		maximum: 2,
		includeZero: true,
		suffix: "g"
	}],
	legend: {
		cursor: "pointer",
		verticalAlign: "bottom",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
	},
	toolTip: {
		shared: true
	},
	data: [
		{
			name: "Z acceleration",
			type: "spline",
			markerType: "cross",
			color: "#0000FF",
			showInLegend: true,
			dataPoints: zaccel,
		}]
};

// emit data every 1second
setInterval(function(){
	io.emit('dataMsg1', chartOptions1);
	io.emit('dataMsg2', chartOptions2);
	io.emit('dataMsg3',	chartOptions3);
	io.emit('dataMsg4',	voltage);
	io.emit('dataMsg5',	temp);
 }, 1000);

//socket io creation. 

var msg2;//second message to be sent to client to toggle LED 
io.on('connection', function(socket){
	//console.log('a user connected');
	io.emit('dataMsg1', chartOptions1);
	io.emit('dataMsg2', chartOptions2);
	io.emit('dataMsg3',	chartOptions3);
	io.emit('dataMsg4',	voltage);
	io.emit('dataMsg5',	temp);
    
    //get led state message to send back to udp_client
    socket.on('data', function (data) {
        msg2 = Buffer.from(data.buttondata);
        server.send(msg2, remote_port, remote_address);
        console.log(msg2);
    });
});



// // keep channel open on ipaddress:3334
http.listen(3334, function(){
});
