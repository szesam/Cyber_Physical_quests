//Create UDP datagram and server
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
var app = require('express')();
const bodyParser = require('body-parser');
app.use(bodyParser.urlencoded({ extended: true })); 
var http = require('http').Server(app);
// initialize data structures to store sensor data
var voltage =[], temp = [], xaccel = [], yaccel = [], zaccel = [], roll = [], pitch = [];
//create stream to write sensor data into csv file called out.csv in same directory.
var fs = require('fs')
var stream = fs.createWriteStream("out.csv");
stream.write("Voltage(mV), Xaccel, Yaccel, Zaccel, Roll, Pitch, Thermistor(C)\n")

//Create acknowledge packet to send back to udp_client upon message received
const msg = Buffer.from('Acknowledged');

// var msg2;
// document.getElementById("input1").onclick = function(){
//     msg2 = "button pressed";
//     console.log(msg2);
// }


//When opening the udp socket. 
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

//when receiving a packet from udp_client (esp32)
var remote_address;
var remote_port;
server.on('message', function (message, remote) {
        console.log(remote.address + ':' + remote.port +' - ' + message.toString());
        remote_address = remote.address;
        remote_port = remote.port;
        server.send(msg, remote.port, remote.address);
        push_sensor_data(message);
        stream.write(String(message));
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
		y: parseFloat(data[1])
	});
	yaccel.push({
		x: parseInt(i),
		y: parseFloat(data[2])
	});
	zaccel.push({
		x: parseInt(i),
		y: parseFloat(data[3])
    });
    roll.push({
		x: parseInt(i),
		y: parseFloat(data[4])
	});
	pitch.push({
		x: parseInt(i),
		y: parseFloat(data[5])
    });
    temp.push({
		x: parseInt(i),
		y: parseFloat(data[6])
	});
    if (temp.length > 15) //number of data poitns visible at any time
    {
        temp.shift();
        xaccel.shift();
        yaccel.shift();
        zaccel.shift();
        roll.shift();
        pitch.shift();

    }
}
//Bind server to PI local address on router with port at 3333
server.bind(3333,'192.168.1.116');

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// socket io script connect server to client (web browser) to display canvasjs charts.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// declare socket io for http
var io = require('socket.io')(http);

//interface with index.html directory to plot chart
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});


  

// first chart format
var chartOptions1 = {
	title:{
		text: "Vibration data"
	},
	axisX: {
		Prefix: "Seconds",
		title: "Time(sec)",
		interval: 1
	},
	axisY: {
		title: "Distance",
		lineColor: "#7F6084",
		tickColor: "#7F6084",
		labelFontColor: "#7F6084",
		titleFontColor: "#7F6084",
		includeZero: true,
		suffix: "m",
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
		color: "#7F6084",
		yValueFormatString: "0.##m",
		showInLegend: true,
		dataPoints: xaccel,
	},
	{
		name: "Y acceleration",
		type: "spline",
		markerType: "cross",
		color: "#ff1a1a",
		yValueFormatString: "0.##m",
		showInLegend: true,
		dataPoints: yaccel,
	}]

};

//second chart with temperature sensor
var chartOptions2 = {
	title:{
		text: "Temperature Sensor data"
	},
	axisX: {
		Prefix: "Seconds",
		title: "Time(sec)",
		interval: 1
	},
	axisY:[
	{
		title: "Temperature",
		lineColor: "#369EAD",
		tickColor: "#369EAD",
		labelFontColor: "#369EAD",
		titleFontColor: "#369EAD",
		includeZero: true,
		suffix: "C"
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
		name: "temperature",
		type: "spline",
		color: "#369EAD",
		yValueFormatString: "####C",
		showInLegend: true,
		dataPoints: temp,
		axisYIndex:1
	},
	]

};
// emit data every 1second
setInterval(function(){
	io.emit('dataMsg1', chartOptions1);
	io.emit('dataMsg2', chartOptions2);
	io.emit('dataMsg3',	voltage);
 }, 1000);

//socket io creation. 
//second message to be sent to client
var msg2;
io.on('connection', function(socket){
	//console.log('a user connected');
	io.emit('dataMsg1', chartOptions1);
	io.emit('dataMsg2', chartOptions2);
    io.emit('dataMsg3', voltage);
    
    //get led state message to send back to udp_client
    socket.on('data', function (data) {
        // msg2 = data.buttondata;
        msg2 = Buffer.from(data.buttondata);
        server.send(msg2, remote_port, remote_address);
        console.log(msg2);
    });
});



// // keep channel open on ipaddress:3334
http.listen(3334, function(){
});
// keep channel open.