/*Carmen Hurtado and Samuel Sze 03-05-2021 
EC444 Quest 2: Tactile Internet
*/

//Declare fs, http, express and socket io for graphing
var app = require('express')();
var http = require('http').Server(app);
var fs = require('fs')

function toogleDataSeries(e){
    if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
        e.dataSeries.visible = false;
    } else{
        e.dataSeries.visible = true;
    }
    chart.render();
}

// initialize data structures to store sensor data
var voltage =[], temp = [], ultra = [], infrared = [];
//including serial port module 
const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
//create stream to write sensor data into csv file called out.csv in same directory.
var stream = fs.createWriteStream("out.csv");
stream.write("Voltage(mV), Thermistor(C), Ultrasonic(m), IR(m)\n")
//create a new serial port connection to my port 
const port = new SerialPort('COM5',{baudRate: 9600});

//create a parser to read in the data 
const parser = port.pipe(new Readline({ delimiter: '\n'}));
parser.on('data', data => {
    //format data input to be displayed on canvasjs
	push_sensor_data(data);
	// save data to csv.
	stream.write(String(data));
	//display data on console.
	console.log(data);
});
// i is x-axis.
var i = 0;
// push_sensor_data is function in parse data. 
function push_sensor_data(data)
{
    data = data.split(",");
    i++
	voltage = data[0];
	temp.push({
		x: parseInt(i),
		y: parseFloat(data[1])
	});
	ultra.push({
		x: parseInt(i),
		y: parseFloat(data[2])
	});
	infrared.push({
		x: parseInt(i),
		y: parseFloat(data[3])
	});
    if (temp.length > 15) //number of data poitns visible at any time
    {
        temp.shift();
        ultra.shift();
        infrared.shift();
    }
}

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
		text: "Distance Sensor data"
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
		name: "ultrasonic distance",
		type: "spline",
		color: "#7F6084",
		yValueFormatString: "0.##m",
		showInLegend: true,
		dataPoints: ultra,
	},
	{
		name: "infrared distance",
		type: "spline",
		markerType: "cross",
		color: "#ff1a1a",
		yValueFormatString: "0.##m",
		showInLegend: true,
		dataPoints: infrared,
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
io.on('connection', function(socket){
	console.log('a user connected');
	io.emit('dataMsg1', chartOptions1);
	io.emit('dataMsg2', chartOptions2);
	io.emit('dataMsg3', voltage);
});

// keep channel open.
http.listen(3000, function(){
});


