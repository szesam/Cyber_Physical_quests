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
stream.write("Voltage, Thermistor, Ultrasonic, IR\n")
//create a new serial port connection to my port 
const port = new SerialPort('COM3',{baudRate: 9600});

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
    // console.log(data[3]);
    i++
    voltage.push({
		x: parseInt(i),
		y: parseInt(data[0])
	});
	temp.push({
		x: parseInt(i),
		y: parseInt(data[1])
	});
	ultra.push({
		x: parseInt(i),
		y: parseInt(data[2])
	});
	infrared.push({
		x: parseInt(i),
		y: parseInt(data[3])
	});
    if (voltage.length > 30) //number of data poitns visible at any time
    {
        voltage.shift();
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
// chart format
  var chartOptions = {
	title:{
		text: "Sensor data"
	},
	axisX: {
		Prefix: "Seconds",
		interval: 1
	},
	axisY:[{
		title: "Voltage",
		lineColor: "#C24642",
		tickColor: "#C24642",
		labelFontColor: "#C24642",
		titleFontColor: "#C24642",
		includeZero: true,
		suffix: "mV"
	},
	{
		title: "Temperature",
		lineColor: "#369EAD",
		tickColor: "#369EAD",
		labelFontColor: "#369EAD",
		titleFontColor: "#369EAD",
		includeZero: true,
		suffix: "C"
	}],
	axisY2: {
		title: "Distance",
		lineColor: "#7F6084",
		tickColor: "#7F6084",
		labelFontColor: "#7F6084",
		titleFontColor: "#7F6084",
		includeZero: true,
		suffix: "cm"
	},
	legend: {
		cursor: "pointer",
		// itemclick: function (e) {
		// 	console.log("legend click: " + e.dataPointIndex);
		// 	console.log(e);
		// 	if (typeof (e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		// 		e.dataSeries.visible = false;
		// 	} else {
		// 		e.dataSeries.visible = true;
		// 	}
		// },
		verticalAlign: "bottom",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
		// itemclick: toogleDataSeries
	},
	toolTip: {
		shared: true
	},
	data: [{
		name: "voltage",
		type: "spline",
		color: "#C24642",
		yValueFormatString: "####mV",
		showInLegend: true,
		dataPoints: voltage,
		axisYIndex:0
	},
	{
		name: "temperature",
		type: "spline",
		color: "#369EAD",
		yValueFormatString: "####C",
		showInLegend: true,
		dataPoints: temp,
		axisYIndex:1
	},
	{
		name: "ultrasonic distance",
		type: "spline",
		color: "#7F6084",
		yValueFormatString: "####cm",
		showInLegend: true,
		dataPoints: ultra,
		axisYType:"secondary"
	},
	{
		name: "infrared distance",
		type: "spline",
		markerType: "cross",
		color: "#7F6084",
		yValueFormatString: "####cm",
		showInLegend: true,
		dataPoints: infrared,
		axisYType:"secondary"
	}]

};
// emit data every 1second
setInterval(function(){
	io.emit('dataMsg', chartOptions);
 }, 1000);

//socket io creation. 
io.on('connection', function(socket){
	console.log('a user connected');
    io.emit('dataMsg', chartOptions);
});

// keep channel open.
http.listen(3000, function(){
});


