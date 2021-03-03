// https://socket.io/get-started/chat/
// source:
var app = require('express')();
var http = require('http').Server(app);
var fs = require('fs')

var voltage =[], temp = [], ultra = [], infrared = [];
// need to populate chart options
// title, ways to sort csv, axis labels, legends, type of chart (spline)
var chartOptions = {
	animationEnabled: true,
	title:{
		text: "Real-time"
	},
	axisX: {
		Prefix: "seconds",
		interval: 1
	},
	axisY: {
		title: "y-axis placeholder",
		Prefix: "C"
	},
	legend: {
		cursor: "pointer",
		verticalAlign: "top",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
		itemclick: toogleDataSeries
	},
	toolTip: {
		shared: true
	},
	data: [{
		name: "voltage",
		type: "spline",
		yValueFormatString: "####mV",
		showInLegend: true,
		dataPoints: voltage
	},
	{
		name: "temperature",
		type: "spline",
		yValueFormatString: "####C",
		showInLegend: true,
		dataPoints: temp
	},
	{
		name: "ultrasonic distance",
		type: "spline",
		yValueFormatString: "####cm",
		showInLegend: true,
		dataPoints: ultra
	},
	{
		name: "infrared distance",
		type: "spline",
		yValueFormatString: "####cm",
		showInLegend: true,
		dataPoints: infrared
	}]

};

//using filestream to import stocks.csv
const filepath = './out.csv'
var csv_lines = fs.readFileSync(filepath, 'utf8').toString().split("\r");
var data = [];
for (i = 1; i < csv_lines.length; i++)
{
	data = csv_lines[i].split(",");
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
}
//https://stackabuse.com/reading-and-writing-csv-files-with-node-js/
//https://dev.to/isalevine/parsing-csv-files-in-node-js-with-fs-createreadstream-and-csv-parser-koi
//create socket
var io = require('socket.io')(http);

//get index.html direction to plot chart
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});
//get stock.csv data 
app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/stocks.csv');
  });

//socket io to emit chartoptions onto local host 
io.on('connection', function(socket){
  io.emit('dataMsg', chartOptions);
});

// nodejs 
http.listen(3000, function(){
  console.log('listening on *:3000');
});

function toogleDataSeries(e){
	if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else{
		e.dataSeries.visible = true;
	}
	chart.render();
}