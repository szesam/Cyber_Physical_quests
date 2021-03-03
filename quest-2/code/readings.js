//including serial port module 
const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
//create a new serial port connection to my port 
const port = new SerialPort('/dev/cu.SLAB_USBtoUART',{baudRate: 9600});
//create a parser to read in the data 
const parser = port.pipe(new Readline({ delimiter: '\n'}));

//for writing the csv file 
const createCsvWriter = require('csv-writer').createObjectCsvWriter;

//global variable to store serial input data
var all_data = []; 

//open port 
port.on('open', () => {
    console.log('serial port open');
});
//read each line of data from the open port
parser.on('data', data => {
    console.log(data);
    //push each line of data each time it reads 
    all_data.push(data);
});

//try and see if it gets all the data we need 
//console.log(all_data);

//function to get data into right format

 console.log("AFTER FORMAT");
 console.log(all_data);


const csvWriter = createCsvWriter({
    //specify path for file 
  path: 'out.csv',
  //create header fields
  header: [
    {id: 'voltage', title: 'Voltage'},
    {id: 'Thermistor', title: 'Thermistor'},
    {id: 'Ultrasonic', title: 'Ultrasonic'},
    {id: 'IR', title: 'IR'},
  ]
});

//need to change this read data from serial port and fill in values after 
//fromat all_data variable with this format, then just do data = all_data
const data = all_data;//[
//   {
//     name: 'John',
//     surname: 'Snow',
//     age: 26,
//     gender: 'M'
//   }, {
//     name: 'Clair',
//     surname: 'White',
//     age: 33,
//     gender: 'F',
//   }, {
//     name: 'Fancy',
//     surname: 'Brown',
//     age: 78,
//     gender: 'F'
//   }
// ];

csvWriter
  .writeRecords(data)
  .then(()=> console.log('The CSV file was written successfully'));
