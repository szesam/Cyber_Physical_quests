//Carmen Hurtado 04-03-2021

const mongoose = require('mongoose');
//create a schema object 
const Schema = mongoose.Schema;

//declare the schema structure specific for this type of data 
const smokeSchema = new Schema({
    time: String,
    id: String,
    smoke: String,
    temp: String
});

//export this schema to be used in main js file
const Smoke = mongoose.model('Smoke', smokeSchema);
module.exports = Smoke;