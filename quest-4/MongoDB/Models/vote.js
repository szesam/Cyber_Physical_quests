
const mongoose = require('mongoose');
//create a schema object 
const Schema = mongoose.Schema;

//declare the schema structure specific for this type of data(vote)
const voteSchema = new Schema({
    vote: String,
    timestamp: String,
    fob_id: String,
}, { timestamps : true}); //timestamps created when object gets saved

//export this schema to be used in main js file
const Vote = mongoose.model('Vote', voteSchema);
module.exports = Vote;