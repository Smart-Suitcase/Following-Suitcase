// Importing express module
const express = require("express")
const app = express()
const rosnodejs = require('rosnodejs');


// Initialisation of the connexion with the ROSMaster
rosnodejs.initNode('/my_node', {onTheFly: true}).then((rosNode) => {

    // Get list of existing publishers, subscribers, and services
    rosNode._node._masterApi.getSystemState("/my_node").then((data) => {
    console.log("getSystemState, result", data, data.publishers[0]);
    });

    const std_msgs = rosnodejs.require('std_msgs').msg;
    const msg_string = new std_msgs.String();
    const msg_float = new std_msgs.Float64();
    const msg_uint = new std_msgs.UInt16();


    // Exemple of the subscribe to the x_pixel topic in continue
//     rosNode.subscribe(
//     '/x_pixel',
//     'std_msgs/Int16',
//     (data) => {
//         console.log('my data', data.data);
//     },
//     {queueSize: 1,
//         throttleMs: 1000}
// )

    // Initalisation of the publishers
    let mode = rosNode.advertise('/mode','std_msgs/String', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    }
    );


// Routes
app.get("/mode:mode_val", (req, res, next) => {
    let msg = req.params.mode_val;
    msg_string.data = (msg);
    mode.publish(msg_string);
    res.send(msg_string.data);
})



});

  

  
// Server setup
app.listen(3000, () => {
    console.log("Server is Running")
})