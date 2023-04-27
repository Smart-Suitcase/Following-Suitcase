// Importing express module
const express = require("express")
const app = express()
  
    /**
    An example of using rosnodejs with turtlesim, incl. services,
    pub/sub, and actionlib. This example uses the on-demand generated
    messages.
 */

    'use strict';

    let rosnodejs = require('rosnodejs');
    rosnodejs.initNode('/my_node', {onTheFly: true}).then((rosNode) => {
    
        // get list of existing publishers, subscribers, and services
        rosNode._node._masterApi.getSystemState("/my_node").then((data) => {
        console.log("getSystemState, result", data, data.publishers[0]);
        });
    
        const std_msgs = rosnodejs.require('std_msgs').msg;
        const msg = new std_msgs.Float64();
    
        rosNode.subscribe(
        '/x_pixel',
        'std_msgs/Int16',
        (data) => {
            console.log('my data', data.data);
        },
        {queueSize: 1,
            throttleMs: 1000}
    )
    let pub = rosNode.advertise('/my_topic','std_msgs/String', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    });
    
    
    // let msgStart = 'my message ';
    // let iter = 0;
    // setInterval(() => {
    //     msg.data = msgStart + iter
    //     pub.publish(msg);
    //     console.log('ca publish la')
    //     ++iter;
    //     if (iter > 200) {
    //         iter = 0;
    //     }
    // }, 5);
    // Handling GET / request
    app.get("/hello", (req, res, next) => {
        let msgStart = 'my message ';
        msg.data = msgStart
        pub.publish(msg);
        res.send("This is the express server")
    })
});

  

  
// Server setup
app.listen(3000, () => {
    console.log("Server is Running")
})