// Importing express module
const express = require("express")
const app = express()
const rosnodejs = require('rosnodejs');
// const swaggerUi = require('swagger-ui-express');
// const swaggerDocument = require('./swagger.yml');
// app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(swaggerDocument));

// Initialisation of the connexion with the ROSMaster
rosnodejs.initNode('/my_node', { onTheFly: true }).then((rosNode) => {

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
    let mode = rosNode.advertise('/mode', 'std_msgs/String', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    }
    );

    let left_speed = rosNode.advertise('/left_speed', 'std_msgs/Float64', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    }
    );

    let right_speed = rosNode.advertise('/left_speed', 'std_msgs/Float64', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    }
    );

    let left_direction = rosNode.advertise('/left_direction', 'std_msgs/UInt16', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    });

    let right_direction = rosNode.advertise('/right_direction', 'std_msgs/UInt16', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    });




    // Routes
    app.get("/mode/:mode_val", (req, res, next) => {
        let msg = req.params.mode_val;
        msg_string.data = (msg);
        mode.publish(msg_string);
        res.send(msg_string.data);
    })

    app.get('/left_speed/:speed', (req, res) => {
        const leftSpeed = parseInt(req.params.speed, 10); // Parse speed parameter from request
        if (isNaN(leftSpeed) || leftSpeed < 0) {
            res.status(400).send('Invalid speed parameter');
            return;
        }
        const message = new std_msgs.Float64({ data: leftSpeed });
        left_speed.publish(message);
        res.status(200).send(`Published left_speed: ${leftSpeed}`);
    });

    app.get('/right_speed/:speed', (req, res) => {
        const rightSpeed = parseInt(req.params.speed, 10); // Parse speed parameter from request
        if (isNaN(rightSpeed) || rightSpeed < 0) {
            res.status(400).send('Invalid speed parameter');
            return;
        }
        const message = new std_msgs.Float64({ data: rightSpeed });
        right_speed.publish(message);
        res.status(200).send(`Published right_speed: ${rightSpeed}`);
    });

    app.get('/left_direction/:direction', (req, res) => {
        const leftDirection = parseInt(req.params.direction, 10); // Parse direction parameter from request
        if (isNaN(leftDirection) || leftDirection < 0) {
            res.status(400).send('Invalid direction parameter');
            return;
        }
        const message = new std_msgs.UInt16({ data: leftDirection });
        left_direction.publish(message);
        res.status(200).send(`Published left_direction: ${leftDirection}`);
    });

    app.get('/right_direction/:direction', (req, res) => {
        const rightDirection = parseInt(req.params.direction, 10); // Parse direction parameter from request
        if (isNaN(rightDirection) || rightDirection < 0) {
            res.status(400).send('Invalid direction parameter');
            return;
        }
        const message = new std_msgs.UInt16({ data: rightDirection });
        right_direction.publish(message);
        res.status(200).send(`Published right_direction: ${rightDirection}`);
    });
});




// Server setup
app.listen(3000, () => {
    console.log("Server is Running")
})