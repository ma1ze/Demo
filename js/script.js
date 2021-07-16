var num = 0;
openSocket = () => {

        socket = new WebSocket("ws://127.0.0.1:9997/");
        let msg = document.getElementById("msg");


        socket.addEventListener('open', (e) => {
            document.getElementById("status").innerHTML = "Opened";
        });

        socket.addEventListener('message', (e) => {
            // let ctx = msg.getContext("2d");
            // let image = new Image();
            // document.getElementById("PositionInfo").innerHTML = "start"
            if (typeof(e.data) == "string") {
                // num = num + 1

                document.getElementById("PositionInfo").innerHTML = e.data
                    // var strList = e.data.split("|");
                    // x = parseFloat(strList[0] * 100)
                    // y = parseFloat(strList[1] * 100)
                    // z = parseFloat(strList[2] * 100)

                // paint(x, y, z)



            } else {

                image.src = URL.createObjectURL(e.data);
                image.addEventListener("load", (e) => {
                    ctx.drawImage(image, 0, 0, msg.width, msg.height);
                });
            }

        });
    }
    // openSocket1 = () => {

//     socket1 = new WebSocket("ws://127.0.0.1:9996/");


// }


function send() {
    var str = document.getElementById("sendText").value;
    socket.send(str);
}

function cameraClick() {
    var str = "camera|on"
    socket.send(str);
}

function posShow() {
    var str = "posShow|on"
    socket.send(str);
}

function stop() {
    var str = "stop|on"
    socket.send(str);
}