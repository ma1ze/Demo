    function sleep(ms) {
            return new Promise(resolve => setTimeout(resolve, ms));
        }

    var socket;
    var current=0;
    var total;
    var beforetime;

    function connect() {
        var host = "ws://" + $("serverIP").value + ":" + $("serverPort").value + "/";
        document.getElementById("btnConnect").value = "连接中";
        document.getElementById("btnConnect").disabled = true;
        socket = new WebSocket(host);
        try {

            socket.onopen = function (msg) {
                document.getElementById("btnConnect").value = "连接成功";
                document.getElementById("btnSend").disabled = "";
                //alert("连接成功！");
            };

            socket.onmessage = function (msg) {
                if (typeof msg.data == "string") {
                    displayContent(msg.data);
                    if(msg.data=="Receive:100%"){
                        current=0;
                        total=0;
                    }
                    else if(msg.data.substr(0,7)=="Receive"){
                        var str1=msg.data.split(':')[1];
                        var str2=str1.split('/')[0];
                        picsend(parseInt(str2))
                    }
                }
                else {
                    alert("非文本消息");
                }
            };

            socket.onerror = function (error) { alert("Error："+ error.name); };

            socket.onclose = function (msg) {
                document.getElementById("btnConnect").value = "连接";
                document.getElementById("btnConnect").disabled = "";
                document.getElementById("btnSend").disabled = true;//alert("连接关闭!");
                 };
        }
        catch (ex) {
            log(ex);
        }
    }

    async function send() {
        var str = document.getElementById("sendText").value;
        socket.send(str);
    }

    async function picsend(pos){
        beforetime=new Date().getTime();
        current=pos;
        socket.close();
        connect();
        while(document.getElementById("btnConnect").value != "连接成功") {await sleep(200);}
        var str = document.getElementById("sendText").value;
        socket.send(str.substring(pos));
    }

    window.onbeforeunload = function () {
        try {
            socket.close();
            socket = null;
        }
        catch (ex) {
        }
    };

    function $(id) { return document.getElementById(id); }

    Date.prototype.Format = function (fmt) { //author: meizz
        var o = {
            "M+": this.getMonth() + 1, //月份
            "d+": this.getDate(), //日
            "h+": this.getHours(), //小时
            "m+": this.getMinutes(), //分
            "s+": this.getSeconds(), //秒
            "q+": Math.floor((this.getMonth() + 3) / 3), //季度
            "S": this.getMilliseconds() //毫秒
        };
        if (/(y+)/.test(fmt)) fmt = fmt.replace(RegExp.$1, (this.getFullYear() + "").substr(4 - RegExp.$1.length));
        for (var k in o)
            if (new RegExp("(" + k + ")").test(fmt)) fmt = fmt.replace(RegExp.$1, (RegExp.$1.length == 1) ? (o[k]) : (("00" + o[k]).substr(("" + o[k]).length)));
        return fmt;
    }

    function displayContent(msg) {
        $("txtContent").value += "\r\n" +new Date().Format("yyyy/MM/dd hh:mm:ss")+ ":  " + msg;
        $("txtContent").scrollTop = $("txtContent").scrollHeight;
    }
    function onkey(event) { if (event.keyCode == 13) { send(); } }