        $(document).ready(function(){
            namespace = '/test'; // change to an empty string to use the global namespace

            // the socket.io documentation recommends sending an explicit package upon connection
            // this is specially important when using the global namespace
            var socket = io.connect('http://' + document.domain + ':' + location.port + namespace);
            socket.on('connect', function() {
                socket.emit('arduino_init', {data: 'I\'m connected!'});
            });

            // event handler for server sent data
            // the data is displayed in the "Received" section of the page
            socket.on('my response', function(msg) {
                $('#log').append('<br>Received #' + msg.count + ': ' + msg.data);
            });

            // handlers for the different forms in the page
            // these send data to the server in a variety of ways
            $('form#emit').submit(function(event) {
                socket.emit('arduino', {data: $('#emit_data').val()});
                return false;
            });
            
            $('button').on('click', function(){
            	var msg = $(this).data('msg');
            	if(msg == 'TEST'){
            	socket.emit('arduino', {data: 'TESTF '+$('#testinput').val()});
            	}
            	else if(msg == 'TEST2'){
            	socket.emit('arduino', {data: 'TESTB '+$('#testinput2').val()});
            	}
            	
            	else{
            	console.log(msg);
            	socket.emit('arduino', {data: msg});
            	}

            	
            });

document.onkeydown = checkKey;

function checkKey(e) {

    e = e || window.event;

	
    if (e.keyCode == '38') {
    	e.preventDefault();
        // up arrow
        socket.emit('arduino', {data: "FWD"});
        console.log('FWD');
    }
    else if (e.keyCode == '40') {
    	e.preventDefault();
        // down arrow
        socket.emit('arduino', {data: "BWD"});
        console.log('BWD');
    }
    else if (e.keyCode == '37') {
    	e.preventDefault();
       // left arrow
       socket.emit('arduino', {data: "TLEFT"});
       console.log('LEFT');
    }
    else if (e.keyCode == '39') {
    	e.preventDefault();
       // right arrow
       socket.emit('arduino', {data: "TRIGHT"});
       console.log('RIGHT');
    }
    else if (e.keyCode == '32') {
    	e.preventDefault();
       // right arrow
       socket.emit('arduino', {data: "STOP"});
       console.log('STOP');
    }
    else if (e.keyCode == '96') {
    	e.preventDefault();
       // right arrow
       socket.emit('arduino', {data: "KICK"});
       console.log('KICK');
    }
    else if (e.keyCode == '97') {
    	e.preventDefault();
       // right arrow
       socket.emit('arduino', {data: "KICKDOWN"});
       console.log('KICK');
    }
}



        });
