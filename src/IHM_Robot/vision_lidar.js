//Lien du serveur 
var ros = new ROSLIB.Ros({
	url : 'ws://192.168.137.82:9090'
});


/* === TOPICS === */

//Declaration du Topic 1
var topic1 = new ROSLIB.Topic({
	ros : ros,
	name : '/deplacement_avant',
	messageType : 'std_msgs/Bool'
});
//Envoi du Topic 1
function deplacement_avant() {
	var message = new ROSLIB.Message({
		data : true
	});
	topic1.publish(message);
	console.log('Message publié sur le topic /message');
}

//Declaration du Topic 2
var topic2 = new ROSLIB.Topic({
	ros : ros,
	name : '/deplacement_arriere',
	messageType : 'std_msgs/Bool'
});
//Envoi du Topic 2
function deplacement_arriere() {
	var message = new ROSLIB.Message({
		data : true
	});

	topic2.publish(message);
	console.log('Message publié sur le topic /message');
}


//Declaration du Topic 3
var topic3 = new ROSLIB.Topic({
	ros : ros,
	name : '/deplacement_droite',
	messageType : 'std_msgs/Bool'
});
//Envoi du Topic 3
function deplacement_droite() {
	var message = new ROSLIB.Message({
		data : true
	});
	topic3.publish(message);
	console.log('Message publié sur le topic /message');
}

//Declaration du Topic 4
var topic4 = new ROSLIB.Topic({
	ros : ros,
	name : '/deplacement_gauche',
	messageType : 'std_msgs/Bool'
});
//Envoi du Topic 4
function deplacement_gauche() {
	var message = new ROSLIB.Message({
		data : true
	});
	topic4.publish(message);
	console.log('Message publié sur le topic /message');
}

//Declaration du Topic 5
var topic5 = new ROSLIB.Topic({
	ros : ros,
	name : '/stop',
	messageType : 'std_msgs/Bool'
});
//Envoi du Topic 5
function stop() {
	var message = new ROSLIB.Message({
		data : true
	});
	topic5.publish(message);
	console.log('Message publié sur le topic /message');
}


/* === SUBSCRIBERS === */

//Definition du Subscriber 1
var subscriber1 = new ROSLIB.Topic({
	ros : ros,
	name : '/vitesse1',
	messageType : 'std_msgs/Int32'
});
//Affiche sur la page web du message present dans le Subscriber 1
subscriber1.subscribe(function(message) {
	console.log('Message reçu :', message.data);
	document.getElementById('vitesse1').innerHTML = message.data;
});

//Definition du Subscriber 2
var subscriber2 = new ROSLIB.Topic({
	ros : ros,
	name : '/vitesse2',
	messageType : 'std_msgs/Int32'
});

//Affiche sur la page web du message present dans le Subscriber 1
subscriber1.subscribe(function(message) {
	console.log('Message reçu :', message.data);
	document.getElementById('vitesse2').innerHTML = message.data;
});

//Definition du Subscriber 3
var subscriber3 = new ROSLIB.Topic({
	ros : ros,
	name : '/inclinaison1',
	messageType : 'std_msgs/Int32'
});
//Affiche sur la page web du message present dans le Subscriber 1
subscriber1.subscribe(function(message) {
	console.log('Message reçu :', message.data);
	document.getElementById('inclinaison1').innerHTML = message.data;
});

//Definition du Subscriber 4
var subscriber4 = new ROSLIB.Topic({
	ros : ros,
	name : '/inclinaison2',
	messageType : 'std_msgs/Int32'
});
//Affiche sur la page web du message present dans le Subscriber 1
subscriber1.subscribe(function(message) {
	console.log('Message reçu :', message.data);
	document.getElementById('inclinaison2').innerHTML = message.data;
});

//Definition du Subscriber 5
var subscriber5 = new ROSLIB.Topic({
	ros : ros,
	name : '/boussole',
	messageType : 'std_msgs/Int32'
});

//Affiche sur la page web du message present dans le Subscriber 1
	subscriber1.subscribe(function(message) {
	console.log('Message reçu :', message.data);
	document.getElementById('boussole').innerHTML = message.data;
});