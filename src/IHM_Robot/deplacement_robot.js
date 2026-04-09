 //Lien du serveur 
 var ros = new ROSLIB.Ros({

  url : 'ws://192.168.137.82:9090'

});

//Declaration du Topic 1
var topic1 = new ROSLIB.Topic({

  ros : ros,

  name : '/joy_ihm',

  messageType : 'sensor_msgs/Joy'

});
//Envoi du Topic 1
function deplacement_avant() {

  var message = new ROSLIB.Message({

    axes: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 2
function deplacement_arriere() {

  var message = new ROSLIB.Message({

    axes: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 3
function deplacement_droite() {

  var message = new ROSLIB.Message({

    axes: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 4
function deplacement_gauche() {

  var message = new ROSLIB.Message({

    axes: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 5
function stop() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  for (let i = 0; i < 5; i++) {
    topic1.publish(message);
}

  console.log('Message publié sur le topic /message');

}


//-------------------Topics flippers (Valeurs a changer en fonction de l'affectation de la manette)---------------------------------------
//Envoi du Topic 6
function flipper_hautAVG() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 1, .0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 7
function flipper_basAVG() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, -1, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 8
function flipper_hautAVD() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);
  console.log('Message publié sur le topic /message');
}

//Envoi du Topic 9
function flipper_basAVD() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);
  console.log('Message publié sur le topic /message');
}

//Envoi du Topic 10
function flipper_hautAR() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);
  console.log('Message publié sur le topic /message');
}

//Envoi du Topic 11
function flipper_basAR() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);
  console.log('Message publié sur le topic /message');
}


//-----------------------Topics controle de la camera--------------------------------------------------------------------------------------------


//Envoi du Topic 14
function camera_haut() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}

//Envoi du Topic 15
function camera_bas() {

  var message = new ROSLIB.Message({

    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
  });

  topic1.publish(message);

  console.log('Message publié sur le topic /message');

}



//---------------------------Subscribers ROS-----------------------------------------------------------------------------------------------------------------------


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


//-----------------------Affectation des actions au clavier--------------------------------------------------------------------------------------------------------------------

let lastSentTime = 0;
const sendInterval = 200; // temps en millisecondes, ici 1 seconde


let modeMontee = true; // true pour montée, false pour descente

document.addEventListener('keydown', function(event) {
switch(event.key) {
    case 'z':
    case 'Z':
        const currentTime = new Date().getTime();
    if (currentTime - lastSentTime > sendInterval) {
        deplacement_avant();
        lastSentTime = currentTime;
    }
        break;
    case 's':
    case 'S':
        deplacement_arriere();
        break;
    case 'q':
    case 'Q':
        deplacement_gauche();
        break;
    case 'd':
    case 'D':
        deplacement_droite();
        break;
    case 'a':
    case 'A':
        flipper_hautAVG();
        break;
    case 'e':
    case 'E':
        flipper_basAVG();
        break;
    case 'p' :
    case 'P': 
        camera_bas();
        break;
    case 'm' :
    case 'M':
        camera_haut();
        break;
    case 'Shift':
        // Bascule entre mode montée et descente sans exécuter d'action directe
        modeMontee = !modeMontee;
        console.log('Mode', modeMontee ? 'Montée' : 'Descente');
        break;
    case 'f': // Exemple de touche pour actionner le flippeur arrière en fonction du mode
        if (modeMontee) flipper_hautAR(); else flipper_basAR();
        break;
    // Correction de la touche pour 'Control' et 'Space'
    case 'Control':
        flipper_hautAR();
        break;
    case ' ':
        stop();
        break;
    default:
        stop();
        break;
}
});

document.addEventListener('keyup', function(event) {
// Simplification du switch pour arrêter le robot après certaines actions
if (['z', 's', 'q', 'd', 'a', 'e', 'w', 'c', 'Control', ' ', 'p', 'm', 'Z', 'S', 'Q', 'D', 'A', 'E', 'P', 'M'].includes(event.key)) {
    stop();
    console.log('Le robot s\'arrête');
}
});
