// Connexion au serveur ROS
var ros = new ROSLIB.Ros({
    url: 'ws://192.168.137.82:9090'
});

// Écouteurs d'événements pour la connexion ROS
ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Création du Topic pour les sliders et les boutons
var topic = new ROSLIB.Topic({
    ros: ros,
    name: '/choix_pos',
    messageType: 'std_msgs/Int16MultiArray'
});

// État global pour les valeurs des sliders et du bouton
var sliderValues = {
    dx: 0,
    dy: 0,
    dz: 0,
    rx: 0,
    ry: 0,
    rz: 0,
    bouton: 0  // Ajout de la propriété bouton
};

// Fonction pour mettre à jour les valeurs des sliders et envoyer le message
function updateSlidersAndSend() {
    var message = new ROSLIB.Message({
        layout: {
            dim: [],
            data_offset: 0,
        },
        data: [
            sliderValues.dx,
            sliderValues.dy,
            sliderValues.dz,
            sliderValues.rx,
            sliderValues.ry,
            sliderValues.rz,
            sliderValues.bouton  // Inclure la valeur du bouton dans le message
        ]
    });

    topic.publish(message);
    console.log('Valeur des sliders envoyée: ', sliderValues);
}

// Fonctions pour les actions des sliders
function deplacement_x() {
    sliderValues.dx = parseFloat(document.getElementById('sliderx').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

function deplacement_y() {
    sliderValues.dy = parseFloat(document.getElementById('slidery').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

function deplacement_z() {
    sliderValues.dz = parseFloat(document.getElementById('sliderz').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

function rotation_x() {
    sliderValues.rx = parseFloat(document.getElementById('sliderrx').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

function rotation_y() {
    sliderValues.ry = parseFloat(document.getElementById('sliderry').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

function rotation_z() {
    sliderValues.rz = parseFloat(document.getElementById('sliderrz').value);
    sliderValues.bouton = 0;
    updateSlidersAndSend();
}

// Fonction pour définir la valeur du bouton et envoyer la mise à jour
function setButtonValueAndSend(buttonId) {
    sliderValues.bouton = buttonId;  // Mise à jour de la valeur du bouton
    updateSlidersAndSend();  // Utiliser la même fonction pour envoyer les données
}

// Vous pouvez définir des valeurs spécifiques si nécessaire pour chaque bouton
function rangement() {
    setButtonValueAndSend(1);
}

function deploiement() {
    setButtonValueAndSend(2);
}

function haut() {
    setButtonValueAndSend(3);
}
