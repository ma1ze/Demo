var container,
    camera,
    scene,
    renderer,
    axes;

// main canvas
// -----------------------------------------------

container = document.getElementById('container');

// renderer
renderer = new THREE.CanvasRenderer();
//renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setSize(640, 380);
container.appendChild(renderer.domElement);
renderer.setClearColor(0xFFFFFF, 1.0);
// scene
scene = new THREE.Scene();

// camera
camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 1, 10000);
camera.position.x = 100;
camera.position.y = 1000;
camera.position.z = 300;
camera.lookAt(scene.position);
scene.add(camera);

// controlls
controls = new THREE.OrbitControls(camera, renderer.domElement);

var geometry = new THREE.Geometry();
var material = new THREE.LineBasicMaterial({
    color: 0x00ffff
});
var color1 = new THREE.Color(0x444444),
    color2 = new THREE.Color(0xFF0000);


axes = new THREE.AxesHelper(200); //
scene.add(axes);
setInterval("paint(x,y,z)", "1000")

function render() {
    renderer.render(scene, camera);

}

function paint(x, y, z) {
    if (x !== 0 || y !== 0 || z !== 0) {
        geometry.vertices.push(new THREE.Vector3(x, y, z));
        var line = new THREE.Line(geometry, material, THREE.LineSegments);
        scene.add(line);
    }

    // axes = new THREE.AxesHelper(200); //
    // scene.add(axes);

}

function sleep(delay) {
    var start = new Date().getTime();
    while (new Date().getTime() < start + delay);
}

(function animate() {

    requestAnimationFrame(animate);

    controls.update();

    render();

})();