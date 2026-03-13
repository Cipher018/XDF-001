const THREE = require('three');
const katex = require('katex');

// ══════════════════════════════════════════════
// STATE
// ══════════════════════════════════════════════
let currentMode = 'waypoint';
let currentLang = 'es';
let currentStep = 1;
let totalSteps = 6;

let droneState = { x: 0, y: -50, z: 0, heading: 90, pitch: 0, speed: 10 };
let targetState = { x: 100, y: 100, z: 20, radius: 50, isCw: true };

let scene, camera, renderer;
let droneMesh, targetMesh, orbitMesh;
let axesGroup, stepVisuals = [];
let animFrames = [];

// CAD controls
let isDragging = false, isRotating = false;
let prevMouse = { x: 0, y: 0 };
// Camera spherical coords
let camTheta = Math.PI / 4;
let camPhi = Math.PI / 3;
let camRadius = 300;
let camTarget = new THREE.Vector3(50, 25, 0);

// ══════════════════════════════════════════════
// i18n CONTENT  ─ full textbook-style paragraphs
// ══════════════════════════════════════════════
const STEPS = {
    waypoint: {
        en: [
            {
                title: "Step 1 — Position Vectors",
                desc: `Every navigation problem begins by locating all agents in a common coordinate frame. 
Here we define two <em>position vectors</em>: the drone's current 3-D position 
<strong>P</strong><sub>drone</sub> and the target waypoint <strong>P</strong><sub>target</sub>, 
both expressed in the inertial NED (North-East-Down) frame with axes X (East), Y (North), Z (Up). 
The blue arrow originates at the world origin and points to the drone; 
the cyan arrow points to the waypoint. These are the raw inputs to all subsequent calculations.`,
                formulas: [
                    `\\vec{P}_{drone} = \\begin{pmatrix} x_d \\\\ y_d \\\\ z_d \\end{pmatrix}`,
                    `\\vec{P}_{target} = \\begin{pmatrix} x_t \\\\ y_t \\\\ z_t \\end{pmatrix}`,
                ]
            },
            {
                title: "Step 2 — Error Vector & Distance",
                desc: `The <em>error vector</em> is the displacement from the drone to the target. 
It is obtained by simple vector subtraction and encodes both the magnitude (straight-line 
distance) and the direction to fly. The 3-D Euclidean distance is the ℓ²-norm of this vector, 
computed via the Pythagorean theorem extended to three dimensions. 
The orange arrow visualises <strong>E</strong> drawn from the drone.`,
                formulas: [
                    `\\vec{E} = \\vec{P}_{target} - \\vec{P}_{drone}`,
                    `|\\vec{E}| = \\sqrt{E_x^2 + E_y^2 + E_z^2}`,
                    `\\hat{E} = \\frac{\\vec{E}}{|\\vec{E}|}\\quad(\\text{unit direction vector})`,
                ]
            },
            {
                title: "Step 3 — 3D Velocity Vector & Dot Product",
                desc: `The drone's <em>velocity unit vector</em> <strong>V̂</strong> is defined by both its 
magnetic heading ψ and its pitch angle θ. In the inertial frame, the X (East), Y (North), 
and Z (Up) components are derived from spherical coordinates. The <em>dot product</em> 
between <strong>V̂</strong> and the error direction <strong>Ê</strong> gives the cosine 
of the total 3-D angular error. This identifies how much the current flight path 
must be corrected to point directly at the waypoint.`,
                formulas: [
                    `\\hat{V} = \\begin{pmatrix} \\cos\\text{pitch} \\sin\\text{heading} \\\\ \\cos\\text{pitch} \\cos\\text{heading} \\\\ \\sin\\text{pitch} \\end{pmatrix}`,
                    `k = \\hat{V} \\cdot \\hat{E} = V_x E_x + V_y E_y + V_z E_z`,
                    `\\theta_{total} = \\arccos(k)`,
                ]
            },
            {
                title: "Step 4 — Cross Product & Turn Direction",
                desc: `The dot product gives the <em>magnitude</em> of the angle, but not whether to turn 
left or right. The <em>cross product</em> <strong>C = V̂ × Ê</strong> solves this. 
The Z-component of <strong>C</strong> carries the sign of the turn: 
positive C_z means the target is to the left (counter-clockwise), 
negative C_z means right (clockwise). This follows directly from the right-hand rule.`,
                formulas: [
                    `\\vec{C} = \\hat{V} \\times \\hat{E} = \\begin{vmatrix}\\mathbf{i}&\\mathbf{j}&\\mathbf{k}\\\\V_x&V_y&0\\\\E_x&E_y&0\\end{vmatrix}`,
                    `C_z = V_x E_y - V_y E_x`,
                    `\\text{Turn} = \\begin{cases}\\text{Left (CCW)}, & C_z > 0 \\\\ \\text{Right (CW)}, & C_z \\leq 0\\end{cases}`,
                ]
            },
            {
                title: "Step 5 — Vertical Angle (Pitch)",
                desc: `Navigation in 3-D space also requires correcting altitude. 
The <em>vertical angle</em> γ (flight path angle) between the horizontal plane 
and the error vector is found using the Z-component and horizontal magnitude. 
This angle feeds the pitch controller. The green vector shows the horizontal 
projection; the yellow shows the vertical component.`,
                formulas: [
                    `E_{horiz} = \\sqrt{E_x^2 + E_y^2}`,
                    `\\gamma = \\arctan\\!\\left(\\frac{E_z}{E_{horiz}}\\right)`,
                    `\\Theta_{cmd} = K_{p,pitch}\\cdot\\gamma`,
                ]
            },
            {
                title: "Step 6 — Proportional Control (Roll & Pitch)",
                desc: `A <em>proportional controller</em> linearly maps angular error to actuator deflection. 
The horizontal error θ drives the roll command Φ<sub>cmd</sub>; the vertical error γ drives the 
pitch command Θ<sub>cmd</sub>. The saturation function min(·, Φ<sub>max</sub>) prevents 
structural overload. The sign operator sgn(C_z) directs the roll into the turn. 
This simple law is the backbone of most fixed-wing autopilots.`,
                formulas: [
                    `\\Phi_{cmd} = \\operatorname{sgn}(C_z)\\cdot\\min\\!\\left(K_{p}\\cdot\\theta,\\,\\Phi_{max}\\right)`,
                    `\\Theta_{cmd} = K_{p,pitch}\\cdot\\gamma`,
                    `K_p = 0.6,\\quad \\Phi_{max}=45°`,
                ]
            }
        ],
        es: [
            {
                title: "Paso 1 — Vectores de Posición",
                desc: `Todo problema de navegación comienza ubicando a todos los agentes en un sistema de referencia común. 
Se definen dos <em>vectores de posición</em>: la posición 3-D actual del dron 
<strong>P</strong><sub>dron</sub> y el waypoint destino <strong>P</strong><sub>target</sub>, 
expresados en el marco inercial NED (Norte–Este–Abajo) con ejes X (Este), Y (Norte), Z (Arriba). 
La flecha azul parte del origen y apunta al dron; la cian apunta al waypoint. 
Estas son las entradas crudas de todos los cálculos subsiguientes.`,
                formulas: [
                    `\\vec{P}_{dron} = \\begin{pmatrix} x_d \\\\ y_d \\\\ z_d \\end{pmatrix}`,
                    `\\vec{P}_{target} = \\begin{pmatrix} x_t \\\\ y_t \\\\ z_t \\end{pmatrix}`,
                ]
            },
            {
                title: "Paso 2 — Vector de Error y Distancia",
                desc: `El <em>vector de error</em> es el desplazamiento desde el dron hasta el objetivo, 
obtenido por sustracción vectorial. Codifica tanto la magnitud (distancia en línea recta) 
como la dirección de vuelo. La distancia euclidiana 3-D es la norma ℓ² de este vector, 
calculada mediante el teorema de Pitágoras extendido a tres dimensiones. 
La flecha naranja visualiza <strong>E</strong> desde el dron.`,
                formulas: [
                    `\\vec{E} = \\vec{P}_{target} - \\vec{P}_{dron}`,
                    `|\\vec{E}| = \\sqrt{E_x^2 + E_y^2 + E_z^2}`,
                    `\\hat{E} = \\frac{\\vec{E}}{|\\vec{E}|}\\quad(\\text{vector unitario de dirección})`,
                ]
            },
            {
                title: "Paso 3 — Vector de Velocidad 3D y Producto Escalar",
                desc: `El <em>vector unitario de velocidad</em> <strong>V̂</strong> se define tanto por su 
rumbo magnético ψ como por su ángulo de cabeceo (*pitch*) θ. En el marco inercial, 
las componentes X (Este), Y (Norte) y Z (Arriba) se derivan usando coordenadas esféricas. 
El <em>producto escalar</em> entre <strong>V̂</strong> y la dirección del error 
<strong>Ê</strong> devuelve el coseno del error angular total en 3-D, identificando 
cuánto debe corregirse la trayectoria actual.`,
                formulas: [
                    `\\hat{V} = \\begin{pmatrix} \\cos\\text{cabeceo} \\sin\\text{rumbo} \\\\ \\cos\\text{cabeceo} \\cos\\text{rumbo} \\\\ \\sin\\text{cabeceo} \\end{pmatrix}`,
                    `k = \\hat{V} \\cdot \\hat{E} = V_x E_x + V_y E_y + V_z E_z`,
                    `\\theta_{total} = \\arccos(k)`,
                ]
            },
            {
                title: "Paso 4 — Producto Vectorial y Dirección de Giro",
                desc: `El producto escalar da la <em>magnitud</em> del ángulo, pero no si girar a la izquierda 
o derecha. El <em>producto vectorial</em> <strong>C = V̂ × Ê</strong> lo resuelve. 
La componente Z de <strong>C</strong> lleva el signo del giro: 
C_z positivo → objetivo a la izquierda (antihorario); 
C_z negativo → derecha (horario). Es consecuencia directa de la regla de la mano derecha.`,
                formulas: [
                    `\\vec{C} = \\hat{V} \\times \\hat{E} = \\begin{vmatrix}\\mathbf{i}&\\mathbf{j}&\\mathbf{k}\\\\V_x&V_y&0\\\\E_x&E_y&0\\end{vmatrix}`,
                    `C_z = V_x E_y - V_y E_x`,
                    `\\text{Giro} = \\begin{cases}\\text{Izq. (CCW)}, & C_z > 0 \\\\ \\text{Der. (CW)}, & C_z \\leq 0\\end{cases}`,
                ]
            },
            {
                title: "Paso 5 — Ángulo Vertical (Cabeceo)",
                desc: `En espacio 3-D, también es necesario corregir la altitud. 
El <em>ángulo vertical</em> γ (ángulo de trayectoria de vuelo) entre el plano horizontal 
y el vector de error se calcula usando la componente Z y la magnitud horizontal. 
Este ángulo alimenta el controlador de cabeceo. El vector verde muestra la proyección horizontal; 
el amarillo la componente vertical.`,
                formulas: [
                    `E_{horiz} = \\sqrt{E_x^2 + E_y^2}`,
                    `\\gamma = \\arctan\\!\\left(\\frac{E_z}{E_{horiz}}\\right)`,
                    `\\Theta_{cmd} = K_{p,cabeceo}\\cdot\\gamma`,
                ]
            },
            {
                title: "Paso 6 — Control Proporcional (Alabeo y Cabeceo)",
                desc: `Un <em>controlador proporcional</em> mapea linealmente el error angular al defleccionamiento del actuador. 
El error horizontal θ dirige el comando de alabeo Φ<sub>cmd</sub>; el error vertical γ dirige el 
comando de cabeceo Θ<sub>cmd</sub>. La función saturación min(·, Φ<sub>máx</sub>) previene sobrecarga 
estructural. El operador signo sgn(C_z) orienta el alabeo hacia el giro. 
Esta ley simple es la columna vertebral de la mayoría de los pilotos automáticos de ala fija.`,
                formulas: [
                    `\\Phi_{cmd} = \\operatorname{sgn}(C_z)\\cdot\\min\\!\\left(K_{p}\\cdot\\theta,\\,\\Phi_{max}\\right)`,
                    `\\Theta_{cmd} = K_{p,cabeceo}\\cdot\\gamma`,
                    `K_p = 0.6,\\quad \\Phi_{max}=45°`,
                ]
            }
        ]
    },
    orbit: {
        en: [
            {
                title: "Step 1 — Position Vectors",
                desc: `Position of the drone and the orbit center are defined in the same 3-D inertial frame. 
The orbit center acts as the <em>geometric reference</em> for all subsequent circular calculations.`,
                formulas: [
                    `\\vec{P}_{drone} = \\begin{pmatrix}x_d\\\\y_d\\\\z_d\\end{pmatrix}, \\quad \\vec{P}_{ctr} = \\begin{pmatrix}x_c\\\\y_c\\\\z_c\\end{pmatrix}`,
                ]
            },
            {
                title: "Step 2 — Radius Vector & Entry Points",
                desc: `The <em>radius vector</em> <strong>R</strong> points from the orbit center to the drone. 
Four <em>orthogonal entry candidates</em> E₁–E₄ are placed on the circle at cardinal offsets. 
The nearest one compatible with the desired rotation sense (CW/CCW) is selected as the insertion waypoint.`,
                formulas: [
                    `\\vec{R} = \\vec{P}_{drone} - \\vec{P}_{ctr}`,
                    `E_k = \\vec{P}_{ctr} + R_{des}\\cdot\\hat{u}_k,\\quad k=1,2,3,4`,
                    `E_{opt} = \\arg\\min_k |\\vec{P}_{drone} - E_k|`,
                ]
            },
            {
                title: "Step 3 — Tangential Approach Alignment",
                desc: `To smoothly enter the orbit, the drone must arrive <em>tangentially</em>. 
The ideal insertion heading ψ<sub>ins</sub> is perpendicular to the radius at E<sub>opt</sub>, 
computed via a 90° rotation of the normalized radius unit vector. 
The heading error drives the yaw pre-alignment.`,
                formulas: [
                    `\\hat{r}_{opt} = \\frac{E_{opt} - \\vec{P}_{ctr}}{R_{des}}`,
                    `\\hat{t}_{ins} = \\text{Rot}_z(\\pm 90°)\\cdot\\hat{r}_{opt}`,
                    `\\Delta\\psi = \\psi_{current} - \\psi_{ins}`,
                ]
            },
            {
                title: "Step 4 — Cross Track Error (CTE)",
                desc: `Once orbit tracking begins, the key quantity is the <em>Cross Track Error</em> (CTE): 
the signed radial deviation from the desired circle. 
It is the difference between the actual radial distance |R| and the target radius R<sub>des</sub>. 
A positive CTE means the drone is outside the orbit; negative means inside.`,
                formulas: [
                    `D_c = |\\vec{P}_{drone} - \\vec{P}_{ctr}| = \\sqrt{R_x^2 + R_y^2}`,
                    `CTE = D_c - R_{des}`,
                ]
            },
            {
                title: "Step 5 — Coupled Roll Command",
                desc: `The orbit roll command combines a fixed <em>base bank angle</em> Φ<sub>base</sub> 
(required to maintain circular flight) with a proportional radial correction. 
The base angle is derived from the desired circular speed and radius (centripetal equations). 
The proportional term corrects drift, suppressing CTE exponentially.`,
                formulas: [
                    `\\Phi_{orbit} = \\Phi_{base} + K_{rad}\\cdot CTE`,
                    `\\Phi_{base} = \\pm 20°\\;(\\text{CW: −, CCW: +})`,
                    `K_{rad} = 0.8`,
                ]
            }
        ],
        es: [
            {
                title: "Paso 1 — Vectores de Posición",
                desc: `Se definen las posiciones del dron y del centro de órbita en el mismo marco inercial 3-D. 
El centro actúa como <em>referencia geométrica</em> de todos los cálculos circulares posteriores.`,
                formulas: [
                    `\\vec{P}_{dron} = \\begin{pmatrix}x_d\\\\y_d\\\\z_d\\end{pmatrix}, \\quad \\vec{P}_{ctr} = \\begin{pmatrix}x_c\\\\y_c\\\\z_c\\end{pmatrix}`,
                ]
            },
            {
                title: "Paso 2 — Vector Radio y Puntos de Entrada",
                desc: `El <em>vector radio</em> <strong>R</strong> apunta del centro al dron. 
Se generan cuatro <em>candidatos de entrada ortogonales</em> E₁–E₄ sobre el círculo. 
Se selecciona el más cercano compatible con el sentido de rotación (hor./antihor.) como waypoint de inserción.`,
                formulas: [
                    `\\vec{R} = \\vec{P}_{dron} - \\vec{P}_{ctr}`,
                    `E_k = \\vec{P}_{ctr} + R_{des}\\cdot\\hat{u}_k,\\quad k=1,2,3,4`,
                    `E_{opt} = \\arg\\min_k |\\vec{P}_{dron} - E_k|`,
                ]
            },
            {
                title: "Paso 3 — Alineación Tangencial de Aproximación",
                desc: `Para entrar suavemente a la órbita, el dron debe llegar <em>tangencialmente</em>. 
El rumbo de inserción ψ<sub>ins</sub> es perpendicular al radio en E<sub>opt</sub>, 
obtenido rotando 90° el vector radio unitario. El error de rumbo guía la pre-alineación.`,
                formulas: [
                    `\\hat{r}_{opt} = \\frac{E_{opt} - \\vec{P}_{ctr}}{R_{des}}`,
                    `\\hat{t}_{ins} = \\text{Rot}_z(\\pm 90°)\\cdot\\hat{r}_{opt}`,
                    `\\Delta\\psi = \\psi_{actual} - \\psi_{ins}`,
                ]
            },
            {
                title: "Paso 4 — Error de Trayectoria Cruzada (CTE)",
                desc: `Una vez en seguimiento circular, la magnitud clave es el <em>Error de Trayectoria Cruzada</em> (CTE): 
la desviación radial signo desde el círculo deseado. 
CTE positivo → el dron está fuera; negativo → dentro de la órbita.`,
                formulas: [
                    `D_c = |\\vec{P}_{dron} - \\vec{P}_{ctr}| = \\sqrt{R_x^2 + R_y^2}`,
                    `CTE = D_c - R_{des}`,
                ]
            },
            {
                title: "Paso 5 — Comando de Alabeo Acoplado",
                desc: `El comando de alabeo de órbita combina un <em>ángulo base</em> Φ<sub>base</sub> 
(necesario para el vuelo circular, derivado de ecuaciones centrípetas) con una corrección proporcional radial. 
El término proporcional suprime el CTE exponencialmente.`,
                formulas: [
                    `\\Phi_{orbit} = \\Phi_{base} + K_{rad}\\cdot CTE`,
                    `\\Phi_{base} = \\pm 20°\\;(\\text{hor.: −, antihor.: +})`,
                    `K_{rad} = 0.8`,
                ]
            }
        ]
    }
};

// ══════════════════════════════════════════════
// DOM READY
// ══════════════════════════════════════════════
document.addEventListener('DOMContentLoaded', () => {
    document.getElementById('mode-waypoint').addEventListener('click', () => setMode('waypoint'));
    document.getElementById('mode-orbit').addEventListener('click', () => setMode('orbit'));
    document.getElementById('lang-toggle').addEventListener('click', toggleLang);
    document.getElementById('btn-prev-step').addEventListener('click', () => changeStep(-1));
    document.getElementById('btn-next-step').addEventListener('click', () => changeStep(1));
    document.getElementById('btn-update').addEventListener('click', updateFromInputs);

    initThreeJS();
    setupCADControls();
    setMode('waypoint');
});

// ══════════════════════════════════════════════
// MODE / LANGUAGE / STEP
// ══════════════════════════════════════════════
function setMode(mode) {
    currentMode = mode;
    currentStep = 1;
    totalSteps = STEPS[mode][currentLang].length;

    document.getElementById('mode-waypoint').classList.toggle('active', mode === 'waypoint');
    document.getElementById('mode-orbit').classList.toggle('active', mode === 'orbit');
    document.body.classList.toggle('orbit-mode', mode === 'orbit');

    const orbitInputs = document.getElementById('orbit-specific-inputs');
    orbitInputs.style.display = mode === 'orbit' ? 'flex' : 'none';

    const orbitMeshVisible = (mode === 'orbit');
    if (orbitMesh) orbitMesh.visible = orbitMeshVisible;

    updateStepUI();
    render3DStep();
}

function toggleLang() {
    currentLang = currentLang === 'en' ? 'es' : 'en';
    totalSteps = STEPS[currentMode][currentLang].length;
    updateStepUI();
    render3DStep();
}

function changeStep(delta) {
    currentStep = Math.max(1, Math.min(totalSteps, currentStep + delta));
    updateStepUI();
    render3DStep();
}

function updateFromInputs() {
    droneState.x = parseFloat(document.getElementById('drone-x').value) || 0;
    droneState.y = parseFloat(document.getElementById('drone-y').value) || 0;
    droneState.z = parseFloat(document.getElementById('drone-z').value) || 0;
    droneState.heading = parseFloat(document.getElementById('drone-heading').value) || 0;
    droneState.pitch   = parseFloat(document.getElementById('drone-pitch').value)   || 0;
    droneState.speed   = parseFloat(document.getElementById('drone-speed').value)   || 10;

    targetState.x = parseFloat(document.getElementById('target-x').value) || 0;
    targetState.y = parseFloat(document.getElementById('target-y').value) || 0;
    targetState.z = parseFloat(document.getElementById('target-z').value) || 0;
    targetState.radius = parseFloat(document.getElementById('orbit-radius').value) || 50;
    targetState.isCw = document.getElementById('orbit-dir').value === 'cw';

    rebuildBaseMeshes();
    render3DStep();
}

function updateStepUI() {
    const steps = STEPS[currentMode][currentLang];
    totalSteps = steps.length;
    const s = steps[currentStep - 1];

    document.getElementById('btn-prev-step').disabled = (currentStep === 1);
    document.getElementById('btn-next-step').disabled = (currentStep === totalSteps);
    document.getElementById('current-step-num').textContent = currentStep;
    document.getElementById('total-step-num').textContent = totalSteps;

    // Dots
    const dc = document.getElementById('dots-container');
    dc.innerHTML = '';
    for (let i = 1; i <= totalSteps; i++) {
        const dot = document.createElement('div');
        dot.className = `step-dot ${i === currentStep ? 'active' : ''}`;
        dc.appendChild(dot);
    }

    // Step title
    document.getElementById('step-title').textContent = s.title;

    // Textbook description
    const descEl = document.getElementById('step-description');
    descEl.innerHTML = s.desc;   // replaces fully each step

    // KaTeX formulas
    const formulaEl = document.getElementById('formula-block');
    formulaEl.innerHTML = s.formulas.map(f =>
        `<div class="formula-line">${katex.renderToString(f, { displayMode: true, throwOnError: false })}</div>`
    ).join('');
}

// ══════════════════════════════════════════════
// THREE.JS INIT
// ══════════════════════════════════════════════
function initThreeJS() {
    const container = document.querySelector('.viz-container');

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x05080f);

    // ── Grid (XY plane, 600×600, 30 divisions) ──
    const size = 600, divs = 30;
    const gridXY = new THREE.GridHelper(size, divs, 0x00e5ff, 0x142030);
    gridXY.material.opacity = 0.35;
    gridXY.material.transparent = true;
    gridXY.rotation.x = Math.PI / 2; // XY horizontal
    scene.add(gridXY);

    // secondary coarser grid lines
    const gridCoarse = new THREE.GridHelper(size, 6, 0x00e5ff, 0x00e5ff);
    gridCoarse.material.opacity = 0.12;
    gridCoarse.material.transparent = true;
    gridCoarse.rotation.x = Math.PI / 2;
    scene.add(gridCoarse);

    // ── World Axes ──
    axesGroup = new THREE.Group();
    const AL = 60; // axis length
    axesGroup.add(makeAxisArrow(new THREE.Vector3(1,0,0), AL, 0xff4444, 'X'));
    axesGroup.add(makeAxisArrow(new THREE.Vector3(0,1,0), AL, 0x44ff44, 'Y'));
    axesGroup.add(makeAxisArrow(new THREE.Vector3(0,0,1), AL, 0x4488ff, 'Z'));
    scene.add(axesGroup);

    // ── Delta-wing drone mesh ──
    droneMesh = buildDeltaWing(0xffffff);
    scene.add(droneMesh);

    // ── Target sphere ──
    const targetGeo = new THREE.SphereGeometry(3, 16, 16);
    const targetMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff });
    targetMesh = new THREE.Mesh(targetGeo, targetMat);
    scene.add(targetMesh);

    // ── Orbit ring (hidden by default) ──
    const orbitGeo = new THREE.RingGeometry(49, 51, 64);
    const orbitMat = new THREE.MeshBasicMaterial({ color: 0xb388ff, side: THREE.DoubleSide, transparent: true, opacity: 0.5 });
    orbitMesh = new THREE.Mesh(orbitGeo, orbitMat);
    orbitMesh.visible = false;
    scene.add(orbitMesh);

    // ── Camera ──
    camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 3000);
    updateCameraFromSpherical();

    // ── Renderer ──
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // ── Resize ──
    new ResizeObserver(entries => {
        for (const e of entries) {
            camera.aspect = e.contentRect.width / e.contentRect.height;
            camera.updateProjectionMatrix();
            renderer.setSize(e.contentRect.width, e.contentRect.height);
        }
    }).observe(container);

    // ── Ambient light to shade the delta wing ──
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(100, 200, 100);
    scene.add(dirLight);

    rebuildBaseMeshes();

    // ── Animation loop ──
    (function loop() {
        requestAnimationFrame(loop);
        // gentle drone hover
        if (droneMesh) droneMesh.position.z = droneState.z + Math.sin(Date.now() * 0.001) * 0.8;
        renderer.render(scene, camera);
    })();
}

// ── Build delta-wing shape ──
function buildDeltaWing(color) {
    const shape = new THREE.Shape();
    // In XY plane, +Y is North. Nose at (0, 15)
    shape.moveTo(0, 15);        // Nose
    shape.lineTo(-10, -10);     // Left wing tip
    shape.lineTo(-2, -6);       // Left notch
    shape.lineTo(0, -9);        // Tail
    shape.lineTo(2, -6);        // Right notch
    shape.lineTo(10, -10);      // Right wing tip
    shape.lineTo(0, 15);        // Back to nose

    const extrudeSettings = {
        depth: 2,
        bevelEnabled: true,
        bevelThickness: 0.5,
        bevelSize: 0.5,
        bevelSegments: 3
    };

    const geo = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    // Center depth (Z)
    geo.translate(0, 0, -1);
    
    // NO rotateX. Keep it in XY plane pointing +Y.
    
    const mat = new THREE.MeshPhongMaterial({ 
        color: 0x00e5ff, 
        emissive: 0x00e5ff,
        emissiveIntensity: 0.3,
        shininess: 100,
        specular: 0xffffff
    });
    return new THREE.Mesh(geo, mat);
}

function makeAxisArrow(dir, length, color, label) {
    const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0,0,0), length, color, 8, 5);
    return arrow;
}

function rebuildBaseMeshes() {
    droneMesh.position.set(droneState.x, droneState.y, droneState.z);

    // Aviation convention: Heading 0 = North (+Y), 90 = East (+X), clockwise.
    // Three.js: Z-axis is Up. Positive rotation is counter-clockwise.
    // Since drone points North (+Y) at rotation 0, a 90 deg clockwise turn
    // (aviation heading 90) must be a -90 deg rotation in Three.js.
    
    const yaw = -droneState.heading * (Math.PI / 180);
    const pitch = droneState.pitch * (Math.PI / 180);
    
    // Apply order: Yaw then Pitch
    droneMesh.rotation.set(0, 0, yaw);
    droneMesh.rotateX(pitch);

    targetMesh.position.set(targetState.x, targetState.y, targetState.z);

    if (currentMode === 'orbit') {
        orbitMesh.visible = true;
        orbitMesh.position.set(targetState.x, targetState.y, targetState.z);
        orbitMesh.geometry.dispose();
        orbitMesh.geometry = new THREE.RingGeometry(
            Math.max(1, targetState.radius - 1), targetState.radius + 1, 64
        );
    } else {
        orbitMesh.visible = false;
    }
}

// ══════════════════════════════════════════════
// CAD CONTROLS
// ══════════════════════════════════════════════
function updateCameraFromSpherical() {
    camera.position.set(
        camTarget.x + camRadius * Math.sin(camPhi) * Math.sin(camTheta),
        camTarget.y - camRadius * Math.sin(camPhi) * Math.cos(camTheta),
        camTarget.z + camRadius * Math.cos(camPhi)
    );
    camera.lookAt(camTarget);
}

function setupCADControls() {
    const cnv = renderer.domElement;

    cnv.addEventListener('mousedown', e => {
        if (e.button === 0 && e.ctrlKey) { changeStep(1); return; }
        isDragging = true;
        isRotating = (e.button === 2 && e.ctrlKey);
        prevMouse = { x: e.clientX, y: e.clientY };
    });

    cnv.addEventListener('mousemove', e => {
        if (!isDragging) return;
        const dx = e.clientX - prevMouse.x;
        const dy = e.clientY - prevMouse.y;
        prevMouse = { x: e.clientX, y: e.clientY };

        if (isRotating) {
            camTheta -= dx * 0.007;
            camPhi = Math.max(0.05, Math.min(Math.PI * 0.95, camPhi - dy * 0.007));
        } else {
            // Pan in camera-right & camera-up
            const right = new THREE.Vector3();
            const up = new THREE.Vector3();
            camera.getWorldDirection(new THREE.Vector3()); // ensure matrices updated
            right.crossVectors(camera.getWorldDirection(new THREE.Vector3()).negate(), camera.up).normalize();
            up.copy(camera.up).normalize();
            const panSpeed = camRadius * 0.0015;
            camTarget.addScaledVector(right, -dx * panSpeed);
            camTarget.addScaledVector(up, dy * panSpeed);
        }
        updateCameraFromSpherical();
    });

    cnv.addEventListener('mouseup', () => { isDragging = false; isRotating = false; });
    cnv.addEventListener('mouseleave', () => { isDragging = false; isRotating = false; });
    cnv.addEventListener('contextmenu', e => e.preventDefault());
    cnv.addEventListener('wheel', e => {
        camRadius = Math.max(30, Math.min(1000, camRadius + e.deltaY * 0.3));
        updateCameraFromSpherical();
    });
}

// ══════════════════════════════════════════════
// STEP VECTOR VISUALIZATION
// ══════════════════════════════════════════════
function clearStepVisuals() {
    stepVisuals.forEach(o => scene.remove(o));
    stepVisuals = [];
}

function addVec(origin, dir, length, color, headLen = 5, headWidth = 3) {
    if (length < 0.01) return;
    const a = new THREE.ArrowHelper(dir.clone().normalize(), origin, length, color, headLen, headWidth);
    scene.add(a);
    stepVisuals.push(a);
    return a;
}

function addSphere(pos, r, color) {
    const m = new THREE.Mesh(
        new THREE.SphereGeometry(r, 12, 12),
        new THREE.MeshBasicMaterial({ color })
    );
    m.position.copy(pos);
    scene.add(m);
    stepVisuals.push(m);
}

function addLine(a, b, color) {
    const points = [a.clone(), b.clone()];
    const geo = new THREE.BufferGeometry().setFromPoints(points);
    const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 });
    const line = new THREE.Line(geo, mat);
    scene.add(line);
    stepVisuals.push(line);
}

function render3DStep() {
    clearStepVisuals();
    updateStepUI();

    const P_d = new THREE.Vector3(droneState.x, droneState.y, droneState.z);
    const P_t = new THREE.Vector3(targetState.x, targetState.y, targetState.z);
    const O   = new THREE.Vector3(0, 0, 0);

    const radH = (90 - droneState.heading) * Math.PI / 180;
    const radP = droneState.pitch * Math.PI / 180;
    const V_dir = new THREE.Vector3(
        Math.cos(radP) * Math.cos(radH),
        Math.cos(radP) * Math.sin(radH),
        Math.sin(radP)
    ).normalize();
    const E = P_t.clone().sub(P_d);
    const dist3D = E.length();
    const E_dir = E.clone().normalize();
    const E_horiz = new THREE.Vector3(E.x, E.y, 0);
    const horizDist = E_horiz.length();

    if (currentMode === 'waypoint') {
        // STEP 1: position vectors from origin
        if (currentStep >= 1) {
            addVec(O, P_d.clone().normalize(), P_d.length(), 0x5599ff); // blue → drone
            addVec(O, P_t.clone().normalize(), P_t.length(), 0x00e5ff); // cyan → target
            addSphere(P_d, 2, 0x5599ff);
        }
        // STEP 2: error vector
        if (currentStep >= 2) {
            addVec(P_d, E_dir, dist3D, 0xff8800, 7, 4); // orange error
            addLine(P_d, P_t, 0x334455);
        }
        // STEP 3: velocity vector + angle arc hint
        if (currentStep >= 3) {
            addVec(P_d, V_dir, 35, 0x44ff88, 6, 3); // green velocity
        }
        // STEP 4: cross product axis line
        if (currentStep >= 4) {
            const cross = new THREE.Vector3().crossVectors(V_dir, E_dir);
            const crossCol = cross.z >= 0 ? 0xff44ff : 0xffff00;
            addVec(P_d, new THREE.Vector3(0, 0, 1), 30, crossCol, 5, 3); // Z axis indicator
        }
        // STEP 5: vertical decomposition
        if (currentStep >= 5) {
            const P_d_flat = new THREE.Vector3(P_d.x, P_d.y, 0);
            const P_t_flat = new THREE.Vector3(P_t.x, P_t.y, 0);
            addVec(P_d, E_horiz.clone().normalize(), horizDist, 0x44ff88, 5, 3); // horiz proj green
            addVec(new THREE.Vector3(P_t.x, P_t.y, P_d.z), new THREE.Vector3(0, 0, 1), Math.abs(E.z), 0xffcc00, 5, 3); // vertical yellow
            addLine(P_d_flat, P_t_flat, 0x336633);
        }
        // STEP 6: roll direction indicator
        if (currentStep >= 6) {
            const dot = V_dir.dot(E_dir);
            const angDeg = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
            const crossZ = V_dir.x * E_dir.y - V_dir.y * E_dir.x;
            const roll = Math.min(angDeg * 0.6, 45) * (crossZ >= 0 ? 1 : -1);
            // roll tilt of drone mesh: animate it
            droneMesh.rotation.y = (roll * Math.PI / 180);
        } else {
            droneMesh.rotation.y = 0;
        }
    } else {
        // ORBIT
        const R = targetState.radius;
        const radVec = P_d.clone().sub(P_t); // from center to drone
        const radDist = radVec.length();
        const radDir = radVec.clone().normalize();

        if (currentStep >= 1) {
            addVec(O, P_d.clone().normalize(), P_d.length(), 0x5599ff);
            addVec(O, P_t.clone().normalize(), P_t.length(), 0xb388ff);
            addSphere(P_d, 2, 0x5599ff);
        }
        if (currentStep >= 2) {
            addVec(P_t, radDir, radDist, 0xff8800, 5, 3); // radius vector
            const candColors = [0xffaa00, 0xffcc44, 0xffdd88, 0xffee99];
            [
                new THREE.Vector3(P_t.x, P_t.y + R, P_t.z),
                new THREE.Vector3(P_t.x + R, P_t.y, P_t.z),
                new THREE.Vector3(P_t.x, P_t.y - R, P_t.z),
                new THREE.Vector3(P_t.x - R, P_t.y, P_t.z)
            ].forEach((ep, i) => addSphere(ep, 2.5, candColors[i]));
        }
        if (currentStep >= 3) {
            // tangent
            const tang = new THREE.Vector3(-radDir.y, radDir.x, 0).normalize();
            const tangScaled = targetState.isCw ? tang.clone().negate() : tang;
            addVec(P_t.clone().add(radDir.clone().multiplyScalar(R)), tangScaled, 30, 0x44ffff, 5, 3);
        }
        if (currentStep >= 4) {
            const CTE = radDist - R;
            const cteColor = CTE > 0 ? 0xff4444 : 0x44ff44;
            addVec(P_t, radDir, radDist, cteColor, 6, 4);
            addLine(P_t.clone().add(radDir.clone().multiplyScalar(R)), P_d, 0xffff00);
        }
        if (currentStep >= 5) {
            droneMesh.rotation.y = (targetState.isCw ? -20 : 20) * Math.PI / 180;
        } else {
            droneMesh.rotation.y = 0;
        }
    }
}
