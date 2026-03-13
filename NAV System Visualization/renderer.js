const THREE = require('three');

// ═══════════════════════════════════════════════════════
// INITIALIZATION & STATE
// ═══════════════════════════════════════════════════════

let currentMode = 'waypoint'; // 'waypoint' or 'orbit'
let currentLang = 'es'; // 'en' or 'es'
let currentStep = 1;
let totalSteps = 4; // Waypoint has 4, Orbit has 5

// Data State
let droneState = { x: 0, y: 0, z: 0, heading: 90, speed: 10 };
let targetState = { x: 100, y: 100, z: 20, radius: 50, isCw: true };

// Three.js State
let scene, camera, renderer;
let droneMesh, targetMesh, orbitMesh;
let arrowHelpers = []; // Store vectors to clear them between steps
let entryPointSpheres = [];

// CAD Controls State
let isDragging = false;
let isRotating = false;
let previousMousePosition = { x: 0, y: 0 };

// i18n Dictionary
const i18n = {
    en: {
        app_title: "NAV System 3D Viz",
        mode_label: "Navigation Mode",
        mode_waypoint: "Waypoint",
        mode_orbit: "Orbit",
        drone_state: "Current Drone State",
        heading: "Heading (°)",
        speed: "Speed (m/s)",
        target_waypoint: "Target Waypoint",
        orbit_center: "Orbit Center",
        orbit_radius: "Radius (m)",
        orbit_dir: "Direction",
        dir_cw: "CW",
        dir_ccw: "CCW",
        update_viz: "Update Visualization",
        calc_steps: "Calculation Steps",
        prev: "Prev",
        next: "Next",
        // Waypoint Steps
        wp_step1_title: "Distance & Error Vector",
        wp_step1_desc: "Calculates the straight line distance to the target using the 3D position. Extracts the horizontal error vector.",
        wp_step2_title: "Horizontal Angle",
        wp_step2_desc: "Determines the angle between the drone's travel direction and target direction using the dot product formula.",
        wp_step3_title: "Turn Direction",
        wp_step3_desc: "Calculates the cross product of the travel and target direction vectors to define the shortest turn.",
        wp_step4_title: "Proportional Control",
        wp_step4_desc: "Translates the angular error into an actuator command (roll) using a proportional gain.",
        // Orbit Steps
        or_step1_title: "Phase & Entry Point",
        or_step1_desc: "Evaluates the optimal entry point to intercept the assigned orbit.",
        or_step2_title: "Approach & Alignment",
        or_step2_desc: "Navigates towards the insertion point, setting up the tangential intercept yaw.",
        or_step3_title: "Cross Track Error (CTE)",
        or_step3_desc: "Calculates the radial deviation from the desired circular path.",
        or_step4_title: "Roll Command",
        or_step4_desc: "Applies a base banking angle summed with a proportional correction to steer into the circle.",
        or_step5_title: "Orbit Maintenance",
        or_step5_desc: "Continuously adjusts roll and pitch to maintain the circle and altitude."
    },
    es: {
        app_title: "Visualización 3D NAV",
        mode_label: "Modo de Navegación",
        mode_waypoint: "Waypoints",
        mode_orbit: "Órbita",
        drone_state: "Estado del Dron",
        heading: "Rumbo (°)",
        speed: "Velocidad (m/s)",
        target_waypoint: "Waypoint Objetivo",
        orbit_center: "Centro de Órbita",
        orbit_radius: "Radio (m)",
        orbit_dir: "Dirección",
        dir_cw: "Horario",
        dir_ccw: "C. Horario",
        update_viz: "Actualizar Vista",
        calc_steps: "Pasos de Cálculo",
        prev: "Ant",
        next: "Sig",
        // Waypoint Steps
        wp_step1_title: "Distancia y Vector de Error",
        wp_step1_desc: "Cálculo de magnitud de la distancia y extracción del vector direccional.",
        wp_step2_title: "Ángulo Horizontal",
        wp_step2_desc: "Cálculo del ángulo de intercepción utilizando la definición del producto escalar.",
        wp_step3_title: "Dirección de Giro",
        wp_step3_desc: "Uso del producto vectorial (componente Z) para definir la dirección más corta de giro.",
        wp_step4_title: "Control Proporcional",
        wp_step4_desc: "Transformación del error angular en un comando físico de alabeo (Roll).",
        // Orbit Steps
        or_step1_title: "Análisis de Inserción",
        or_step1_desc: "Cálculo matemático del punto óptimo de entrada a la trayectoria circular geométrica.",
        or_step2_title: "Alineación Tangencial",
        or_step2_desc: "Aproximación inicial y cálculo del componente tangencial deseado del vector de velocidad.",
        or_step3_title: "Error de Trayectoria Cruzada (CTE)",
        or_step3_desc: "Cálculo de la desviación escalar entre el radio analítico y la distancia radial actual.",
        or_step4_title: "Comando de Alabeo Acoplado",
        or_step4_desc: "Superposición del alabeo base (centrípeto) con el término proporcional de corrección radial.",
        or_step5_title: "Mantenimiento Periódico",
        or_step5_desc: "Lazo cerrado estabilizado para sostener parámetros de la cinemática circular."
    }
};

// ═══════════════════════════════════════════════════════
// SETUP & EVENT LISTENERS
// ═══════════════════════════════════════════════════════

document.addEventListener('DOMContentLoaded', () => {
    // Mode toggles
    document.getElementById('mode-waypoint').addEventListener('click', () => setMode('waypoint'));
    document.getElementById('mode-orbit').addEventListener('click', () => setMode('orbit'));
    
    // Language toggle
    document.getElementById('lang-toggle').addEventListener('click', toggleLanguage);
    
    // Step navigation
    document.getElementById('btn-prev-step').addEventListener('click', () => changeStep(-1));
    document.getElementById('btn-next-step').addEventListener('click', () => changeStep(1));
    
    // Update visualization
    document.getElementById('btn-update').addEventListener('click', updateDataFromInputs);
    
    initThreeJS();
    setupCADControls();
    
    // Initial Setup
    applyI18n();
    updateStepUI();
    renderMathAndVectors();
});

// ═══════════════════════════════════════════════════════
// UI & STATE LOGIC
// ═══════════════════════════════════════════════════════

function setMode(mode) {
    currentMode = mode;
    currentStep = 1;
    totalSteps = (mode === 'waypoint') ? 4 : 5;
    
    document.getElementById('mode-waypoint').classList.toggle('active', mode === 'waypoint');
    document.getElementById('mode-orbit').classList.toggle('active', mode === 'orbit');
    
    if(mode === 'orbit') {
        document.body.classList.add('orbit-mode');
        document.getElementById('orbit-specific-inputs').style.display = 'flex';
        document.getElementById('target-title').setAttribute('data-i18n', 'orbit_center');
    } else {
        document.body.classList.remove('orbit-mode');
        document.getElementById('orbit-specific-inputs').style.display = 'none';
        document.getElementById('target-title').setAttribute('data-i18n', 'target_waypoint');
    }
    
    applyI18n();
    updateStepUI();
    renderMathAndVectors();
}

function toggleLanguage() {
    currentLang = (currentLang === 'en') ? 'es' : 'en';
    applyI18n();
    updateStepUI();
}

function applyI18n() {
    const dict = i18n[currentLang];
    document.querySelectorAll('[data-i18n]').forEach(el => {
        const key = el.getAttribute('data-i18n');
        if (dict[key]) {
            if(el.tagName === 'INPUT' && el.type === 'button') {
                el.value = dict[key];
            } else {
                el.textContent = dict[key];
            }
        }
    });
}

function updateDataFromInputs() {
    droneState.x = parseFloat(document.getElementById('drone-x').value) || 0;
    droneState.y = parseFloat(document.getElementById('drone-y').value) || 0;
    droneState.heading = parseFloat(document.getElementById('drone-heading').value) || 0;
    droneState.speed = parseFloat(document.getElementById('drone-speed').value) || 0;
    
    targetState.x = parseFloat(document.getElementById('target-x').value) || 0;
    targetState.y = parseFloat(document.getElementById('target-y').value) || 0;
    targetState.radius = parseFloat(document.getElementById('orbit-radius').value) || 50;
    targetState.isCw = document.getElementById('orbit-dir').value === 'cw';
    
    updateBaseMeshes();
    renderMathAndVectors();
}

function changeStep(delta) {
    currentStep += delta;
    if (currentStep < 1) currentStep = 1;
    if (currentStep > totalSteps) currentStep = totalSteps;
    updateStepUI();
    renderMathAndVectors();
}

function updateStepUI() {
    document.getElementById('btn-prev-step').disabled = (currentStep === 1);
    document.getElementById('btn-next-step').disabled = (currentStep === totalSteps);
    
    document.getElementById('current-step-num').textContent = currentStep;
    document.getElementById('total-step-num').textContent = totalSteps;
    
    const dotsContainer = document.getElementById('dots-container');
    dotsContainer.innerHTML = '';
    for(let i=1; i<=totalSteps; i++) {
        const dot = document.createElement('div');
        dot.className = `step-dot ${i === currentStep ? 'active' : ''}`;
        dotsContainer.appendChild(dot);
    }
    
    const dict = i18n[currentLang];
    let prefix = currentMode === 'waypoint' ? 'wp_' : 'or_';
    document.getElementById('step-title').textContent = dict[`${prefix}step${currentStep}_title`];
    document.getElementById('step-description').textContent = dict[`${prefix}step${currentStep}_desc`];
    document.getElementById('formula-block').innerHTML = '';
}

// ═══════════════════════════════════════════════════════
// THREE.JS & CAD CONTROLS
// ═══════════════════════════════════════════════════════

function initThreeJS() {
    const container = document.querySelector('.viz-container');
    
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x05080f);
    
    // Grid Helper
    const gridHelper = new THREE.GridHelper(500, 50, 0xffffff, 0xffffff);
    gridHelper.material.opacity = 0.1;
    gridHelper.material.transparent = true;
    gridHelper.rotation.x = Math.PI / 2; // XY plane
    scene.add(gridHelper);

    camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 2000);
    // Move camera back to see the XY plane
    camera.position.set(0, -100, 100);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Basic geometries
    const droneGeo = new THREE.ConeGeometry(3, 10, 3);
    droneGeo.rotateX(Math.PI / 2); // Point along Y
    const droneMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
    droneMesh = new THREE.Mesh(droneGeo, droneMat);
    scene.add(droneMesh);

    const targetGeo = new THREE.SphereGeometry(2, 16, 16);
    const targetMat = new THREE.MeshBasicMaterial({ color: 0x00e5ff });
    targetMesh = new THREE.Mesh(targetGeo, targetMat);
    scene.add(targetMesh);
    
    const orbitGeo = new THREE.RingGeometry(49, 50, 64);
    const orbitMat = new THREE.MeshBasicMaterial({ color: 0xb388ff, side: THREE.DoubleSide, transparent: true, opacity: 0.5 });
    orbitMesh = new THREE.Mesh(orbitGeo, orbitMat);
    scene.add(orbitMesh);

    // Initial positioning
    updateBaseMeshes();

    // Resize handling
    const resizeObserver = new ResizeObserver(entries => {
        for (let entry of entries) {
            camera.aspect = entry.contentRect.width / entry.contentRect.height;
            camera.updateProjectionMatrix();
            renderer.setSize(entry.contentRect.width, entry.contentRect.height);
        }
    });
    resizeObserver.observe(container);
    
    // Animation loop
    const animate = function () {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    };
    animate();
}

function updateBaseMeshes() {
    droneMesh.position.set(droneState.x, droneState.y, 0);
    // Heading in aviation is 0=North/Y-up. Rotate drone around Z axis.
    const radHeading = (-droneState.heading) * Math.PI / 180;
    droneMesh.rotation.z = radHeading;
    
    targetMesh.position.set(targetState.x, targetState.y, 0);
    
    if(currentMode === 'orbit') {
        orbitMesh.visible = true;
        orbitMesh.position.set(targetState.x, targetState.y, 0);
        // recreate geometry if radius changed
        orbitMesh.geometry.dispose();
        orbitMesh.geometry = new THREE.RingGeometry(Math.max(1, targetState.radius - 0.5), targetState.radius + 0.5, 64);
    } else {
        orbitMesh.visible = false;
    }
}

function setupCADControls() {
    const canvas = renderer.domElement;
    
    // Ctrl + Left Click (Advance step), Left Click (Pan), Ctrl + Right Click (Rotate)
    canvas.addEventListener('mousedown', (e) => {
        if (e.button === 0 && e.ctrlKey) { // Left + Ctrl
            changeStep(1);
            return;
        }
        
        isDragging = true;
        isRotating = (e.button === 2 && e.ctrlKey); // Right + Ctrl
        previousMousePosition = { x: e.offsetX, y: e.offsetY };
    });

    canvas.addEventListener('mousemove', (e) => {
        if (!isDragging) return;

        const deltaMove = {
            x: e.offsetX - previousMousePosition.x,
            y: e.offsetY - previousMousePosition.y
        };

        if (isRotating) {
            // Rotate camera around origin (simplified)
            const rotateSpeed = 0.01;
            camera.position.applyAxisAngle(new THREE.Vector3(0, 0, 1), -deltaMove.x * rotateSpeed);
            camera.position.applyAxisAngle(new THREE.Vector3(1, 0, 0), -deltaMove.y * rotateSpeed);
            camera.lookAt(0,0,0);
        } else if (e.button === 0) { // Left-click Pan
            const panSpeed = 0.5;
            // Simplified pan on XY plane
            camera.position.x -= deltaMove.x * panSpeed;
            camera.position.y += deltaMove.y * panSpeed;
        }

        previousMousePosition = { x: e.offsetX, y: e.offsetY };
    });

    canvas.addEventListener('mouseup', () => { isDragging = false; isRotating = false; });
    canvas.addEventListener('mouseleave', () => { isDragging = false; isRotating = false; });
    canvas.addEventListener('contextmenu', e => e.preventDefault()); // Prevent normal right-click menu
    
    // Zoom
    canvas.addEventListener('wheel', (e) => {
        const zoomSpeed = 0.1;
        camera.position.z += e.deltaY * zoomSpeed;
        camera.position.z = Math.max(10, Math.min(camera.position.z, 1000));
    });
}

function clearStepVisuals() {
    arrowHelpers.forEach(ah => scene.remove(ah));
    arrowHelpers = [];
    entryPointSpheres.forEach(s => scene.remove(s));
    entryPointSpheres = [];
}

function addArrow(origin, dir, length, color) {
    const arrow = new THREE.ArrowHelper(dir, origin, length, color, 4, 3);
    scene.add(arrow);
    arrowHelpers.push(arrow);
    return arrow;
}

// ═══════════════════════════════════════════════════════
// MATH RENDERING (Pure Math Notation & 3D Vectors)
// ═══════════════════════════════════════════════════════

function renderMathAndVectors() {
    clearStepVisuals();
    const formulaDiv = document.getElementById('formula-block');
    let mathStr = "";
    
    const P_d = new THREE.Vector3(droneState.x, droneState.y, 0);
    const P_t = new THREE.Vector3(targetState.x, targetState.y, 0);
    
    // Travel Direction
    const radH = (90 - droneState.heading) * Math.PI / 180;
    const V_dir = new THREE.Vector3(Math.cos(radH), Math.sin(radH), 0);
    
    // Error Vector
    const E = new THREE.Vector3().subVectors(P_t, P_d);
    const dist = E.length();
    const E_dir = E.clone().normalize();

    if (currentMode === 'waypoint') {
        // WAYPOINT LOGIC
        if (currentStep >= 1) {
            addArrow(P_d, E_dir, dist, 0x00e5ff); // Error arrow
            mathStr += `\\text{Error Vector: } \\vec{E} = P_{target} - P_{drone}\\\\`;
            mathStr += `\\vec{E} = (${E.x.toFixed(1)}, ${E.y.toFixed(1)})\\\\`;
            mathStr += `\\text{Distance: } |\\vec{E}| = \\sqrt{E_x^2 + E_y^2} = ${dist.toFixed(1)}\\text{ m}`;
        }
        
        if (currentStep >= 2) {
            addArrow(P_d, V_dir, 20, 0x00ff00); // Velocity arrow
            let dot = V_dir.dot(E_dir);
            let angle = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
            
            mathStr += `\\text{Travel Dir: } \\hat{V} = (${V_dir.x.toFixed(2)}, ${V_dir.y.toFixed(2)})\\\\`;
            mathStr += `\\text{Target Dir: } \\hat{E} = (${E_dir.x.toFixed(2)}, ${E_dir.y.toFixed(2)})\\\\`;
            mathStr += `\\text{Scalar Prod: } k = \\hat{V} \\cdot \\hat{E} = ${dot.toFixed(3)}\\\\`;
            mathStr += `\\text{Horiz Angle: } \\theta = \\arccos(k) = ${angle.toFixed(1)}^{\\circ}`;
        }
        
        if (currentStep >= 3) {
            let cross = new THREE.Vector3().crossVectors(V_dir, E_dir);
            mathStr += `\\text{Vector Prod: } \\vec{C} = \\hat{V} \\times \\hat{E}\\\\`;
            mathStr += `\\vec{C}_z = (V_{x}E_{y} - V_{y}E_{x}) = ${cross.z.toFixed(3)}\\\\`;
            mathStr += `\\text{Turn: } \\begin{cases} \\text{Left,} & C_z > 0 \\\\ \\text{Right,} & C_z \\leq 0 \\end{cases}\\\\`;
            mathStr += `\\text{Result: } ${cross.z > 0 ? '\\text{Left}' : '\\text{Right'}`;
        }
        
        if (currentStep >= 4) {
            let dot = V_dir.dot(E_dir);
            let angleRad = Math.acos(Math.max(-1, Math.min(1, dot)));
            let angleDeg = angleRad * 180 / Math.PI;
            let crossZ = (V_dir.x*E_dir.y - V_dir.y*E_dir.x);
            let turnLeft = crossZ > 0;
            
            let roll = (angleDeg * 0.6); // KP = 0.6
            roll = Math.min(roll, 45); // Limit limit
            let finalRoll = turnLeft ? roll : -roll;
            
            mathStr += `\\text{Proportional Feedback Controller:}\\\\`;
            mathStr += `\\Phi_{cmd} = \\min(K_p \\cdot \\theta,\\, \\Phi_{max}) \\cdot \\operatorname{sgn}(C_z)\\\\`;
            mathStr += `\\Phi_{cmd} = ${finalRoll.toFixed(1)}^{\\circ}`;
        }
    } else {
        // ORBIT LOGIC
        const R = targetState.radius;
        
        if (currentStep >= 1) {
            const e1 = new THREE.Vector3(P_t.x, P_t.y + R, 0); const e2 = new THREE.Vector3(P_t.x + R, P_t.y, 0);
            const e3 = new THREE.Vector3(P_t.x, P_t.y - R, 0); const e4 = new THREE.Vector3(P_t.x - R, P_t.y, 0);
            [e1, e2, e3, e4].forEach(ep => {
                const s = new THREE.Mesh(new THREE.SphereGeometry(2,8,8), new THREE.MeshBasicMaterial({color: 0xffaa00}));
                s.position.copy(ep);
                scene.add(s);
                entryPointSpheres.push(s);
            });
            
            mathStr += `\\text{Search Phase - Orthogonal Entry Points:}\\\\`;
            mathStr += `E_1 = P_{center} + (0, R, 0)\\\\`;
            mathStr += `E_2 = P_{center} + (R, 0, 0)\\\\`;
            mathStr += `\\dots`;
        }
        
        if (currentStep >= 2) {
            mathStr += `\\text{Tangential Insertion Evaluation:}\\\\`;
            mathStr += `\\Psi_{ideal} = f(E_{opt},\\, \\omega_{dir})\\\\`;
            mathStr += `\\text{Orientation bounded by CW/CCW vector field.}`;
        }
        
        if (currentStep >= 3) {
            addArrow(P_t, new THREE.Vector3(-E.x, -E.y, 0).normalize(), E.length(), 0xffff00);
            let CTE = E.length() - R;
            mathStr += `\\text{Cross Track Error (CTE) Formulation:}\\\\`;
            mathStr += `D_c = |P_{drone} - P_{center}| = ${E.length().toFixed(1)}\\text{ m}\\\\`;
            mathStr += `CTE = D_c - R_{desired} = ${CTE.toFixed(1)}\\text{ m}`;
        }
        
        if (currentStep >= 4) {
            let CTE = E.length() - R;
            let rollBase = targetState.isCw ? -20 : 20;
            let prop = CTE * 0.8;
            mathStr += `\\text{Coupled Roll Equation:}\\\\`;
            mathStr += `\\Phi_{orbit} = \\Phi_{base} + (K_{rad} \\cdot CTE)\\\\`;
            mathStr += `\\Phi_{orbit} = ${rollBase} + (0.8 \\cdot ${CTE.toFixed(1)}) = ${(rollBase + prop).toFixed(1)}^{\\circ}`;
        }
        
        if (currentStep >= 5) {
            mathStr += `\\text{State: } \\mathbb{ORBIT\\_MAINTENANCE}\\\\`;
            mathStr += `\\text{Continuous integration of } \\Phi_{orbit} \\text{ to cancel radial error.}`;
        }
    }
    
    // Render with KaTeX if available
    try {
        const katex = require('katex');
        formulaDiv.innerHTML = katex.renderToString(mathStr, {
            displayMode: true,
            throwOnError: false
        });
    } catch (e) {
        formulaDiv.innerHTML = mathStr;
        console.error("KaTeX failed", e);
    }
}

