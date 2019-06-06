let gl;
let canvas;
let legacygl;
let drawutil;
let camera;
let moving = false;
let linkages = [
  { angle : 2, length : 0.8 },
  { angle : 9, length : 0.9 },
  { angle : 10, length : 1.5 },
  { angle : 50, length : 0.7 },
];
let is_dragging = false;

function mouseauto() {
  moving = true;
  let count = 0;
  let r = 0.2;
  let id = setInterval(function(){
    count++;

    let t = count / 10;
    compute_ik([r * (t - Math.sin(t)), r * 5 * (1 - Math.cos(t))]);
    draw();
    if (count > 200) {
      clearInterval(id);
      moving = false;
      document.getElementById("anime").disabled = false;
    }
  }, 30);
};

function update_position() {
  linkages.forEach(function(linkage, index){
    linkage.position = [0, 0];
    let angle_sum = 0;
    for (let j = 0; j <= index; ++j) {
      angle_sum += linkages[j].angle;
      linkage.position[0] += linkages[j].length * Math.cos(angle_sum * Math.PI / 180);
      linkage.position[1] += linkages[j].length * Math.sin(angle_sum * Math.PI / 180);
    }
  });
};

// Cyclic Coodininate Descent
function ccd(target_position) {
  for (let i = linkages.length - 2; i >= -1; i--) {
    let last = linkages[linkages.length-1];
    let lasttoe = new THREE.Vector2(last.position[0], last.position[1]);
    let target = new THREE.Vector2(target_position[0], target_position[1]);

    let cur;
    if (i == -1)
      cur = new THREE.Vector2(0, 0);
    else
      cur = new THREE.Vector2(linkages[i].position[0], linkages[i].position[1]);

    let toecur = lasttoe.sub(cur);
    let tarcur = target.sub(cur);

    let toean = toecur.angle()*(180/Math.PI);
    let taran = tarcur.angle()*(180/Math.PI);
    linkages[i+1].angle += taran - toean;
    update_position();
  }
};

// Particle IK
function pik(target_position) {
  let target = new THREE.Vector2(target_position[0], target_position[1]);
  let root = new THREE.Vector2(0, 0);
  let particles = [];
  linkages.forEach(function(linkage, index){
    particles.push(new THREE.Vector2(linkage.position[0], linkage.position[1]));
  });

  for (let it = 0; it < 1000; it++) {
    let dv = target.clone();
    dv.sub(particles[particles.length - 1]);
    particles[particles.length - 1].add(dv);

    for (let pid = particles.length - 1; pid > 0; pid--) {
      dv = particles[pid - 1].clone();
      dv.sub(particles[pid]);
      dv.multiplyScalar(0.5 - linkages[pid].length/dv.length() * 0.5);
      particles[pid].add(dv);
      particles[pid - 1].sub(dv);
    }

    dv = root.clone();
    dv.sub(particles[0]);
    dv.multiplyScalar(1.0 - linkages[0].length / dv.length());
    particles[0].add(dv);
  }

  let angles = [];
  for (let i = 0; i < linkages.length; i++) {
    let cur = particles[i].clone();
    if (i == 0) {
      let prev = root.clone();
      let vec = cur.sub(prev);
      let angle = vec.angle()*(180/Math.PI);
      angles.push(angle);
      linkages[i].angle = angle;
    } else {
      let prev = particles[i-1];
      let vec = cur.sub(prev);
      let shita = angles[i-1];
      let fai = vec.angle()*(180/Math.PI);
      angles.push(fai);
      let angle;
      if (fai > shita)
        angle = fai - shita;
      else
        angle = fai + (360 - shita);
      linkages[i].angle = angle;
    }
    update_position();
  }
};

// Jacobian IK
function jacob(target_position) {
  let angles = [];
  let l = [];
  for (let i in linkages) {
    angles.push(linkages[i].angle/(180/Math.PI));
    l.push(linkages[i].length);
  }

  let	d1x = -l[0]*Math.sin(angles[0]) - l[1]*Math.sin(angles[0]+angles[1])
    - l[1]*Math.sin(angles[0]+angles[1]+angles[2])-l[3]*Math.sin(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d2x = - l[1]*Math.sin(angles[0]+angles[1]) - l[1]*Math.sin(angles[0]+angles[1]+angles[2])
    -l[3]*Math.sin(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d3x = - l[1]*Math.sin(angles[0]+angles[1]+angles[2])-l[3]*Math.sin(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d4x = -l[3]*Math.sin(angles[0]+angles[1]+angles[2]+angles[3])

  let	d1y = l[0]*Math.cos(angles[0]) + l[1]*Math.cos(angles[0]+angles[1]) + l[1]*Math.cos(angles[0]+angles[1]+angles[2])+l[3]*Math.cos(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d2y = l[1]*Math.cos(angles[0]+angles[1]) + l[1]*Math.cos(angles[0]+angles[1]+angles[2])+l[3]*Math.cos(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d3y = l[1]*Math.cos(angles[0]+angles[1]+angles[2])+l[3]*Math.cos(angles[0]+angles[1]+angles[2]+angles[3]);
  let	d4y = l[3]*Math.cos(angles[0]+angles[1]+angles[2]+angles[3])

  let J = m4th.matrix(2, [
    d1x, d2x, d3x, d4x,
    d1y, d2y, d3y, d4y
  ]);

  let jInv = J.transp().mult(m4th.lu(J.mult(J.transp())).getInverse());
  let target = m4th.matrix(2, [target_position[0], target_position[1]]);
  let cur = m4th.matrix(2,[linkages[linkages.length - 1].position[0], linkages[linkages.length - 1].position[1]]);
  let d = jInv.mult(target.minus(cur));

  for(var i = 0; i < angles.length; i++) {
    linkages[i].angle = (angles[i] + d.times(.1).get(i,0))*(180/Math.PI);
  }

  update_position();
};

function compute_ik(target_position) {
  if (document.getElementById("ccd").checked)
     ccd(target_position);
  else if (document.getElementById("pik").checked)
    pik(target_position);
  else if (document.getElementById("jacob").checked)
    jacob(target_position);
};

function draw() {
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  // projection & camera position
  mat4.perspective(legacygl.uniforms.projection.value, Math.PI / 6, canvas.aspect_ratio(), 0.1, 1000);
  let modelview = legacygl.uniforms.modelview;
  camera.lookAt(modelview.value);

  // xy grid
  gl.lineWidth(1);
  legacygl.color(0.5, 0.5, 0.5);
  drawutil.xygrid(100);

  // linkages
  let selected = Number(document.getElementById("input_selected").value);
  legacygl.begin(gl.LINES);
  linkages.forEach(function(linkage, index){
    if (index == selected)
      legacygl.color(1, 0, 0);
    else
      legacygl.color(0, 0, 0);
    if (index == 0)
      legacygl.vertex(0, 0, 0);
    else
      legacygl.vertex2(linkages[index - 1].position);
    legacygl.vertex2(linkage.position);
  });
  legacygl.end();
  legacygl.begin(gl.POINTS);
  legacygl.color(0, 0, 0);
  legacygl.vertex(0, 0, 0);
  linkages.forEach(function(linkage, index){
    if (index == selected)
      legacygl.color(1, 0, 0);
    else
      legacygl.color(0, 0, 0);
    legacygl.vertex2(linkage.position);
  });
  legacygl.end();
};

function mousemove(mousepos) {
  let mouse_win = mousepos;
  if (camera.is_moving()) {
    camera.move(mouse_win);
    draw();
    return;
  }
  if (!is_dragging) return;
  let viewport = [0, 0, canvas.width, canvas.height];
  mouse_win.push(1);
  let mouse_obj = glu.unproject(mouse_win, 
    legacygl.uniforms.modelview.value,
    legacygl.uniforms.projection.value,
    viewport);
  // just reuse the same code as the 3D case
  let plane_origin = [0, 0, 0];
  let plane_normal = [0, 0, 1];
  let eye_to_mouse = numeric.sub(mouse_obj, camera.eye);
  let eye_to_origin = numeric.sub(plane_origin, camera.eye);
  let s1 = numeric.dot(eye_to_mouse, plane_normal);
  let s2 = numeric.dot(eye_to_origin, plane_normal);
  let eye_to_intersection = numeric.mul(s2 / s1, eye_to_mouse);
  let target_position = numeric.add(camera.eye, eye_to_intersection);
  compute_ik(target_position);
  draw();
  document.getElementById("input_selected").onchange();
};

function init() {
  // OpenGL context
  canvas = document.getElementById("canvas");
  gl = canvas.getContext("experimental-webgl");
  if (!gl)
    alert("Could not initialise WebGL, sorry :-(");
  let vertex_shader_src = "\
  attribute vec3 a_vertex;\
  attribute vec3 a_color;\
  varying vec3 v_color;\
  uniform mat4 u_modelview;\
  uniform mat4 u_projection;\
  void main(void) {\
    gl_Position = u_projection * u_modelview * vec4(a_vertex, 1.0);\
    v_color = a_color;\
    gl_PointSize = 5.0;\
  }\
  ";
  let fragment_shader_src = "\
  precision mediump float;\
  varying vec3 v_color;\
  void main(void) {\
    gl_FragColor = vec4(v_color, 1.0);\
  }\
  ";
  legacygl = get_legacygl(gl, vertex_shader_src, fragment_shader_src);
  legacygl.add_uniform("modelview", "Matrix4f");
  legacygl.add_uniform("projection", "Matrix4f");
  legacygl.add_vertex_attribute("color", 3);
  legacygl.vertex2 = function(p) {
    this.vertex(p[0], p[1], 0);
  };
  drawutil = get_drawutil(gl, legacygl);
  camera = get_camera(canvas.width);
  camera.center = [2, 0, 0];
  camera.eye = [2, 0, 7];
  update_position();
  // event handlers
  canvas.onmousedown = function(evt) {
    let mouse_win = this.get_mousepos(evt);
    if (evt.altKey) {
      camera.start_moving(mouse_win, evt.shiftKey ? "zoom" : "pan");
      return;
    }
    is_dragging = true;
  };
  canvas.onmousemove = function(evt) {
    let mouse_win = this.get_mousepos(evt);
    mousemove(mouse_win);
  }
  document.onmouseup = function (evt) {
    if (camera.is_moving()) {
      camera.finish_moving();
      return;
    }
    is_dragging = false;
  };
  document.getElementById("input_selected").max = linkages.length - 1;
  document.getElementById("input_selected").onchange = function(){
    document.getElementById("input_angle").value = linkages[this.value].angle;
    draw();
  };
  document.getElementById("input_angle").onchange = function(){
    let selected = document.getElementById("input_selected").value;
    linkages[selected].angle = Number(document.getElementById("input_angle").value);
    update_position();
    draw();
  };
  document.getElementById("anime").onclick = function(){
    if (!moving) {
      mouseauto();
      document.getElementById("anime").disabled = true;
    }
  }
  // init OpenGL settings
  gl.viewport(0, 0, canvas.width, canvas.height);
  gl.clearColor(1, 1, 1, 1);
};
