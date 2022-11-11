main();

var squareRotation = 0.0;
var wait_flag = false;
function main() {
  // Initialize
  var c = document.getElementById('webgl');
  c.width = window.innerWidth;
  c.height = window.innerHeight;
  var gl = c.getContext('webgl');

  // Vertex shader program

  const vsSource = `
    attribute vec4 aVertexPosition;

    uniform mat4 uModelViewMatrix;
    uniform mat4 uProjectionMatrix;

    void main() {
      gl_Position = uProjectionMatrix * uModelViewMatrix * aVertexPosition;
    }
  `;

  const vsSourceColor = `
    attribute vec4 aVertexPosition;
    attribute vec4 aVertexColor;
    uniform mat4 uModelViewMatrix;
    uniform mat4 uProjectionMatrix;
    varying lowp vec4 vColor;
    void main(void) {
      gl_Position = uProjectionMatrix * uModelViewMatrix * aVertexPosition;
      vColor = aVertexColor;
    }
  `;

  // Fragment shader program

  const fsSource = `
    void main() {
      gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
  `;

  const fsSourceColor = `
    varying lowp vec4 vColor;
    void main(void) {
      gl_FragColor = vColor;
    }
  `;

  // Initialize a shader program; this is where all the lighting
  // for the vertices and so forth is established.
  const shaderProgram = initShaderProgram(gl, vsSource, fsSource);
  const shaderProgramColor = initShaderProgram(gl, vsSourceColor, fsSourceColor);

  // Collect all the info needed to use the shader program.
  // Look up which attribute our shader program is using
  // for aVertexPosition and look up uniform locations.
  const programInfo = {
    program: shaderProgram,
    attribLocations: {
      vertexPosition: gl.getAttribLocation(shaderProgram, 'aVertexPosition'),
    },
    uniformLocations: {
      projectionMatrix: gl.getUniformLocation(shaderProgram, 'uProjectionMatrix'),
      modelViewMatrix: gl.getUniformLocation(shaderProgram, 'uModelViewMatrix'),
    },
  };
  // Color object
  const programInfoColor = {
    program: shaderProgramColor,
    attribLocations: {
      vertexPosition: gl.getAttribLocation(shaderProgramColor, 'aVertexPosition'),
      vertexColor: gl.getAttribLocation(shaderProgramColor, 'aVertexColor'),
    },
    uniformLocations: {
      projectionMatrix: gl.getUniformLocation(shaderProgramColor, 'uProjectionMatrix'),
      modelViewMatrix: gl.getUniformLocation(shaderProgramColor, 'uModelViewMatrix'),
    },
  };

  // Here's where we call the routine that builds all the
  // objects we'll be drawing.
  // const buffers = initBuffers(gl);

  // Draw the scene
  var then = 0;
  function render(now) {
	now *= 0.001;  // convert to seconds
	const deltaTime = now - then;
        const buffers = initBuffers(gl);
        const buffersAttach = initBuffersAttach(gl);
	then = now;
        drawScene(gl, programInfo, programInfoColor, buffers, buffersAttach, 12, 4);
        update(deltaTime);
	requestAnimationFrame(render);
  }
  requestAnimationFrame(render);
}

//
// initBuffers
//
// Initialize the buffers we'll need. For this demo, we just
// have one object -- a simple two-dimensional square.
//
function initBuffers(gl) {

  // Create a buffer for the square's positions.

  const positionBuffer = gl.createBuffer();

  // Select the positionBuffer as the one to apply buffer
  // operations to from here out.

  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);

  // Now create an array of positions for the square.

  var rad1 = squareRotation;
  var rad2 = squareRotation;
  var rad3 = squareRotation;
  var rad4 = squareRotation;
  var BROW_LEN = 3;
  var BROW_WID = 1.5;
  var LEFT_LEN = 1;
  var RIGHT_LEN = 1;
  var pt1_x = 0.0;
  var pt1_y = 0.0;
  if (mode == 1) {
    var rad1 = 0;
    var rad2 = -1 * squareRotation;
    var rad3 = squareRotation;
    var rad4 = squareRotation;
    var BROW_WID = 1.5 - Math.abs(squareRotation / 2);
    var pt1_x = 0.0;
    var pt1_y = squareRotation;
  }
  var pt2_x = pt1_x + BROW_LEN * Math.cos(rad1);
  var pt2_y = pt1_y + BROW_LEN * Math.sin(rad1);
  var pt3_x = pt2_x + RIGHT_LEN * Math.cos(rad2);
  var pt3_y = pt2_y + RIGHT_LEN * Math.sin(rad2);
  var pt5_x = pt1_x - BROW_WID * Math.sin(rad1);
  var pt5_y = pt1_y + BROW_WID * Math.cos(rad1);
  var pt4_x = pt5_x + BROW_LEN * Math.cos(rad1);
  var pt4_y = pt5_y + BROW_LEN * Math.sin(rad1);
  var pt6_x = pt5_x - LEFT_LEN * Math.cos(rad3);
  var pt6_y = pt5_y - LEFT_LEN * Math.sin(rad3);
  var pt7_x = pt1_x - LEFT_LEN * Math.cos(rad4);
  var pt7_y = pt1_y - LEFT_LEN * Math.sin(rad4);
  const positions = [
      pt2_x, pt2_y,
      pt3_x, pt3_y,
      pt4_x, pt4_y,
      pt2_x, pt2_y,
      pt1_x, pt1_y,
      pt5_x, pt5_y,
      pt4_x, pt4_y,
      pt1_x, pt1_y,
      pt5_x, pt5_y,
      pt6_x, pt6_y,
      pt7_x, pt7_y,
      pt1_x, pt1_y,
  ];

  // Now pass the list of positions into WebGL to build the
  // shape. We do this by creating a Float32Array from the
  // JavaScript array, then use it to fill the current buffer.

  gl.bufferData(gl.ARRAY_BUFFER,
                new Float32Array(positions),
                gl.STATIC_DRAW);

  return {
    position: positionBuffer,
  };
}

function initBuffersAttach(gl) {

  // Create a buffer for the square's positions.

  const positionBuffer = gl.createBuffer();

  // Select the positionBuffer as the one to apply buffer
  // operations to from here out.

  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);

  // Now create an array of positions for the square.
  const positions = [
     1.0,  1.0,
    -1.0,  1.0,
     1.0, -1.0,
    -1.0, -1.0,
  ];

  // Now pass the list of positions into WebGL to build the
  // shape. We do this by creating a Float32Array from the
  // JavaScript array, then use it to fill the current buffer.

  gl.bufferData(gl.ARRAY_BUFFER,
                new Float32Array(positions),
                gl.STATIC_DRAW);

  // add color
  var colors = [
    1.0,  1.0,  1.0,  1.0,    // white
    1.0,  0.0,  0.0,  1.0,    // red
    0.0,  1.0,  0.0,  1.0,    // green
    0.0,  0.0,  1.0,  1.0,    // blue
  ];

  const colorBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.STATIC_DRAW); 

  return {
    position: positionBuffer,
    color: colorBuffer,  
  };
}

//
// Draw the scene.
//
function drawScene(gl, programInfo, programInfoAttach, buffers, buffersAttach, vertexCount, vertexCountAttach) {
  // black:0, white:1
  gl.clearColor(0.9, 0.9, 0.9, 1.0);  // Clear to black, fully opaque
  gl.clearDepth(1.0);                 // Clear everything
  gl.enable(gl.DEPTH_TEST);           // Enable depth testing
  gl.depthFunc(gl.LEQUAL);            // Near things obscure far things

  // Clear the canvas before we start drawing on it.

  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  // Create a perspective matrix, a special matrix that is
  // used to simulate the distortion of perspective in a camera.
  // Our field of view is 45 degrees, with a width/height
  // ratio that matches the display size of the canvas
  // and we only want to see objects between 0.1 units
  // and 100 units away from the camera.
  const mat4 = glMatrix.mat4

  const fieldOfView = 45 * Math.PI / 180;   // in radians
  const aspect = gl.canvas.clientWidth / gl.canvas.clientHeight;
  const zNear = 0.1;
  const zFar = 100.0;
  const projectionMatrix = mat4.create();
  
  // note: glmatrix.js always has the first argument
  // as the destination to receive the result.
  mat4.perspective(projectionMatrix,
                   fieldOfView,
                   aspect,
                   zNear,
                   zFar);

  // Set the drawing position to the "identity" point, which is
  // the center of the scene.
  const modelViewMatrix = mat4.create();

  // Now move the drawing position a bit to where we want to
  // start drawing the square.

  mat4.translate(modelViewMatrix,     // destination matrix
                 modelViewMatrix,     // matrix to translate
                 [-1.5, -0.5, -6.0]);  // amount to translate
  // mat4.rotate(modelViewMatrix,        // destination matrix
  //  	      modelViewMatrix,        // matrix to rotate
  //  	      squareRotation,         // amount to rotate in radians
  // 	      [0, 0, -1]);             // axis to rotate around

  // Tell WebGL how to pull out the positions from the position
  // buffer into the vertexPosition attribute.
  {
    const numComponents = 2;
    const type = gl.FLOAT;
    const normalize = false;
    const stride = 0;
    const offset = 0;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffers.position);
    gl.vertexAttribPointer(
        programInfo.attribLocations.vertexPosition,
        numComponents,
        type,
        normalize,
        stride,
        offset);
    gl.enableVertexAttribArray(
        programInfo.attribLocations.vertexPosition);
  }

  // Tell WebGL to use our program when drawing

  gl.useProgram(programInfo.program);

  // Set the shader uniforms

  gl.uniformMatrix4fv(
      programInfo.uniformLocations.projectionMatrix,
      false,
      projectionMatrix);
  gl.uniformMatrix4fv(
      programInfo.uniformLocations.modelViewMatrix,
      false,
      modelViewMatrix);
  {
    const offset = 0;
    // const vertexCount = 12;
    gl.drawArrays(gl.TRIANGLE_STRIP, offset, vertexCount);
  }

  // draw other object
  {
    const numComponents = 2;
    const type = gl.FLOAT;
    const normalize = false;
    const stride = 0;
    const offset = 0;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffersAttach.position);
    gl.vertexAttribPointer(
        programInfoAttach.attribLocations.vertexPosition,
        numComponents,
        type,
        normalize,
        stride,
        offset);
    gl.enableVertexAttribArray(
        programInfoAttach.attribLocations.vertexPosition);
  }

  // Tell WebGL how to pull out the colors from the color buffer
  // into the vertexColor attribute.
  {
    const numComponents = 4;
    const type = gl.FLOAT;
    const normalize = false;
    const stride = 0;
    const offset = 0;
    gl.bindBuffer(gl.ARRAY_BUFFER, buffersAttach.color);
    gl.vertexAttribPointer(
        programInfoAttach.attribLocations.vertexColor,
        numComponents,
        type,
        normalize,
        stride,
        offset);
    gl.enableVertexAttribArray(
        programInfoAttach.attribLocations.vertexColor);
  }

  // Tell WebGL to use our program when drawing

  gl.useProgram(programInfoAttach.program);

  // Set the shader uniforms

  gl.uniformMatrix4fv(
      programInfoAttach.uniformLocations.projectionMatrix,
      false,
      projectionMatrix);
  gl.uniformMatrix4fv(
      programInfoAttach.uniformLocations.modelViewMatrix,
      false,
      modelViewMatrix);
  {
    const offset = 0;
    // const vertexCount = vertexCountAttach;
    gl.drawArrays(gl.TRIANGLE_STRIP, offset, vertexCountAttach);
  }
}

function update(deltaTime) {
  // Update the rotation for the next draw
  if (Math.abs(squareRotation - maxRotation) > 0.01) {
    if (squareRotation < maxRotation) {
      squareRotation += deltaTime * 0.5;
    } else if (squareRotation > maxRotation) {
      squareRotation -= deltaTime * 0.5;
    }
  } else {
    if (wait_flag) {
      mode = new_mode;
      maxRotation = new_maxRotation;
      wait_flag = false;
    } else {
      get_degree();
      if (new_mode == mode) {
	maxRotation = new_maxRotation;
      } else {
        maxRotation = 0;
        wait_flag = true;
      }
    }
  }
}

//
// Initialize a shader program, so WebGL knows how to draw our data
//
function initShaderProgram(gl, vsSource, fsSource) {
  const vertexShader = loadShader(gl, gl.VERTEX_SHADER, vsSource);
  const fragmentShader = loadShader(gl, gl.FRAGMENT_SHADER, fsSource);

  // Create the shader program

  const shaderProgram = gl.createProgram();
  gl.attachShader(shaderProgram, vertexShader);
  gl.attachShader(shaderProgram, fragmentShader);
  gl.linkProgram(shaderProgram);

  // If creating the shader program failed, alert

  if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
    alert('Unable to initialize the shader program: ' + gl.getProgramInfoLog(shaderProgram));
    return null;
  }

  return shaderProgram;
}

//
// creates a shader of the given type, uploads the source and
// compiles it.
//
function loadShader(gl, type, source) {
  const shader = gl.createShader(type);

  // Send the source to the shader object

  gl.shaderSource(shader, source);

  // Compile the shader program

  gl.compileShader(shader);

  // See if it compiled successfully

  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    alert('An error occurred compiling the shaders: ' + gl.getShaderInfoLog(shader));
    gl.deleteShader(shader);
    return null;
  }

  return shader;
}
