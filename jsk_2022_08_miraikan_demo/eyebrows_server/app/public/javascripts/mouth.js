main();

var squareRotation = 0.0;
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

  // Fragment shader program

  const fsSource = `
    void main() {
      gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
  `;

  // Initialize a shader program; this is where all the lighting
  // for the vertices and so forth is established.
  const shaderProgram = initShaderProgram(gl, vsSource, fsSource);

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

  // Here's where we call the routine that builds all the
  // objects we'll be drawing.
  // const buffers = initBuffers(gl);

  // Draw the scene
  var then = 0;
  function render(now) {
	now *= 0.001;  // convert to seconds
	const deltaTime = now - then;
        const buffers = initBuffers(gl);
	then = now;
	drawScene(gl, programInfo, buffers, deltaTime);
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


  const rad = squareRotation;
  const MOUTH_LEN = 2;
  const MOUTH_WID = 0.6;
  const ADD_LEN = 1.2;
  var pt1_x = 0.0;  //2.5;
  var pt1_y = 0.0;  //-0.75;
  var pt2_x = ADD_LEN * Math.cos(rad);
  var pt2_y = ADD_LEN * Math.sin(rad);
  var pt3_x = pt2_x;
  var pt3_y = pt2_y + MOUTH_WID;
  var pt4_x = pt1_x;
  var pt4_y = pt1_y + MOUTH_WID;
  var pt5_x = pt4_x - MOUTH_LEN;
  var pt5_y = pt4_y;
  var pt6_x = pt5_x - ADD_LEN * Math.cos(rad);
  var pt6_y = pt5_y + ADD_LEN * Math.sin(rad);
  var pt7_x = pt6_x;
  var pt7_y = ADD_LEN * Math.sin(rad);
  var pt8_x = pt1_x - MOUTH_LEN;
  var pt8_y = pt1_y;
  const positions = [
       pt1_x, pt1_y,
       pt2_x, pt2_y,
       pt3_x, pt3_y,
       pt4_x, pt4_y,
       pt1_x, pt1_y,
       pt8_x, pt8_y,
       pt5_x, pt5_y,
       pt6_x, pt6_y,
       pt7_x, pt7_y,
       pt8_x, pt8_y,
       pt4_x, pt4_y,
       pt5_x, pt5_y,
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

//
// Draw the scene.
//
function drawScene(gl, programInfo, buffers, deltaTime) {
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
                 [1.0, -0.3, -6.0]);  // amount to translate
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
    const vertexCount = 12;
    gl.drawArrays(gl.TRIANGLE_STRIP, offset, vertexCount);
  }

  // Update the rotation for the next draw
  if (Math.abs(squareRotation - maxRotation) > 0.01) {
    if (squareRotation < maxRotation) {
      squareRotation += deltaTime * 0.5;
    } else if (squareRotation > maxRotation) {
      squareRotation -= deltaTime * 0.5;
    }
  } else {
    get_degree();
  }
  // console.log(squareRotation, maxRotation);

  function playerClick (evt) {
    var position = [
      evt.pageX - evt.target.offsetLeft,
      gl.drawingBufferHeight - (evt.pageY - evt.target.offsetTop),
    ];
    console.log(position);
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

