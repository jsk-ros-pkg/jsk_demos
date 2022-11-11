// --- mode ---
// 0 normal
// 1 happy
// 2 relief
// 3 intrigue
// 4 surprized
// 5 sorrow
// 6 anger
// 7 embarrassed
// 8 fear
// 9 love
// 10 wink
// 11 boring
// 12 confusion
// -------------

main();

var squareRotation = 0.0;
var pre_tear_y = 0.0;
var wait_flag = false;
var tenRad = 10 * Math.PI / 180;
var back_red = 1.0;
var back_green = 1.0;
var back_blue = 0.9;
var PT1_X = -0.5;
var PT1_Y = -0.6;
var BROW_LEN = 3;
var BROW_WID = 1.5;
var LEFT_LEN = 1.2;
var RIGHT_LEN = 1;

function main() {
  // Initialize
  var c = document.getElementById('webgl');
  c.width = window.innerWidth;
  c.height = window.innerWidth * 4 / 7;  // For Rakuten mini
  // c.height = window.innerHeight;
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
      then = now;
      const buffersTear = initBuffersTear(gl);
      if (mode < 6 || mode == 11) {
        const buffers = initBuffers(gl);
        drawScene(gl, programInfo, buffers, deltaTime, 12);
      } else if (mode == 6 || mode == 7 || mode == 8) {
        const buffersColor = initBuffersColor(gl);
        drawSceneColor(gl, programInfoColor, buffersColor, 12);
      } else if (mode == 9) {
        const buffers = initBuffers(gl);
        const buffersHeart = initBuffersHeart(gl);
        drawSceneSome(gl, programInfo, programInfoColor, buffers, buffersHeart, 12, 17);
      } else if (mode == 10) {
	const buffers = initBuffers(gl);
	const buffersStar = initBuffersStar(gl);
	drawSceneSome(gl, programInfo, programInfoColor, buffers, buffersStar, 12, 25);
      } else if (mode == 12) {
	const buffers = initBuffers(gl);
        drawSceneSome(gl, programInfo, programInfoColor, buffers, buffersTear, 12, 12);
      }
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
  var brow_len = BROW_LEN;
  var brow_wid = BROW_WID;
  var left_len = LEFT_LEN;
  var right_len = RIGHT_LEN;
  var pt1_x = PT1_X;
  var pt1_y = PT1_Y;
  if (mode == 1) {  // yorokobi
    var rad1 = 0;
    var rad2 = -1 * squareRotation * 1.5;
    var rad3 = squareRotation * 1.5;
    var rad4 = squareRotation * 1.5;
    // var brow_len = BROW_LEN + squareRotation;
    var brow_wid = BROW_WID - Math.abs(squareRotation / 2);
    var pt1_x = PT1_X;
    var pt1_y = PT1_Y + squareRotation;
    var left_len = LEFT_LEN + squareRotation;
    var right_len = RIGHT_LEN + squareRotation;
  } else if (mode == 2 || mode == 11) {  // anshin || taikutsu
    if (Math.abs(squareRotation) < tenRad * 3) {
      var rad1 = squareRotation * -0.125;
      var rad2 = squareRotation * -0.125;
      var rad3 = squareRotation * -0.125;
      var rad4 = squareRotation * -0.125;
      var pt1_x = PT1_X + squareRotation * 0.75;
      var pt1_y = PT1_Y + squareRotation * 1.5;
    } else if (Math.abs(squareRotation) < tenRad * 4) {
      var rad1 = tenRad * -0.125 * 3;
      var rad2 = tenRad * -0.125 * 3;
      var rad3 = tenRad * -0.125 * 3;
      var rad4 = tenRad * -0.125 * 3;
      var pt1_x = PT1_X + tenRad * 0.75 * 3;
      var pt1_y = PT1_Y + tenRad * 1.5 * 3;
    } else {
      var rad1 = tenRad * -0.125 * 3;
      var rad2 = tenRad * -0.125 * 3;
      var rad3 = tenRad * -0.125 * 3;
      var rad4 = tenRad * -0.125 * 3;
      var pt1_x = PT1_X + tenRad * 0.75 * 3;
      var pt1_y = PT1_Y + tenRad * 1.5 * 3 - (squareRotation - 4 * tenRad) * 1.5;
    }
    if (squareRotation > 9 * tenRad) {
      var pt1_y = PT1_Y + tenRad * 1.5 * 3 - 5 * tenRad * 1.5;
    }
  } else if (mode == 3) {  // warudakumi
    var tmp_num = squareRotation / tenRad;
    if (squareRotation <= 2 * tenRad) {
      var rad1 = 0;
      var rad2 = 0;
      var rad3 = 0;
      var rad4 = 0;
      // var brow_len = BROW_LEN - squareRotation * 0.5;
      var brow_wid = BROW_WID - squareRotation * 0.75;
      var pt1_x = PT1_X + squareRotation;
      var pt1_y = PT1_Y - squareRotation;
      var left_len = LEFT_LEN - squareRotation * 0.25;
      var right_len = RIGHT_LEN + squareRotation * 0.25;
    } else {
      // var brow_len = BROW_LEN - tenRad;
      var brow_wid = BROW_WID - tenRad * 1.5;
      var pt1_x = PT1_X + tenRad * 2 - 0.5;
      var pt1_y = PT1_Y - tenRad * 2 - 0.1;
      var left_len = LEFT_LEN - 0.5;
      var right_len = RIGHT_LEN + 0.5;
      if (wait_flag) {
        var rad1 = 0;
        var rad2 = 0;
        var rad3 = 0;
        var rad4 = 0;
     } else if (squareRotation <= 4 * tenRad) {
        var rad1 = squareRotation - 2 * tenRad;
        var rad2 = (squareRotation - 2 * tenRad) * 0.5;
        var rad3 = 0;
        var rad4 = 0;
     } else if (squareRotation <= 5 * tenRad) {
	var rad1 = 2 * tenRad;
        var rad2 = tenRad;
        var rad3 = 0;
        var rad4 = 0;
     } else if (Math.abs(maxRotation - squareRotation) > 0.1 &&
		squareRotation < 10 * tenRad) {
	if (Math.floor(tmp_num) % 2 == 1) {  // down
          var rad1 = (2 + Math.floor(tmp_num)) * tenRad - squareRotation;
          var rad2 = (1 + Math.floor(tmp_num)) * tenRad - squareRotation;
          var rad3 = 0;
          var rad4 = 0;
	} else {  // up
          var rad1 = (1 - Math.floor(tmp_num)) * tenRad + squareRotation;
	  var rad2 = (0 - Math.floor(tmp_num)) * tenRad + squareRotation;
	  var rad3 = 0;
	  var rad4 = 0;
        }
      } else if (squareRotation < 11 * tenRad) {
	  var rad1 = (1 + Math.floor(tmp_num)) * tenRad - squareRotation;
          var rad2 = (0 + Math.floor(tmp_num)) * tenRad - squareRotation;
          var rad3 = 0;
          var rad4 = 0;
      } else {
	var rad1 = tenRad * 0.5;  // 2 * tenRad;
        var rad2 = tenRad * -0.5;  // tenRad;
        var rad3 = 0;
        var rad4 = 0;
      }
    }
  } else if (mode == 4) {  // odoroki || tere
    if (Math.abs(squareRotation) < tenRad * 2) {
      var rad1 = squareRotation * -0.5;
      var rad2 = squareRotation * -0.5;
      var rad3 = squareRotation * -0.5;
      var rad4 = squareRotation * -0.5;
      var brow_len = BROW_LEN - squareRotation * 1.5;
      var pt1_x = PT1_X + squareRotation * 3;
      var pt1_y = PT1_Y + squareRotation;
    } else {
      var rad1 = tenRad * -1;
      var rad2 = tenRad * -1;
      var rad3 = tenRad * -1;
      var rad4 = tenRad * -1;
      var brow_len = BROW_LEN - tenRad * 3;
      if (wait_flag) {
        var pt1_x = Math.min(PT1_X + tenRad * 6 + (squareRotation - 2 * tenRad) * 3, PT1_X + tenRad * 12);
        var pt1_y = Math.min(PT1_Y + tenRad * 2 + (squareRotation - 2 * tenRad) * 3, PT1_Y + tenRad * 8);
      } else if (Math.abs(squareRotation) < 2.5 * tenRad) {
        var pt1_x = PT1_X + tenRad * 2;
        var pt1_y = PT1_Y;
      } else if (2.5 * tenRad < Math.abs(squareRotation)) {
        if (Math.abs(squareRotation) < tenRad * 3) {
	  var pt1_x = PT1_X + tenRad * 6 + (squareRotation - 2.5 * tenRad) * 10;
	  var pt1_y = PT1_Y + tenRad * 2 + (squareRotation - 2.5 * tenRad) * 15;
	// } else if (4 * tenRad < Math.abs(squareRotation) &&  Math.abs(squareRotation) < tenRad * 6) {
	//   var tmp_rotation = 5 * tenRad - (squareRotation - 5 * tenRad);
	//   var pt1_x = PT1_X + tenRad * 6 + (tmp_rotation - 2 * tenRad) * 3;
        //   var pt1_y = PT1_Y + tenRad * 2 + (tmp_rotation - 2 * tenRad) * 3;
	// } else if (6 * tenRad < Math.abs(squareRotation)) {
	//   var pt1_x = PT1_X + tenRad * 12;
	//   var pt1_y = PT1_Y + tenRad * 8;
	} else {
          var pt1_x = PT1_X + tenRad * 11;
          var pt1_y = PT1_Y + tenRad * 9.5;
	}
      } else {
        var pt1_x = PT1_X + tenRad * 11;
        var pt1_y = PT1_Y + tenRad * 9.5;
      }
    }
  } else if (mode == 12) {  // kanashimi
    if (squareRotation < 2 * tenRad) {
      var rad1 = squareRotation * -0.6;
      var rad2 = squareRotation * -0.6;
      var rad3 = squareRotation * -0.6;
      var rad4 = squareRotation * -0.6;
      var brow_len = BROW_LEN + squareRotation;
      // var pt1_x = PT1_X + squareRotation * 0.5;
      var pt1_y = PT1_Y + squareRotation * 2.5;
    } else {
      var rad1 = tenRad * -1.2;
      var rad2 = tenRad * -1.2;
      var rad3 = tenRad * -1.2;
      var rad4 = tenRad * -1.2;
      var brow_len = BROW_LEN + tenRad * 2;
      // var pt1_x = PT1_X + squareRotation;
      var pt1_y = PT1_Y + tenRad * 5;
    }
  } else if (mode == 9) {  // suki
    if (squareRotation < tenRad * 2) {
      var rad1 = squareRotation * -0.25;
      var rad2 = squareRotation * -0.25;
      var rad3 = squareRotation * -0.25;
      var rad4 = squareRotation * -0.25;
      var pt1_y = PT1_Y - squareRotation * 1.0;
    } else {
      var rad1 = tenRad * -0.5;
      var rad2 = tenRad * -0.5;
      var rad3 = tenRad * -0.5;
      var rad4 = tenRad * -0.5;
      var pt1_y = PT1_Y - tenRad * 2;
    }
  } else if (mode == 10) {  // wink
    var rad1 = 0;
    var rad2 = 0;
    var rad3 = 0;
    var rad4 = 0;
    if (squareRotation < tenRad * 2) {
      var pt1_y = PT1_Y + squareRotation * 3;
    } else if (squareRotation < 3.5 * tenRad) {
      var pt1_y = PT1_Y + 6 * tenRad - (squareRotation - 2 * tenRad) * 8;
    } else {
      var pt1_y = PT1_Y -6 * tenRad;
    }
  } else if (mode == 5) {  // konran
    var rad1 = 0;
    var rad2 = 0.2 * squareRotation;
    var rad3 = -1.5 * squareRotation;
    var rad4 = -1.5 * squareRotation;
    // var brow_wid = BROW_WID - Math.abs(squareRotation / 2);
    // var brow_len = BROW_LEN + squareRotation * 0.5;
    // var left_len = LEFT_LEN + squareRotation;
    var pt1_x = PT1_X - squareRotation;
    var pt1_y = PT1_Y - squareRotation;
  }
  var pt2_x = pt1_x + brow_len * Math.cos(rad1);
  var pt2_y = pt1_y + brow_len * Math.sin(rad1);
  var pt3_x = pt2_x + right_len * Math.cos(rad2);
  var pt3_y = pt2_y + right_len * Math.sin(rad2);
  var pt5_x = pt1_x - brow_wid * Math.sin(rad1);
  var pt5_y = pt1_y + brow_wid * Math.cos(rad1);
  var pt4_x = pt5_x + brow_len * Math.cos(rad1);
  var pt4_y = pt5_y + brow_len * Math.sin(rad1);
  var pt6_x = pt5_x - left_len * Math.cos(rad3);
  var pt6_y = pt5_y - left_len * Math.sin(rad3);
  var pt7_x = pt1_x - left_len * Math.cos(rad4);
  var pt7_y = pt1_y - left_len * Math.sin(rad4);
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

function initBuffersColor(gl) {

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
  var brow_len = BROW_LEN;
  var brow_wid = BROW_WID;
  var left_len = LEFT_LEN;
  var right_len = RIGHT_LEN;
  var pt1_x = PT1_X;
  var pt1_y = PT1_Y;
  if (mode == 6) {  // ikari
    var rad1 = squareRotation * 2.0;
    var rad2 = squareRotation * 1.5;
    var rad3 = squareRotation * 0.0;
    var rad4 = squareRotation * 0.0;
    // var brow_len = BROW_LEN + squareRotation;
    var right_len = RIGHT_LEN + squareRotation;
    var left_len = LEFT_LEN - squareRotation / 2;
    // var brow_wid = BROW_WID + squareRotation;
    var pt1_x = PT1_X + squareRotation;
    var pt1_y = PT1_Y - squareRotation;
  } else if (mode == 7) {  // odoroki || tere
    if (Math.abs(squareRotation) < tenRad * 2) {
      var rad1 = squareRotation * -0.5;
      var rad2 = squareRotation * -0.5;
      var rad3 = squareRotation * -0.5;
      var rad4 = squareRotation * -0.5;
      var brow_len = BROW_LEN - squareRotation * 1.5;
      var pt1_x = PT1_X + squareRotation * 3;
      var pt1_y = PT1_Y + squareRotation;
    } else {
      var rad1 = tenRad * -1;
      var rad2 = tenRad * -1;
      var rad3 = tenRad * -1;
      var rad4 = tenRad * -1;
      var brow_len = BROW_LEN - tenRad * 3;
      if (wait_flag) {
        var pt1_x = Math.min(PT1_X + tenRad * 6 + (squareRotation - 2 * tenRad) * 3, PT1_X + tenRad * 12);
        var pt1_y = Math.min(PT1_Y + tenRad * 2 + (squareRotation - 2 * tenRad) * 3, PT1_Y + tenRad * 8);
      } else if (Math.abs(squareRotation) < 2.5 * tenRad) {
        var pt1_x = PT1_X + tenRad * 2;
        var pt1_y = PT1_Y;
      } else if (2.5 * tenRad < Math.abs(squareRotation)) {
        if (Math.abs(squareRotation) < tenRad * 3) {
	  var pt1_x = PT1_X + tenRad * 6 + (squareRotation - 2.5 * tenRad) * 10;
	  var pt1_y = PT1_Y + tenRad * 2 + (squareRotation - 2.5 * tenRad) * 15;
	} else {
	  var pt1_x = PT1_X + tenRad * 11;
	  var pt1_y = PT1_Y + tenRad * 9.5;
	}
      } else {
        var pt1_x = PT1_X + tenRad * 11;
        var pt1_y = PT1_Y + tenRad * 9.5;
      }
    }
  } else if (mode == 8) {  // kyoufu
    if (squareRotation < tenRad * 2) {
      var rad1 = squareRotation * -0.5;
      var rad2 = squareRotation * -0.5;
      var rad3 = squareRotation * -1.0;
      var rad4 = squareRotation * -1.0;
      var pt1_x = PT1_X + squareRotation * 1.5;
      var pt1_y = PT1_Y + squareRotation;
      var brow_len = BROW_LEN - Math.abs(squareRotation * 0.5);
    } else if (wait_flag) {
      var rad1 = tenRad * -1;
      var rad2 = tenRad * -1;
      var rad3 = tenRad * -2;
      var rad4 = tenRad * -2;
      var pt1_x = PT1_X + tenRad * 3;
      var pt1_y = PT1_Y + tenRad * 2;
      var brow_len = BROW_LEN - tenRad;
    } else {
      var rad1 = tenRad * -1;
      var rad2 = tenRad * -1;
      var rad3 = tenRad * -2;
      var rad4 = tenRad * -2;
      var pt1_x = PT1_X + tenRad * 3;
      var brow_len = BROW_LEN - tenRad;
      if (squareRotation < 3 * tenRad) {
        var pt1_y = PT1_Y + tenRad * 2 + Math.abs(squareRotation - 2 * tenRad) * -0.5;
      } else if (squareRotation < 4 * tenRad) {
	var diff_rot = Math.abs(squareRotation - 3 * tenRad);
        var pt1_y = PT1_Y + tenRad * 2 - 0.5 * tenRad + diff_rot * 0.5;
      } else {
	var pt1_y = PT1_Y + tenRad * 2;
      }
    }
  }
  var pt2_x = pt1_x + brow_len * Math.cos(rad1);
  var pt2_y = pt1_y + brow_len * Math.sin(rad1);
  var pt3_x = pt2_x + right_len * Math.cos(rad2);
  var pt3_y = pt2_y + right_len * Math.sin(rad2);
  var pt5_x = pt1_x - brow_wid * Math.sin(rad1);
  var pt5_y = pt1_y + brow_wid * Math.cos(rad1);
  var pt4_x = pt5_x + brow_len * Math.cos(rad1);
  var pt4_y = pt5_y + brow_len * Math.sin(rad1);
  var pt6_x = pt5_x - left_len * Math.cos(rad3);
  var pt6_y = pt5_y - left_len * Math.sin(rad3);
  var pt7_x = pt1_x - left_len * Math.cos(rad4);
  var pt7_y = pt1_y - left_len * Math.sin(rad4);
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

  // add color
  var r = 0.0;
  var g = 0.0;
  var b = 0.0;
  if (mode == 6) {  // ikari
    var r = Math.min(1.0, squareRotation * 3);
  } else if (mode == 7) {  // tere
    if (squareRotation < 2.5 * tenRad) {
      var color_deg = squareRotation - tenRad * 0.5;
      var r = Math.min(1.0, color_deg * 1.5);
      var g = Math.min(1.0, color_deg * 0.8);
      var b = Math.min(1.0, color_deg * 0.8);
    } else if (squareRotation < 4.5 * tenRad) {
      var r = 3 * tenRad;
      var g = 1.6 * tenRad;
      var b = 1.6 * tenRad;
    } else if (4.5 * tenRad < squareRotation){
      var color_deg = squareRotation - tenRad * 4.5;
      var r = Math.min(1.0, 3 * tenRad + color_deg * 1.5);
      var g = Math.min(1.0, 1.6 * tenRad + color_deg * 0.8);
      var b = Math.min(1.0, 1.6 * tenRad + color_deg * 0.8);
    }
  } else if (mode == 8) {  // kyoufu
    var r = 0;
    var g = Math.min(0.2, squareRotation * 0.2);
    var b = Math.min(0.5, squareRotation * 0.7);
  }
  var colors = [
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
  ];

  const colorBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.STATIC_DRAW);

  return {
    position: positionBuffer,
    color: colorBuffer,
  };
}

function initBuffersTear(gl) {  // -> tears

  // Create a buffer for the square's positions.

  const positionBuffer = gl.createBuffer();

  // Select the positionBuffer as the one to apply buffer
  // operations to from here out.

  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);

  // Now create an array of positions for the square.
  const TEAR_LEN = 1.5;
  const TEAR_WID = 1.0;
  var tear_len = TEAR_LEN;
  var tear_wid = TEAR_WID;
  var pt1_x = PT1_X + BROW_LEN + RIGHT_LEN + 0.12;
  if (squareRotation < 1.5 * tenRad && !wait_flag) {
    var tear_len = squareRotation * TEAR_LEN / (1.5 * tenRad);
    var tear_wid = squareRotation * TEAR_WID / (1.0 * tenRad);
    var pt1_x = PT1_X + BROW_LEN + RIGHT_LEN + squareRotation / 2.5
    var pt1_y = PT1_Y + 0.15;
  } else if (squareRotation < 2.5 * tenRad && !wait_flag) {
    var pt1_y = PT1_Y + 0.15;
  } else {
    var pt1_y = pre_tear_y - 0.02;
  }
  pre_tear_y = pt1_y;
  var pt2_x = pt1_x - tear_wid / 2;
  var pt2_y = pt1_y - tear_len / 2;
  var pt3_x = pt1_x - tear_wid / 2;
  var pt3_y = pt1_y - tear_len * 3 / 4;
  var pt4_x = pt1_x - tear_wid / 4;
  var pt4_y = pt1_y - tear_len;
  var pt5_x = pt1_x + tear_wid / 4;
  var pt5_y = pt1_y - tear_len;
  var pt6_x = pt1_x + tear_wid / 2;
  var pt6_y = pt1_y - tear_len * 3 / 4;
  var pt7_x = pt1_x + tear_wid / 2;
  var pt7_y = pt1_y - tear_len / 2;
  const positions = [
      pt2_x, pt2_y,
      pt1_x, pt1_y,
      pt7_x, pt7_y,
      pt2_x, pt2_y,
      pt3_x, pt3_y,
      pt6_x, pt6_y,
      pt7_x, pt7_y,
      pt3_x, pt3_y,
      pt4_x, pt4_y,
      pt5_x, pt5_y,
      pt6_x, pt6_y,
      pt3_x, pt3_y,
  ];

  // Now pass the list of positions into WebGL to build the
  // shape. We do this by creating a Float32Array from the
  // JavaScript array, then use it to fill the current buffer.

  gl.bufferData(gl.ARRAY_BUFFER,
                new Float32Array(positions),
                gl.STATIC_DRAW);

  // add color
  var r = 0.3;
  var g = 0.3;
  var b = 1.0;
  var colors = [  // blue
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
  ];

  const colorBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.STATIC_DRAW);

  return {
    position: positionBuffer,
    color: colorBuffer,
  };
}

function initBuffersStar(gl) {

  // Create a buffer for the square's positions.

  const positionBuffer = gl.createBuffer();

  // Select the positionBuffer as the one to apply buffer
  // operations to from here out.

  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);

  // Now create an array of positions for the square.
  const CENTER_SCALE = 0.7;
  const STAR_SCALE = 1.2;
  if (!wait_flag && 2 * tenRad < squareRotation) {
    if (squareRotation < 3 * tenRad) {
      var center_x = 1.8 + squareRotation * 2;
      var center_y = 1 - (squareRotation - 2 * tenRad) * 8;
      var center_vec = squareRotation * 2;
    } else {
      var center_x = 1.8 + squareRotation * 4.2;
      var center_y = squareRotation * 1.2;
      var center_vec = squareRotation * 3;
    }
  } else {
    var center_y = 5;  // out of screen
  }
  // center
  var pt1_x = center_x + CENTER_SCALE * Math.cos(Math.PI * -0.5 + center_vec);
  var pt1_y = center_y + CENTER_SCALE * Math.sin(Math.PI * -0.5 + center_vec);
  var pt2_x = center_x + CENTER_SCALE * Math.cos(-1 * Math.PI / 10 + center_vec);
  var pt2_y = center_y + CENTER_SCALE * Math.sin(-1 * Math.PI / 10 + center_vec);
  var pt3_x = center_x + CENTER_SCALE * Math.cos(Math.PI * 3 / 10 + center_vec);
  var pt3_y = center_y + CENTER_SCALE * Math.sin(Math.PI * 3 / 10 + center_vec);
  var pt4_x = center_x + CENTER_SCALE * Math.cos(Math.PI * 7 / 10 + center_vec);
  var pt4_y = center_y + CENTER_SCALE * Math.sin(Math.PI * 7 / 10 + center_vec);
  var pt5_x = center_x + CENTER_SCALE * Math.cos(-9 * Math.PI / 10 + center_vec);
  var pt5_y = center_y + CENTER_SCALE * Math.sin(-9 * Math.PI / 10 + center_vec);
  // vertex
  var pt6_x = center_x + STAR_SCALE * Math.cos(-3 * Math.PI / 10  + center_vec);
  var pt6_y = center_y + STAR_SCALE * Math.sin(-3 * Math.PI / 10 + center_vec);
  var pt7_x = center_x + STAR_SCALE * Math.cos(Math.PI / 10 + center_vec);
  var pt7_y = center_y + STAR_SCALE * Math.sin(Math.PI / 10 + center_vec);
  var pt8_x = center_x + STAR_SCALE * Math.cos(Math.PI / 2 + center_vec);
  var pt8_y = center_y + STAR_SCALE * Math.sin(Math.PI / 2 + center_vec);
  var pt9_x = center_x + STAR_SCALE * Math.cos(Math.PI * 9 / 10 + center_vec);
  var pt9_y = center_y + STAR_SCALE * Math.sin(Math.PI * 9 / 10 + center_vec);
  var pt10_x = center_x + STAR_SCALE * Math.cos(-7 * Math.PI / 10 + center_vec);
  var pt10_y = center_y + STAR_SCALE * Math.sin(-7 * Math.PI / 10 + center_vec);
  const positions = [
      pt5_x,  pt5_y,
      pt2_x,  pt2_y,
      pt1_x,  pt1_y,
      pt5_x,  pt5_y,
      pt4_x,  pt4_y,
      pt3_x,  pt3_y,
      pt2_x,  pt2_y,
      pt5_x,  pt5_y,
      // vertex
      pt6_x,  pt6_y,
      pt1_x,  pt1_y,
      pt5_x,  pt5_y,
      pt10_x, pt10_y,
      pt4_x,  pt4_y,
      pt5_x,  pt5_y,
      pt3_x,  pt3_y,
      pt9_x,  pt9_y,
      pt4_x,  pt4_y,
      pt3_x,  pt3_y,
      pt8_x,  pt8_y,
      pt2_x,  pt2_y,
      pt3_x,  pt3_y,
      pt1_x,  pt1_y,
      pt7_x,  pt7_y,
      pt2_x,  pt2_y,
      pt1_x,  pt1_y,
  ];

  // Now pass the list of positions into WebGL to build the
  // shape. We do this by creating a Float32Array from the
  // JavaScript array, then use it to fill the current buffer.

  gl.bufferData(gl.ARRAY_BUFFER,
                new Float32Array(positions),
                gl.STATIC_DRAW);

  // add color
  var r = 1.0;
  var g = 0.8;
  var b = 0.0;
  var colors = [  // yellow
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    // vertex
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
    r,  g,  b,  1.0,
  ];

  const colorBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.STATIC_DRAW);

  return {
    position: positionBuffer,
    color: colorBuffer,
  };
}

function initBuffersHeart(gl) {

  // Create a buffer for the square's positions.

  const positionBuffer = gl.createBuffer();

  // Select the positionBuffer as the one to apply buffer
  // operations to from here out.

  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);

  // Now create an array of positions for the square.
  var HEART_WID = 3.2;
  var HEART_HEI = 2.8;
  // center
  var CENTER_X = 2.1;
  var pt1_x = CENTER_X;
  var pt1_y = 5.0;
  if (!wait_flag && squareRotation > 2 * tenRad) {
    var diff_rad = squareRotation - 2 * tenRad;
    var HEART_WID = 1.5 + diff_rad * 1;
    var HEART_HEI = 1.5 + diff_rad * 1;
    var pt1_y = squareRotation * 2.5 - 1.5;
    if (squareRotation < 5 * tenRad) {
      var diff_rad = squareRotation - 2 * tenRad;
      var pt1_x = diff_rad * 2 + CENTER_X;
    } else if (squareRotation < 8 * tenRad) {
      var diff_rad = squareRotation - 5 * tenRad;
      var pt1_x = 6 * tenRad - diff_rad * 2.5 + CENTER_X;
    } else if (squareRotation < 11 * tenRad) {
      var diff_rad = squareRotation - 8 * tenRad;
      var pt1_x = -1.5 * tenRad + diff_rad * 3 + CENTER_X;
    } else {
      var diff_rad = squareRotation - 11 * tenRad;
      var pt1_x = 7.5 * tenRad - diff_rad * 2.5 + CENTER_X;
    }
  }
  var pt2_x = pt1_x + HEART_WID / 2;
  var pt2_y = pt1_y + HEART_HEI / 2;
  var pt3_x = pt1_x + HEART_WID / 2;
  var pt3_y = pt1_y + HEART_HEI * 3 / 4;
  var pt4_x = pt1_x + HEART_WID * 2 / 5;
  var pt4_y = pt1_y + HEART_HEI * 9 / 10;
  var pt5_x = pt1_x + HEART_WID / 5;
  var pt5_y = pt1_y + HEART_HEI - 0.05;
  var pt6_x = pt1_x;
  var pt6_y = pt1_y + HEART_HEI * 5 / 6;
  var pt7_x = pt1_x - HEART_WID / 5;
  var pt7_y = pt1_y + HEART_HEI - 0.05;
  var pt8_x = pt1_x - HEART_WID * 2 / 5;
  var pt8_y = pt1_y + HEART_HEI * 9 / 10;
  var pt9_x = pt1_x - HEART_WID / 2;
  var pt9_y = pt1_y + HEART_HEI * 3 / 4;
  var pt10_x = pt1_x - HEART_WID / 2;
  var pt10_y = pt1_y + HEART_HEI / 2;
  const positions = [
      pt10_x, pt10_y,
      pt2_x,  pt2_y,
      pt1_x,  pt1_y,
      pt10_x, pt10_y,
      pt9_x,  pt9_y,
      pt3_x,  pt3_y,
      pt2_x,  pt2_y,
      pt10_x, pt10_y,
      pt6_x,  pt6_y,
      pt7_x,  pt7_y,
      pt8_x,  pt8_y,
      pt9_x,  pt9_y,
      pt6_x,  pt6_y,
      pt3_x,  pt3_y,
      pt4_x,  pt4_y,
      pt5_x,  pt5_y,
      pt6_x,  pt6_y,
  ];

  // Now pass the list of positions into WebGL to build the
  // shape. We do this by creating a Float32Array from the
  // JavaScript array, then use it to fill the current buffer.

  gl.bufferData(gl.ARRAY_BUFFER,
                new Float32Array(positions),
                gl.STATIC_DRAW);

  // add color
  var C = 0.9;
  var colors = [  // red
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
    C,  0.0,  0.0,  1.0,
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
function drawScene(gl, programInfo, buffers, deltaTime, vertexCount) {
  // black:0, white:1
  gl.clearColor(back_red, back_green, back_blue, 1.0);  // Clear to black, fully opaque
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
    gl.drawArrays(gl.TRIANGLE_STRIP, offset, vertexCount);
  }
}

function drawSceneSome(gl, programInfo, programInfoAttach, buffers, buffersAttach, vertexCount, vertexCountAttach) {
  // black:0, white:1
  gl.clearColor(back_red, back_green, back_blue, 1.0);  // Clear to black, fully opaque
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

function drawSceneColor(gl, programInfoAttach, buffersAttach, vertexCount) {
  // black:0, white:1
  gl.clearColor(back_red, back_green, back_blue, 1.0);  // Clear to black, fully opaque
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
    gl.drawArrays(gl.TRIANGLE_STRIP, offset, vertexCount);
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
      date = new Date();
      current_time = formattedDateTime(date);
      current_date = current_time.split(',')[0];
      current_ms = parseInt(current_time.split(',')[1], 10);
      // console.log("date: ", current_date, posted_date);
      // console.log("current_time: ", current_ms);
      // console.log("reflect_time" , reflect_ms);
      if ((current_date == posted_date) && (current_ms > reflect_ms)) {
        new_mode = tmp_mode;
        if (new_mode == mode) {
	  maxRotation = new_maxRotation;
        } else {
          maxRotation = 0;
          wait_flag = true;
        }
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
