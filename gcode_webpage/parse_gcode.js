let units_to_m = 0.001;

// let gcode_translate = { X: 2, Y: 0 }, gcode_scale = 2;
// let gcode_translate = { X: 2.45, Y: 0.9 }, gcode_scale = 1.0;
// let gcode_translate = { X: BOUNDS.X[0], Y: BOUNDS.Y[0] }, gcode_scale_x = 1.0, gcode_scale_y = 1.0;
let gcode_translate = { X: 1.9667, Y: 0.6685 }, gcode_scale_x = 1.0, gcode_scale_y = 1.0;
let homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

function updateValue(id, target, message, checkPositive = false) {
  let value = parseFloat(document.getElementById(id).value);
  if (!isNaN(value) && (!checkPositive || value > 0)) {
    return value;
  } else {
      alert(`Please enter a valid ${checkPositive ? "positive " : ""}number for ${message}.`);
      return target;
  }
}
function updateTranslateX() { gcode_translate.X = updateValue('translateX', gcode_translate.X, 'X translation'); }
function updateTranslateY() { gcode_translate.Y = updateValue('translateY', gcode_translate.Y, 'Y translation'); }
function updateScaleX() { gcode_scale_x = updateValue('scaleValueX', gcode_scale_x, 'ScaleX', true); }
function updateScaleY() { gcode_scale_y = updateValue('scaleValueY', gcode_scale_y, 'ScaleY', true); }
document.getElementById('translateX').value = gcode_translate.X;
document.getElementById('translateY').value = gcode_translate.Y;
document.getElementById('scaleValueX').value = gcode_scale_x;
document.getElementById('scaleValueY').value = gcode_scale_y;
function updateHomography() {
  let value = JSON.parse(document.getElementById('homography').value);
  if (value.length != 3 || value[0].length != 3 || value[1].length != 3 || value[2].length != 3) {
    alert('Please enter a valid 3x3 matrix for the homography.');
    return;
  }
  homography = value;
}
document.getElementById('homography').value = JSON.stringify(homography);

CALLBACKS_READABLE = {
  undefined: (args) => '',
  G0: (args) => 'moveTo ' + JSON.stringify(args),
  G1: (args) => 'lineTo ' + JSON.stringify(args),
  G20: (args) => { units_to_m = 0.0254; return 'set units to inches!  Doing implicit conversion to meters.' },
  G21: (args) => { units_to_m = 0.001; return 'set units to millimeters! ' },
  G90: (args) => 'Absolute positioning, this is good and expected.',
  G91: (args) => '**** CANNOT HANDLE RELATIVE POSITIONING YET!!!! ****',
  G92: (args) => 'TODO(gerry): set current position - a zeroing-type operation.',
  M2: (args) => 'End of program :)',
  default: (cmd, args, raw) => `UNKNOWN command: ${cmd} with args ${args}.  ${raw}`
}
CALLBACKS_CDPR_COMMAND = {
  undefined: (args) => '', // empty line
  G0: (args) => `M0,${args.X},${args.Y}`,
  G1: (args) => `L0,${args.X},${args.Y}`,
  G20: (args) => { units_to_m = 0.0254; return '' }, // inch units
  G21: (args) => { units_to_m = 0.001; return '' },
  G90: (args) => '',
  G91: (args) => '**** CANNOT HANDLE RELATIVE POSITIONING YET!!!! ****',
  G92: (args) => '**** TODO(gerry): set current position - a zeroing-type operation ****',
  M2: (args) => '',
  default: (cmd, args, raw) => `**** UNKNOWN command: ${cmd} with args ${args}.  ${raw} ****`
}
CALLBACKS_OBJECT = {
  default: (cmd, args, raw) => { return { cmd: cmd, args: args, raw: raw } }
}

function parseGCodeLine(line, callbacks = CALLBACKS_CDPR_COMMAND, display_callback = console.log) {
  // Split the line at the semicolon and only keep the first part
  line = line.split(';')[0];

  // Split arguments by spaces.
  const parts = line.trim().split(' ').filter(Boolean);
  const command = parts[0];
  const args = parts.slice(1).reduce((acc, arg) => {
    const [letter, ...value] = arg;
    const scale = letter.toUpperCase() === 'X' ? gcode_scale_x : gcode_scale_y;
    acc[letter.toUpperCase()] = parseFloat(value.join('')) * units_to_m * scale + gcode_translate[letter.toUpperCase()];
    return acc;
  }, {});
  if (command == 'G0') {
    console.log(args);
  }
  if (("X" in args) && ("Y" in args)) {
    const x = args.X, y = args.Y;
    args.X = x * homography[0][0] + y * homography[0][1] + homography[0][2];
    args.Y = x * homography[1][0] + y * homography[1][1] + homography[1][2];
    const w = x * homography[2][0] + y * homography[2][1] + homography[2][2];
    args.X /= w;
    args.Y /= w;
  }
  if (command == 'G0') {
    console.log(args);
  }

  ret = callbacks[command] ? callbacks[command](args) : callbacks.default(command, args, line);
  display_callback(ret);
  return ret;
}

function parseGCode(gcode, callbacks = CALLBACKS_CDPR_COMMAND, display_callback = console.log) {
  const lines = gcode.split('\n');
  return lines.map((line) => parseGCodeLine(line, callbacks, display_callback));
}

function correct_ends_of_strokes_to_U_inplace(cdpr_command_lines) {
  let last_L_line = null;
  for (let i = 0; i < cdpr_command_lines.length; i++) {
    switch (cdpr_command_lines[i][0]) {
      case 'L':
        last_L_line = i;
        break;
      case 'M':
        if (last_L_line) {
          cdpr_command_lines[last_L_line] = cdpr_command_lines[last_L_line].replace('L', 'U');
          last_L_line = null;
        }
        break;
    }
  }
  if (last_L_line) {
    cdpr_command_lines[last_L_line] = cdpr_command_lines[last_L_line].replace('L', 'U');
  }
  return cdpr_command_lines;
}
