let units_to_m = 0.001;

// let gcode_translate = { X: 2, Y: 0 }, gcode_scale = 2;
let gcode_translate = { X: 2.45, Y: 0.9 }, gcode_scale = 1.0;

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
function updateScale() { gcode_scale = updateValue('scaleValue', gcode_scale, 'Scale', true); }


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
    acc[letter] = parseFloat(value.join('')) * units_to_m * gcode_scale + gcode_translate[letter.toUpperCase()];
    return acc;
  }, {});

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
