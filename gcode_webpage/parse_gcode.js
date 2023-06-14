let units_to_m = 0.001;
let cur_pos = null;

CALLBACKS_READABLE = {
  G0: (args) => 'moveTo ' + JSON.stringify(args),
  G1: (args) => 'lineTo ' + JSON.stringify(args),
  G20: (args) => { units_to_m = 0.0254; return 'set units to inches!  Doing implicit conversion to meters.' },
  G21: (args) => { units_to_m = 0.001; return 'set units to millimeters! ' },
  G90: (args) => 'Absolute positioning, this is good and expected.',
  G91: (args) => '**** CANNOT HANDLE RELATIVE POSITIONING YET!!!! ****',
  M2: (args) => 'End of program :)',
  default: (cmd, args, raw) => `UNKNOWN command: ${cmd} with args ${args}.  ${raw}`
}
CALLBACKS_CDPR_COMMAND = {
  G0: (args) => (cur_pos ? (`U0,${cur_pos.X},${cur_pos.Y};`) : '') + `M0,${args.X},${args.Y}`,
  G1: (args) => `L0,${args.X},${args.Y}`,
  G20: (args) => { units_to_m = 0.0254; return '' }, // inch units
  G21: (args) => { units_to_m = 0.001; return '' },
  G90: (args) => '',
  G91: (args) => '**** CANNOT HANDLE RELATIVE POSITIONING YET!!!! ****',
  M2: (args) => '',
  default: (cmd, args, raw) => `**** UNKNOWN command: ${cmd} with args ${args}.  ${raw} ****`
}
CALLBACKS_OBJECT = {
  default: (cmd, args, raw) => { return { cmd: cmd, args: args, raw: raw } }
}

function parseGCodeLine(line, callbacks = CALLBACKS, display_callback = console.log) {
  // Split the line at the semicolon and only keep the first part
  line = line.split(';')[0];

  // Split arguments by spaces.
  const parts = line.trim().split(' ').filter(Boolean);
  const command = parts[0];
  const args = parts.slice(1).reduce((acc, arg) => {
    const [letter, ...value] = arg;
    acc[letter] = parseFloat(value.join('')) * units_to_m;
    return acc;
  }, {});

  ret = callbacks[command] ? callbacks[command](args) : callbacks.default(command, args, line);
  if (command == 'G0' || command == 'G1') {
    cur_pos = { X: args.X, Y: args.Y };
  }
  display_callback(ret);
  return ret;
}

function parseGCode(gcode, callbacks = CALLBACKS, display_callback = console.log) {
  cur_pos = null;
  const lines = gcode.split('\n');
  return lines.map((line) => parseGCodeLine(line, callbacks, display_callback));
}
