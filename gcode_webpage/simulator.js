// const W = 1,
//   H = 0.7;
const W = 5.8,
  H = 3.7;
const SCALE = 500;
const BRUSH_DIAMETER = 0.025; // mm
const CURSOR_SIZE = 40; // Units

// BOUNDS = {Y: [0.70, 2.3], X: [2.0, 3.93]}
// BOUNDS = {Y: [0.65, 2.35], X: [1.9, 3.98]}
// These are limits without 60mm buffer, but brush-radius buffer
BOUNDS = {Y: [0.628, 2.411], X: [1.9262, 4.0222]}

// Get the canvas with ID "previewCanvas"
const canvas = document.getElementById("previewCanvas");
const ctx = canvas.getContext("2d");
const canvas_cursor = document.getElementById("cursorCanvas");
const ctx_cursor = canvas_cursor.getContext("2d");

// Set the canvas size to the size of the preview image
canvas.width = W * SCALE;
canvas.height = H * SCALE;
canvas_cursor.width = W * SCALE;
canvas_cursor.height = H * SCALE;

// Make the background red
ctx.fillStyle = "#eee";
// ctx.fillRect(0, 0, W, H) ;
ctx.fillRect(0, 0, SCALE * W, SCALE * H);

function drawWorkspace() {
  ctx.fillStyle = "none";
  ctx.strokeStyle = "#888";
  ctx.rect(BOUNDS.X[0] * SCALE, (H - BOUNDS.Y[1]) * SCALE, (BOUNDS.X[1] - BOUNDS.X[0]) * SCALE, (BOUNDS.Y[1] - BOUNDS.Y[0]) * SCALE);
}

function drawCommands(lines) {
  // Lines has a format of "M" for move, "L" for line, and "U" for finishing a stroke.  It looks like this:
  // M0,0,0.1584458843191785
  // L0,0.008556241426611795,0.1498939295866682
  // ...
  // L0,0.19999999999999996,0.1584458843191787
  // U0,0,0.1584458843191785
  // M0,0.2,0.1584458843191785
  // L0,0.2085562414266118,0.1498939295866682
  // ...
  // L0,0.21296933573661248,0.14549094351124087
  // U0,0.24638203017832655,0.11259969090354069

  // Ignore the leading 0's.
  console.log(lines);

  // First clear the existing canvas and set the marker diameter
  ctx.clearRect(0, 0, SCALE * W, SCALE * H);

  drawWorkspace();

  // Now draw each line as it will look painted
  ctx.lineWidth = BRUSH_DIAMETER * SCALE;
  ctx.strokeStyle = "#000";
  for (const line of lines) {
    const [_, x, y_] = line.split(",");
    const y = H - y_;
    if (line.at(0) == "M") {
      ctx.moveTo(x * SCALE, y * SCALE);
    } else if (line.at(0) == "L") {
      ctx.lineTo(x * SCALE, y * SCALE);
    } else if (line.at(0) == "U") {
      ctx.lineTo(x * SCALE, y * SCALE);
      ctx.stroke();
      ctx.beginPath();
    }
  }

  // Now draw each line again but with a skinny green line.
  ctx.lineWidth = (BRUSH_DIAMETER / 10) * SCALE;
  if (lines.length > 0) {
    const [_, x0, y0] = lines[0].split(",");
    ctx.moveTo(x0 * SCALE, (H - y0) * SCALE);
    ctx.beginPath();
  }
  for (const line of lines) {
    const [_, x, y_] = line.split(",");
    const y = H - y_;
    ctx.lineTo(x * SCALE, y * SCALE);
    if (line.at(0) == "M") {
      ctx.strokeStyle = "#f00";
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(x * SCALE, y * SCALE);
    } else if (line.at(0) == "L") {
    } else if (line.at(0) == "U") {
      ctx.strokeStyle = "#0f0";
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(x * SCALE, y * SCALE);
    }
  }
}

function drawSimCursor(line_past, line_next) {
  ctx_cursor.clearRect(0, 0, SCALE * W, SCALE * H);

  function drawWithColor(line, color) {
    const [_, x, y] = line.split(",");
    ctx_cursor.fillStyle = color;
    ctx_cursor.fillRect(
      x * SCALE - CURSOR_SIZE / 2,
      (H - y) * SCALE - CURSOR_SIZE / 2,
      CURSOR_SIZE,
      CURSOR_SIZE
    );
  }

  if (line_past) drawWithColor(line_past, "#55f8");
  if (line_next) drawWithColor(line_next, "#55f");

  if (line_past && line_next) {
    // Draw a white line between the two points
    const [_, x0, y0] = line_past.split(",");
    const [__, x1, y1] = line_next.split(",");
    ctx_cursor.strokeStyle = "#fff";
    ctx_cursor.lineWidth = 2;
    ctx_cursor.beginPath();
    ctx_cursor.moveTo(x0 * SCALE, (H - y0) * SCALE);
    ctx_cursor.lineTo(x1 * SCALE, (H - y1) * SCALE);
    ctx_cursor.stroke();
  }
}
