let gcodeLines = [];
let parsedLinesReadable = [];
let parsedLines = [];
let parsedLinesObjects = [];
let lineIndex = 0;

function parseGCodeString(gcode) {
  document.getElementById('gcodeDisplay').value = gcode;

  gcodeLines = gcode.split('\n');

  parsedLinesReadable = parseGCode(gcode, callbacks = CALLBACKS_READABLE);
  parsedLinesObjects = parseGCode(gcode, callbacks = CALLBACKS_OBJECT);
  parsedLines = parseGCode(gcode, callbacks = CALLBACKS_CDPR_COMMAND);
  correct_ends_of_strokes_to_U_inplace(parsedLines);
  drawCommands(parsedLines);
  
  document.getElementById('readableDisplay').value = parsedLinesReadable.join('\n');
  document.getElementById('commandsDisplay').value = parsedLines.join('\n');

  reset();
  highlight(lineIndex);
}

function parseFile() {
  const fileInput = document.getElementById('gcodeFile');
  const file = fileInput.files[0];

  if (file) {
    const reader = new FileReader();

    reader.onload = function (e) {
      parseGCodeString(e.target.result);
    };

    reader.readAsText(file);
  } else {
    console.error('No file selected!');
  }
}

function appendToCallbackDisplay(text) {
  document.getElementById('callbackDisplay').value += text + '\n';
}

function highlight(lineNumber) {
  // Auto-scroll the textareas if the highlight is off the screen
  // Calculate the new scroll position
  const lineHeight = gcodeDisplay.scrollHeight / gcodeLines.length;
  const linePos = lineHeight * lineNumber;
  const visibleHeight = gcodeDisplay.clientHeight - 30;
  if (linePos < gcodeDisplay.scrollTop || linePos > (gcodeDisplay.scrollTop + visibleHeight)) {
    gcodeDisplay.scrollTop = linePos - visibleHeight;
  }

  // Do Highlighting
  // highlight_caret(lineNumber);
  highlight_overlay(lineNumber);

  // Display in simulation
  drawSimCursor(parsedLines[lineNumber - 1], parsedLines[lineNumber]);
}

function highlight_caret(lineNumber) {
  const gcodeDisplay = document.getElementById('gcodeDisplay');
  const highlightedContent = gcodeLines.map((line, index) =>
    index === lineNumber ? `> ${line}` : `  ${line}`
  ).join('\n');
  gcodeDisplay.value = highlightedContent;

  const readableDisplay = document.getElementById('readableDisplay');
  readableDisplay.value = parsedLinesReadable.map((line, index) =>
    index === lineNumber ? `> ${line}` : `  ${line}`
  ).join('\n');

  const commandsDisplay = document.getElementById('commandsDisplay');
  commandsDisplay.value = parsedLines.map((line, index) =>
    index === lineNumber ? `> ${line}` : `  ${line}`
  ).join('\n');
}
function highlight_overlay(lineNumber) {
  const overlay = document.getElementById('highlightOverlay');
  const scrollOffset = document.getElementById('gcodeDisplay').scrollTop;
  const padding = 3;  // textarea padding + border + margin

  overlay.style.top = `calc(${lineNumber}em - ${scrollOffset}px + ${padding}px)`;
  if (parseFloat(getComputedStyle(overlay).top) > gcodeDisplay.clientHeight + 50) {
    overlay.style.display = "none";
  } else {
    overlay.style.display = "block";
  }
}

function syncScroll(id) {
  const target = document.getElementById(id);
  const source = event.target;
  target.scrollTop = source.scrollTop;
  highlight_overlay(lineIndex);
}

function sendCurLineAndAdvance() {
  return sendCurLine() && advance();
}
function sendCurLine() {
  if (lineIndex < parsedLines.length) {
    return send_line(parsedLines[lineIndex]);
  } else {
    setTimeout(() => { alert('End of file!') }, 100);
    return false;
  }
}
function advance() {
  if (lineIndex < parsedLines.length - 1) {
    lineIndex++;
    highlight(lineIndex);
    return true;
  } else {
    setTimeout(() => { alert('End of file!') }, 100);
    return false;
  }
}
function back() {
  if (lineIndex > 0) {
    lineIndex--;
    highlight(lineIndex);
    return true;
  } else {
    setTimeout(() => { alert('Start of file!') }, 0);
    return false;
  }
}
function reset() {
  lineIndex = 0;
  highlight(lineIndex);
}
function runInLoopRedraw(func, redrawInterval_ms = 10, startTime = Date.now()) {
  while (func()) {
    if (Date.now() - startTime > redrawInterval_ms) {
      setTimeout(runInLoopRedraw, 0, func, redrawInterval_ms);
      break;
    }
  }
  setTimeout(highlight, 0, lineIndex);
}
function runTillEnd() {
  runInLoopRedraw(sendCurLineAndAdvance);
}
function runTillEndOfStroke() {
  runInLoopRedraw(() => sendCurLineAndAdvance() && (parsedLinesObjects[lineIndex].cmd === 'G1'));
}

// Initialize with some sample gcode
parseGCodeString(`G21
G90;svg#Default > path
;
G0 X0 Y158.4458843191785
G1 X8.556241426611795 Y149.89392958666818 F300
G1 X12.969335736612468 Y145.49094351124086 F300
G1 X17.52400548696845 Y140.95617238502214 F300
G1 X22.127602195230633 Y136.3851021987246 F300
G1 X202.36595506054766 Y151.26995808803932 F300
G1 X201.20185116469713 Y154.8206351166626 F300
G1 X199.99999999999994 Y158.4458843191787 F300
G1 X0 Y158.4458843191785 F300;svg#Default > path
G0 X200 Y158.4458843191785
G1 X208.5562414266118 Y149.89392958666818 F300
G1 X212.96933573661246 Y145.49094351124086 F300
G1 X217.52400548696843 Y140.95617238502214 F300
G1 X222.12760219523057 Y136.38510219872467 F300
G1 X226.85185185185182 Y131.70977320806736 F300
G1 X230.01921309633485 Y128.58524334460836 F300
G1 X233.23188177703443 Y125.42508552597474 F300
G1 X236.48834019204386 Y122.23189254963117 F300
G1 X239.75029548208764 Y119.04415613849372 F300
G1 X243.04874795829383 Y115.83251010029966 F300
G1 X246.38203017832654 Y112.59969090354069 F300
M2`);
