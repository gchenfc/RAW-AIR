
const HOST = '143.215.95.166'
const websocket = new WebSocket("ws://" + HOST + ":5900/");

function send_line(line) {
  if (line == '') {
    return true;
  }
  line = line.replace(/;/g, '\n');
  for (l of line.split('\n')) {
    if (l.startsWith('*')) {
      alert('AN INVALID COMMAND WAS ATTEMPTED TO BE SENT.  PLEASE DOUBLE CHECK THAT THIS COMMAND CAN BE SAFELY IGNORED.');
      return false;
    }
    const textarea = document.getElementById('sentCommands');
    textarea.value = (textarea.value + l + '\n').slice(-3000);
    textarea.scrollTop = textarea.scrollHeight;
    websocket.send(l);
  }
  return true;
}

websocket.onmessage = function (event) {
  console.log(event.data);
  let command = event.data[0];
  xy = event.data.slice(1).split(',');
  x_raw = parseFloat(xy[0]);
  y_raw = parseFloat(xy[1]);
  // x = x_raw * canvas.width;
  // y = y_raw * canvas.width;

  // console.log(command, x, y);
  console.log(command, x_raw, y_raw);
};

websocket.onopen = function (event) {
  const elem = document.getElementById('websocket-status');
  elem.style.backgroundColor = 'green';
  elem.textContent = "Websocket connected :)";
  console.log('yay connected');
}

websocket.onclose = function (event) {
  const elem = document.getElementById('websocket-status');
  elem.style.backgroundColor = 'red';
  elem.textContent = "WEBSOCKET NOT CONNECTED";
}

websocket.onerror = function (event) {
  const elem = document.getElementById('websocket-status');
  elem.style.backgroundColor = 'red';
  elem.textContent = "WEBSOCKET ERRORED OUT";
}
