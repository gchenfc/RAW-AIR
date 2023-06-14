
const HOST = '192.168.0.12'
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
    textarea.value += l + '\n';
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
