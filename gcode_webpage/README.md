# GCode Webpage

This webpage loads a gcode file, parses it, displays it (together with the parse results), and sends commands over a websocket to the cable robot control panel.

## Setup
* open up the cable robot control panel (from the `cable-robot` repo)
* start `server.py` (this is copy-pasted from the art_skills repo, so if you're already running that server, don't also run this one)
* copy the ip address to the `HOST` variable at the very top of `cdpr_websocket.js`.  The ip address is printed out by `server.py` where it says: `Serving whiteboard at: xxx.xxx.xxx.xxx:5900`.  The ip address is just the `xxx.xxx.xxx.xxx` part, do NOT include the 5900 (you can see in the javascript that the 5900 is already coded into the javascript).
* open [`index.html`](index.html) in a browser (I've only tested this in Chrome)

## Usage
* Load a gcode file by choosing a file with the top-left button then clicking `Parse File`.  The gcode file will be parsed and the results will be displayed.
  * Check that there's no "UNKNOWN COMMAND"s in the parsed section, or if there are, that they are safe to ignore.
  * Check that everything is as you expect, especially the rough order of magnitude of the motions.
* You can click the following buttons to send commands over the websocket:
  * Run All - sends all the commands in the gcode file, in order
  * Run Stroke - sends any G1 commands up until the next non-G1 command.  Since G1 is "line to", this will send a single stroke of the robot.
  * Step - sends the highlighted command and advances to the next line
  * Reset - resets the highlighted command to the first command in the gcode file
  * Send current but don't advance - sends the highlighted command but does not advance to the next line
  * Advance without sending current - advances to the next line without sending the current line.  This might be necessary if there's any UNKNOWN commands you'd like to ignore, since the code won't let you send/advance past an UNKNOWN command otherwise.
  * Back without sending - goes back to the previous line.


## TODO
* I think cable robot control panel is scaling all the motions, where it's expecting the total robot workspace to be [0, 1] in width and height, so if there are any motions >1m or <0m, you should definitely be careful and read the code yourself thoroughly to make sure of what the cable robot ipad.js code is doing with respect to scaling, and adjust accordingly.
* Visualize the gcode file / drawing commands.  This shouldn't actually be too hard if we re-use some ipad.js code from `art_skills`.

## svg to gcode
Temporarily, I tried using this svg2gcode converter: [https://sameer.github.io/svg2gcode/](https://sameer.github.io/svg2gcode/) ([github](https://github.com/sameer/svg2gcode)), but it seems to only be able to handle svg path commands, no circles or rectangles etc.  But as an example, InkScape may be able to do this conversion (see [FAQ](https://github.com/sameer/svg2gcode#faq--interesting-details))

Otherwise, tristan said he can generate gcode ;) but his Rhino license expired so he's trying to figure out something else :)
