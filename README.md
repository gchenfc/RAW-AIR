# RAW-AIR
Code for Tristan's and my [GT Library's Artist-in-residence](https://library.gatech.edu/AIR) robot, and ME senior capstone

# Code Modules / checklist
* Cable Robot Control Panel (cable-robot/site/index.html)
  * Set correct cable robot dimensions in main.js and index.html
  * Set correct js imports in index.html, e.g. comment-out ipad if not using ipad
* Arm Server (RAW-AIR/arm/arm_server.py)
* Cable Robot Server (RAW-AIR/gcode_webpage/server.py), (only needed if using ipad or gcode interpreter)
* gcode interpreter (RAW-AIR/gcode_webpage/index.html), use liveserver (?)
  * Set correct ip address in cdpr_websocket.js
  * Verify that "websocket connected :)" is green
* Continuous Recalibration (cable-robot/src/apriltag/continuous_recalibration.py)

## SOP
1. Cable Robot
   1. Calibrate
      1. `c4`
      2. Press controller `4/x` to activate motors in "HOLD" mode
      3. Move around to make sure cables are wound nicely
      4. Move to center of sandbags
      5. `c14`
   2. Set configuration
      1. Copy-paste the lines below
   3. Set limits
      1. press controller `4/X` then `1/Y` to switch to "TRACKING" mode
      2. Move to limits, record, and update
      3. Switch to "HOLD" mode when done, to be safe
2. Arm
   1. Turn on arm
   2. Launch Arm Server
   3. Ensure connection by pressing "Get Angles" on cable robot control panel
3. gcode interpreter
   1. Launch Cable Robot Server
   2. Set correct ip address in cdpr_websocket.js (ensure VPNs are turned off)
   3. Open gcode_webpage/index.html
   4. Verify "websocket connected" is green.
4. Start Running
   1. Switch to "gs1" mode if not already
   2. Load up the stroke(s) from the gcode_webpage
      1. oeu
   3. Switch to "TRACKING" mode and the robot should start painting

## Cable Robot
* Turn down speed to ts0.04
* Turn down speed in cdpr.js to 0.04
* Check anticogging calibration - Motors 0/1 seemed to be ok but Motors 2/3 did not appear to be ok.
* Set L/R/U/D limits in cable robot manually via serial
  * ```xLu1.8;xLr5;xLl0.8;xLd0.3```
* Set custom gains `kKp10000;kKd500;kKi0;kKw0`
* Set tracking fail-distance threshold: `kd0.25;xd0.05`
* Set max odrive vel/current: `0n15c10,30` sets vel limit to 10 and current limit to 30A (=150N for 1" diameter winch, so ~300N for bottom cables and ~600N for top cables)

** One-time only ** - update the current limit on the ODrive.  Since Kt = 0.06366197764873505 Nm/A, then for 1" diameter winch, 20A = 100N.  So maybe 30-40A would be a good limit?  (40A = 200N, or 400N for the top winches)

** TODO ONCE I GET TO LIBRARY: consider just setting Ki to 0, it doesn't seem to do much at all in Klaus, and even the integrator windup doesn't help that much because the integrator accumulates so incredibly slowly, e.g. (50 N/m.s)*(0.01m) = (0.5N/s).

During calibration:
```
kKL16;kKm225;kKM450
```

Set all controller modes:
```
gs0;ts0.04;gs1;ts0.04;gs2;ts0.04;gs0
gs0;kKp15000;kKd500;kKi0;kKw0;gs1;kKp15000;kKd500;kKi0;kKw0;gs2;kKp15000;kKd500;kKi0;kKw0;gs0
gs0;kKp15000;kKd1000;kKi0;kKw0;gs1;kKp15000;kKd1000;kKi0;kKw0;gs2;kKp15000;kKd1000;kKi0;kKw0;gs0
gs0;kd0.25;xd0.05;gs1;kd0.25;xd0.05;gs2;kd0.25;xd0.05;gs0
gs0;kKL16;kKm200;kKM300;gs1;kKL16;kKm200;kKM300;gs2;kKL16;kKm200;kKM300;gs0
gs1;xA0.03;gs0
gs0;xs0.1;xa0.5;gs0
gs0;xld0;xlu0;xlr0;xll0;gs1;xld0;xlu0;xlr0;xll0;gs2;xld0;xlu0;xlr0;xll0;gs0
0n15c10,15;1n15c10,30;2n15c10,30;3n15c10,15
```

single window pane:
```
xLl2.24;xLr3.805;xLu2.30  # old
xLl2.0;xLr3.93;xLu2.30  # new

# for control
gs0;xLl2.0;xLr3.93;xLd0.0;xLu2.30;gs1;xLl2.0;xLr3.93;xLd0.0;xLu2.30;gs2;xLl2.0;xLr3.93;xLd0.0;xLu2.30;gs0

# for bottom safety
gs0;xLl2.0;xLr3.93;xLd0.0;xLu2.30;gs1;xLl2.0;xLr3.93;xLd0.65;xLu2.30;gs2;xLl2.0;xLr3.93;xLd0.0;xLu2.30;gs0

# tight
gs0;xLl1.9;xLr3.98;xLd0.65;xLu2.35;gs1;xLl1.9;xLr3.98;xLd0.65;xLu2.35;gs2;xLl1.9;xLr3.98;xLd0.65;xLu2.35;gs0



gs0;xLl1.9;xLr3.98;xLd0.65;xLu2.25;gs1;xLl1.9;xLr3.98;xLd0.65;xLu2.25;gs2;xLl1.9;xLr3.98;xLd0.65;xLu2.25;gs0
```

Individual controller mode:
```
ts0.04
xLu1.8;xLr5;xLl0.8;xLd0.3
kKp10000;kKd500;kKi0;kKw0
kd0.25
kKL16;kKm100;kKM300
xs0.1;xa0.5
```

gcode webpage params:
* x: 1.9
* y: 0.67
* xscale: 0.975
* yscale: 0.92

# Recovering from Weird States

## Fixing weak/slack cables or poor tracking control

This is due to motor encoders slipping causing motors to go out of calibration.  We just need to recalibrate the motors.

*Expect the lines to be up* (fill in this description)

1. On the GUI, Press "pause"
2. On the gamepad, press "4"/"X" to switch to "hold" mode.
   1. Be prepared, this will *gently* drop the robot!!!
3. Pull the robot to rest on the sandbags.
4. ESTOP the robot (press right joystick or press-then-twist-release ESTOP button) then press "3"/"A" on the gamepad to re-enable the motors.
5. Pull each of the 4 cables to introduce some slack (about 6-12 inches), because the motors need some slack to calibrate properly.
6. Type `c4` in the GUI terminal and hit enter or "send".
   1. You should hear a beep come from each motor.
   2. In the "state" column of the GUI, you should see all the motors go from 1 to 4 to 6 to 7 then back to 1.
7. On the gamepad press "4"/"X" to switch to "hold" mode.
   1. Be prepared, this might pull the robot sideways off the sandbags!!!  Hold it down / catch it if necessary.
8. If any of the pulleys are wound-up weirdly (gaps, overlaps), then manually pull the cables and slowly release them to get them to wind up properly.
9. Pull the robot to be "resting" on the center of the sandbags.
10. Type `c14` in the GUI terminal to set the "zero" position of the motors.
11. On the gamepad, press "1"/"Y" to switch back to tracking mode.  It should move to the last commanded position.
    1.  Wait for it to finish moving
12. On the GUI, Press "resume"

# TODO LIST
* [x] Get a longer ethernet cable for Arm
* [x] Brush tool-rest
  * [x] Brush-to-arm mount
  * [x] Tool-rest
    * [x] Fix issue where after ~1hr of operation the servos get tired and might miss the tool-rest
    * [x] Make cone horn slipperier
    * [ ] More testing of horn alignment when servos get hot
  * [x] Paint bucket
* [x] Make brush/arm trajectories so that they don't crash into the mullions
* [x] Fix bug where sometimes if the callback isn't called (e.g. manually disrupted) then the state machine won't proceed
  * Resolved with the pause/resume buttons, by just pausing and re-resuming, you can get the robot out of the stuck state.
* [x] Add a "pause" button that pauses the painting execution without disrupting the state
* [x] Update the gcode_webpage to display the "current line" more efficiently, because currently it's very slow to re-write the entire tex box each time
* [x] Figure out why sometimes one motor dies (probably because the phase wires are loose)
  * I think it was because the encoder was slipping.  I remounted it more centered and more securely, so we'll see if it's resolved.
  * Edit: found another issue which was that the phase wire screw terminals were loose.  In particular, motor 3's middle phase wire melted the plastic and caused too much contact resistance error-ing out the Odrive.  I needed to solder the phase wires to fix it.  I tightened all the phase wires (many were very loose) so hopefully that resolves it.  Lesson for the future: always make sure the phase wires are tight, and they may loosen over time so periodically check, especially if the motors are having intermittent connection issues.
* [x] Arm State-tracker to make sure the arm always goes through the appropriate intermediate configurations, e.g. if in "paint", go to "prep" before going to "home" or "dip".
* [x] Add ETA timer
* [x] Bring tripod
* [x] Arm overheat detection & pre-emptive rest
* [ ] Resolve why sometimes cable robot motors get misaligned
* [ ] Bring in different computer/laptop
* [x] Test edge half-panels - can reach when motors calibrated, seems pretty accurate too but a little warped
* [x] Set screws in the other 3 motors
* [x] Program 4-corner calibration
  * Currently this is part of the calibration script.  There's also a homography computation if lazy.

42m

# Pane parameters

* Unsplit panes: 84" x 71.5"
* Split panes: 72" x 71.5"

60mm buffer on each side
