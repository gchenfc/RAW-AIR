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

## Cable Robot
* Turn down speed to ts0.04
* Turn down speed in cdpr.js to 0.04
* Check anticogging calibration - Motors 0/1 seemed to be ok but Motors 2/3 did not appear to be ok.
* Set L/R/U/D limits in cable robot manually via serial
  * ```xLu1.8;xLr5;xLl0.8;xLd0.3```
* Set custom gains
* Set tracking fail-distance threshold: `kd0.25`

```
gs0;ts0.04;gs1;ts0.04;gs2;ts0.04;gs0
gs0;xLu1.8;xLr5;xLl0.8;xLd0.3;gs1;xLu1.8;xLr5;xLl0.8;xLd0.3;gs2;xLu1.8;xLr5;xLl0.8;xLd0.3;gs0
gs0;kKp10000;kKd500;kKi50;gs1;kKp10000;kKd500;kKi50;gs2;kKp10000;kKd500;kKi50;gs0
gs0;kd0.25;gs1;kd0.25;gs2;kd0.25;gs0
gs0;kKL8;kKm100;kKM200;gs1;kKL8;kKm100;kKM200;gs2;kKL8;kKm100;kKM200;gs0
gs1;xA0.03;gs0
```

```
ts0.04
xLu1.8;xLr5;xLl0.8;xLd0.3
kKp10000;kKd500;kKi50
kd0.25
kKL8;kKm100;kKM200
```
