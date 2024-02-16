# HybridRocketAeroRep

This repo holds the diagrams and code used in the flight data recording system for an hybrid rocket. This eletronic system is used to measure maximum flight altitude and activate a parachute, using COTS components for the MCU, IMU, SD card reader, battery and power regulation. The code is based on a finite state machine operating during the three flight phases: setup + verification; reading + saving data; save file + sleep.

<p align="middle">
  <img src="https://github.com/pedrokappaa/HybridRocketAeroRep/blob/main/img/prototype_front.jpeg" width="500"/>
  <img src="https://github.com/pedrokappaa/HybridRocketAeroRep/blob/main/img/rocket_capsule.jpeg"  width="500"/>
</p>