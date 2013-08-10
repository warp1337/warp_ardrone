# falkor_ardrone: A ROS package to control an AR.Drone autonomously

## Introduction

"warp_ardrone" is a [ROS](http://ros.org/ "Robot Operating System") package which uses the "ardrone_autonomy" package
(originally from Brown, now from AutonomyLab) to implement autonomous control functionality on an AR.Drone. "warp_ardrone"
package is  additionally based on the great "falkor_ardrone package" [Falkor Systems] (https://github.com/FalkorSystems),
therefore I left their BSD and Copyright statements intact!

More detail on this particular package follow...

### Updates

- *August 10th, 2013*: Initial Release

## Usage

Once the AR.Drone is connected to your computer via WiFi, run

     ```bash
     $ roslaunch warp_ardrone warp-init.launch
     ```

## License

Copyright (c) 2012, Falkor Systems, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.