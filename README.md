# LOCK

Localization, Obstacle avoidance , Control and Kinematics framework for differential steering systems.

# Introduction

* The Problem Statement assumes a static camera mounted with the complete view of the arena.
* The obstacles are assumed to be of a paticular predefined color.
* Uses A* for Path Planning with Erosion/Dilation to provide the bot with sufficient cleareance from obstacles.
* Controls written specifically for differential steering systems.

# Some Sample Tracks the bot successfully completed:

## Sample Tracks at NIT Pragyan:
![Track 1](https://raw.githubusercontent.com/quinasura/LOCK/master/Localization/Example.jpg)

## Sample Track at Kurukshetra:
![Track at Kurukshetra](https://raw.githubusercontent.com/quinasura/LOCK/master/AI-path_planning/Dilation_Erosion/frame_screenshot_31.01.2015.png)

Disclaimer: This is code developed specifically for competition and are certain to contain idiosyncrasies but it won in all
the competitions it participated (2015-2016) :)

# Competition Runs
[![DEMO Video](http://img.youtube.com/vi/KyFhYlVsC_k/0.jpg)](http://www.youtube.com/watch?v=KyFhYlVsC_k)

[![DEMO Video](http://img.youtube.com/vi/bM3tn3CD7cA/0.jpg)](http://www.youtube.com/watch?v=bM3tn3CD7cA)

[![DEMO Video](http://img.youtube.com/vi/1ySnonQtAqg/0.jpg)](http://www.youtube.com/watch?v=1ySnonQtAqg)

####Tools needed
* USB HD webcam
* opencv 2.4.x
* Arduino UNO/MSP430
* HC06/Serial
