## Project: Search and Sample Return

### Notebook Analysis

#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

Obstacles are just inverted navigable pixels. Lower left and lower right corners of image are masked by transformed white image. `warped_white` is created in *cell [7]* then applied as mask in *cell [13] #3*.

For rock samples I created new function `sample_thresh()` in *cell [6]*. Color of rocks is checked against corresponding RGB boundaries.

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

For this task I mostly used code from the previous lessons. Obstacles and rocks are processed analogously to navigable pixels except for threshed image creation. Obstacles and rocks are set to 0 for navigable pixels in `worldmap` image as their coordinates are much more reliable.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

For `perception_step()` I used the same code as in the notebook. `worldmap` populated only with relatively reliable data when Rover is oriented close to horizontal plane (*line #143*). I lowered Rover speed so that most of the data would be reliable.

At the end of `perception_step()` all three coordinate sets are converted to polar ones. `Rover.nav_angles` is adjusted so as to keep Rover to the left. Coordinates of obstacles are packed into the list of tuples. It's a workaround for extracting data conditionally.

Several modes where added to `decision_step()`. In `forward` mode Rover additionally examines if it:
- got stuck (*line #20*)
- has an obstacle in front of it (*line #42*)
- is near a rock (*line #58*)
- objectives was achieved (*line #63*).

In `stuck` mode (*line #95*) Rover turns right to an angle, defined by `Rover.stuck` variable.

In `rock` mode (*line #119*) Rover tries to pick up a spotted sample. At first Rover steers towards the rock while keeps it on the left side. When distance to the rock is suitable for picking it up Rover turns round in attempt to find a right position. Sometimes coordinates of the rock evaluated incorrectly because only the top of the rock is visible or obstacles are preventing Rover from getting closer. In that cases Rover turns round to check if the rock can be picked up.

Polar coordinates of a spotted rock are converted to absolute coordinates and memorized (*line #132*) because during maneuvers the rock often becomes invisible through the camera.

In `parked` mode (*line #179*) Rover is standing still close to the starting position.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

Simulator was running with the following settings and FPS:

| Parameter       | Value |
------------------|---
Screen resolution | 1024 x 768
Graphics quality  | Good
FPS               | 18-20

The program in its current state gives relatively stable results. Average totals for each run are the following:

| Metric  | Average value|
----------|---
Mapped    | 97
Fidelity  | 70
Located   | 4-6
Collected | 3-6
Time      | 1000 s

Generally Rover is navigating along the left wall, tries to round obstacles on the right. If a rock is spotted, Rover approaches it and picks it up.

There are still a few blind spots and unreachable places for sample picking up. I managed to navigate them with other parameter settings, but it led to emerging of problems in other places.

There is a lot of space for improvements. It seems that HSB color space is much more suitable than RGB for selection of navigable region and samples so the system have to be recalibrated for using HSB images first. Pitch and roll correction for transformation of camera images would allow to speed up Rover and improve fidelity at the same time. Obstacles can be observed from a longer distance so hanging rocks could be avoided, but that will require complex steering decisions and taking into account already visited areas.