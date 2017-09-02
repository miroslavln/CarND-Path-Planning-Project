# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
# Introuction
The purpos of the project is to safely navigate a virtual highway without leaving the road, exceeding the speed limit, 
colliding with other vehicles or making uncomfortable moves measured by the car jerk.
 
 The simulator provides information about the other vehicles speed and positions expressed in freenet coordinates.
 
 #Implementation
  The simulator provides information about the car's freenet coordinates in terms of distance along the track(s) and 
 position across the road (d). The car needs to keep driving with a speed that is less than 50mph. This is accomplished
 by providing a path to the simulator that describes the next few positions the car needs to be in the future. 
  The simulator periodically queries the code for the next path. I am providing 50 points for the car to follow. 
  The simulator gives back the last path it was given but with the points that have already been traveled missing. 
  The code determines the end x, y and yaw coordinates from the previouspath to use them when calculating the next points. 
 The algorithm stores the last 2 points from the previous path in a list along with 3 more points 30, 60 and 90 meters
 ahead in the s direction. The points are then converted to local car coordinates and used to define a spline.
  This is  necessary to smooth out the path and minimize jerk as the way points are spaced out and would create 
  a lot of jerk when trying to follow the trajectory using only them. 
  The algorithm then looks ahead 30 meters and using the spline looks up the new reference points that it needs to send
  to the simulator spaced out in 0.02 seconds making sure that the desired speed is not exceeded.
  
  The algorithm also observes other cars on the road in order to avoid collisions. Using the provided sensor fusion data
  it predicts where the other cars are going to be at the end of the previous path point. It then measures the 
  distance to check if it is coming close to any of them in the same lane. If that is the case it makes a decision
  whether to change lane by measuring the longest distance in the adjacent lanes and chooses the one with the largest
  distance to the next vehicle. It prioritizes the left lane when making the choice. 
   If the lance change is not desirable it slows down the car to maintain a distance and avoid a collision.
   

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
