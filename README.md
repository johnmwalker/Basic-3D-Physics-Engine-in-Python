# Basic 3D Physics Engine in Python

This is an evolution of my [2D Python Physics Game](https://github.com/johnmwalker/Basic-2D-Physics-Game-in-Python) which I developed to gain some insight on the feasibility of guiding students to build such a physics engine for future offerings of PHYS 398 at the University of St. Thomas in St. Paul, MN.

My personal goal with this project was to learn about the basics of making a true physics engine while also keeping things as efficient and entertaining as possible. Hence I resorted to PyBullet for collision detection as I failed to find any resources to guide me explicitly through efficient detection and sorting algorithms.

Select your level (in App.py) and run the App.py file.

### Requires:

* Numpy
* Scipy
* Pyglet
* [ModernGL 5](https://github.com/cprogrammer1994/ModernGL)
* PyBullet

### Features: 

* N-body gravity
* Basic collisions
* RK4 Solver

### Levels: 

Note that levels 1-5 are holdovers from previous versions and do not currently work.
6-11 are functional.

### Controls: 

* Numpad + and - speed up and slow down the simulation.
    
### Issues:

* Collisions are awkward. Some look totally fine while others clearly don't behave like they should.

* Normal forces and non-penetration constraints are not currently working, so things will gradually phase into each other.


