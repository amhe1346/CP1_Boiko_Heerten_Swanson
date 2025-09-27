Instructions: 

CP1_test2.py  is the main source code for the challenge problem one. It runs the project. Download and run it . 

thepdf of the report is uploaded as Challenge Problem 1 Report - ROBO 5000.pdf
grid.py is a practice of how to make the mapping system and is not used in cp1_test2.py






to do 
1. figure out switching (amy) 
2. add debugging - exit if stuck in loop (done?) 
3. editing - remove unnecessary codes  (Evan) done

4. Paper(done?)



Amy - Debugging updates 
1. updated so that the switch is called to check EVERY adjacent robot after all the robots have made their initial moves and switches if beneficial. not just when in conflict.
    it makes it so that switch is called every time step and checks
   
2..  updated so that it does not take a full timestep to put each robot to goal if there are multiple robots that shoudl reach goal in the same time step
3.  working on "Deadlocks" - why is the code looping sometimes
  a.Its possible moves (primary and secondary) are blocked by the grid boundary or by other robots.
  The can_move method returns False for both possible moves, due to another robot occupying the target cell or due to the movement rules for robot types.
  Its goal is occupied by another robot that cannot move away, or the only available moves are not allowed by the robot type rules.

  3.b dead lock type -  robot1 is occupying the goal for robot 2. robot 2 is in the path of robot 1 
    the can move blocks it if robots are in the positions . need to fix it to allow switching to happen  
    FIX: removed the check of can move in the "Switch" function. the can move is only about checking if robots can go into the spaces shared by another robot, so obviosly it was blocking alot of switches!!

3a. !! self.position = (row, col) means self.position[0] is the y-coordinate (row), and self.position[1] is the x-coordinate (column).
When you print self.position, it shows as (y, x), not (x, y).
When plotting with matplotlib, you use plt.plot(x, y, ...), so you pass self.position[1] as x and self.position[0] as y.!! 

so all the math has been working, but the x and y variables have actually been switched this whole time ! 

4. notices sometimes arrows anddots dont amtch colors green arrow to bluedot

Evan: Debugging
1. added labels to the robots, their goals, and when robots are at their goals. This will help with identificaiton.
2. Removed some bits of redundant code, renamed unnecessarily long variables, replaced if chains with switch statements when necessary, removed unnecessary print statements.
