# RobotSystems
Assignment for Moving the arm, tracking and placing in the position: 

The solution is placed in MoveNew.py file, please place the file in armpi_fpv/src folder and do 
`chmod a+x MoveNew.py`  then run using `rosrun armpi_fpv MoveNew.py`
 

Assignment for Perception on HiWonderArm: 

For testing, please make sure to place this file armpi_fpv/src/ and copy all the packages from Ai_FPV to src 

1)Since getAreaMaxContour has no change across all 3 files I haven't  changed  anything on it.

2)Skipped move and init() functions, since they are control functions.

3)Tested run_track() for tracking the red-cube with the arm.

*Perception Flow Chart for tracking and palletising*

![Flow Chart for Tracking](img/1.png)
![Flow Chart for Sorting and Palletizing](img/2.png)



