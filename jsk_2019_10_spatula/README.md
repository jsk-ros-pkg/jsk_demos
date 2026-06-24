# PR2 Force Test

The goal is to find out how accurate the effort data from PR2 is.

### pr2-spatula-force-test.l
- with this little demo Pr2 scrapes along a bowl 40 times, which it is holding in its left gripper using a spatula with its right gripper
- the demo does not include picking up the bowl and spatula, PR2 waits 5 seconds before closing one griper at a time, the spatula and bowl have to be placed in the gripper manually

### plot_bag.py, PR2_joints.pdf and bagfiles
- the task was executed three times:
  1. Once as intended with spatula and bowl in both hands.
  2. Once without the bowl
  3. Once without the spatula
  This is used to compare the effort signals with and without contact.
  The effort of the right hand from version 1 is compared to the effort of the right arm version 2.
  The effort of the left hand version one is compared to the effort of the left arm version 3.
  This comparison is used to see if contact can be detected in the effort signals.
  
- the data can then be extracted split and plot with the python file. To do so:
  1. download the bagfiles from the following link: https://drive.google.com/drive/folders/1gq6TnKa5HTYQjAxiZ2W5w2jUel8rgWeL
  2. change the absolute path to the bagfiles in the python file manually
  3. decide in which joint you are interested and put that also manually in the main() of the python file
      PR2_joints.pdf (adapted from the pr2 manual) can be used to find the name of the joint you are interested in: the upper blue names are the joint names used in the python file
  4.) The python script will split the bag file at the position where the scraping movement starts, so that all 40 scraping movements of one bag file are plot on top of each other
  5.) The plot shows the effort in the top and the position in the bottom, by default it also plots crosses in the bottom view, where the scraping movement stopped
      - the experiment with spatula and bowl is plot in red
      - the experiment without spatula is plot in blue
      - the experiment without bowl is plot in green
- the data can also be lowpass filtered if wished, eg. add the argument *cutoff_f = 10* when calling plot_data inside the main() to apply a low pass filter with a cutoff frequency of 10Hz, you can also define the *order* (default is 6) and the sampling rate *fs* (default is 100.3)
      
