PR2 Force Test
- the goal is to find out how accurate the effort data from PR2 is
- with the little demo in pr2-spatula-force-test.l Pr2 scrapes along a bowl, which it is holding in its left gripper using a spatula with its right gripper
- the task was executed three times, once as intended with spatula and bowl in both hands. To be able to see if contact can be detected in effort data that comes from the current, the task is once executed without the bowl in the left Hand, the data of PR2's right Arm can now be compared bewteen the one with contact with the bowl and the one without any contact. The task is execited again, this time without the spatula. Here the data of the left arm can be compared to the data without any contact

-to run the python file that visualizes the effort with and without contact, download the bagfiles from the link shown below and change the path in the python file 

the log files and a video of the task can be found here
https://drive.google.com/drive/folders/1gq6TnKa5HTYQjAxiZ2W5w2jUel8rgWeL

- PR2_joints.pdf can be used to find the names of all joints, the upper blue names can be used in plot_bag.py to specify the joint of which the effort should be plot
