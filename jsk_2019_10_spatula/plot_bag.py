import rosbag
import numpy as np
import matplotlib.pyplot as plt


def main():
	#change the path here to the absolute path, where you downloaded the bagfiles to
	bag_spatula_and_bowl = '/home/leus/experiment_with_spatula_and_bowl_2019-09-27-16-21-18.bag'
	bag_no_spatula = '/home/leus/experiment_without_spatula_2019-09-27-17-09-42.bag'
	bag_no_bowl = '/home/leus/experiment_without_bowl_2019-09-27-17-02-24.bag'

	#change the joint here to the joint you want to plot
	joint = "r_upper_arm_roll_joint"

	d_spatula_and_bowl = data_analysis(joint,bag_spatula_and_bowl)
	d_spatula_and_bowl.extract_bag_data()
	d_spatula_and_bowl.split_data()
	d_spatula_and_bowl.color = 'r'

	d_no_spatula = data_analysis(joint,bag_no_spatula)
	d_no_spatula.extract_bag_data()
	d_no_spatula.split_data()
	d_no_spatula.color = 'b'

	d_no_bowl = data_analysis(joint,bag_no_bowl)
	d_no_bowl.extract_bag_data()
	d_no_bowl.split_data()
	d_no_bowl.color = 'g'

	if joint[0] == "l":
		print "you picked a joint from left arm, plotting spatula_and_bowl and no_spatula"
		data_list = [d_spatula_and_bowl,d_no_spatula]
	elif joint[0] == "r":
		print "you picked a joint from right arm, plotting spatula_and_bowl and no_bowl"
		data_list = [d_spatula_and_bowl,d_no_bowl]
	else:
		print "you picked a joint that is not from either arm plotting all three experiments"
		data_list = [d_spatula_and_bowl,d_no_spatula,d_no_bowl]

	plot_data(data_list, show_av2=True)



class data_analysis:

	def __init__(self,joint_name,path,debug_mode=False):
		#enter the joint name of the effort you want to plot here
		#joint_name = "r_upper_arm_roll_joint"
		#debug_mode = True

		#the path to the logfiles
		self.bag = rosbag.Bag(path)
		print self.bag
		self.joint_name = joint_name
		self.debug_mode = debug_mode

	def extract_bag_data(self):
		self.effort = []
		self.position = []
		self.velocity = []
		self.split_indices = []
		self.av2_indices = []
		first = True

		#when the robot starts scraping it is at av_1, when it lifts the spatula off the bowl it is at position av_2
		#in order to split the bag file that contains 40 iterations, this angle vector is 
		#compared to the angle vector of the bag file, if it the difference is small, the file is split
		self.av_1 = (2*np.pi/360) * np.array([49.9331,62.6833,33.1418,127.946,-117.444,-7.41574,-5.72958,51.833,-16.9966,-9.03369,-111.73,-116.714,-76.193,-57.7095,266.18,3.0727,-21.2682])
		self.av_2 = (2*np.pi/360) * np.array([49.9331,62.3888,32.6716,129.774,-117.502,-7.41574,-5.72958,51.833,-29.1714,-9.19364,-117.398,-118.456,-81.5623,-50.4353,263.348,3.0727,-21.2682])
		#names of 17 jointpositions in av_1 and av_2
		self.cmd_joints = ["torso_lift_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint","r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint","head_pan_joint","head_tilt_joint"]

		n_split = 0
		if self.debug_mode:
			max_diff = 0
			min_diff = float('inf')

		split_window = dict()
		av2_window = dict()
		
		i = 0
		for topic, msg, t in self.bag.read_messages(''):
		    self.effort.append(msg.effort)
		    self.position.append(msg.position)
		    self.velocity.append(msg.velocity)
		    if first:
		        self.name = msg.name
		        cmd_indices_bag = []
		        for joint in self.cmd_joints:
		            if joint in self.name:
		                cmd_indices_bag.append(self.name.index(joint))
		        first = False
		    
		    pos_array = np.array(msg.position)
		    if self.debug_mode:
		        if np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1)) < min_diff:
		        	min_diff = np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1))
		        if np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1)) > max_diff:
		        	max_diff = np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1))

		    diff_limit = 1.1
		    if np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1)) < diff_limit:
		        n_split = n_split + 1 
		        split_window[np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_1)/self.av_1))] = i

		    elif split_window:
		    	self.split_indices.append(split_window[np.min(split_window.keys())])
		        split_window = dict()

		    if np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_2)/self.av_2)) < diff_limit:
		        n_split = n_split + 1 
		        av2_window[np.sum(np.abs((pos_array[cmd_indices_bag]-self.av_2)/self.av_2))] = i

		    elif av2_window:
		    	self.av2_indices.append(av2_window[np.min(av2_window.keys())])
		        av2_window = dict()
		    i = i+1

		self.av2_indices = np.array(self.av2_indices) - np.array(self.split_indices)

		if self.debug_mode:
		    print "n_split"
		    print n_split
		    print "max difference"
		    print max_diff
		    print "min difference"
		    print min_diff

		self.bag.close();
		self.effort = np.array(self.effort)
		self.position = np.array(self.position)
		self.velocity = np.array(self.velocity)
		self.ind_joint = self.name.index(self.joint_name)

	def split_data(self):
		old_ind = 0
		self.n_exp = len(self.split_indices)
		effort_shape = np.shape(self.effort)
		n_joint = effort_shape[1]
		tmp_indices = self.split_indices[0:-1]
		tmp_indices.insert(0,0)
		n_time = np.max(np.array(self.split_indices) - np.array(tmp_indices))
		self.split_effort = np.array(np.zeros([self.n_exp,n_time,n_joint]))
		self.split_position = np.array(np.zeros([self.n_exp,n_time,n_joint]))
		i = 0
		for split_ind in self.split_indices:
			diff_ind = split_ind - old_ind
			self.split_effort[i,0:split_ind-old_ind,:] = self.effort[old_ind:split_ind,:]
			self.split_position[i,0:split_ind-old_ind,:] = self.position[old_ind:split_ind,:]
			old_ind = split_ind
			i = i+1


def plot_data(data_list,split_plot = True,show_av2=False,lowpass_filter=False):
	t1 = np.array(range(920))
	fig, axs = plt.subplots(2, 1)
	for data in data_list:
		for i in range(3,data.n_exp):
			s1 = data.split_effort[i,0:920,data.ind_joint]
			axs[0].plot(t1, s1, data.color)

		for i in range(3,data.n_exp):
			s2 = data.split_position[i,0:920,data.ind_joint]
			axs[1].plot(t1, s2, data.color)
		if show_av2:
			for av2_idx in data.av2_indices:
				axs[1].plot(av2_idx,data.av_2[data.cmd_joints.index(data.joint_name)],'%s x' % data.color)
	axs[0].set_xlabel('time')
	axs[0].set_ylabel('effort_%s' % data.name[data.ind_joint])
	axs[0].grid(True)  
	axs[1].set_xlabel('time')
	axs[1].set_ylabel('position_%s' % data.name[data.ind_joint])
	axs[1].grid(True) 

	fig.tight_layout()
	plt.show()


if __name__ == "__main__":
	main()