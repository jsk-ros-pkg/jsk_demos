"""Running eus scripts as demonstration.
"""
#!/usr/bin/env python
import rospy
import os
from os.path import join
import shlex
import subprocess


class ScriptDemonstrationCollector(object):

    def __init__(self, initializer_script_path, setting_script_path, main_script_path, save_dir_base='/home/amabe/rosbag/panda/cable-insertion_heuristic'):
        """Euslisp script based demonstration collector.

        Args:
            initializer_script_path (str): Path to initializer script.
            main_script_path (str): Path to main script.
            save_dir_base (str): Path to save rosbag file
        """
        self.initializer_script_path = initializer_script_path
        self.setting_script_path = setting_script_path
        self.main_script_path = main_script_path
        self.save_dir_base = save_dir_base

    def get_demonstration(self):
        """Getting demonstration
        """
        self.run_eus_script(self.initializer_script_path, wait=True)
        self.start_rosbag_record()
        self.run_eus_script(self.main_script_path, wait=True)
        self.stop_rosbag_record()
        self.check_if_save_demo()

    def run_eus_script(self, file_path, wait=True):
        """Run euslistp script

        Args:
            file_path (str): executable file path
            wait (bool, optional): Wait until the script to be finished Defaults to True.

        Raises:
            RuntimeError: File not found.

        Returns:
            int: PID of subproces.
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(
                "{}, file path dosen't exist".format(file_path))
        command = shlex.split("{}".format(file_path))
        rospy.loginfo('Running {}...', file_path)
        proc = subprocess.Popen(command)
        if wait:
            proc.communicate()
        return proc

    def start_rosbag_record(self):
        """Start rosbag record subprocess.
        """
        print("start rosbag")
        self.save_dir_base
        options = "save_dir:={}".format(self.save_dir_base)
        rospy.loginfo(
            "Start recording, saving file in {}...".format(self.save_dir_base))
        command = "roslaunch jsk_panda_startup panda_record.launch {}".format(
            options)
        command = shlex.split(command)
        self.rosbag_proc = subprocess.Popen(command)

    def stop_rosbag_record(self):
        """Kill rosbag subprocess to end record.
        """
        rospy.loginfo("Stop recording rosbag...")
        if self.rosbag_proc is not None:
            self.rosbag_proc.send_signal(subprocess.signal.SIGINT)

    def check_if_save_demo(self):
        """Ask user if you want to save this demonstration or not.
        """
        while True:
            save_demo = raw_input("Success, fault, don't save [s/f/n]")
            if save_demo == 's':
                self.target_dir = join(self.save_dir_base, 'success')
                self.num_bagfiles = len([name for name in os.listdir( self.target_dir) if os.path.isfile(os.path.join(self.target_dir, name))])
                for f in os.listdir(self.save_dir_base):
                    if os.path.isfile(os.path.join(self.save_dir_base,f)):
                        bagfile_name_origin = join(self.save_dir_base, f)
                        bagfile_name_con = join(self.target_dir, '{}_'.format(self.num_bagfiles) + f)
                        cmd = "mv {} {}".format(bagfile_name_origin,bagfile_name_con)
                        cmd = shlex.split(cmd)
                        subprocess.call(cmd)
                break
                    
            if save_demo == 'f':
                self.target_dir = join(self.save_dir_base, 'fault')
                self.num_bagfiles = len([name for name in os.listdir( self.target_dir) if os.path.isfile(os.path.join(self.target_dir, name))])
                for f in os.listdir(self.save_dir_base):
                    if os.path.isfile(os.path.join(self.save_dir_base,f)):
                        bagfile_name_origin = join(self.save_dir_base, f)
                        bagfile_name_con = join(self.target_dir, '{}_'.format(self.num_bagfiles) + f)
                        cmd = "mv {} {}".format(bagfile_name_origin,bagfile_name_con)
                        cmd = shlex.split(cmd)
                        subprocess.call(cmd)
                break
                    
            if save_demo == 'n':
                print("trashing this data")
                for f in os.listdir(self.save_dir_base):
                    if os.path.isfile(os.path.join(self.save_dir_base,f)):
                        bagfile_name = join(self.save_dir_base, f)
                    cmd = "rm -rf {}".format(bagfile_name)
                    print(cmd)
                    cmd = shlex.split(cmd)
                    subprocess.call(cmd)
                break

initializer_script_path  = "/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/euslisp/cable-insertion_heuristic_init.l"
setting_script_path = "/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/euslisp/cable-insertion_heuristic_grasp.l"
main_script_path = "/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/euslisp/cable-insertion_naive_insert.l"
node = ScriptDemonstrationCollector(initializer_script_path=initializer_script_path,setting_script_path=setting_script_path,main_script_path=main_script_path)
node.get_demonstration()
