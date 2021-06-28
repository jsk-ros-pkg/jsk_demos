"""Running eus scripts as demonstration.
"""
#!/usr/bin/env python
import rospy
import os
from os.path import join
import shlex
import subprocess


class ScriptDemonstrationCollector(object):

    def __init__(self, initializer_script_path, main_script_path, save_dir_base='/home/amabe/rosbag/panda'):
        """Euslisp script based demonstration collector.

        Args:
            initializer_script_path (str): Path to initializer script.
            main_script_path (str): Path to main script.
            save_dir_base (str): Path to save rosbag file
        """
        self.initializer_script_path = initializer_script_path
        self.main_script_path = main_script_path
        self.save_dir_base = save_dir_base

    def get_demonstration(self):
        """Getting demonstration
        """
        self.run_eus_scrpt(self.initializer_script_path, wait=True)
        self.start_rosbag_record()
        self.run_eus_scrpt(self.main_script_path, wait=True)
        self.stop_rosbag_record()
        self.check_if_save_demo()

    def run_eus_scrpt(self, file_path, wait=True):
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
        self.target_dir = join(self.save_dir_base, 'cable-insertion_naive')
        self.num_bagfies = len([name for name in os.listdir(
            self.target_dir) if os.path.isfile((self.target_dir, name))])
        options = "save_dir:={} bagfile_prefix:={:03}".format(
            self.target_dir, self.num_bagfies)
        rospy.loginfo(
            "Start recording, saving file in {}...".format(self.target_dir))
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
        save_demo = raw_input("Save this trajectory? [yes/no]")
        if save_demo == 'no':
            for f in os.listdir(self.target_dir):
                if f.startswith('{:03}'.format(self.num_bagfies)):
                    bagfile_name = join(self.target_dir, f)
                    cmd = "rm -rf {}".format(bagfile_name)
                    cmd = shlex.split(cmd)
                    subprocess.call(cmd)

initializer_script_path = "/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/euslisp/cable-insertion_naive_init.l"
main_script_path = "/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/euslisp/cable-insertion_naive_insert.l"
node = ScriptDemonstrationCollector(initializer_script_path=initializer_script_path,main_script_path=main_script_path)
node.get_demonstration()
