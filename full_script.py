import subprocess
import time

# cmd = ["roslaunch", "decomp_test_node", "test_path_decomp_2d.launch"]
cmd = ["roslaunch decomp_test_node test_path_decomp_2d.launch"]
# proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
proc = subprocess.Popen(cmd, shell=True)#, shell=False, stdout=subprocess.PIPE)

# out1, _ = proc.communicate()
# # out1 = out1.decode("utf-8")
# print("out1")
# print(out1)
time.sleep(3)  

# cmd2 = ["roslaunch", "decomp_test_node", "test_png2pcd.launch"]
# stdout = subprocess.PIPE
cmd2 = ["roslaunch decomp_test_node test_png2pcd.launch"]
# proc2 = subprocess.Popen(cmd2, shell=True, stdout=subprocess.PIPE)
proc2 = subprocess.Popen(cmd2, shell=True)#, shell=False, stdout=subprocess.PIPE)

# out2, _ = proc2.communicate()
# out2 = out2.decode("utf-8")
# print("out2")
# print(out2)
time.sleep(3)  

# cmd3 = ["rosrun" "rviz" "rviz"]# "-d" "catkin_ws/png2pcl.rviz"]
cmd3 = ["rosrun rviz rviz -d png2pcl.rviz"]
proc3 = subprocess.Popen(cmd3, shell=True)

# print(proc3.stdout)

# print("done with both scripts")
time.sleep(3)

cmd4 = ["./../../../mosek/9.0/tools/examples/fusion/cxx/lownerjohn_ellipsoid "]
proc4 = subprocess.Popen(cmd4, shell=True)

time.sleep(3)

proc4.kill()
proc3.kill()
proc2.kill()
proc.kill()