import sys
import tempfile

import yaml


with open(sys.argv[1], encoding="utf-8") as source:
    params = yaml.safe_load(source)

params["bt_navigator"]["ros__parameters"]["wait_for_service_timeout"] = 10000
params["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] = "cmd_vel_safe"
# goal_checker = params["controller_server"]["ros__parameters"]["general_goal_checker"]
# goal_checker["xy_goal_tolerance"] = 0.5
# goal_checker["yaw_goal_tolerance"] = 0.5
amcl = params["amcl"]["ros__parameters"]
for parameter in ("alpha1", "alpha2", "alpha3", "alpha4", "alpha5"):
    amcl[parameter] = 0.05
amcl["z_hit"] = 0.8
amcl["z_rand"] = 0.2
amcl["laser_likelihood_max_dist"] = 10.0
amcl["update_min_d"] = 0.1
amcl["update_min_a"] = 0.1
for costmap, layer in (("local_costmap", "voxel_layer"), ("global_costmap", "obstacle_layer")):
    observation_layer = params[costmap][costmap]["ros__parameters"][layer]
    observation_layer["observation_sources"] = "scan scan_3d"
    observation_layer["scan_3d"] = {**observation_layer["scan"], "topic": "scan_3d"}

with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as output:
    yaml.safe_dump(params, output)
    sys.stdout.write(output.name)