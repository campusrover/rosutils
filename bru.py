#!/usr/bin/env python
"""
BRU: working with robots from the Brandeis Robotics Lab

$ BRU [command] [subcommand] [-arguments]

All commands and subcommands can be abbreviated to a single letter

COMMANDS:

s[tatus] - show the current settings
e[vironment] - set the execution enviornment. Values are c (cloud), r (robot)
r[robot] - set the robot. Values are tb3 (turtlebot3), mr (minirover), bull (bullet)
l[launch] - a set of actual action commands

ARGUMENTS
-l - list

EXAMPLES
BRU l bringup (will only work if running on a robot)
BRU l navigate
BRU l slam
BRU e robot 
BRU r pitosalas

INSTALLATION
Something like this but specifics vary depending on where you are installing

Install click library, see: https://click.palletsprojects.com/en/7.x/quickstart/#virtualenv
$ ln -s /my_ros_data/rosutils/bru.py /usr/local/bin/bru
$ ~/rosutils$ chmod +x bru.py 

"""
import click
import os
import subprocess

MODES = ['sim', 'robot', 'onboard']
TYPES = ['tb3', 'minirover', 'bullet']
ROBOTS = ['pitosalas', 'bullet', 'robc']
IP = {'pitosalas': '100.72.171.120', 'bullet': '100.120.93.84', 'robc': 'x.x.x.x'}
MASTER_IP = {'pitosalas': '100.72.171.120', 'bullet': '100.120.93.84', 'robc' : 'x.x.x.x'}
TYPE_MAP = {'pitosalas':'minirover', 'bullet':'bullet', 'robc' : 'tb3'}
LAUNCH_TYPES = ['stage_2']
LAUNCH_COMMANDS = { ('tb3', 'sim', 'stage4'): ["roslaunch", "turtlebot3_gazebo", "turtlebot3_stage_4.launch"],
                    ('tb3', 'sim',  'slam'):  ["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"]
                    }

class Bru(object):
    def __init__(self):
        self.get_env_variables()
        self.my_ip = os.environ.get("BRU_MY_IP")
        self.my_vpn_ip = os.environ.get("BRU_VPN_IP")
        if not all([self.my_vpn_ip, self.my_ip]):
            click.echo("You have to set up the two environment variables BRU_MY_IP and BRU_VPN_IP")
            exit(1)

    def get_env_variables(self):
        config = {}
        self.append_env_value(config, "BRU_TYPE", "minirover")
        self.append_env_value(config, "BRU_MODE", "cloud")
        self.append_env_value(config, "BRU_NAME", "pitosalas")
        self.append_env_value(config, "ROS_IP", "na")
        self.append_env_value(config, "ROS_MASTER_URI", "na")
        self.save_env_variables()
        self.cfg = config

    def append_env_value(self, dict, key, default):
        val = os.environ.get(key)
        dict[key] = val if val != None else default

    def set_mode(self, name):
        self.cfg["BRU_MODE"] = name
        if name == "sim":
            self.cfg["ROS_IP"] = self.my_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_ip)
        else:
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(MASTER_IP[self.cfg["BRU_NAME"]])
        self.save_env_variables()

    def save_env_variables(self):
        with open(os.path.expanduser('~/.bruenv'), "w") as f:
            f.write("# environment variables set by rpt.py\n")
            { f.write("export {0}={1}\n".format(k,v)) for (k,v) in self.cfg.items() }
        click.echo("# Remember to source ~/.bruenv!")

    def set_robot(self, name):
        self.cfg["BRU_NAME"] = name
        self.cfg["ROS_MASTER_URI"]="http://{0}:11311".format(MASTER_IP[name])
        self.cfg["ROS_IP"] = self.my_vpn_ip
        self.cfg["BRU_TYPE"] = TYPE_MAP[name]
        self.save_env_variables()

    def echo_status(self):
        click.echo("# Current bru status")
        click.echo("robot-name:     {0}".format(self.cfg["BRU_NAME"]))
        click.echo("robot-type:     {0}".format(self.cfg["BRU_TYPE"]))
        click.echo("run-mode:       {0}".format(self.cfg["BRU_MODE"]))
        click.echo("my-ip:          {0}".format(self.my_ip))
        click.echo("vpn-ip:         {0}".format(self.my_vpn_ip))
        click.echo("ros-master-uri: {0}".format(self.cfg["ROS_MASTER_URI"]))
        click.echo("ros-ip:         {0}".format(self.cfg["ROS_IP"]))

    def launch(self, launch_name):
        # list_files = subprocess.run(["roslaunch", "turtlebot3_gazebo", "turtlebot3_stage_4.launch"])
        list_dir = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"])
        list_dir.wait()

@click.group(help="Brandeis Robotics utilities. Configure for different kinds of robots.")
@click.pass_context
def cli(ctx):
    ctx.obj = Bru()

@cli.command()
@click.pass_obj
def status(bru):
    bru.echo_status()

@cli.command()
@click.option('-l', '--list', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(MODES))
@click.pass_obj
def mode(bru, list, name):
    if (list):
        click.echo("# Available environments: ")
        [click.echo("{0}".format(envname)) for envname in MODES]
    else:
        bru.set_mode(name)

@cli.command()
@click.option('--list', '-l', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(ROBOTS))
@click.pass_obj
def robot(bru, list, name):
    if (list):
        click.echo("# Available robots: ")
        [click.echo("{0} ({1})".format(rname, IP[rname])) for rname in ROBOTS]
    else:
        bru.set_robot(name)

@cli.command(help="Launch ROS packages. This will customize and run a launch file based on your current configuration.")
@click.option('--list', help='list available options')
@click.argument('launch', type=click.Choice(LAUNCH_TYPES))
@click.pass_obj
def launch(bru, launch, list):
    if (list):
        click.echo("# Available launches: ")
        [click.echo("{0}".format(lt)) for lt in LAUNCH_TYPES]
    else:
        bru.launch(launch)

if __name__ == '__main__':
    cli()


