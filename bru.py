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
LAUNCH_TYPES = ['bringup', 'stage_2']
LAUNCH_PERMITTED_MODES = {'bringup' : ['onboard']}
LAUNCH_PERMITTED_OPTIONS = {'bringup' : ['camera', 'lidar']}

# Launch commands maps a set of ('mode', 'robot type', 'launch command' to a launch string)
LAUNCH_MAP = { ('onboard', 'minirover', 'bringup'): "xxx"}

class Bru(object):
    def __init__(self):
        self.get_env_variables()
        self.my_ip = os.environ.get("BRU_MY_IP")
        self.my_vpn_ip = os.environ.get("BRU_VPN_IP")
        if not all([self.my_vpn_ip, self.my_ip]):
            click.echo("You have to set up the two environment variables BRU_MY_IP and BRU_VPN_IP")
            click.echo("export BRU_MY_IP=$(myip); export BRU_VPN_IP=$(myvpnip)")
            exit(1)

    def get_env_variables(self):
        config = {}
        self.append_env_value(config, "BRU_TYPE", "minirover")
        self.append_env_value(config, "BRU_MODE", "cloud")
        self.append_env_value(config, "BRU_NAME", "pitosalas")
        self.append_env_value(config, "BRU_MY_IP", "invalid")
        self.append_env_value(config, "BRU_VPN_IP", "invalid")
        self.append_env_value(config, "ROS_IP", "invalid")
        self.append_env_value(config, "ROS_MASTER_URI", "na")
        self.cfg = config

    def append_env_value(self, dict, key, default):
        val = os.environ.get(key)
        dict[key] = val if val != None else default

    def set_mode(self, name):
        self.cfg["BRU_MODE"] = name
        if name == "sim":
            self.cfg["ROS_IP"] = self.my_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_ip)
        elif name == "cloud":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(MASTER_IP[self.cfg["BRU_NAME"]])
        elif name == "onboard":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_vpn_ip)
        else:
            click.echo("*** bug in bru.py set_mode")
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

    def echo_env(self):
        click.echo("# Relevant global environment variables\n")
        cmd1 = subprocess.Popen("printenv | grep ROS", shell=True, stdout=subprocess.PIPE)
        cmd2 = subprocess.Popen("printenv | grep BRU", shell=True, stdout=subprocess.PIPE)
        cmd1_out = cmd1.stdout.read()
        cmd2_out = cmd2.stdout.read()
        click.echo(cmd1_out)
        click.echo(cmd2_out)

    def launch(self, launch_name, lidar, camera):
        click.echo("Launching...{}: lidar: {}, camera: {}".format(launch_name, lidar, camera))
        launch_pattern = (self.cfg["BRU_MODE"], self.cfg["BRU_TYPE"], launch_name)
        if launch_pattern in LAUNCH_MAP:
            click.echo(LAUNCH_MAP[launch_pattern])
        else:
            click.echo('That launch option is not available')
            

@click.group(help="Brandeis Robotics utilities. Configure for different kinds of robots.")
@click.pass_context
def cli(ctx):
    ctx.obj = Bru()

@cli.command(help='Display current Bru sttings')
@click.pass_obj
def status(bru):
    bru.echo_status()

@cli.command(help='Display all relevant Environment variables')
@click.pass_obj
def env(bru):
    bru.echo_env()

@cli.command(help='Set running modes')
@click.option('-l', '--list', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(MODES))
@click.pass_obj
def mode(bru, list, name):
    if (list):
        click.echo("# Available environments: ")
        [click.echo("{0}".format(envname)) for envname in MODES]
    else:
        bru.set_mode(name)

@cli.command(help='Set type of Robot')
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
@click.option('--list', '-l', help='list available options', flag_value='list')
@click.option('--camera/--nocamera', default=True)
@click.option('--lidar/--nolidar', default=True)
@click.argument('launch', type=click.Choice(LAUNCH_TYPES), required=False)
@click.pass_obj
def launch(bru, launch, list, lidar, camera):
    if (list or launch == None):
        click.echo("# Available launches options: ")
        [click.echo("{0}".format(lt)) for lt in LAUNCH_TYPES]
    else:
        bru.launch(launch, lidar, camera)

if __name__ == '__main__':
    cli()


