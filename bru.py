#!/usr/bin/env python
"""
BRU: working with robots from the Brandeis Robotics Lab

$ BRU [command] [subcommand] [-arguments]

All commands and subcommands can be abbreviated to a single letter

COMMANDS:

s[tatus] - show the current bru settings
e[vironment] - display all environment variables
n[ame] - set the robot name
m[ode] - set mode, from real, sim and oboard
l[launch] - a set of actual action commands

ARGUMENTS
-l - list
INSTALLATION
Something like this but specifics vary depending on where you are installing

Install click library, see: https://click.palletsprojects.com/en/7.x/quickstart/#virtualenv
$ ln -s /my_ros_data/rosutils/bru.py /usr/local/bin/bru
$ ~/rosutils$ chmod +x bru.py 

"""
import click
import os
import subprocess

MODES = ['sim', 'real', 'onboard']
TYPES = ['tb3', 'minirover', 'bullet']
ROBOTS = ['pitosalas', 'bullet', 'robc', 'mr1', 'mr2']
ROBOT_VPNIP = {'pitosalas': '100.120.93.84', 'bullet': '100.120.93.84', 'mr1': '100.80.161.82', 'mr2' : '100.120.93.84'}
TYPE_MAP = {'pitosalas':'minirover', 'bullet':'bullet', 'robc' : 'tb3', 'mr1': 'minirover', 'mr2' : 'minirover'}
LAUNCH_TYPES = ['bringup', 'stage_2', 'rviz']

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
        self.append_env_value(config, "BRU_MODE", "real")
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
        self.calc_ip()
        self.save_env_variables()

    def save_env_variables(self):
        with open(os.path.expanduser('~/.bruenv'), "w") as f:
            f.write("# environment variables set by rpt.py\n")
            { f.write("export {0}={1}\n".format(k,v)) for (k,v) in self.cfg.items() }
        click.echo("# Remember to source ~/.bruenv!")

    def set_robot(self, name):
        self.cfg["BRU_NAME"] = name
        self.cfg["BRU_TYPE"] = TYPE_MAP[name]
        self.calc_ip()
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

    def  calc_ip(self):
        if self.cfg["BRU_MODE"] == "sim":
            self.cfg["ROS_IP"] = self.my_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_ip)
        elif self.cfg["BRU_MODE"] == "real":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(ROBOT_VPNIP[self.cfg["BRU_NAME"]])
        elif self.cfg["BRU_MODE"] == "onboard":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_vpn_ip)
        else:
            click.echo("*** bug in bru.py calc_ip")


    def echo_env(self):
        click.echo("# Relevant global environment variables\n")
        cmd1 = subprocess.Popen("printenv | grep ROS", shell=True, stdout=subprocess.PIPE)
        cmd2 = subprocess.Popen("printenv | grep BRU", shell=True, stdout=subprocess.PIPE)
        cmd1_out = cmd1.stdout.read()
        cmd2_out = cmd2.stdout.read()
        click.echo(cmd1_out)
        click.echo(cmd2_out)

    # Launch commands maps a set of ('mode', 'robot type', 'launch command' to a launch string)
    LAUNCH_MAP = { ('onboard', 'minirover', 'bringup'): "roslaunch minirover mr_bru_bringup.launch lidar:={0} camera:={1} joy:={3}",
                ('real', 'minirover', 'rviz'): "roslaunch minirover mr_bru_rviz.launch desc:={2} gmapping:={4}" }

    def launch(self, launch_name, lidar, camera, desc, joy, gmapping):
        click.echo("Launching...{}: lidar: {}, camera: {}, desc: {}".format(launch_name, lidar, camera, desc))
        launch_pattern = (self.cfg["BRU_MODE"], self.cfg["BRU_TYPE"], launch_name)
        if launch_pattern in Bru.LAUNCH_MAP:
            click.echo(Bru.LAUNCH_MAP[launch_pattern].format(lidar, camera, desc, joy, gmapping))
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

@cli.command(help='Set name of Robot')
@click.option('--list', '-l', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(ROBOTS))
@click.pass_obj
def name(bru, list, name):
    if (list):
        click.echo("# Available robots: ")
        [click.echo("{0} ({1})".format(rname, IP[rname])) for rname in ROBOTS]
    else:
        bru.set_robot(name)

@cli.command(help="Launch ROS packages. This will customize and run a launch file based on your current configuration.")
@click.option('--list', '-l', help='list available options', flag_value='list')
@click.option('--camera/--nocamera', default=True)
@click.option('--lidar/--nolidar', default=True)
@click.option('--desc/--nodesc', default=True)
@click.option('--joy/--nojoy', default=False)
@click.option('--gmapping/--nogmapping', default=False)
@click.argument('launch', type=click.Choice(LAUNCH_TYPES), required=False)
@click.pass_obj
def launch(bru, launch, list, lidar, camera, desc, joy, gmapping):
    if (list or launch == None):
        click.echo("# Available launches options: ")
        [click.echo("{0}".format(lt)) for lt in LAUNCH_TYPES]
    else:
        bru.launch(launch, lidar, camera, desc, joy, gmapping)

if __name__ == '__main__':
    cli()


