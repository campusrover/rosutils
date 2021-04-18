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
from dotenv import dotenv_values

MODES = ['cloud', 'robot']
TYPES = ['tb3', 'minirover', 'bullet']
ROBOTS = ['pitosalas', 'bullet']
IP = {'pitosalas': '100.72.171.120', 'bullet': '100.120.93.84'}
MASTER_IP = {'pitosalas': '100.72.171.120', 'bullet': '100.120.93.84'}
TYPE_MAP = {'pitosalas':'minirover', 'bullet':'bullet', 'robc' : 'tb3'}

@click.group(help="Brandeis Robotics utilities. Configure for different kinds of robots.")
def cli():
    pass

@cli.command()
def status():
    click.echo("# Current bru status")
    click.echo("robot-name:     {0}".format(config["BRU_NAME"]))
    click.echo("robot-type:     {0}".format(config["BRU_TYPE"]))
    click.echo("run-mode:       {0}".format(config["BRU_MODE"]))
    click.echo("rosmaster-ip:   {0}".format(config["ROS_MASTER_URI"]))
    click.echo("self-ip:        {0}".format(config["ROS_IP"]))


@cli.command()
@click.option('-l', '--list', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(MODES, case_sensitive=False))
def mode(list, name):
    if (list):
        click.echo("# Available environments: ")
        [click.echo("{0}".format(rname)) for envname in MODES]
    else:
        set_mode(name)

@cli.command()
@click.option('--list', '-l', help='list available options', is_flag=True)
@click.argument('name', type=click.Choice(ROBOTS, case_sensitive=False))
def robot(list, name):
    if (list):
        click.echo("# Available robots: ")
        [click.echo("{0} ({1})".format(rname, IP[rname])) for rname in ROBOTS]
    else:
        set_robot(name)

@cli.command(help="Launch ROS packages. This will customize and run a launch file based on your current configuration.")
@click.option('--list', help='list available options')
def launch(list):
    click.echo("launch")

def append_env_value(dict, key, default):
    val = os.environ.get(key)
    dict[key] = val if val != None else default

def get_env_variables():
    config = {}
    append_env_value(config, "BRU_TYPE", "na")
    append_env_value(config, "BRU_MODE", "na")
    append_env_value(config, "BRU_NAME", "na")
    append_env_value(config, "BRU_TYPE", "na")
    append_env_value(config, "BRU_VPN_IP", "na")
    append_env_value(config, "ROS_IP", "na")
    append_env_value(config, "ROS_MASTER_URI", "na")
    
    return dict(config)

def save_env_variables(config):
    with open(os.path.expanduser('~/.bruenv'), "w") as f:
        f.write("# environment variables set by rpt.py\n")
        { f.write("export {0}={1}\n".format(k,v)) for (k,v) in config.items() }
    click.echo("# Remember to source ~/.bruenv!")

def set_robot(name):
    cfg = get_env_variables()
    cfg["BRU_NAME"] = name
    cfg["ROS_MASTER_URI"]="http://{0}:11311".format(MASTER_IP[name])
    cfg["ROS_IP"] = cfg["BRU_VPN_IP"]
    cfg["BRU_TYPE"] = TYPE_MAP[name]
    save_env_variables(cfg)

def set_mode(name):
    cfg = get_env_variables()
    cfg["BRU_MODE"] = name
    save_env_variables(cfg)

if __name__ == '__main__':
    config = get_env_variables()
    cli()


