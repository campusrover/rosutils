#!/usr/bin/python3

"""
BRU: working with robots from the Brandeis Robotics Lab
$ BRU [command] [subcommand] [-arguments]
All commands and subcommands can be abbreviated to a single letter. The goal of this tool is, 
to the extent possible, unify commands for controlling all the different robots we have. 
Each Robot has a name and a type. The names are found in the ROBOTS array and the types are in the TYPES array.
Mapping of a robot to a type can be found in the TYPE_MAP dict.
Setting a mode controls how ROS_MASTER_URI and ROS_IP are set. There are the following modes:
    sim - in the web environment, running with a simulated robot
    real - in the web environment, running with a real robot
    onboard - in the physical robot, using vpn IP
    labonboard - in the physical robot, using non-vpn IP
In order to be able to modify environment variables, bru commands must be invoked as follows:
`eval $(bru arg arg arg)`
COMMANDS:
s[tatus] - show the current bru settings
e[vironment] - display all environment variables
n[ame] - set the robot name
m[ode] - set mode, from real, sim and oboard
r[obot] - control a robot remotely
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
import paramiko
import time

MODES = ["sim", "real", "onboard", "labonboard"]
TYPES = ["tb3", "minirover", "bullet", "cat", "cube", "platform", "pupper"]
ROBOTS = [
    "pitosalas",
    "cat1",
    "bullet1",
    "cat1",
    "robc",
    "roba",
    "robb",
    "rafael",
    "donatello",
    "mr1",
    "august",
    "mr2",
    "alien",
    "mutant",
    "platform1",
    "platform2",
    "platform3",
    "platform4",
    "sim"
]
TYPE_MAP = {
    "pitosalas": "minirover",
    "bullet1": "bullet",
    "mr1": "minirover",
    "mr2": "minirover",
    "august": "minirover",
    "roba": "tb3",
    "robb": "tb3",
    "robc": "tb3",
    "rafael": "tb3",
    "donatello": "tb3",
    "cat1": "cat",
    "alien":"cube",
    "mutant":"cube",
    "platform1": "platform",
    "platform2": "platform",
    "platform3": "platform",
    "platform4": "platform",
    "doc": "pupper",
    "vnc": "vnc",
    "sim": "tb3"
}


class Bru(object):
    def __init__(self):
        self.get_env_variables()
        self.my_ip = os.environ.get("BRU_MY_IP")
        self.my_vpn_ip = os.environ.get("BRU_VPN_IP")
        if not all([self.my_vpn_ip, self.my_ip]):
            click.echo(
                "You have to set up the two environment variables BRU_MY_IP and BRU_VPN_IP"
            )
            click.echo("export BRU_MY_IP=$(myip); export BRU_VPN_IP=$(myvpnip)")
            exit(1)

    def get_env_variables(self):
        config = {}
        self.append_env_value(config, "BRU_TYPE", "invalid")
        self.append_env_value(config, "BRU_MODE", "invalid")
        self.append_env_value(config, "BRU_NAME", "invalid")
        self.append_env_value(config, "BRU_MY_IP", "invalid")
        self.append_env_value(config, "BRU_MASTER_IP", "invalid")
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
        self.export_env()

    def export_env(self):
        {click.echo("export {0}={1}".format(k, v)) for (k, v) in self.cfg.items()}

    def set_robot(self, name, master_ip):
        self.cfg["BRU_NAME"] = name
        self.cfg["BRU_TYPE"] = TYPE_MAP[name]
        self.cfg["BRU_MASTER_IP"] = master_ip
        self.calc_ip()
        self.export_env()

    def echo_status(self):
        click.echo("# Current bru status")
        click.echo("robot-name:     {0}".format(self.cfg["BRU_NAME"]))
        click.echo("robot-type:     {0}".format(self.cfg["BRU_TYPE"]))
        click.echo("run-mode:       {0}".format(self.cfg["BRU_MODE"]))
        click.echo("my-ip:          {0}".format(self.my_ip))
        click.echo("vpn-ip:         {0}".format(self.my_vpn_ip))
        click.echo("ros-master-uri: {0}".format(self.cfg["ROS_MASTER_URI"]))
        click.echo("ros-ip:         {0}".format(self.cfg["ROS_IP"]))

    def calc_ip(self):
        if self.cfg["BRU_MODE"] == "sim":
            self.cfg["ROS_IP"] = self.my_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_ip)
        elif self.cfg["BRU_MODE"] == "real":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(
                self.cfg["BRU_MASTER_IP"]
            )
        elif self.cfg["BRU_MODE"] == "onboard":
            self.cfg["ROS_IP"] = self.my_vpn_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_vpn_ip)
        elif self.cfg["BRU_MODE"] == "labonboard":
            self.cfg["ROS_IP"] = self.my_ip
            self.cfg["ROS_MASTER_URI"] = "http://{0}:11311".format(self.my_ip)
        else:
            click.echo("*** bug in bru.py calc_ip")

    def echo_env(self):
        click.echo("# Relevant global environment variables\n")
        cmd1 = subprocess.Popen(
            "printenv | grep ROS", shell=True, stdout=subprocess.PIPE
        )
        cmd2 = subprocess.Popen(
            "printenv | grep BRU", shell=True, stdout=subprocess.PIPE
        )
        cmd1_out = cmd1.stdout.read()
        cmd2_out = cmd2.stdout.read()
        click.echo(cmd1_out)
        click.echo(cmd2_out)

    # Launch commands maps a set of ('mode', 'robot type', 'launch command' to a launch string)
    SUBCOMMAND_DISPATCH = {
        (
            "bringup",
            "real",
            "minirover",
        ): "roslaunch minirover mr_bru_bringup.launch lidar:={0} camera:={1} joy:={3}",
        ("kill", "real", "minirover"): "rosnode kill -a",
        ("status", "real", "minirover"): "rostopic list",
    }

    def remote(self, command, lidar, camera, desc, joy):
        click.echo(
            "Launching...{}: lidar: {}, camera: {}, joy: {}, desc: {}".format(
                command, lidar, camera, joy, desc
            )
        )
        dispatch_pattern = (command, self.cfg["BRU_MODE"], self.cfg["BRU_TYPE"])

        if dispatch_pattern in Bru.SUBCOMMAND_DISPATCH:
            self.sshp(
                Bru.SUBCOMMAND_DISPATCH[dispatch_pattern].format(
                    lidar, camera, desc, joy
                ),
                10,
            )
        else:
            click.echo("BUG: That launch option is not available")

    def ssh(self, command_line):
        ssh_cmd = ["ssh", "pi@" + self.cfg["BRU_MASTER_IP"], command_line]
        click.echo(ssh_cmd)
        click.echo("1")
        # cmd1 = subprocess.Popen(" ".join(ssh_cmd), shell=True, stdout=subprocess.PIPE)
        # completed_process = subprocess.run(" ".join(ssh_cmd), shell=True, capture_output=True, text=True)
        completed_process = subprocess.Popen(
            " ".join(ssh_cmd), shell=True, stdout=subprocess.PIPE
        )
        click.echo("2")
        # completed_process.communicate()
        click.echo("2a")
        # cmd1_out = completed_process.stdout.read()
        click.echo(completed_process.stdout.read())
        click.echo("3")

    def sshp(self, command_line, sleep):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.load_system_host_keys()
        ssh_client.connect(
            hostname=self.cfg["BRU_MASTER_IP"], username="pi", password="ROSlab134"
        )
        stdin, stdout, stderr = ssh_client.exec_command(command_line, timeout=sleep)
        out = stdout.read().decode().strip()
        error = stderr.read().decode().strip()
        if error:
            raise Exception("There was an error pulling the runtime: {}".format(error))
        else:
            print(out)
        ssh_client.close()


@click.group(
    help="Brandeis Robotics utilities. Configure for different kinds of robots."
)
@click.pass_context
def cli(ctx):
    ctx.obj = Bru()


@cli.command(help="Display current Bru sttings")
@click.pass_obj
def status(bru):
    bru.echo_status()


@cli.command(help="Display all relevant Environment variables")
@click.pass_obj
def env(bru):
    bru.echo_env()


@cli.command(help="Display bash commands to export state")
@click.pass_obj
def export(bru):
    bru.export_env()


@cli.command(help="Set running modes")
@click.option("-l", "--list", help="list available options", is_flag=True)
@click.argument("name", type=click.Choice(MODES))
@click.pass_obj
def mode(bru, list, name):
    if list:
        click.echo("# Available environments: ")
        [click.echo("{0}".format(envname)) for envname in MODES]
    else:
        bru.set_mode(name)


@cli.command(help="Set name of Robot")
@click.option("--list", "-l", help="list available options", is_flag=True)
@click.option("--master_ip", "-m", prompt=True, help="vpn ip address of robot")
@click.argument("name", type=click.Choice(ROBOTS))
@click.pass_obj
def name(bru, list, name, master_ip):
    if list:
        click.echo("# Available robots: ")
        [click.echo("{0}".format(rname)) for rname in ROBOTS]
    else:
        bru.set_robot(name, master_ip)


ROBOT_SUBCOMMANDS = ["bringup", "kill", "status"]


@cli.command(help="Control the attached robot remotely")
@click.option("--list", "-l", help="list available options", flag_value="list")
@click.option("--camera/--nocamera", default=True)
@click.option("--lidar/--nolidar", default=True)
@click.option("--desc/--nodesc", default=True)
@click.option("--joy/--nojoy", default=True)
@click.argument("command", type=click.Choice(ROBOT_SUBCOMMANDS), required=True)
@click.pass_obj
def robot(bru, command, list, lidar, camera, desc, joy):
    bru.remote(command, lidar, camera, desc, joy)


if __name__ == "__main__":
    cli()
