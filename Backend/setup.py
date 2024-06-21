#!/usr/bin/env python3

import os
import subprocess
import sys

# Configuration
PROJECT_DIR = "/"
VENV_DIR = os.path.join(PROJECT_DIR, "Backend")
PYTHON_VERSION = "3.10"
PIP_VERSION = "23.2"
PACKAGES = [
    "--ignore-installed blinker flask==2.3.2",
    "flask-socketio==5.2.0",
    "pyserial==3.5",
    "numpy==1.21.5",
    "redis==5.0.3",
    "pyyaml",
    "lark-parser",
    "opencv-python-headless"
]

def run_command(command, env=None):
    result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    if result.returncode != 0:
        print(f"Error running command: {command}")
        print(result.stderr.decode())
        sys.exit(result.returncode)
    return result.stdout.decode().strip()

def add_command_to_bashrc(command, user):
    """
    Add a command to the user's .bashrc file if it doesn't already exist.

    Parameters:
    - command: Command to be added (e.g., 'cd /home/user/Backend && source run.sh')
    - user: Username of the user whose .bashrc should be modified
    """
    # Construct the command entry
    command_entry = f'{command}'

    # Check if the command already exists in .bashrc
    bashrc_file = f'/home/{user}/.bashrc'
    command_exists = False
    with open(bashrc_file, 'r') as f:
        for line in f:
            if line.strip() == command_entry:
                command_exists = True
                break

    # If command does not exist, append it to .bashrc
    if not command_exists:
        try:
            # Construct and execute the command to append to .bashrc
            cmd = f'echo -e \"\n# Source ROS 2 Humble setup script\n{command_entry}\" >> {bashrc_file}'
            subprocess.run(['sudo', '-u', user, 'bash', '-c', cmd], check=True)
            print(f"Command '{command}' added to {bashrc_file} for user {user}.")
        except subprocess.CalledProcessError as e:
            print(f"Error adding command '{command}' to {bashrc_file} for user {user}: {e}")
    else:
        print(f"Command '{command}' already exists in {bashrc_file} for user {user}. Skipping addition.")

def add_alias_to_bashrc(alias_name, alias_command, user):
    """
    Add an alias to the user's .bashrc file if it doesn't already exist.

    Parameters:
    - alias_name: Name of the alias (e.g., 'br')
    - alias_command: Command to be aliased (e.g., '(cd {CURRENT_DIR} && ./run.sh)')
    - user: Username of the user whose .bashrc should be modified
    """
    # Construct the alias entry
    alias_entry = f'alias {alias_name}={alias_command}'

    # Check if the alias already exists in .bashrc
    bashrc_file = f'/home/{user}/.bashrc'
    alias_exists = False
    with open(bashrc_file, 'r') as f:
        for line in f:
            if line.strip() == alias_entry:
                alias_exists = True
                break

    # If alias does not exist, append it to .bashrc
    if not alias_exists:
        try:
            # Construct and execute the command
            cmd = f'echo -e \"\n# Backend Run\n{alias_entry}\" >> {bashrc_file}'
            run_command(f'sudo -u {user} {cmd}')
            print(f"Alias '{alias_name}' added to {bashrc_file} for user {user}.")
        except subprocess.CalledProcessError as e:
            print(f"Error adding alias '{alias_name}' to {bashrc_file} for user {user}: {e}")
    else:
        print(f"Alias '{alias_name}' already exists in {bashrc_file} for user {user}. Skipping addition.")

def main():
    # Ensure the script is run with sudo
    if os.geteuid() != 0:
        print("This script must be run with sudo.")
        sys.exit(1)

    # Get Current Directory
    CURRENT_DIR = os.getcwd()

    # Alias Name
    alias_name = 'br'

    # Alias Command
    alias_command = f"\'cd {CURRENT_DIR} && source run.sh\'"

    # Get User
    sudo_user = os.getenv('SUDO_USER')

    # Add Alias to Bashrc
    if sudo_user:
        add_alias_to_bashrc(alias_name, alias_command, sudo_user)
    else:
        print("No user identified who called sudo. Cannot proceed.")

    # Create project directory if it does not exist
    os.makedirs(PROJECT_DIR, exist_ok=True)
    os.chdir(PROJECT_DIR)

    # Install Python 3.10 and necessary tools
    print("Installing Python 3.10 and necessary tools...")
    run_command("apt update")
    run_command("apt install -y python3.10 python3.10-venv python3.10-distutils build-essential python3-dev python3-pip python3-setuptools python3-wheel curl lsb-release net-tools nano software-properties-common lsof")

    # Create the virtual environment using Python 3.10
    print(f"Creating virtual environment in {VENV_DIR} with Python {PYTHON_VERSION}...")
    run_command(f"python3.10 -m venv {VENV_DIR}")

    # Upgrade pip to the specified version
    print(f"Upgrading pip to version {PIP_VERSION}...")
    venv_pip = os.path.join(VENV_DIR, "bin", "pip")
    run_command(f"{venv_pip} install --upgrade pip=={PIP_VERSION}")

    # Ensure setuptools and cython are up-to-date
    print("Upgrading setuptools and cython...")
    run_command(f"{venv_pip} install build")
    run_command(f"{venv_pip} install --upgrade setuptools cython wheel")

    # Install the required packages
    for package in PACKAGES:
        print(f"Installing {package}...")
        run_command(f"{venv_pip} install {package}")

    # Install ROS 2 Humble
    print("Installing ROS 2 Humble...")
    run_command("curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg")
    command = (
        'echo "deb [arch=$(dpkg --print-architecture) '
        'signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] '
        'http://packages.ros.org/ros2/ubuntu '
        '$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | '
        'sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'
    )
    run_command(command)
    run_command("apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools")
    run_command("rm /etc/apt/sources.list.d/ros2.list")
    command = 'source /opt/ros/humble/setup.bash'
    add_command_to_bashrc(command, sudo_user)
    run_command("apt clean")

    print("Setup complete.")
    print(f"Virtual environment created in {VENV_DIR}")
    print("To activate it, run:")
    print(f"source {os.path.join(VENV_DIR, 'bin', 'activate')}")

if __name__ == "__main__":
    main()
