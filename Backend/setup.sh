#!/bin/bash

# Configuration
PROJECT_DIR="/"
VENV_DIR="${PROJECT_DIR}Backend"
PYTHON_VERSION="3.10"
PIP_VERSION="23.2"
PACKAGES=(
    "--ignore-installed blinker flask==2.3.2"
    "flask-socketio==5.2.0"
    "flask-cors==4.0.1"
    "pyopenssl==24.2.1"
    "pyserial==3.5"
    "numpy==1.21.5"
    "redis==5.0.3"
    "pyyaml"
    "lark-parser"
    "opencv-python-headless"
)

run_command() {
    command=$1
    eval "$command"
    if [ $? -ne 0 ]; then
        echo "Error running command: $command"
        exit 1
    fi
}

add_command_to_bashrc() {
    command=$1
    user=$2
    bashrc_file="/home/${user}/.bashrc"
    command_entry="${command}"

    if ! grep -Fxq "$command" "$bashrc_file"; then
        echo -e "\n$command_entry" | sudo -u "$user" tee -a "$bashrc_file" > /dev/null
        echo "Command '$command' added to $bashrc_file for user $user."
    else
        echo "Command '$command' already exists in $bashrc_file for user $user. Skipping addition."
    fi
}

add_alias_to_bashrc() {
    alias_name=$1
    alias_command=$2
    user=$3
    bashrc_file="/home/${user}/.bashrc"
    alias_entry="alias $alias_name='$alias_command'"

    if ! grep -Fxq "$alias_entry" "$bashrc_file"; then
        echo -e "\n# Backend Run\n$alias_entry" | sudo -u "$user" tee -a "$bashrc_file" > /dev/null
        echo "Alias '$alias_name' added to $bashrc_file for user $user."
    else
        echo "Alias '$alias_name' already exists in $bashrc_file for user $user. Skipping addition."
    fi
}

main() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "This script must be run with sudo."
        exit 1
    fi

    # Get the directory the script is being run from
    CURRENT_DIR=$(dirname "$(readlink -f "$0")")

    # Define alias name and command
    alias_name='br'
    alias_command="cd $CURRENT_DIR && source run.sh"

    # Retrieve the user who invoked sudo
    # If SUDO_USER is not set, fall back to logname
    sudo_user=${SUDO_USER:-$(logname)}

    # Validate that we have a user
    if [ -n "$sudo_user" ]; then
        add_alias_to_bashrc "$alias_name" "$alias_command" "$sudo_user"
    else
        echo "No user identified who called sudo. Cannot proceed."
        exit 1
    fi

    mkdir -p "$PROJECT_DIR"
    cd "$PROJECT_DIR" || exit

    echo "Installing Python 3.10 and necessary tools..."
    run_command "apt update"
    run_command "apt install -y python3.10 python3.10-venv python3.10-distutils build-essential python3-dev python3-pip python3-setuptools python3-wheel curl lsb-release net-tools nano software-properties-common lsof redis-server redis-tools"
    run_command "service redis-server start"

    echo "Creating virtual environment in ${VENV_DIR} with Python ${PYTHON_VERSION}..."
    run_command "python3.10 -m venv ${VENV_DIR}"

    venv_pip="${VENV_DIR}/bin/pip"

    echo "Upgrading pip to version ${PIP_VERSION}..."
    run_command "${venv_pip} install --upgrade pip==${PIP_VERSION}"

    echo "Upgrading setuptools and cython..."
    run_command "${venv_pip} install build"
    run_command "${venv_pip} install --upgrade setuptools cython wheel"

    for package in "${PACKAGES[@]}"; do
        echo "Installing ${package}..."
        run_command "${venv_pip} install ${package}"
    done

    echo "Installing ROS 2 Humble..."
    run_command "add-apt-repository universe"
    run_command "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    run_command "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(lsb_release -sc) main\" | tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    run_command "apt update"
    run_command "apt upgrade -y"
    run_command "apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools"
    run_command "rm /etc/apt/sources.list.d/ros2.list"
    add_command_to_bashrc "source /opt/ros/humble/setup.bash" "$sudo_user"
    add_command_to_bashrc "export ROS_DOMAIN_ID=1" "$sudo_user"
    run_command "apt clean"

    echo "Setup complete."
    echo "Virtual environment created in ${VENV_DIR}"
    echo "To activate it, run:"
    echo "source ${VENV_DIR}/bin/activate"
}

main
