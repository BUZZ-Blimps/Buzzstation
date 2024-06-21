#!/usr/bin/env python3

import os
import subprocess
import sys

def run_command(command, env=None):
    result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    if result.returncode != 0:
        print(f"Error running command: {command}")
        print(result.stderr.decode())
        sys.exit(result.returncode)
    return result.stdout.decode().strip()

def add_alias_to_bashrc(alias_name, alias_command, user):
    """
    Add an alias to the user's .bashrc file if it doesn't already exist.

    Parameters:
    - alias_name: Name of the alias (e.g., 'fr')
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
            cmd = f'echo -e \"\n# Frontend Run\n{alias_entry}\" >> {bashrc_file}'
            run_command(f'sudo -u {user} {cmd}')
            print(f"Alias '{alias_name}' added to {bashrc_file} for user {user}.")
        except subprocess.CalledProcessError as e:
            print(f"Error adding alias '{alias_name}' to {bashrc_file} for user {user}: {e}")
    else:
        print(f"Alias '{alias_name}' already exists in {bashrc_file} for user {user}. Skipping addition.")

def add_lines_to_bashrc(lines):
    """
    Add specified lines to the user's .bashrc file if they don't already exist.

    Parameters:
    - lines: List of strings, each representing a line to add.
    """
    bashrc_path = os.path.expanduser('~/.bashrc')

    # Read the .bashrc file
    with open(bashrc_path, 'r') as f:
        bashrc_content = f.read()

    # Check if all lines are present
    missing_lines = [line for line in lines if line not in bashrc_content]

    # If any lines are missing, append them to .bashrc
    if missing_lines:
        with open(bashrc_path, 'a') as f:
            f.write("\n# NVM configuration\n")
            for line in missing_lines:
                f.write(f"{line}\n")
        print("NVM configuration added to .bashrc.")
    else:
        print("NVM configuration already exists in .bashrc.")

def main():

    # Get Current Directory
    CURRENT_DIR = os.getcwd()

    # Alias Name
    alias_name = 'fr'

    # Alias Command
    alias_command = f"\'cd {CURRENT_DIR} && source run.sh\'"

    # Get User
    user = os.environ.get('USER') or os.environ.get('USERNAME')

    # Install Necessary Tools
    print("Installing necessary tools...")
    run_command("sudo apt update")
    run_command("sudo apt install -y curl lsb-release net-tools nano")

    # Install NVM, NodeJS, Yarn, Expo Ngrok, and Typescript
    print("Installing NVM, NodeJS, Yarn, Expo Ngrok, and Typescript...")
    os.chdir(f"{CURRENT_DIR}/Expo/src")

    # Install Node Version Manager (nvm)
    run_command("curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash")

    # NVM Variables
    nvm_lines = [
        'export NVM_DIR="$HOME/.nvm"',
        '[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm',
        '[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion'
    ]

    # Add NVM Variables to Bashrc
    add_lines_to_bashrc(nvm_lines)

    # Reload .bashrc
    #run_command(nvm_lines)

    # Install Node.js version 20.12.1
    #run_command("nvm install 20.12.1")
    #run_command("nvm use 20.12.1")

    # Install Yarn 1.22.22, Ngrok, and Typescript
    #run_command("npm install -g yarn@1.22.22 @expo/ngrok@^4.1.0 typescript")
    #run_command("yarn upgrade")

    # Run nvm and subsequent commands in a new shell that sources .bashrc
    nvm_commands = '''
        source ~/.bashrc
        nvm install 20.12.1
        nvm use 20.12.1
        npm install -g yarn@1.22.22 @expo/ngrok@^4.1.0 typescript
        yarn upgrade
    '''
    run_command(f'bash -c "{nvm_commands}"')

    # Clean up packages
    run_command("sudo apt clean")

    # Add alias
    add_alias_to_bashrc(alias_name, alias_command, user)

    print("Setup complete.")

if __name__ == "__main__":
    main()
