#!/bin/bash

# Function to add an alias to .bashrc
add_alias_to_bashrc() {
    local alias_name="$1"
    local alias_command="$2"
    local user="$3"
    local bashrc_file="/home/$user/.bashrc"
    local alias_entry="alias $alias_name=$alias_command"

    # Check if alias exists in .bashrc
    if ! grep -qxF "$alias_entry" "$bashrc_file"; then
        echo -e "\n# Frontend Run\n$alias_entry" >> "$bashrc_file"
        echo "Alias '$alias_name' added to $bashrc_file for user $user."
    else
        echo "Alias '$alias_name' already exists in $bashrc_file for user $user. Skipping addition."
    fi
}

# Function to add lines to .bashrc if they don't already exist
add_lines_to_bashrc() {
    local bashrc_file="$HOME/.bashrc"

    for line in "$@"; do
        if ! grep -qxF "$line" "$bashrc_file"; then
            echo "$line" >> "$bashrc_file"
        fi
    done
    echo "NVM configuration complete."
}

# Main setup script
main() {
    CURRENT_DIR=$(pwd)
    USER=$(whoami)

    # Install necessary tools
    echo "Installing necessary tools..."
    sudo apt update > /dev/null 2>&1
    sudo apt install -y curl lsb-release net-tools nano > /dev/null 2>&1

    # Install NVM
    echo "Installing NVM..."
    cd "$CURRENT_DIR/Expo/src"
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh > /dev/null 2>&1 | bash > /dev/null 2>&1

    # NVM configuration
    NVM_LINES=(
        'export NVM_DIR="$HOME/.nvm"'
        '[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm'
        '[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion'
    )

    # Add NVM lines to .bashrc
    add_lines_to_bashrc "${NVM_LINES[@]}"

    # Export NVM_DIR
    export NVM_DIR="$HOME/.nvm" && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"

    # Install Node.js version 20.12.1, Yarn, Expo Ngrok, and Typescript
    echo "Installing NodeJS, Yarn, Expo Ngrok, and Typescript..."
    nvm install 20.12.1 > /dev/null 2>&1
    npm install -g yarn@1.22.22 @expo/ngrok@^4.1.0 typescript > /dev/null 2>&1
    yarn upgrade > /dev/null 2>&1

    # Clean up packages
    sudo apt clean

    # Add alias
    ALIAS_NAME='fr'
    ALIAS_COMMAND="'cd $CURRENT_DIR && ./run.sh'"
    add_alias_to_bashrc "$ALIAS_NAME" "$ALIAS_COMMAND" "$USER"

    # Return to starting directory
    cd $CURRENT_DIR

    echo "Setup complete."
}

# Execute main function
main
