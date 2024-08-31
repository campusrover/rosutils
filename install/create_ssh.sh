#!/bin/bash

# Variables
KEY_NAME="cluster"  # Change this to your desired key name
KEY_PATH="/root/.ssh/$KEY_NAME"

# 1. Create a new SSH key
ssh-keygen -t ed25519 -f "$KEY_PATH" -N ""

# 2. Start the SSH agent
eval "$(ssh-agent -s)"

# 3. Add the key to the SSH agent
ssh-add "$KEY_PATH"

echo "SSH key created and added to the agent."
cat "$KEY_PATH.ssh"
