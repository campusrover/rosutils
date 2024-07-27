#!/bin/bash

# Define variables
EMAIL="pitosalas@gmail.com"

# Generate the SSH key and capture the output
keygen_output=$(yes "" | ssh-keygen -t ed25519 -C "$EMAIL")
private_key_path=$(echo "$keygen_output" | grep -oP '(?<=Your identification has been saved in )[^ ]+')

# Start the ssh-agent in the background
eval "$(ssh-agent -s)" >/dev/null 2>&1

# Check if the key was successfully generated
if [ -n "$private_key_path" ]; then
    echo "Private key generated at: $private_key_path"
    # Add the key to the ssh-agent
    ssh-add "$private_key_path"
else
    echo "Failed to generate SSH key. Probably there was already a key with the same name"
    exit 1
fi
echo "SSH key generation complete."