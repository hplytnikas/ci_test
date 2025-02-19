#!/bin/bash

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# starting the ssh-agent
# Start ssh-agent and capture its output

if [[ -e "/tmp/ssh-amz" ]]; then
  rm /tmp/ssh-amz
fi

output=$(ssh-agent -a "/tmp/ssh-amz")
eval "$output"

# Check if ssh-agent started successfully
if [[ $output == *"Agent pid"* ]]; then
  # Prettify and display the output
  echo -e "\033[32m[Ssh agent started]\033[0m Successfully started ssh-agent."
else
  # Display an error message if ssh-agent did not start correctly
  echo -e "\033[31m[Ssh agent error]\033[0m Failed to start ssh-agent."
fi

# I guess it is fine to add all of them to the ssh agent ?
for possiblekey in ${HOME}/.ssh/*; do
    if [[ $keyfile == known_hosts* || $keyfile == config* ]]; then
        continue
    fi
    if grep -q PRIVATE "$possiblekey"; then
        ssh-add "$possiblekey"
    fi
done 2>&1 | $SCRIPTS_DIR/prettify.sh "Setting up ssh"
