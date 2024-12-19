#!/bin/bash

# User to change to, default is AndreasZachariae
GITHUB_USER=${1:-AndreasZachariae}

# This has to started with 'source' to change the environment of the current shell
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "This script must be sourced: use 'source $0'" >&2
    exit 1
fi

# Add the new user's ssh key to the ssh-agent
if [ -z "$SSH_AUTH_SOCK" ]; then
    eval "$(ssh-agent -s)" > /dev/null
fi
ssh-add -D
ssh-add ~/.ssh/${GITHUB_USER}
GITHUB_EMAIL=$(ssh-add -L | awk '{print $NF}')

# Change global git config
git config --global user.name ${GITHUB_USER}
git config --global user.email ${GITHUB_EMAIL}
ssh -T git@github.com
