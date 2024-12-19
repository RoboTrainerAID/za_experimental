#!/bin/sh

# User to change to, default is AndreasZachariae
GITHUB_USER=${1:-AndreasZachariae}

# Add the new user's ssh key to the ssh-agent
eval "$(ssh-agent -s)"
ssh-add -D
ssh-add ~/.ssh/${GITHUB_USER}
GITHUB_EMAIL=$(ssh-add -L | awk '{print $NF}')

# Change global git config
git config --global user.name ${GITHUB_USER}
git config --global user.email ${GITHUB_EMAIL}
ssh -T git@github.com
