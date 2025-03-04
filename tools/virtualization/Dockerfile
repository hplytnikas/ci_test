# syntax=docker/dockerfile:1

# This Dockerfile creates an image of the repository that allows to build it
# and run it on any system which has Docker Engine.

# Pull full desktop humble ROS image, important since has a lot of stuff that would normally be installed as subdependencies
FROM  osrf/ros:humble-desktop-full

RUN adduser --disabled-login --gecos '' --uid 1000 amz
RUN echo 'amz ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/amz

# Update & upgrade apt and install required packages
RUN   --mount=type=cache,target=/var/cache/apt \
      --mount=type=cache,target=/root/.cache/pip \
      rm -f /etc/apt/apt.conf.d/docker-clean && \
      apt update && \
      # apt -y upgrade && \
      apt -y install \
        pip \
        git \
        curl \
        ssh \
        wget \
        software-properties-common \
        xterm \
        foot \
        nano

# Nvim install
RUN   add-apt-repository -y ppa:neovim-ppa/unstable && \
      apt -y update && \
      apt -y install neovim

# Default branch main
ARG   BRANCH=main

# Cool way to clone the autonomous_2024, invalidates cache on new commits, and lets for specifying branches !
# ADD   --keep-git-dir=true --chown=amz:amz git@github.com:AMZ-Racing/autonomous_2024.git#$BRANCH /autonomous_2024
ADD   --keep-git-dir=true --chown=amz:amz https://github.com/hplytnikas/ci_test.git#$BRANCH /autonomous_2024

WORKDIR /autonomous_2024

# Reuse the apt pip and amz cache from previous builds
RUN   --mount=type=cache,target=/var/cache/apt \
      --mount=type=cache,target=/root/.cache/pip \
      --mount=type=cache,target=/usr/local/share/amz/cache \
      rm -f /etc/apt/apt.conf.d/docker-clean && \
      cd /autonomous_2024 && \
      ./tools/scripts/install_deps.sh

ENV CMAKE_PREFIX_PATH="/usr/local/share/amz/lib/cmake/easy_profiler:/usr/local/share/amz/lib/cmake/GTSAM:$CMAKE_PREFIX_PATH"

USER amz

RUN pre-commit install

CMD tail -f /dev/null
