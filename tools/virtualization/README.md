# Docker-based Virtualization

## Project Structure

```
.
├── Dockerfile          # Docker configuration for setting up the virtualization environment
├── README.md           # This file, containing project documentation
├── files               # Directory for shared volumes between the host and the container
├── dotfiles            # Store ~/.config files here for Docker container use, maintaining consistent settings.
├── .scripts            # Scripts for container setup and management
└── run.sh             # Main script for building, running, and interacting with the container
```

### Key Components

- **Dockerfile**: Contains the setup instructions for the Docker image, including the installation of necessary packages and configurations for GUI application support.
- **files/**: A shared volume directory that facilitates file sharing between the Docker container and the host machine.
- **.scripts/**: Includes scripts that automate various tasks such as sharing X and Wayland sockets, handling SSH keys, and naming containers and images.
- **run.sh**: A utility script to easily build, run, and interact with the Docker container. It abstracts away Docker commands for ease of use.

## Getting Started

### Building the Docker Image

You can build the Docker image using the `run.sh` script. This is the recommended way as it simplifies the process:
```bash
./run.sh
```
or
```bash
./run.sh build
```

Alternatively, you can inspect and run the build process manually by checking the `./.scripts/build.sh` script.

### Running the Virtualization Docker Image

The `run.sh` script without any arguments will also execute this action by default, as it's designed to handle both building (if necessary) and running:

```bash
./run.sh
```
or
```bash
./run.sh run
```

For a detailed understanding of the running process, you can refer to `./.scripts/run.sh`.

### Spawning a Terminal within the Docker Container

To interact with the running container through a terminal:

```bash
./run.sh
```

This command opens a Bash session in your current terminal window. If you prefer to use Docker commands directly:

```bash
docker exec -it virtualization bash
```

Or, to spawn a Docker-installed terminal emulator via the shared socket:

```bash
docker exec virtualization xterm
```

*Note: Replace `xterm` with any terminal emulator installed in the Docker container.*

### Using Text Editors within the Docker Container

#### VSCode

[For VSCode](https://code.visualstudio.com/docs/devcontainers/attach-container) you can attach to the container and develop inside it, haven't tested it too much yet.
For this you have to do the above first. (Will improve in the future)

#### Terminal editors

For enthusiasts of text-based interfaces, Neovim and Nano are also available options for editing files:

- **Neovim**: To edit a specific file with Neovim, use the following command:
  ```
  nvim /autonomous_2024/<file>
  ```

- **Nano**: If you prefer Nano, you can edit files by running:
  ```
  nano /autonomous_2024/<file>
  ```

While the container includes these editors by default, you're welcome to install any other editor of your choice. However, please note that compatibility and functionality of other editors cannot be guaranteed.

### Configuring Your Development Environment

To personalize your development experience, you can add configuration files for your preferred text editors. For example, to use your existing Visual Studio Code settings, simply copy your configuration files from your local `~/.config` directory to the container's `virtualization/dotfiles/` directory. This allows you to maintain a consistent development environment, even within the container.
