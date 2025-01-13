# Digital Twin Project

This is a digital twin of the Robot Living Studio. The project simulates the environment in GAZEBO and includes an integrated Fetch robot. The topics and control commands are identical to those of the real robot, ensuring that code developed in simulation will work seamlessly with the physical robot.

## System Requirements

### For Ubuntu Users

If you're running Ubuntu 20.04 natively, we recommend using Docker for development to ensure consistency across different development environments.

#### Prerequisites for Docker Setup

- Docker installed on your system
- VSCode with [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed
- Git installed on your system

### For Windows/macOS Users

If you're running Windows or macOS, we recommend using either:

- Virtual Machine running Ubuntu 20.04 (recommended for most users)
- Windows Subsystem for Linux (WSL) with Ubuntu 20.04 (for Windows users)

Please follow the setup instructions in [VM_SETUP.md](VM_SETUP.md) for virtual machine setup.

## Getting Started

### For Ubuntu Users with Docker

1. Clone the repository:

   ```bash
   git clone https://github.com/H-tr/digital_twin
   cd digital-twin
   ```

2. Open VSCode:

   ```bash
   code .
   ```

3. When VSCode prompts to "Reopen in Container", click "Yes". Alternatively:
   - Press `F1` or `Ctrl+Shift+P`
   - Type "Dev Containers: Reopen in Container"
   - Press Enter

4. Wait for the container to build. This may take several minutes on the first run as it downloads and sets up all required dependencies.

### Setting up the ROS Workspace

Once your development environment is ready (either Docker or VM):

1. Navigate to the ROS workspace:

   ```bash
   cd rls_fetch_ws
   ```

2. Build the workspace:

   ```bash
   catkin_make
   ```

3. Add the workspace to your bash environment:

   ```bash
   echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
   ```

4. Apply the changes:

   ```bash
   source ~/.bashrc
   ```

### Installing the Project

Install the project in development mode:

```bash
cd /workspace  # Skip this step if you're using VM
pip install -e .
git lfs install
git lfs pull
```

## Examples

The project includes several examples to help you get started with controlling the robot. These examples can be found in the `examples` folder and demonstrate various control commands and interactions with the robot.

To run an example:

1. Navigate to the examples folder
2. Follow the instructions in [example's README](examples/README.md)
3. Try modifying the examples to understand how different commands affect the robot's behavior

## Development

After setting up the environment, you can start developing:

- All necessary dependencies are provided in either Docker or VM setup
- ROS commands and tools are available in the terminal
- Use the provided examples as a reference for implementing your own robot control logic

## License

This project is licensed under the MIT License.
