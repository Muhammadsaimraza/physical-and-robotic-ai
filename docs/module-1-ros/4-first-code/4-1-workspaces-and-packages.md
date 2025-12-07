# Lesson 4.1: Workspaces & Packages

Before we write code, we need to organize our files. ROS 2 has a standard, conventional structure for development called a **workspace**.

## The ROS 2 Workspace

A workspace is a directory that contains your ROS 2 packages. The standard structure looks like this:
```
ros2_ws/
├── src/
├── build/
├── install/
└── log/
```
*   `src/`: This is the **source** directory. It's the only one you will touch directly. You will put all of your packages inside this folder.
*   `build/`: This is a temporary directory where ROS 2's build tool, `colcon`, will put intermediate files when it compiles your code. You should never edit this directory yourself.
*   `install/`: After a successful build, `colcon` will place the final, installable files here. This includes your node executables and the setup files needed to run them.
*   `log/`: Contains log files from running your nodes.

Let's create our workspace. In your terminal, run:
```bash
mkdir -p ros2_ws/src
cd ros2_ws
```

## Creating a Python Package

Now that we are inside our workspace, we can create our first package. We will use the `ros2 pkg create` command that we've seen before.
```bash
cd src
ros2 pkg create --build-type ament_python my_first_package
```
This command creates a new directory called `my_first_package` inside `src`. The `--build-type ament_python` flag tells ROS that this will be a Python package.

The generated package will have this structure:
```
my_first_package/
├── my_first_package/
│   └── __init__.py
├── resource/
│   └── my_first_package
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

### Key Files
*   `package.xml`: This is the package manifest. It contains metadata like the package name, version, author, and dependencies. You **must** edit this file to add your dependencies. For a Python node, you'll always need `rclpy`.
    ```xml
    <!-- Inside package.xml -->
    <depend>rclpy</depend>
    ```
*   `setup.py`: This is a standard Python setup file. It tells `colcon` how to install your package and, crucially, where to find your executables. You will need to add an `entry_points` section to register your nodes.
*   `my_first_package/`: This is the directory where you will put your actual Python source code files.

## Building the Workspace

Now that we have a package, we need to build it. Navigate to the root of your workspace (`ros2_ws`) and run the build command:
```bash
cd ..  # Go back to the root of ros2_ws
colcon build
```
`colcon` is the ROS 2 build tool. It will go through all the packages in your `src` directory, figure out their dependencies, and build them in the correct order.

If the build is successful, you will see new `build` and `install` directories appear in your workspace.

## Sourcing the Workspace

The executables and resources for your new package are now in the `install` directory. To use them, you need to source your workspace's local setup file.
```bash
source install/setup.bash
```
This command tells your terminal where to find the packages you just built. **You must do this in every new terminal you open.**

Now, if you run `ros2 pkg list`, you should see `my_first_package` in the list of available packages.

You now have a complete, buildable ROS 2 package. In the next lesson, we will add our first Python node to it.
