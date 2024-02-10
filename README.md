
# IntroRoboticProject

Welcome to the IntroRoboticProject! This project contains code for various robotic functionalities.

## Prerequisites

Before you begin, ensure you have the following installed:

- Eigen lib

## Setup the Workspace
### Clone the Repository

```bash
git clone https://github.com/rikirandon/IntroRoboticProject.git
cd IntroRoboticProject
```
### Build the Catkin Workspace
```bash
catkin_make
nano ~/.bashrc
```
Add the path for the setup.bash.
```bash
source $HOME/IntroRoboticProject/devel/setup.bash
```
Save, exit.
```bash 
source ~/.bashrc
```
## Directory Structure

### Directories

```bash
|-- build/                          # Store build files and compiled binaries
|-- devel/                          # Development files and symbolic links
|-- src/                            # Source directory
    |-- package/                    # Code related to motion planning functionality
          |-- src/
          |    |-- motion_planner/    # Code related to task planning functionality
          |    |-- vision/          # Code related to vision functionality
          |    |-- tests/           # Code related to testing motion_planning funcionality
          |-- include/
          |-- srv/
```
