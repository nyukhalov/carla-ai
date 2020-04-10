# carla-ai

AI for Carla Simulator

## Installation

- Follow [the official guide](https://carla.readthedocs.io/en/latest/start_quickstart/) to install Carla
- Create a conda environment from the `environment.yaml` file in this repository and activate it
  ```bash
  conda env create -f environment.yaml
  conda activate carla-ai
  ```
- Install Carla PythonAPI
  ```bash
  easy_install /opt/carla/PythonAPI/carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg
  ```

## Running

1. Run Carla

   For headless mode execute
   ```bash
   DISPLAY= /opt/carla/cin/CarlaUE4.sh -opengl
   ```

   or for normal mode execute
   ```bash
   /opt/carla/CarlaUE4.sh
   ```
