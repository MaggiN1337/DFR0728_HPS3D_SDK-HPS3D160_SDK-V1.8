# HPS3D Distance CLI

## Overview
The HPS3D Distance CLI is a command-line tool designed to interact with the HPS3D sensor. It allows users to query the distance measurements from the sensor based on specific pixel range inputs.

## Project Structure
```
hps3d-distance-cli
├── src
│   ├── main.c          # Entry point of the command-line tool
│   └── HPS3DUser_IF.h  # Header file for HPS3D sensor interface
├── Makefile            # Build instructions for the project
└── README.md           # Documentation for the project
```

## Requirements
- C Compiler (e.g., GCC)
- HPS3D SDK

## Building the Project
To compile the project, navigate to the project directory and run the following command:

```
make
```

This will generate the executable for the command-line tool.

## Running the Tool
After building the project, you can run the tool using the following command:

```
./hps3d-distance-cli
```

## Usage
Once the tool is running, you can input pixel range values to query the distance measurements from the HPS3D sensor. Follow the prompts in the command line to enter your inputs.

## Dependencies
Ensure that the HPS3D SDK is properly installed and configured on your system before running the tool.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.