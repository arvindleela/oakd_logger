# OAKD sensor logger / replay / preview


The goal of this project is to record / replay / preview sensor streams from an OAKD sensor https://store.opencv.ai/products/oak-d


![OakD_Logger](https://user-images.githubusercontent.com/7820970/186074234-de63feed-a71b-4f6b-86b0-9d882b2e99af.gif)

## Build instructions

Tested on macbook pro with the following specs
```
Software:
System Version:	macOS 12.5.1 (21G83)
  Kernel Version:	Darwin 21.6.0
  
Hardware: 
Model Name:	MacBook Pro
  Model Identifier:	MacBookPro14,1
  Processor Name:	Dual-Core Intel Core i5
  Processor Speed:	2.3 GHz
  Number of Processors:	1
  Total Number of Cores:	2
  L2 Cache (per Core):	256 KB
  L3 Cache:	4 MB
  Hyper-Threading Technology:	Enabled
  Memory:	8 GB
```
### Prerequisites:
`find_package` needs to be able to find `OpenCV`.

0. Either install opencv by following instructions here https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
1. (or) Build opencv and set `OpenCV_DIR` variable in CMakeLists.txt to the opencv build dir like  `set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../build_opencv")`.

### Building
0. Clone the repo by running `https://github.com/arvindleela/oakd_logger.git`.
1. `cd` into the root directory.
2. Update all submodule by running `git submodule update --init --recursive`.
3. Create make files by running `cmake -S. -Bbuild` in the root directory.
4. Compile with `cmake --build build`

## Run instructions
The `logger.py` file in the root directory is the main entry point. Use `python3 logger.py -h` for some help.

### To preview
1. Connect the OAK-D sensor via a high speed USB connection.
0. Run something like `python3 logger.py data/` to show the sensor preview.

### To log data
1. Connect the OAK-D sensor via a high speed USB connection.
0. Run something like `python3 logger.py data/ --output <filename.bin>` to log sensor data to a binary file in addition to showing the preview.

### To replay logs
0. Run something like `python3 logger.py data/ --input calibration_target/data.bin` to replay data from a binary file stored from the previous step.
