pibt2
===
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

A simulator and visualizer of Multi-Agent Path Finding (MAPF), used in a paper.
It is written in C++(17) with [CMake](https://cmake.org/) (≥v3.16) build.
The repository uses [Google Test](https://github.com/google/googletest) and [the original library for 2D pathfinding](https://github.com/Kei18/grid-pathfinding) as git submodules.
The visualizer uses [openFrameworks](https://openframeworks.cc) and works only on macOS.

The implementations include: HCA\* [1], PIBT [2], and PIBT+.

| platform | status (public) | status (dev) |
| ---: | :--- |:--- |
| macos-10.15 | ![test_macos](https://github.com/Kei18/pibt2/workflows/test_macos/badge.svg?branch=public) ![build_visualizer_macos](https://github.com/Kei18/pibt2/workflows/build_visualizer_macos/badge.svg?branch=public) | ![test_macos](https://github.com/Kei18/pibt2/workflows/test_macos/badge.svg?branch=dev) ![build_visualizer_macos](https://github.com/Kei18/pibt2/workflows/build_visualizer_macos/badge.svg?branch=dev) |
| ubuntu-latest | ![test_ubuntu](https://github.com/Kei18/pibt2/workflows/test_ubuntu/badge.svg?branch=public) | ![test_ubuntu](https://github.com/Kei18/pibt2/workflows/test_ubuntu/badge.svg?branch=dev) |

You can see the performance of each solver from [auto\_record repo](https://github.com/Kei18/pibt2/tree/auto_record). The records were created by Github Actions.

Please cite the following paper if you use the code in your published research:
```
@article{okumura2021iterative,
  title={Iterative Refinement for Real-Time Multi-Robot Path Planning},
  author={Okumura, Keisuke and Tamura, Yasumasa and D{\'e}fago, Xavier},
  journal={arXiv preprint arXiv:2102.12331},
  year={2021}
}
```

## Demo
![100 agents in arena](/material/arena_100agents.gif)

100 agents in arena, planned by PIBT in ~~67ms~~ 5ms.

![1000 agents in brc202d](/material/brc202d_1000agents.gif)

1000 agents in brc202d, planned by PIBT in ~~84sec~~ 1348ms.
The gif shows a part of an MAPF plan.

## Building

```sh
git clone --recursive https://github.com/Kei18/pibt2.git
cd pibt2
mkdir build
cd build
cmake ..
make
```

## Usage
PIBT
```sh
./app -i ../instances/sample.txt -s PIBT -o result.txt -v
```

IR (the result will be saved in result.txt)
```sh
./app -i ../instances/random-32-32-20_70agents_1.txt -s IR_HYBRID -n 300 -t 100 -v
```

You can find details and explanations for all parameters with:
```sh
./app --help
```

Please see `instances/sample.txt` for parameters of instances, e.g., filed, number of agents, time limit, etc.

### Output File

This is an example output of `../instances/sample.txt`.
Note that `(x, y)` denotes location.
`(0, 0)` is the left-top point.
`(x, 0)` is the location at `x`-th column and 1st row.
```
instance= ../instances/sample.txt
agents=100
map_file=arena.map
solver=PIBT
solved=1
soc=3403
makespan=68
comp_time=58
starts=(32,21),(40,4),(20,22),(26,18), [...]
goals=(10,16),(30,21),(11,42),(44,6), [...]
solution=
0:(32,21),(40,4),(20,22),(26,18), [...]
1:(31,21),(40,5),(20,23),(27,18), [...]
[...]
```

## Visualizer

### Building
It takes around 10 minutes.

#### macOS 10.5
```sh
bash ./visualizer/scripts/build_macos.sh
```

Note: The script of openFrameworks seems to contain bugs. Check this [issue](https://github.com/openframeworks/openFrameworks/issues/6623). I fixed this in my script :D

#### macOS 11.4
```sh
git submodule update --remote
bash ./third_party/openFrameworks/scripts/osx/download_libs.sh
cd visualizer
make build
cd ..
chmod +x ./visualize.sh
```

### Usage
```sh
cd build
../visualize.sh result.txt
```

You can manipulate it via your keyboard. See printed info.


## Experimental Environment


## Notes
- Maps in `maps/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
  When you add a new map, please place it in the `maps/` directory.
- The font in `visualizer/bin/data` is from [Google Fonts](https://fonts.google.com/).

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Reference
