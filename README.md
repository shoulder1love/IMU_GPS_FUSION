# IMU_GPS_FUSION

## Introduction
We use the IMU information built into the mobile phone, as well as the GPS information obtained, to fuse the bicycle tracks.
For more information on data structures and their introduction, please refer [here](https://github.com/simra-project/dataset).

This repository is divided into two parts: the first part is an introduction to GPS and IMU fusion algorithms. The second part is about the visualization of the fusion results between the front-end and the back-end.

## Quick Start

use git clone from this repository
```shell
git clone git@github.com:shoulder1love/IMU_GPS_FUSION.git
cd IMU_GPS_FUSION
```

### IMU fusion algorithms

<details>
<summary>Installation</summary>

Build files from source.
```shell
cd IMU_GPS_FUSION_2023
mkdir -p build
cd build
```

Build files.
```shell
cmake ..
make
```
  
### Front- and Backend algorithms(Visualisation)

<details>
<summary>Installation</summary>

Install [Django](https://docs.djangoproject.com/zh-hans/4.1/topics/install/#installing-official-release) from source.
```shell
python3 -m pip install Django
```
Run the sever.
```shell
cd VIS_front_backend
python3 manage.py runserver
```

