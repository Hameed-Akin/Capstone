Json C++ 
======================

Overview
---------------

This is a simple code to access the data in a json file and print the 2d coordinates of hands key point.

To get started install the Jsoncpp library 
```
sudo apt-get install libjsoncpp-dev
```

COMPILE THE CODE 
---------------

```
mkdir build
cmake ..
make
```

COMPILE THE CODE 
---------------

The executable is called `Main` and it takes location of the json as first parameter (here we assume your running it from `build` directory.

```
./Main ../vid.json 
```

with sample provided your expected to have
```
[x,y] = [242.284,209.586]
```
