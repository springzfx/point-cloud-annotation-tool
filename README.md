##  Overview
This point cloud annotation tool is forked from springzfx with new modification.

![example1 image](example_newe.png)
![Demo](pcl_demo.gif)

##  point cloud annotation tool
It is a tool used to annotate 3D box in point cloud. Point cloud in KITTI-bin format is supported. Annotation format is the same as Applo 3D format. Data examples can be found at [here](http://data.apollo.auto/help?name=data_intro_3d&data_key=lidar_obstacle_label&data_type=0&locale=en-us&lang=en).

### supported functions
- load, save, visiualize
- point cloud selection
- 3d box generation
- ground remove using threhold or plane detect

### New Features
- 3 Additional camera view (top, front, side) that snaps to object
- Ability to move box object in any render window
- Added a full screen feature to toggle current render window and normal view
- Real time update of box details in message box below
- Points in boxs are highlighted green
- Box Opacity reduce to see through box 
- Lowered point intensity to see clearer

### usage 
- if *cloud.bin* is open, then *cloud.bin.txt* will be the annotation file to be loaded if exist.
- click to select an annotaion, then edit it, rotate it or just press 'Del' to delete.
- 'x' to toggle selection mode,then left click with ctrl or shift to help select.
- click label button to annotate a 3D box.

### additional usage
- ctrl + z to toggl full screen and selected render window


### dependency
Tested with pcl 1.8, vtk 8.1, Qt5  under both ubuntu 16.04 and windows 10.
to build:
```
mkdir build && cd build && cmake .. && make
```

## Authors
1. Louis Goh 
2. springzfx- Initial Work

## License
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.




##  Acknowledgement
1. Fancy Zhang

(Meow!!) 
.....\ 
........_ 
.....<(o )___ 
......( .__> / 
........`-----' 
