# PSL-SLAM
**Authors:** Xuesong Xu,Wei Fan,Boyi Sun<br>
    Visual Simultaneous Localization and Mapping (SLAM) is a fundamental technology enabling autonomous robot navigation. Traditional SLAM algorithms heavily rely on point features, which often struggle in low-texture environments, leading to degraded tracking performance     or even failure. To address this limitation, multi-feature fusion strategies integrating line and plane features have been explored. However, existing line feature methods face issues such as endpoint drift, poor contour representation, and matching ambiguities. In        this paper, we propose a novel structural line feature composed of two coplanar intersecting line segments, effectively capturing 3D object contours using geometric constraints. We present a dual-modal VSLAM framework that integrates point and structural line              features, featuring robust extraction, structured matching using plane constraints, and a composite error model. Experimental results on the ICL-NUIM and TUM datasets demonstrate significant improvements in trajectory accuracy and feature stability in complex, low-        texture environments.
# PSL-SLAM 
We build our SLAM system based on ORB-SLAM2 RGBD version. For some prerequisites, you could read their page, [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)


We only add PCL library to deal with point cloud, we tested on PCL-1.80. The system only supports RGBD sensor. We have not removed those unrelevent components coming from ORB-SLAM2.
# RGB-D Example
## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```
## ICL Dataset
1. Download a sequence from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Use `ICL.yaml`  for ICL sequences . Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/ICL.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```
