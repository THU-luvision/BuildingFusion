# BuildingFusion: Semantic-aware Structural Building-scale 3D Reconstruction (TPAMI 2021)

By Tian Zheng; Guoqing Zhang; Lei Han; Lan Xu; Lu Fang*. (\*) Corresponding author.
[[Paper]](https://ieeexplore.ieee.org/abstract/document/9286413) 


<!-- <div align="center">
<img src="docs/scene0249_00_output_2.gif" width="48%" />
<img src="docs/scene0430_00_output_2.gif" width="48%" />
</div>

<br> -->

## Introduction
This is the official code repository for BuildingFusion, a semantic-aware structural building-scale reconstruction system, which allows collaborative building-scale dense reconstruction, with online semantic and structural understanding. It is able to handle large scale scenes (~1000m^2).

This is a project from LuVision SIGMA, Tsinghua University. Visit our website for more interesting works: http://www.luvision.net/

## License
This project is released under the [GPLv3 license](LICENSE). We only allow free use for academic use. For commercial use, please contact us to negotiate a different license by: `fanglu at tsinghua.edu.cn`

## Citing

If you find our code useful, please kindly cite our paper:

```bibtex
@ARTICLE{9286413,
  author={Zheng, Tian and Zhang, Guoqing and Han, Lei and Xu, Lan and Fang, Lu},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence}, 
  title={Building Fusion: Semantic-aware Structural Building-scale 3D Reconstruction}, 
  year={2020},
  volume={},
  number={},
  pages={1-1},
  doi={10.1109/TPAMI.2020.3042881}}
```

## Quickstart with docker
0. Install docker and nvidia runtime following [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html). A CUDA capable GPU is required.
1. Download the test data from http://www.example.com
2. Modify the correct paths to the data in `docker_run.sh`
3. Start with `bash docker_run.sh`

