<h1 align="center">
  MaskUKF
</h1>

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/picture.png" alt=""/></p>


<h3 align="center">
  An Instance Segmentation Aided Unscented Kalman Filter for 6D Object Pose and Velocity Tracking
</h3>

<div align="center">
  Frontiers in Robotics and AI
</div>

<div align="center">
  <a href="https://www.frontiersin.org/articles/10.3389/frobt.2021.594583/full"><b>Paper</b></a> |
  <a href="https://www.youtube.com/watch?v=UZ1CGojdxrA"><b>Video</b></a>
</div>

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5448472.svg)](https://doi.org/10.5281/zenodo.5448472)
![CI badge](https://github.com/robotology/mask-ukf/workflows/C++%20CI%20Workflow/badge.svg)

## Reproducing the experiments

We support running the experiments via the provided Docker image.

1. Pull the docker image:
    ```console
    docker pull ghcr.io/robotology/mask-ukf:latest
    ```
1. Launch the container:
    ```console
    docker run -it --rm --user user ghcr.io/robotology/mask-ukf:latest
    ```
1. Clone and build the project:
    ```console
    git clone https://github.com/robotology/mask-ukf.git
    cd mask-ukf
    mkdir build && cd build
    cmake ../
    make
    ```
1. Download and unzip the accompanying data [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5419201.svg)](https://doi.org/10.5281/zenodo.5419201):
    ```console
    cd /home/user/mask-ukf
    wget https://zenodo.org/record/5419201/files/data.zip?download=1 -O data.zip
    unzip data.zip
    ```
1. Run the experiments (optional):
    ```console
    cd /home/user/mask-ukf
    bash test/test_all.sh
    ```
    > The accompanying data contains the pre-evaluated results. If desired, the results can be re-evaluated using the above command.
1. Run the evaluation:
    ```console
    cd /home/user/mask-ukf
    bash evaluation/evaluate_<mask_set>_<metric>_<algorithm>.sh
    ```
    where `<mask_set>` can be `mrcnn` (Mask R-CNN) or `posecnn` (PoseCNN), `<metric>` can be `add_s` (ADD-S) or `rmse` (RMSE) and `<algorithm>` can be empty (for MaskUKF), `icp` (ICP) or `densefusion` (DenseFusion).

If you want to install the repository manually, please refer to the recipe contained in the [**`Dockerfile`**](./dockerfiles/Dockerfile). Please be aware that the results might differ if unsupported versions of the dependencies are used.

## Citing MaskUKF

If you find the MaskUKF code useful, please consider citing the associated publication:

```bibtex
@ARTICLE{10.3389/frobt.2021.594583,
AUTHOR={Piga, Nicola A. and Bottarel, Fabrizio and Fantacci, Claudio and Vezzani, Giulia and Pattacini, Ugo and Natale, Lorenzo},
TITLE={MaskUKF: An Instance Segmentation Aided Unscented Kalman Filter for 6D Object Pose and Velocity Tracking},
JOURNAL={Frontiers in Robotics and AI},
VOLUME={8},
PAGES={38},
YEAR={2021},
URL={https://www.frontiersin.org/article/10.3389/frobt.2021.594583},
DOI={10.3389/frobt.2021.594583},
ISSN={2296-9144}
}
```

and/or the repository itself by pressing on the `Cite this respository` button in the **About** section.


## Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
