# Tesla_core

![main](https://github.com/ethz-msrl/Tesla_core/workflows/main/badge.svg)

![Lint Code Base](https://github.com/ethz-msrl/Tesla_core/actions/workflows/linter.yml/badge.svg)

![navion-integration-test](https://github.com/ethz-msrl/Tesla_core/workflows/navion_integration_test/badge.svg)

![dockerize](https://github.com/ethz-msrl/Tesla_core/actions/workflows/docker.yaml/badge.svg)

This is the Tesla\_core repository for reviewed, application-independent code.

Please hold to the [guidelines](GUIDELINES.md) when working in this repository. You can also find additional information about contributing to Tesal_core and the CI in the wiki.

## Installation

### Installing using APT

Note that the APT server can only be accessed from within ETH network. You can use the VPN if you are not located at ETH.

To install the packages on a machine, first setup the sources.list

```bash
sudo sh -c 'echo "deb [arch=amd64] http://129.132.73.77/apt/debian $(lsb_release -sc) main" > /etc/apt/sources.list.d/tesla-core.list'
curl -sSL 'http://129.132.73.77/apt/conf/tesla_core.gpg.key' | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-mag-calculator
```

### Installing from Debian Packages

You can find the compiled debian packages as an asset in each release. For example go
[here](https://github.com/ethz-msrl/Tesla_core/releases/tag/v3.1.0) and download the `noetic.zip` file.
After unzipping, you can install all packages by using

```bash
cd focal
sudo dpkg -i *.deb
```

### Building from Source

#### Set up a workspace

First you need to add an SSH key to your account. Go to your Github account
settings and [add a key under SSH
Keys](https://help.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account).

```bash
mkdir -p <workspace_name>/src
cd <workspace_name>
catkin init
catkin config --extend /opt/ros/$ROS_DISTRO
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
git clone
git@github.com:ethz-msrl/Tesla_core.git
```

#### Install dependencies

ROS packages:

```bash
wstool init
wstool merge Tesla_core/dependencies.rosinstall
wstool update
```

Wstool pulls the repositories from Github. In case they are not available
anymore, you can find them in the [old Tesla repository external
folder](https://gogs.msrl.ethz.ch/MSRL/Tesla/src/4ab2b2d9d58842e2aaddad4011266410fe96b4cc/external).

```bash
sudo apt-get install swig
```

#### Installing Git LFS

Git LFS is used to store the tensorflow binaries and to store pretrained models.
For the mag_tensorflow package to work, you need
git-lfs installed. On Ubuntu, run the following:

<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
```
<!-- markdownlint-restore -->

From ~/tc_ws/src/Tesla_core, you need to run

```bash
git lfs install
# this will get all the large files needed for the Tesla_core repo
git lfs pull
```

#### Build packages

```bash
cd ~/<workspace_name>
source devel/setup.bash
catkin build
```

### Changes in ROS Noetic

* Python 3 is supported exclusively. All imports of mag_manip and mpem need to
be changed from `import mag_manip` to `from mag_manip import mag_manip`. See #64.

## Citation

If you use Tesla_core for your research, please cite the following publications based on the packages that you use.

### mpem

```markdown
@ARTICLE{Petruska2017,
    author = {Petruska, Andrew and Edelmann, Janis and Nelson, Brad},
    year = {2017},
    month = {01},
    pages = {1-1},
    title = {Model-Based Calibration for Magnetic Manipulation},
    volume = {PP},
    journal = {IEEE Transactions on Magnetics},
    doi = {10.1109/TMAG.2017.2653080}
}
```

### mag_calculator

```markdown
@ARTICLE{Charreyron2021,
    author={Charreyron, Samuel L. and Boehler, Quentin and Kim, Byungsoo and Weibel, Cameron and Chautems, Christophe and Nelson, Bradley J.},
    journal={IEEE Transactions on Robotics},
    title={Modeling Electromagnetic Navigation Systems},
    year={2021},
    volume={37},
    number={4},
    pages={1009-1021},
    doi={10.1109/TRO.2020.3047053}
}
```

### mag_tensorflow

```markdown
@INPROCEEDINGS{Yu2020,
    author={Yu, Ruoxi and Charreyron, Samuel L. and Boehler, Quentin and Weibel, Cameron and Chautems, Christophe and Poon, Carmen C. Y. and Nelson, Bradley J.},
    booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
    title={Modeling Electromagnetic Navigation Systems for Medical Applications using Random Forests and Artificial Neural Networks},
    year={2020},
    volume={},
    number={},
    pages={9251-9256},
    doi={10.1109/ICRA40945.2020.9197212}
}
```
