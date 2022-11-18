# Installation Steps

## Installing Git LFS

Git LFS is used to store the tensorflow binaries and to store pretrained models.
For this package to work, you need
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

## Building

When building the default pretrained CardioMag model is extracted to models/cmag_cnn_v1.

Note that these folders are not cleaned up when running catkin clean and you need
to delete them yourself.

## Using GPU version (Advanced)

By default this compiles a version of TensorFlow that is compiled to run on CPUs.
To use the GPU Version, you need to install a NVidia compatible driver and the
Cuda libraries.

So far, we support TensorFlow 1.15 which uses Cuda version 10.0. To install cuda
on Ubuntu 18.04.

```bash
# Install CUDA Toolkit 10
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/
7fa2af80.pub && sudo apt update
sudo dpkg -i cuda-repo-ubuntu1804_10.0.130-1_amd64.deb

sudo apt update
sudo apt install -y cuda

# Install CuDNN 7 and NCCL 2
wget https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
sudo dpkg -i nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb

sudo apt update
sudo apt install -y libcudnn7 libcudnn7-dev libnccl2 libc-ares-dev

sudo apt autoremove
sudo apt upgrade
```

You need to make sure the cuda libraries are visible by the dynamic linker.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.0/lib64
```

On a desktop with a Titan Xp GPU, a single model query took 2 ms vs 17 ms on CPU.
A succesful run will display the following (if you did not disable TensorFlow messages)

<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
```bash
name: TITAN Xp major: 6 minor: 1 memoryClockRate(GHz): 1.582
pciBusID: 0000:02:00.0
2021-03-18 16:29:40.593334: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcudart.so.10.0
2021-03-18 16:29:40.593360: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcublas.so.10.0
2021-03-18 16:29:40.593380: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcufft.so.10.0
2021-03-18 16:29:40.593401: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcurand.so.10.0
2021-03-18 16:29:40.593421: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcusolver.so.10.0
2021-03-18 16:29:40.593441: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcusparse.so.10.0
2021-03-18 16:29:40.593461: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcudnn.so.7
2021-03-18 16:29:40.594288: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1746] Adding visible gpu devices: 0
2021-03-18 16:29:40.594328: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1159] Device interconnect StreamExecutor with strength 1 edge matrix:
2021-03-18 16:29:40.594344: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1165]      0
2021-03-18 16:29:40.594357: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1178] 0:   N
2021-03-18 16:29:40.595249: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1304] Created TensorFlow device (/job:localhost/replica:0/task:0/device:GPU:0 with 11387 MB memory) -> physical GPU (device: 0, name: TITAN Xp, pci bus id: 0000:02:00.0, compute capability: 6.1)
2021-03-18 16:29:40.605795: I tensorflow/cc/saved_model/loader.cc:202] Restoring SavedModel bundle.
2021-03-18 16:29:40.675770: I tensorflow/cc/saved_model/loader.cc:311] SavedModel load for tags { serve }; Status: success. Took 87522 microseconds.
bm_runModel                    2796058 ns     217188 ns       2378
```
<!-- markdownlint-restore -->

## Supressing TensorFlow Messages

To suppress the messages that tensorflow displays to the console, you can set
the following environment variable in the terminal that you are running from.

```bash
export TF_CPP_MIN_LOG_LEVEL=3
```

## Disable TensorFlow

To disable tensorflow when building ros packages add the argument `--cmake-args -DDISABLE_TENSORFLOW=On`.

For example:

```bash  
catkin build mag_manip --cmake-args -DDISABLE_TENSORFLOW=On
```

Make sure compile mag_tensorflow is also compiled.
