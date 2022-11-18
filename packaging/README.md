# Debian packages

Build docker image on which the packages will be built

```bash
docker build . -t tc-noetic-pkger
```

Then run the packaging command

```bash
docker run --rm -v ~/tc_ws/packages:/tc_ws/packaging tc-noetic-pkger /tc_ws/build-packages.sh
```

The packages will go in `~/tc_ws/packaging/focal`.

To install the packages on a machine, first setup the sources.list

```bash
sudo sh -c 'echo "deb [arch=amd64] http://129.132.73.77/apt/debian $(lsb_release -sc) main" > /etc/apt/sources.list.d/tesla-core.list'
curl -sSL 'http://129.132.73.77/rep.gpg.key' | sudo apt-key add -
```

Then update the cache with

```bash
sudo apt update
```

And you can install packages with

```bash
sudo apt install ros-noetic-mag-calculator
```
