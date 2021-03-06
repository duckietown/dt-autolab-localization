# dt-autolab-localization

# Requirements

In order to build this image, you need two things,

- Docker
- Duckietown Shell

## Install Duckietown Shell

You can install the Duckietown Shell with the command,

```shell script
pip3 install -U duckietown-shell
```

# Build

From the root of this repository, run

```shell script
dts devel build -f
```

This will build a Docker image out of the project.


# Run

From the root of this repository, run

```shell script
dts devel run -f -M -X
```

This will run the image you just built. The script that will
run inside the container is (by default), the file 
`launchers/default.sh`.

If you want to create your own launcher, 
say `launchers/my_launcher.sh`, you can then run it with

```shell script
dts devel run -f -M -X -L my_launcher
```
