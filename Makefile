# simplebgc32-ros2 — developer conveniences.
#
# The packages are built with colcon, not this Makefile. This exists because
# the workstation this is developed on has no ROS installation, so every build
# runs inside the official ros image. On a machine that already has ROS
# sourced, ignore this file and use colcon directly.
#
#   make build      colcon build inside a ros container
#   make test       colcon build && colcon test, then print the results
#   make shell      interactive shell in the container, workspace mounted
#   make clean      remove build/ install/ log/
#
# Override the distro to test the forward target:
#   make test ROS_DISTRO=lyrical

ROS_DISTRO ?= jazzy
IMAGE      ?= simplebgc32-ros2-dev:$(ROS_DISTRO)
WS         := /ws

# The ros images declare no USER, so without --user every build would write
# root-owned build/, install/ and log/ trees into the bind mount and the host
# could not clean them up afterwards. HOME is redirected because the mapped UID
# has no passwd entry in the image, and colcon wants somewhere writable.
UIDGID := $(shell id -u):$(shell id -g)

# --network=host so rosdep and apt can reach the network; --rm so nothing is
# left behind. The workspace is bind-mounted rather than copied, so build
# artefacts land in the host's build/ and incremental builds work.
DOCKER_RUN = docker run --rm -v "$(CURDIR)":$(WS) -w $(WS) --network=host \
             --user $(UIDGID) -e HOME=/tmp

.PHONY: all build test shell clean submodule image

all: build

# The build fails with a clear message if this has not been run, but doing it
# here means a fresh clone needs one command rather than two.
submodule:
	@git submodule update --init --recursive

# Rebuilding is a no-op once the layers are cached, so every target can depend
# on this without anyone having to remember to run it.
image:
	@docker build -q -t $(IMAGE) \
	  --build-arg ROS_DISTRO=$(ROS_DISTRO) \
	  -f docker/Dockerfile.dev docker/ >/dev/null

# Deliberately no --symlink-install. Its symlinks record absolute paths under
# the container's /ws, so an install tree built this way is full of dangling
# links when read from the host. The convenience it buys is not worth an
# install/ directory that only resolves inside a container.
build: submodule image
	$(DOCKER_RUN) $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && \
	  colcon build'

test: submodule image
	$(DOCKER_RUN) $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && \
	  colcon build && \
	  colcon test && \
	  colcon test-result --verbose'

shell: submodule image
	$(DOCKER_RUN) -it $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && exec bash'

clean:
	rm -rf build install log
