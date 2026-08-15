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
IMAGE      ?= ros:$(ROS_DISTRO)-ros-base
WS         := /ws

# --network=host so rosdep and apt can reach the network; --rm so nothing is
# left behind. The workspace is bind-mounted rather than copied, so build
# artefacts land in the host's build/ and incremental builds work.
DOCKER_RUN = docker run --rm -v "$(CURDIR)":$(WS) -w $(WS) --network=host

.PHONY: all build test shell clean submodule

all: build

# The build fails with a clear message if this has not been run, but doing it
# here means a fresh clone needs one command rather than two.
submodule:
	@git submodule update --init --recursive

build: submodule
	$(DOCKER_RUN) $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && \
	  colcon build --symlink-install'

test: submodule
	$(DOCKER_RUN) $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && \
	  colcon build --symlink-install && \
	  colcon test && \
	  colcon test-result --verbose'

shell: submodule
	$(DOCKER_RUN) -it $(IMAGE) bash -lc '\
	  . /opt/ros/$(ROS_DISTRO)/setup.sh && exec bash'

clean:
	rm -rf build install log
