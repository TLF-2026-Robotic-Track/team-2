# IMAGE       our image, the one this Makefile builds. Shows up in "docker images".
#             The same for everybody, it is built from the requirements files.
# BASE_IMAGE  the image we start FROM. Lives on Docker Hub. We never build it.
# NAME        the name of the running container. One container per robot, the
#             whole team shares it. Override it if you need your own:
#             make run NAME=duckiebot-vasya
IMAGE      ?= duckie-image
BASE_IMAGE ?= spgc/duckiebot-base-image:latest
NAME       ?= duckiebot

# The folder this Makefile sits in, no matter where you run make from.
REPO_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

# Build the image, then build the ROS2 workspace inside it.
# --symlink-install links the node files instead of copying them, so editing
# a .py file takes effect straight away, without building again.
build:
	docker build -t $(IMAGE) --build-arg BASE_IMAGE=$(BASE_IMAGE) --progress=plain $(REPO_DIR)
	docker run --rm -v $(REPO_DIR):/workspace --entrypoint=bash $(IMAGE) \
		-c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Start the container in the background and leave it running. Your nodes keep
# running even if the SSH connection drops.
#   --network=host        so the container can see the robot's topics
#   --privileged          access to hardware
#   -v /dev/shm:/dev/shm  shared memory with the robot's own ROS2 containers
#   -v $(REPO_DIR):/workspace  your code, live
up:
	docker start $(NAME) 2>/dev/null || \
	docker run -d --name $(NAME) --network=host --privileged \
		-e VEHICLE_NAME -e USER_NAME \
		-v /dev/shm:/dev/shm -v $(REPO_DIR):/workspace $(IMAGE) sleep infinity

# Open a shell inside the container. Run it in as many terminals as you like.
# "-e VEHICLE_NAME" with no value hands over the value from your own shell,
# because docker exec does not inherit your environment by itself.
shell: up
	docker exec -it -e VEHICLE_NAME -e USER_NAME $(NAME) bash

# Same as "make shell". Starts the container if it is not running yet.
run: shell

# Stop and delete the container. Your code is outside, so nothing is lost.
# Do this after moving or renaming the project folder, then "make up" again.
down:
	docker rm -f $(NAME)

# Delete the folders colcon generates. Use it when colcon behaves strangely.
clean:
	rm -rf $(REPO_DIR)/build $(REPO_DIR)/install $(REPO_DIR)/log

.PHONY: build up shell run down clean
