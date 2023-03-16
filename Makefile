

IMAGE_NAME=foxy-uam:v0

all: build run
	@echo "Building and Running"

build:
	docker image build -t ${IMAGE_NAME} .

run:
	docker run --rm --gpus '"device=0"' --net=host --env DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}/src:/home/ubuntu/ros2_ws/src  -v /dev/shm:/dev/shm -it ${IMAGE_NAME}

	