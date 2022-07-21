#!/bin/bash
IMAGE_NAME="darko_ms1_pybullet_throw:python3"
USERNAME=ros2
CONTAINER_NAME=""
GPUS="all"
RUN_FLAGS=()

USER_ID=$(id -u "${USER}")
GROUP_ID=$(id -g "${USER}")
COMMAND_FLAGS+=(--uid "${USER_ID}")
COMMAND_FLAGS+=(--gid "${GROUP_ID}")

RUN_FLAGS+=(--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw)
RUN_FLAGS+=(--device=/dev/dri:/dev/dri)

if [ -z "${CONTAINER_NAME}" ]; then
  CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
  CONTAINER_NAME="${CONTAINER_NAME/:/-}-runtime"
fi

if [ -n "${USERNAME}" ]; then
  RUN_FLAGS+=(-u "${USERNAME}")
fi



if [ -n "${GPUS}" ]; then
  RUN_FLAGS+=(--gpus "${GPUS}")
  RUN_FLAGS+=(--env DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(--env NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}")
  RUN_FLAGS+=(--env NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics")
fi

xhost +
RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
RUN_FLAGS+=(--device=/dev/dri:/dev/dri)

if [ ${#FWD_ARGS[@]} -gt 0 ]; then
  echo "Forwarding additional arguments to docker run command:"
  echo "${FWD_ARGS[@]}"
fi

docker run -it --rm \
  --net host \
  "${RUN_FLAGS[@]}" \
  "${FWD_ARGS[@]}" \
  "${IMAGE_NAME}" /bin/bash
