#! /bin/bash

DOCKER_BUILDKIT=1 docker build \
  --no-cache \
  --network=host \
  --build-arg BUILDKIT_STEP_TIMEOUT=600 \
  -t tfg-img .
