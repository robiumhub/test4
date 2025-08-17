# Dev-friendly base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive     TZ=Etc/UTC

RUN apt-get update  && apt-get install -y --no-install-recommends     bash ca-certificates curl git sudo build-essential  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Default command keeps the container running for multi-terminal dev
CMD ["bash"]
