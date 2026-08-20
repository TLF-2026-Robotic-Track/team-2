# Base image for the Duckiebot. Override it if this one is not available:
#   make build BASE_IMAGE=some/other-image:tag
ARG BASE_IMAGE=spgc/duckiebot-base-image:latest
FROM ${BASE_IMAGE}

# Default shell is /bin/sh, which has no "source". We need bash.
SHELL ["/bin/bash", "-c"]

# The code is NOT copied into the image. It is mounted at run time
# so you can edit files and rebuild without rebuilding the image.
WORKDIR /workspace

COPY requirements-apt.txt .
COPY requirements-python.txt .

RUN set -e; \
    if [ -s requirements-apt.txt ]; then \
        apt update; \
        xargs -a requirements-apt.txt apt install -y; \
        rm -rf /var/lib/apt/lists/*; \
    fi

RUN set -e; \
    if [ -s requirements-python.txt ]; then \
        pip install -r requirements-python.txt; \
    fi

# Every new shell inside the container gets ROS2 and this workspace sourced.
#
# Note: local_setup.bash, not setup.bash. setup.bash also re-sources the
# parent ROS prefixes, and /opt/ros/humble_local in this base image is
# incomplete, which prints an ugly error. local_setup.bash adds only the
# packages of this workspace, which is all we need here.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /workspace/install/local_setup.bash ] && source /workspace/install/local_setup.bash' >> /root/.bashrc && \
    echo 'cd /workspace' >> /root/.bashrc

CMD ["bash"]
