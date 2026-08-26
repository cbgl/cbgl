FROM ros:lyrical-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ARG FSM_LOCALISATION_EXACT_FP=OFF
ARG FSM_LOCALISATION_DETERMINISTIC_FFT=OFF

RUN apt-get update \
  && apt-get install -y ca-certificates curl
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
RUN apt-get -y update                                                       && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo apt-utils build-essential g++ git curl lsb-release pkg-config \
    python3-rosdep ros-lyrical-rmw-cyclonedds-cpp gosu clang \
    bash-completion vim nano bc                                             && \
    rm -rf /var/lib/apt/lists/*
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ARG USER=cbgl
RUN useradd --create-home ${USER}                                           && \
    echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER}         && \
    chmod 0440 /etc/sudoers.d/${USER}                                       && \
    mkdir -p /home/${USER}
COPY --chown=${USER}:${USER} cbgl/docker/bashrc      /home/${USER}/.bashrc
COPY --chown=${USER}:${USER} cbgl/docker/bashrc_base /home/${USER}/.bashrc_base
COPY --chown=${USER}:${USER} cbgl/docker/bashrc_ros2 /home/${USER}/.bashrc_ros2
USER ${USER}
RUN mkdir /home/${USER}/shared-input
RUN mkdir /home/${USER}/shared-output

# rosdep update has to run as the account that runs rosdep install
# below, here ${USER}, or the cache it writes lands under root's home
# while everything else in this build, and the container itself, runs
# as ${USER}; the install then finds no cache and refuses outright.
# rosdep init would need root, since it writes under /etc, but this
# base image seeds its rosdep sources already, so there is nothing
# left here for root to do.
RUN rosdep update

# These two COPY lines are the reason this image's build context is
# not this repository alone: it is the directory holding this
# repository and fsm-localisation side by side, so both can be reached.
COPY --chown=${USER}:${USER} cbgl/cbgl/                        /home/${USER}/ros2_ws/src/cbgl/
COPY --chown=${USER}:${USER} fsm-localisation/fsm_localisation/ /home/${USER}/ros2_ws/src/fsm_localisation/

# libfftw3-dev is named directly because the libfftw3 rosdep key
# resolves, through ros/rosdistro's rule for it, to libfftw3-3, a
# package Ubuntu dropped in favour of one per floating point precision.
# The same key is skipped below so resolving the rest of both
# manifests does not fail looking for a package that is not there.
RUN sudo apt-get -y update                                                  && \
    sudo apt-get install -y --no-install-recommends libfftw3-dev libeigen3-dev && \
    rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -r -y   \
      --skip-keys "libfftw3"                                           && \
    sudo rm -rf /var/lib/apt/lists/*

# Both switches belong to fsm_localisation and cbgl follows whatever that
# package was built with. The matcher is a header cbgl compiles for itself,
# so two arithmetics over one header would put two definitions of one inline
# function into a single process. cbgl reads the choice out of what
# fsm_localisation installs rather than offering a switch of its own, which
# is why only two arguments reach this build rather than four.
RUN source /opt/ros/lyrical/setup.bash                                      && \
    cd /home/${USER}/ros2_ws                                                && \
    colcon build --packages-up-to cbgl                                        \
      --cmake-args                                                             \
        -DFSM_LOCALISATION_EXACT_FP=${FSM_LOCALISATION_EXACT_FP}               \
        -DFSM_LOCALISATION_DETERMINISTIC_FFT=${FSM_LOCALISATION_DETERMINISTIC_FFT}

USER root
COPY --chown=${USER}:${USER} cbgl/docker/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /home/${USER}

# The default command launches the node, which comes up in stand-by and computes
# nothing until its service is called. For a shell instead, override the command:
#
#   docker compose run --rm cbgl bash
#
# `exec` matters here. Without it the shell stays as the container's first
# process and the launch system is only its child, so a stop signal reaches the
# shell and dies there.
#
# The stop signal is an interrupt rather than a termination. A first process
# receives no default signal handling from the kernel: it acts only on signals it
# has installed a handler for. The launch system handles an interrupt, which is
# what a keyboard interrupt sends, and shuts its nodes down in order. It does not
# handle a termination, which is what a stop sends by default. Measured: a stop
# took the full ten second grace period and killed the node with no shutdown at
# all, against one second and an orderly shutdown with this line.
STOPSIGNAL SIGINT
CMD ["/bin/bash", "-lc", "exec ros2 launch cbgl cbgl.launch.xml"]
