FROM ros:lyrical-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ARG CBGL_EXACT_FP=OFF
ARG CBGL_DETERMINISTIC_FFT=OFF
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
    bash-completion vim nano bc patch                                       && \
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

# cbgl is gaining a second refinement stage built on csm, Andrea Censi's
# canonical scan matcher, and no apt package name will ever put it on this
# image: it never had a ROS 2 release, its last release was for a ROS 1
# distribution, and it is absent from this distribution's own package
# mirror. There is no rosdep rule for the name either, and there never was
# one, so once a manifest does declare a dependency on it the key will
# need skipping explicitly below, in the same manner libfftw3 already is
# on that line, rather than resolved. It is built from source here
# instead, placed ahead of the colcon build below so that it is already on
# the system once cbgl is built against it. Eigen headers are installed
# directly here too, for the same reason libfftw3-dev is named directly
# further down: nothing has put them on the image yet at this point, since
# that only happens once rosdep resolves fsm_localisation's manifest below.
#
# The Eigen variant is built, on the csm_eigen branch, and a fixed commit
# is checked out rather than the branch tip, because the three patches
# carried beside this file are written against the sources exactly as that
# commit stands and are not guaranteed to still apply, or to still be
# needed, further along the branch.
#
# It is built shared rather than left at the static default its own build
# file produces. csm is LGPL v3, and the relinking obligation that licence
# carries is met by linking against it dynamically, not by a static
# archive baked into whatever links against it. Its build file already
# carries a second target forced static regardless of this switch. That
# target is built and then its archive is deleted after the install, so
# the only csm on the image is the shared one. Upstream installs both, and
# leaving the archive there would leave a route to a static link that
# nothing announces and that a reader of a linker command is the only
# person likely to catch.
#
# Three patches are applied. The first is needed before the sources
# configure at all here: the build file asks for CMake 2.8, and CMake
# 4.2.3, which this image carries, refuses anything short of 3.5 outright
# and stops before reading another line.
#
# The second corrects a neighbour gathering routine that writes one entry
# past the end of its own buffer, on the code path the angle based
# rejection of correspondences uses. cbgl's own compiled default leaves
# that rejection off, so an unpatched matcher would build and run happily
# until something switched it on, which the settings the ROS 1 build ships
# do. Switched on, the standard library's own bounds check stops the
# process rather than letting the write land quietly, and cbgl's own tests
# exercise that very settings set, so an unpatched build fails the suite
# rather than reaching a robot. Built with both, the sources compile clean
# under GCC 15.2 at C++23, and neither the address sanitizer nor the
# undefined behaviour sanitizer has anything to say about the result.
#
# The third patch corrects what the matcher's own pkg-config file answers
# with. That file names the static library, so anybody who asks
# pkg-config how to link against the matcher, which is the obvious thing
# to do and the reason the file is shipped, links the static one instead
# and undoes the whole point of the switch above. Nothing announces it;
# reading the linker command afterwards is the only way to notice. The
# patch makes the file name the shared library, and changes nothing else
# about it.
#
# The optimisation flag is passed by hand rather than by asking for a
# release build. The matcher's own build file sets no flags and names no
# default build type, so configuring it without one compiles the whole
# library at no optimisation at all. That is several times slower than it
# needs to be, in the one place the refinement spends its time, and it
# also leaves the toolchain's own buffer checks inactive, since those need
# optimisation to be compiled in at all. Asking for a release build would
# fix both and cost something worse: a release build also defines NDEBUG,
# which disarms the assertions guarding the matcher's own fixed size pools
# and turns what is now a clean stop into a quiet write past the end of
# one. So the flag is given on its own and the assertions stay armed.
COPY --chown=${USER}:${USER} cbgl/docker/csm_cmake_floor.patch      /tmp/csm-patches/csm_cmake_floor.patch
COPY --chown=${USER}:${USER} cbgl/docker/csm_neighbour_bounds.patch /tmp/csm-patches/csm_neighbour_bounds.patch
COPY --chown=${USER}:${USER} cbgl/docker/csm_pkgconfig_shared.patch /tmp/csm-patches/csm_pkgconfig_shared.patch
RUN sudo apt-get -y update                                                     && \
    sudo apt-get install -y --no-install-recommends libeigen3-dev              && \
    sudo rm -rf /var/lib/apt/lists/*                                           && \
    git clone https://github.com/AndreaCensi/csm.git /tmp/csm                  && \
    cd /tmp/csm                                                                && \
    git checkout 04c4a31072e8690bd0ff14de50be20c5b6b43e66                      && \
    for patch_file in /tmp/csm-patches/*.patch; do                                \
      patch -p1 --forward --batch --fuzz=0 < "${patch_file}" || {                  \
        echo "FATAL: ${patch_file} no longer applies to csm's csm_eigen"   >&2;   \
        echo "branch at commit 04c4a31072e8690bd0ff14de50be20c5b6b43e66."  >&2;   \
        echo "The build stops here rather than continue against sources"  >&2;   \
        echo "it cannot trust are patched."                                >&2;   \
        exit 1; };                                                                \
    done                                                                       && \
    cmake -S /tmp/csm -B /tmp/csm/build                                           \
      -DBUILD_SHARED_LIBS=ON -DCMAKE_CXX_FLAGS=-O2                                 \
      -DCMAKE_INSTALL_PREFIX=/usr/local                                        && \
    cmake --build /tmp/csm/build                                               && \
    sudo cmake --install /tmp/csm/build                                        && \
    sudo rm -f /usr/local/lib/libcsm-static.a                                  && \
    sudo ldconfig                                                              && \
    rm -rf /tmp/csm /tmp/csm-patches

# libfftw3-dev is named directly because the libfftw3 rosdep key
# resolves, through ros/rosdistro's rule for it, to libfftw3-3, a
# package Ubuntu dropped in favour of one per floating point precision.
# The same key is skipped below so resolving the rest of both
# manifests does not fail looking for a package that is not there.
#
# csm is skipped for a different reason with the same shape. There is no
# rosdep rule for that name at all and there never was one, so the key
# resolves to nothing rather than to the wrong thing. It is built from
# source further up, so by the time this runs it is already installed and
# there is nothing for rosdep to do about it either way.
RUN sudo apt-get -y update                                                  && \
    sudo apt-get install -y --no-install-recommends libfftw3-dev            && \
    rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -r -y   \
      --skip-keys "libfftw3 csm"                                           && \
    sudo rm -rf /var/lib/apt/lists/*

# Both packages are built in one invocation because the deterministic
# transform switch lives in a header both packages include; setting it
# for cbgl alone would still leave fsm_localisation built against the
# default.
RUN source /opt/ros/lyrical/setup.bash                                      && \
    cd /home/${USER}/ros2_ws                                                && \
    colcon build --packages-up-to cbgl                                        \
      --cmake-args                                                             \
        -DCBGL_EXACT_FP=${CBGL_EXACT_FP}                                       \
        -DCBGL_DETERMINISTIC_FFT=${CBGL_DETERMINISTIC_FFT}                     \
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
