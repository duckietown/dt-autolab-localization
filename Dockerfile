# parameters
ARG REPO_NAME="dt-autolab-localization"
ARG DESCRIPTION="Localization system for the Duckietown Autolab"
ARG MAINTAINER="Andrea F. Daniele (afdaniele@duckietown.com)"
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG ICON="map-marker"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO=daffy
ARG DOCKER_REGISTRY=docker.io
ARG BASE_IMAGE=dt-autolab-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as base

# recall all arguments
ARG DISTRO
ARG REPO_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG ICON
ARG BASE_TAG
ARG BASE_IMAGE
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# check build arguments
RUN dt-build-env-check "${REPO_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# define/create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${REPO_PATH}" "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE="${REPO_NAME}" \
    DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
    DT_MODULE_ICON="${ICON}" \
    DT_MAINTAINER="${MAINTAINER}" \
    DT_REPO_PATH="${REPO_PATH}" \
    DT_LAUNCH_PATH="${LAUNCH_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# download and install opencv-4.3.0 for the ArUco library
RUN apt-get update && apt-get install unzip
RUN cd / && wget -t 0 -T 15 -c https://github.com/opencv/opencv/archive/4.3.0.zip && \
    unzip 4.3.0.zip && rm -r 4.3.0.zip && \
    mkdir /opencv-4.3.0/build && cd /opencv-4.3.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j$(nproc) && make install && \
    cd / && rm -r /opencv-4.3.0

# download, fix the bug and install Boost.NumPy for python-boost binding
COPY ./packages/processing_node/src/boost_numpy_bug_fixer.py /boost_numpy_bug_fixer.py
RUN cd / && git clone https://github.com/ndarray/Boost.NumPy.git && python3 boost_numpy_bug_fixer.py && \
    cd Boost.NumPy && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j$(nproc) && make install && \
    cp lib/libboost_numpy.so /usr/local/lib/libboost_numpy.so && \
    cd / && rm -r /Boost.NumPy && rm boost_numpy_bug_fixer.py

# download, fix the bug and install the ArUco library
COPY ./packages/processing_node/src/aruco_bug_fixer.py /aruco_bug_fixer.py
RUN cd / && wget -t 0 -T 15 -c https://sourceforge.net/projects/aruco/files/3.1.12/aruco-3.1.12.zip && \
    unzip aruco-3.1.12.zip && rm -r aruco-3.1.12.zip && python3 aruco_bug_fixer.py && \
    mkdir /aruco-3.1.12/build && cd /aruco-3.1.12/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_CXX_FLAGS="-std=c++11" .. && \
    make -j$(nproc) && make install && ldconfig && \
    cd / && rm -r /aruco-3.1.12 && rm aruco_bug_fixer.py

# install apt dependencies
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple/"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${REPO_PATH}/"
RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

# copy the source code
COPY ./packages "${REPO_PATH}/packages"

# build ArUco library adapter for python
RUN apt-get update && apt-get install ros-noetic-image-geometry
RUN cd "${REPO_PATH}/packages/processing_node/lib/aruco" && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && make -j$(nproc) aruco_caller && \
    cp aruco_caller.so ../../../src/aruco_caller.so && cd .. && rm -r build

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.module.icon="${ICON}" \
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
# <== Do not change the code above this line
# <==================================================

# remove pip package `dataclasses`
RUN python3 -m pip uninstall -y dataclasses

# copy the ApriltagsDB code
# TODO: this is temporary, only used because the apriltag-postproc node is a mess
COPY ./assets/apriltagsDB.yaml "${REPO_PATH}/assets/apriltagsDB.yaml"
COPY ./assets/maps "${REPO_PATH}/assets/maps"
