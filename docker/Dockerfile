# Base Image
FROM ros:humble-ros-base-jammy

# Set environment variables to prevent interaction during build
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && \
	apt-get -y upgrade && \
	apt-get install -y \
	build-essential \
	curl \
	ffmpeg \
	fontconfig \
	git \
	libcairo2-dev \
	libjpeg-dev \
	libpng-dev \
	libsm6 \
	libssl-dev \
	libtiff5-dev \
	libxext6 \
	pkg-config \
	python3-colcon-common-extensions \
	python3-dev \
	python3-pip \
	python3-rosdep \
	ros-"${ROS_DISTRO}"-lanelet2 \
	ros-"${ROS_DISTRO}"-launch \
	ros-"${ROS_DISTRO}"-launch-ros \
	ros-"${ROS_DISTRO}"-rmw-cyclonedds-cpp \
	sudo \
	wget \
	unzip \
	zlib1g-dev \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

# Set working directory
ENV WORK_DIR=/carla
WORKDIR ${WORK_DIR}

# Copy files
COPY . .

# Install Python dependencies
RUN python3 -m pip install --upgrade pip wheel \
	&& python3 -m pip install "setuptools<=70.3.0" \
	&& python3 -m pip install -r docker/requirements.txt --default-timeout=2000 \
	&& python3 -m pip cache purge

# Import repositories
RUN mkdir -p src/external \
	&& vcs import --recursive src/external < docker/carla.repos \
	&& vcs import --recursive src/external < docker/ros2.repos

# Install dependencies and build packages
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
	&& rosdep update \
	&& apt update \
	&& rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 



# Set additional environment variables
#ENV SCENARIO_RUNNER_ROOT=${WORK_DIR}/external/scenario_runner/scenario_runner_docker/scenario_runner
#ENV AGENTS_ROOT=${WORK_DIR}/external/scenario_runner/scenario_runner_docker
#ENV LEADERBOARD_ROOT=${WORK_DIR}/external/leaderboard
#ENV PROJECT_ROOT=${WORK_DIR}/src/autoware_agent
#ENV CONFIG_ROOT=${WORK_DIR}/config
ENV ROS_PATH=/opt/ros/"${ROS_DISTRO}"/lib/python3.10/site-packages
ENV PYTHONPATH="${PYTHONPATH}":"${ROS_PATH}"

# Create entrypoint
RUN chmod +x docker/entrypoint.sh
# Add sourcing of the ROS2 workspace to .bashrc file
RUN if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then \
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc; \
	fi && \
	if  [ -f "${WORK_DIR}/install/setup.bash" ]; then \
	echo  "source ${WORK_DIR}/install/setup.bash" >> ~/.bashrc; \
	fi


# Default entrypoint and command
ENTRYPOINT ["docker/entrypoint.sh"]
CMD ["bash"]
