ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive


# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    wget \
    libclang-dev \
    tmux \
    neovim \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pytest

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

WORKDIR /workspace

# Install Rust and the cargo-ament-build plugin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install cargo-ament-build
RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip install --user -U empy==3.3.4 pyros-genmsg 

RUN echo ". /workspace/portal/install/setup.bash" >> ~/.bashrc

WORKDIR /workspace/src

RUN git clone https://github.com/PX4/px4_msgs.git
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/ros2-rust/ros2_rust.git
RUN git clone https://github.com/OpenHORYZN/argus_common.git

WORKDIR /workspace

RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos

RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /workspace/src/singularity
COPY . .

WORKDIR /workspace

RUN . /opt/ros/humble/setup.sh && colcon build --packages-select singularity

CMD  ["/bin/sh", "-c",  ". install/setup.sh && ros2 run singularity singularity --ros-args -p \"machine:=sim\""]




