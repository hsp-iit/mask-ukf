FROM ubuntu:focal
LABEL org.opencontainers.image.title="MaskUKF Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing MaskUKF experiments on YCB-Video"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/robotology/mask-ukf/master/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Nicola A. Piga <nicola.piga@iit.it>"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Install essentials
RUN apt install -y build-essential cmake cmake-curses-gui emacs-nox git htop iputils-ping locales nano vim wget

# Set the locale
RUN locale-gen en_US.UTF-8

# Install dependencies
RUN apt install -y libeigen3-dev libmlpack-dev libopencv-dev libpcl-dev libtclap-dev libconfig++-dev

# Install dependencies (from source)

# Install bayes-filters-lib
RUN git clone https://github.com/robotology/bayes-filters-lib && \
    cd bayes-filters-lib && mkdir build && cd build && \
    cmake .. && \
    make install

# Install nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann && \
    cd nanoflann && mkdir build && cd build && \
    cmake -DNANOFLANN_BUILD_EXAMPLES:BOOL=OFF -DNANOFLANN_BUILD_TESTS:BOOL=OFF .. && \
    make install

# Create user
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user

# Switch to user
USER user

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
