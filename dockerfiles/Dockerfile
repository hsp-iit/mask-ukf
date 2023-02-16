FROM ubuntu:focal
LABEL org.opencontainers.image.title="MaskUKF Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing MaskUKF experiments on YCB-Video"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/hsp-iit/mask-ukf/master/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Nicola A. Piga <nicola.piga@iit.it>"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Install essentials
RUN apt update && \
    apt install -y build-essential cmake cmake-curses-gui curl emacs-nox git htop iputils-ping locales nano python3 python3-pip sudo vim wget && \
    rm -rf /var/lib/apt/lists/*

# Install GitHub cli
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt update && \
    apt install --no-install-recommends -y -qq gh && \
    rm -rf /var/lib/apt/lists/*

# Set the locale
RUN locale-gen en_US.UTF-8

# Install dependencies
RUN apt update && \
    apt install -y libeigen3-dev libmlpack-dev libopencv-dev libpcl-dev libtclap-dev libconfig++-dev

# Install dependencies (from source)

# Install bayes-filters-lib
RUN git clone https://github.com/robotology/bayes-filters-lib && \
    cd bayes-filters-lib && mkdir build && cd build && \
    cmake .. && \
    make install

# Install nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann && \
    cd nanoflann && git checkout 05d9c35d175fbcedd9f1f854ab49d04ae0bc5dbc && mkdir build && cd build && \
    cmake -DNANOFLANN_BUILD_EXAMPLES:BOOL=OFF -DNANOFLANN_BUILD_TESTS:BOOL=OFF .. && \
    make install

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user

# Configure emacs
RUN echo "(setq-default indent-tabs-mode nil)" >> /home/user/.emacs.el && \
    echo "(setq-default tab-width 4)" >> /home/user/.emacs.el && \
    echo "(setq make-backup-files nil)" >> /home/user/.emacs.el && \
    echo "(setq auto-save-default nil)" >> /home/user/.emacs.el && \
    echo "(setq c-default-style \"linux\"" >> /home/user/.emacs.el && \
    echo "      c-basic-offset 4)" >> /home/user/.emacs.el && \
    echo "(global-subword-mode 1)" >> /home/user/.emacs.el && \
    echo "(add-hook 'before-save-hook 'delete-trailing-whitespace)" >> /home/user/.emacs.el && \
    echo "(custom-set-variables '(custom-enabled-themes '(tango-dark)))" >> /home/user/.emacs.el && \
    echo "(custom-set-faces)" >> /home/user/.emacs.elx

# Install python dependencies
RUN pip3 install pyquaternion pyrender opencv-python

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
