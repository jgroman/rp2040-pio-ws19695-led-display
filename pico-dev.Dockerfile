ARG base_tag=bullseye
ARG base_img=mcr.microsoft.com/vscode/devcontainers/base:dev-${base_tag}

FROM --platform=linux/amd64 ${base_img} AS builder-install

RUN apt-get update --fix-missing && apt-get -y upgrade
RUN apt-get install -y --no-install-recommends \
    apt-utils \
    autoconf \
    automake \
    build-essential \
    cmake \
    git \
    libftdi-dev \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libtool \
    libusb-1.0-0-dev \
    ninja-build \
    pkg-config \
    texinfo \
    && rm -rf /var/lib/apt/lists/*

ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen \
    && /usr/sbin/locale-gen \
    && echo "alias ll='ls -laGFh'" >> /root/.bashrc

ENV PICO_SDK_PATH=/home/vscode/pico/pico-sdk
ENV PICO_EXAMPLES_PATH=/home/vscode/pico/pico-examples

USER vscode

RUN cd \
    && mkdir pico \
    && cd pico/ \
    && git clone https://github.com/raspberrypi/pico-sdk.git --branch master --depth 1 --single-branch \
    && cd pico-sdk \
    && git submodule update --init \
    && cd ~/pico/ \
    && git clone https://github.com/raspberrypi/picotool.git --branch master --depth 1 --single-branch \
    && cd picotool \
    && mkdir build \
    && cd build \
    && cmake ../ \
    && make \
    && sudo cp picotool /usr/local/bin/ \
    && cd ~/pico/ \
    && git clone https://github.com/raspberrypi/openocd.git --branch rp2040 --recursive --depth=1 --single-branch \
    && cd openocd \
    && ./bootstrap \
    && ./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio \
    && make -j4 \
    && sudo make install \
    && cd ~/pico \
    && git clone https://github.com/raspberrypi/pico-examples.git --branch master --depth 1 --single-branch 

# Official ARM GNU toolchain has broken gdb, we will use using xpack release which has this fixed
ARG ARM_TOOLCHAIN_ARCHIVE=xpack-arm-none-eabi-gcc-12.2.1-1.2-linux-x64.tar.gz
RUN cd \
    && wget https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v12.2.1-1.2/$ARM_TOOLCHAIN_ARCHIVE \
    && tar xf $ARM_TOOLCHAIN_ARCHIVE -C ~/pico/ \
    && rm -rf $ARM_TOOLCHAIN_ARCHIVE

ENV PATH="/home/vscode/pico/xpack-arm-none-eabi-gcc-12.2.1-1.2/bin:${PATH}"

VOLUME ["/builder/mnt"]
WORKDIR /builder/mnt
