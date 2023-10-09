FROM ubuntu:jammy

# Prepare environment
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y build-essential git cmake curl python3

# Install toolchain
RUN ARM_TOOLCHAIN_VERSION=$(curl -s https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads | grep -Po '<h4>Version \K.+(?=</h4>)') && curl -Lo gcc-arm-none-eabi.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz"
RUN mkdir /opt/gcc-arm-none-eabi
RUN tar xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C /opt/gcc-arm-none-eabi
RUN rm gcc-arm-none-eabi.tar.xz
RUN echo 'export PATH=$PATH:/opt/gcc-arm-none-eabi/bin' | tee -a /etc/profile.d/gcc-arm-none-eabi.sh

# Install SDK
RUN git clone https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk
RUN git -C /opt/pico-sdk submodule update --init
RUN echo 'export PICO_SDK_PATH=/opt/pico-sdk' | tee -a /etc/profile.d/pico-sdk.sh


ENTRYPOINT ["bash", "-l"]







