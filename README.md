# Pico Toolchain Image

All this repository does it translate the instructions at https://lindevs.com/set-up-raspberry-pi-pico-sdk-on-ubuntu into a docker image.  This image can be used to build binaries for Raspberry Pico device.


To build the image:

     docker build -t picobuild .


Note, just add "--no-cache" to make sure you download all the latest stuff for this image rather than using potentially old cached packages.


You can then use the image to build binaries.  For example to build ALL the SDK example binaries, just do this:

     git clone https://github.com/raspberrypi/pico-examples.git
     docker run --rm -it -v `pwd`/pico-examples:/opt/src picobuild
     cd /opt/src
     mkdir build
     cd build
     cmake ..
     make



