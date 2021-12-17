FROM ubuntu:18.04
RUN apt-get -y update
RUN apt-get -y install wget libssl-dev build-essential

WORKDIR /tmp

# install cmake
RUN wget https://github.com/Kitware/CMake/archive/refs/tags/v3.20.2.tar.gz -O cmake.tar.gz
RUN tar zxvf cmake.tar.gz
RUN cd CMake-3.20.2 && ./bootstrap && make -j4 && make install && cd ..
RUN hash -r
RUN rm -rf CMake-3.20.2

RUN echo 'alias l="ls -all"' >> ~/.bashrc
WORKDIR /workspace
