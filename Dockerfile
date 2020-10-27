# ubuntu as basis
FROM ubuntu:18.04

# preparation
RUN apt-get update && apt-get install -y build-essential make cmake git
RUN apt-get update && apt-get install -y doxygen doxygen-doc graphviz

# copy code
COPY . /app

# download libs
RUN cd /app/lib && rm -rf *
RUN cd /app/lib && git clone --branch "release-1.10.0" --recursive https://github.com/google/googletest.git
RUN cd /app/lib && git clone --branch dev --recursive https://github.com/JensKlimke/SimCore.git

# build process
RUN cd /app \
    && mkdir -p build \
    && cd build \
    && cmake -DBUILD_TESTING=ON -DCREATE_DOXYGEN_TARGET=ON -DBUILD_GTEST_LIBRARY=ON -DENABLE_COVERAGE=ON .. \
    && make

# test process
RUN cd /app/build && make test

# installation process
RUN cd /app/build && make install

# command
CMD bash