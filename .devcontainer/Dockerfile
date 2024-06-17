# Use the base image from Microsoft Dev Containers
FROM mcr.microsoft.com/devcontainers/base:jammy

# Install essential packages and dependencies
RUN apt-get update && \
    apt-get install -y python3 python3-pip git wget gfortran liblapack-dev pkg-config cmake gcc swig g++ pybind11-dev

# Upgrade pip and setuptools
RUN pip3 install --upgrade pip setuptools

# Find the python site packages directory
RUN export PYTHON_SITE_PACKAGES=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") && \
    echo $PYTHON_SITE_PACKAGES > /tmp/python_site_packages

# Install Casadi with Python bindings from source
RUN git clone https://github.com/casadi/casadi.git && \
    mkdir casadi/build && cd casadi/build && \
    export PYTHON_SITE_PACKAGES=$(cat /tmp/python_site_packages) && \
    cmake -DWITH_IPOPT=OFF -DWITH_BUILD_IPOPT=OFF \
    -DWITH_BUILD_MUMPS=OFF -DWITH_BUILD_METIS=OFF \
    -DWITH_OPENMP=OFF -DWITH_THREAD=OFF -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DPYTHON_PREFIX=$PYTHON_SITE_PACKAGES \
    -DINSTALL_INTERNAL_HEADERS=ON .. && \
    make -j && sudo make install 

# Install Fatropy with Spectool from source
RUN git clone https://github.com/meco-group/fatrop.git --recursive && \
    export CMAKE_ARGS="-DBLASFEO_TARGET=X64_AUTOMATIC -DENABLE_MULTITHREADING=OFF -DWITH_SPECTOOL=ON" && \
    cd fatrop/fatropy && \
    python3 -m pip install .

# install latex
# RUN apt-get -y --no-install-recommends install texlive-latex-base texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra

# install ffmpeg
RUN apt-get -y --no-install-recommends install ffmpeg


# install numpy, matplotlib, scipy, panda
RUN pip3 install numpy matplotlib scipy pandas tqdm