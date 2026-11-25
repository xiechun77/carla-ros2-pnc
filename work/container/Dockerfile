FROM ros@sha256:078ac8eaffc953e97d4abf72f57da1fc33080b720b94bb1dc8c05ef4f7690367 AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates python3-pip python3-numpy \
    python3-scipy python3-networkx python3-pygame libeigen3-dev \
    gnuplot-nox ros-foxy-derived-object-msgs ros-foxy-cv-bridge \
    ros-foxy-rviz2 ros-foxy-ackermann-msgs ros-foxy-ament-lint-auto \
    ros-foxy-ament-lint-common && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir 'pip==24.0' && \
    python3 -m pip install --no-cache-dir 'transforms3d==0.4.1' 'carla==0.9.13'
RUN source /opt/ros/foxy/setup.bash && python3 -c \
    'import carla, rclpy, cv_bridge, transforms3d, pygame, networkx; import pkg_resources; print("CARLA", pkg_resources.get_distribution("carla").version)'

FROM base AS libraries
RUN git clone --recursive --depth 1 --branch v0.6.3 https://github.com/osqp/osqp.git /tmp/osqp && \
    test "$(git -C /tmp/osqp rev-parse HEAD)" = 0dd00a578cf1c2691c5c379965d504c75bf6cfad && \
    cmake -S /tmp/osqp -B /tmp/osqp/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/osqp -DENABLE_MKL_PARDISO=OFF && \
    cmake --build /tmp/osqp/build -j2 && cmake --install /tmp/osqp/build
RUN git clone --recursive --depth 1 --branch v0.8.0 https://github.com/robotology/osqp-eigen.git /tmp/osqp-eigen && \
    test "$(git -C /tmp/osqp-eigen rev-parse HEAD)" = fe34491d419227d0db74741a549d7e6e9e38971a && \
    cmake -S /tmp/osqp-eigen -B /tmp/osqp-eigen/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/osqp_eigen -DCMAKE_PREFIX_PATH=/opt/osqp -DBUILD_TESTING=OFF && \
    cmake --build /tmp/osqp-eigen/build -j2 && cmake --install /tmp/osqp-eigen/build
COPY matplot-source.tar /tmp/matplot-source.tar
RUN mkdir /tmp/matplot && tar -xf /tmp/matplot-source.tar -C /tmp/matplot && \
    cmake -S /tmp/matplot -B /tmp/matplot/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/matplot \
      -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_TESTS=OFF -DMATPLOTPP_BUILD_INSTALLER=ON -DMATPLOTPP_BUILD_PACKAGE=OFF && \
    cmake --build /tmp/matplot/build -j2 && cmake --install /tmp/matplot/build
ENV CMAKE_PREFIX_PATH=/opt/osqp:/opt/osqp_eigen:/opt/matplot
ENV LD_LIBRARY_PATH=/opt/osqp/lib:/opt/osqp_eigen/lib:/opt/matplot/lib
COPY smoke /tmp/dependency-smoke
RUN cmake -S /tmp/dependency-smoke -B /tmp/dependency-smoke/build && \
    cmake --build /tmp/dependency-smoke/build -j2 && /tmp/dependency-smoke/build/dependency_smoke

FROM libraries AS dependencies
COPY carla-agents.tar /tmp/carla-agents.tar
# CARLA 0.9.13 annotated tag resolves to commit a1b37f7f1cf34b0f6f77973c469926ea368d1507.
RUN mkdir /opt/carla-source && tar -xf /tmp/carla-agents.tar -C /opt/carla-source
ENV PYTHONPATH=/opt/carla-source/PythonAPI/carla
RUN python3 -c 'from agents.navigation.global_route_planner import GlobalRoutePlanner; print("CARLA agents import OK")'
