FROM ghcr.io/sdustio/ros2:base

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 libgazebo11-dev ros-foxy-gazebo-ros-pkgs curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ARG GH_REF

RUN set -eux; \
    curl -LsSf -o spotng.deb https://raw.githubusercontent.com/sdustio/openspot_node/main/dockerfiles/spotng.deb; \
    dpkg -i spotng.deb; rm spotng.deb

RUN set -ex; \
    mkdir -p /openspot/src/openspot_sim; \
    curl -LsSf -o src.tar.gz https://github.com/sdustio/openspot_sim/archive/${GH_REF}.tar.gz; \
    tar --gzip --extract --directory /openspot/src/openspot_sim --strip-components=1 --file src.tar.gz; \
    rm src.tar.gz; \
    cd /openspot; \
    . "/opt/ros/$ROS_DISTRO/setup.sh"; \
    . "/usr/share/gazebo/setup.sh"; \
    colcon build; \
    rm -rf build src

# setup environment
EXPOSE 11345

COPY deploy-entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
