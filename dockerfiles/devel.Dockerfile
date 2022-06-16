FROM ghcr.io/sdustio/ros2:dev

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 libgazebo11-dev ros-foxy-gazebo-ros-pkgs curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    curl -LsSf -o spotng.deb https://raw.githubusercontent.com/sdustio/openspot_node/main/dockerfiles/spotng.deb; \
    dpkg -i spotng.deb; rm spotng.deb

# setup environment
EXPOSE 11345

COPY devel-entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
