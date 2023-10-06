FROM osrf/mbzirc:mbzirc_sim_latest

USER root

RUN usermod -u 1000 developer
RUN groupmod -g 1000 developer
RUN chown -R 1000:1000 /home/developer

USER developer

RUN sudo apt-get update && sudo apt install -y \
    ros-galactic-laser-filters \
    ros-galactic-laser-geometry \
    ros-galactic-sensor-msgs-py

RUN pip install \
    scipy
