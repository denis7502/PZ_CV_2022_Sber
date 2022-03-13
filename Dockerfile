FROM dustynv/ros:galactic-pytorch-l4t-r32.6.1
RUN apt update && apt install -y python3 python3-pip
RUN python -m pip install --upgrade pip
RUN sudo apt-get install -y v4l-utils
RUN pip install seaborn matplotlib pyyaml requests scipy tqdm facenet-pytorch
