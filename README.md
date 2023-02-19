# How to Set Up the Environment
1. Install the docker environment
docker build -f Dockerfile --tag=visual_frontend_docker .

2. To access the GUI, run the following command
xhost + 

3. Run the docker image 
docker run -d -it --ipc=host --net=host --privileged -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw  --name dev_vslam_frontend -v /home/hiel/Job/VisualFrontend:/usr/visual_frontend/ -v /home/hiel/Data/rgbd_dataset_freiburg3_long_office_household:/usr/src/data visual_frontend_docker:latest



4. Build the code
bazel build --copt=-fdiagnostics-color=always //src/...

5. Run the code with a sample dataset
bazel run //src:v_frontend /usr/src/data/
