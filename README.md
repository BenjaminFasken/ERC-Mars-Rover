# MARS
Mars Rover

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

#Leorover
docker run -it --entrypoint /bin/bash erc-mars-rover:latest

scp erc-mars-rover_perception2.tar leorover@192.168.0.61:/home/leorover/


#Home: 192.168.0.61




#Docker


sudo docker buildx build --platform linux/arm64 -t erc-mars-rover_base:latest -f Dockerfile-base --load .


sudo docker buildx build --platform linux/arm64 -t erc-mars-rover_perception:latest -f Dockerfile-perception --load --no-cache=false .

docker save -o erc-mars-rover_perception.tar erc-mars-rover_perception:latest


docker load -i erc-mars-rover_perception.tar

#allow gpu::::::
sudo docker run --runtime=nvidia -it     --name zed_container     --privileged     --network=host     -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)     -v /run/nvargus:/run/nvargus     -v /tmp/argus_socket:/tmp/argus_socket     --device=/dev/video0     --entrypoint /bin/bash erc-mars-rover_perception:latest

# after closed, open again via
docker start zed_container
docker attach zed_container




# Build upon local image.
Build image
Save image

## if needed create swap?
sudo fallocate -l 20G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

## "create" load the prebuild image into buildx, which is different from build???
sudo docker buildx imagetools create --file erc-mars-rover_base.tar --tag latest

## if swap was made, remove it
sudo swapoff /swapfile
sudo rm /swapfile

## build upon base image:
sudo docker buildx build --platform linux/arm64 -t erc-mars-rover_perception3:latest -f Dockerfile_perception --load .






#
sudo docker build -t erc-mars-rover_base:latest -f Dockerfile-base .