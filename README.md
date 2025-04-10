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

docker save -o /mnt/ssd/erc-mars-rover_perception.tar erc-mars-rover_perception:latest


docker load -i erc-mars-rover_perception.tar

#allow gpu::::::
sudo docker run --runtime=nvidia -it     --name zed_container     --privileged     --network=host     -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)     -v /run/nvargus:/run/nvargus     -v /tmp/argus_socket:/tmp/argus_socket     --device=/dev/video0     --entrypoint /bin/bash erc-mars-rover_perception:latest

sudo docker run --runtime=nvidia -it     --name perception_container     --privileged     --network=host     -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)     -v /run/nvargus:/run/nvargus     -v /tmp/argus_socket:/tmp/argus_socket     --device=/dev/video0     --entrypoint /bin/bash erc-mars-rover_perception:latest

sudo docker run --runtime=nvidia -it     --name nav_container     --privileged     --network=host     -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)     -v /run/nvargus:/run/nvargus     -v /tmp/argus_socket:/tmp/argus_socket     --device=/dev/video0     --entrypoint /bin/bash erc-mars-rover_nav:latest

# after closed, open again via
docker start zed_container
docker attach zed_container

docker start perception_container
docker attach perception_container

#
sudo docker build -t erc-mars-rover_base:latest -f Dockerfile_base .

docker build -t erc-mars-rover_perception:latest -f Dockerfile_perception .

pip3 uninstall numpy -y

cd src/probe_detection/scripts/



scp -r datasets/raw_images/OnRover daroe@192.168.0.10:~/ERC-Mars-Rover/

nmap -sP 192.168.0.1/23

scp -r /mnt/ssd/erc-mars-rover_perception.tar robotlab@192.168.0.5:~/ERC-Mars-Rover/


docker load -i erc-mars-rover_perception.tar