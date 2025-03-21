# MARS
Mars Rover



#Leorover
docker run -it --entrypoint /bin/bash erc-mars-rover:latest

scp erc-mars-rover.tar leorover@192.168.0.68


#Home: 192.168.0.61




#Docker


sudo docker buildx build --platform linux/arm64 -t erc-mars-rover_base:latest -f Dockerfile-base --load .


sudo docker buildx build --platform linux/arm64 -t erc-mars-rover_perception:latest -f Dockerfile-perception --load --no-cache=false .


docker load -i erc-mars-rover_perception.tar

#allow gpu::::::
sudo docker run --runtime=nvidia -it     --name zed_container     --privileged     --network=host     -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)     -v /run/nvargus:/run/nvargus     -v /tmp/argus_socket:/tmp/argus_socket     --device=/dev/video0     --entrypoint /bin/bash erc-mars-rover_perception:latest

# after closed, open again via
docker start zed_container
docker attach zed_container
