# Guide to setup UCLOUD remote machine for yolo training in python
For this project a uCloud remote server with an NVIDIA a40 GPU was used to train everything related to the YOLO11 model used for image segmentation. Following is a guide on how to setup the remote server and how to execute python scripts on it.

## 0: Start ucloud
An Ubuntu virtual machine instance must be launched from the UCloud website. General guides can be found at [https://hpc.aau.dk/](https://hpc.aau.dk/). Once access and resources have been granted the following steps should be enough to get an instance running:
- Find Ubuntu(virtual machine) 22.04 with CUDA form the application store
- Enter name and machine type
- Public ssh key needs to be added before launch
- Press submit

## 1: SSH into the machine
The machine must be accessed though an SSH connection
- Replace `<IP-address>` with the one given after launching your machine and run the following command:
```bash
ssh ucloud@<IP-address>
```
- write 'yes' when prompted and press enter
- You will see a confirmation once you're connected to the machine
> **Note**: If the machine does not connect immediately, try again after a few seconds.

## 2: Copy training folder to the remote machine
Data and training scripts can be palced in a folder and transferred onto the virtual machine:
- Open local terminal and write the following command:
```bash
scp -r <path to github>/GitHub/ERC-Mars-Rover/src/probe_detection/Training ucloud@<IP-address>:/home/ucloud
```
- After completion return to the remote terminal and enter the directory
```bash
cd Training
```

## 3: Data for training
- Data for training should be uploaded in the marsYardData folder
- If data is uploaded as a .zip file a bash script is provided for free:
```bash
bash unzip.sh
```
> **Note**: bash script only looks for zip files in training/marsYardData.

## 4: Install and start the virtual environment
A requirements.txt file should be provided. Running following script will install all dependancies and setup a virtual environment.
```bash
source setup_venv.sh
```

## 5: Install clearml if logging is required
clearML can be used to log progress online.
- activate file with credentials:
```bash
source setup_clearml.sh
```
- If the file does not exist: 
	- rename `setup_clearml_template.sh` to `setup_clearml.sh` and fill in the blanks with you clearml API information

## 6: Run python script
Finally the training python script can be executed.
```bash
python3 TrainYolo_[some version].py
```
- If output needs to be directed away from the terminal session:
```bash
nohup python3 TrainYolo_[some version].py > output.log 2>&1 &
```
- Running nohup will allow you to exit the ssh terminal without closing the python script
	
