# Guide to setup UCLOUD remote machine for yolo training in python

## 0: Start ucloud
- Find Ubuntu(virtual machine) 22.04 with CUDA form the application store
- Enter name and machine type
- Public ssh key needs to be added before launch
- Press submit

## 1: SSH into the machine
- Replace `<IP-address>` with the one given after launching your machine and run the following command:
```bash
ssh ucloud@<IP-address>
```
- write 'yes' when prompted and press enter
- You will see a confirmation once you're connected to the machine
> **Note**: If the machine does not connect immediately, try again after a few moments.

## 2: Copy training folder to the remote machine
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

## 4: Install and start the virtual environment
```bash
source setup_venv.sh
```

## 5: Install clearml if logging is required
- activate file with credentials:
```bash
source setup_clearml.sh
```
- If the file does not exist: 
	- rename `setup_clearml_template.sh` to `setup_clearml.sh` and fill in the blanks with you clearml API information

## 6: Run python script
```bash
python3 TrainYolo_[some version].py
```
- If output needs to be directed away from the terminal session:
```bash
nohup python3 TrainYolo_[some version].py > output.log 2>&1 &
```
- Running nohup will allow you to exit the ssh terminal without closing the python script
	
