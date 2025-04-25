# Guide til setup af ucloud mht. yolo træning

#0: Start ucloud
	- Start Ubuntu(virtual machine) 22.04 med CUDA
	- public key skal være tilføjet ved launch
	- vigtigt at private key ligger i ~/.ssh

#1: SSH ind i maskinen
	ssh ucloud@[IP-adresse]
	- yes you are sure you want to connect

#2: Kopier uCloud mappe ind i /home/ucloud
	- Åben lokal terminal og skriv følgende
	scp -r ~/home/emil/Documents/GitHub/ERC-Mars-Rover/src/probe_detection/Training
 ucloud@[IP-adresse]:/home/ucloud
	- vent venligst
	- gå tilbage til ssh terminal
	cd Training
	- now we cook

#3: Data til træning
	- Data til træning skal lægges i marsYardData
	- Hvis data er uploadet som .zip er der bash script provided for free:
	bash unzip.sh

#3: Installér og start python virtuel environment
	source setup_venv.sh

#4: Installér clearml hvis der skal logges træning
	source setup_clearml.sh
	- denne fil indeholder clearml credentials
	- Hvis ikke filen eksisterer: 
		- omdøb setup_clearml_template.sh og fyld ud med API info

#5: Kør whatever
	python3 TrainYolo_[some version].py
	- Hvis output skal sendes væk fra terminal:
	nohup python3 TrainYolo_[some version].py > output.log 2>&1 &
	
