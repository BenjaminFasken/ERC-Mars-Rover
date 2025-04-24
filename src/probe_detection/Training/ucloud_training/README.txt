# Guide til setup af ucloud mht. yolo træning

#1: SSH ind i maskinen
	ssh ucloud@[IP-adresse]
	- public key skal være tilføjet ved lainch
	- vigtigt at private key ligger i ~/.ssh
	- yes you are sure you want to connect

#2: Kopier uCloud mappe ind i /home/ucloud
	- Åben lokal terminal og skriv følgende
	scp -r ~/Documents/ucloud_training ucloud@[IP-adresse]:/home/ucloud
	- vent venligst

#3: Installér og start python virtuel environment
	- tilbage i ssh terminal
	cd ucLoud_training
	source setup_venv.sh

#4: Installér clearml hvis der skal logges træning
	source setup_clearml.sh
	- denne fil indeholder clearml credentials

#5: Kør whatever
	- hvis ikke data er uploadet allerede skal det lægges i
          marsYardData mappen
	- Hvis du uploader en zip fil:
	bash unzip.sh
	
	- Og så kan der køres træningsscript
	python3 TrainYolo_[some version].py
	
