# pythonVision
Team 4611's python-based vision, designed to run on a raspberry pi

## Installation
1) Install Raspbian as the OS
2) Run these commands for installation
	
	`sudo apt-get update`
	
    `sudo apt-get install realvnc-vnc-server realvnc-vnc-viewer`
	
	`sudo apt-get install python-opencv`
    
    `pip install pynetworktables`
    
    `sudo pip install "picamera[array]"`
    
    `sudo rpi-update`
3) Edit the startup file to run vncserver

    `sudo nano /etc/rc.local`
    
    Add this line above exit 0
    
    `su pi -c 'vncserver &'`
