# pythonVision
Team 4611's python-based vision, designed to run on a raspberry pi

## Installation
1) Install Raspbian as the OS
2) Edit the crontab
    ```
    sudo crontab -e
    ```
    
    Add these lines to the bottom of the crontab file:
    
    ```
    @reboot su pi -c vncserver
    @reboot /home/pi/vision_autostart.sh
    ```
    
3) Copy vision files to the pi.  Use a program like WinSCP if on Windows.
    
4) Change permission of build.sh script and execute
    ```
    chmod +x /home/pi/build.sh
    /home/pi/build.sh
    ```

## Usage
### Automated
On reboot, *__vision_autostart.sh__* will automatically run.  This activates the virtual environment and runs *__vision.py__*

### Manual
Run this script:
```
./vision_manual.sh
```
This will activate the virtual environment and run *__visionTape.py__*
It's intended that vision.py will be modified with options so that there will only be a single py executable.

