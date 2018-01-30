#!/usr/bin/env bash


update_packages() {
    sudo apt-get update
    sudo apt-get upgrade
}

setup_virtual() {
    sudo pip install virtualenv virtualenvwrapper
    echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.profile
    echo "export WORKON_HOME=/home/pi/.virtualenvs" >> ~/.profile
    echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.profile
    source ~/.profile
    mkvirtualenv cv -p python2
    source ~/.profile
    workon cv
}

install_packages() {
    sudo apt-get install python-opencv
    pip install numpy
    pip install "picamera[array]"
    pip install pynetworktables
}

opencv_link() {
    cd ~/.virtualenvs/cv/lib/python2.7/site-packages
    ln -s /usr/lib/python2.7/dist-packages/cv2.arm-linux-gnueabihf.so cv2.so
}

remove_unused_directories() {
    sudo rm -r /home/pi/python_games/
    sudo rm -r /home/pi/Documents/
    sudo rm -r /home/pi/Music/
    sudo rm -r /home/pi/Pictures/
    sudo rm -r /home/pi/Templates/
    sudo rm -r /home/pi/Downloads/
    sudo rm -r /home/pi/Public/
    sudo rm -r /home/pi/Videos/
    sudo rm -r /home/pi/oldconffiles/
}

static_ip() {
    sudo cp /home/pi/interfaces.static /etc/network/interfaces
}

permissions() {
    chmod +x /home/pi/vision*
}

main() {
    update_packages
    setup_virtual
    install_packages
    opencv_link
    remove_unused_directories
    static_ip
    permissions
}

main
