-- Odrive config (GUI)
https://gui.odriverobotics.com/

-- Prerequisites
sudo apt install python3 python3-pip
pip install --upgrade odrive
pip3 install matplotlib
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
source ~/.bashrc

-- 

