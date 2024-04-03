echo -e 'SUBSYSTEM=="usb", ATTRS{idProduct}=="f100", ATTRS{idVendor}=="fccf", GROUP="users", MODE="0666"\n' >> ./99-fccf-f100.rules
# change idProduct and idVendor according to actual usb port
sudo mv ./99-fccf-f100.rules /etc/udev/rules.d/
udevadm trigger
sudo udevadm control --reload && sudo udevadm trigger