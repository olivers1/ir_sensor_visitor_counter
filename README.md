# ir_sensor_visitor_counter
--description--  
dual ir sensors to detect if a visitor enter or leaves the room. when a visitor entering/exiting the room is detected the action along with a timestamp is stored

--raspberry_pi_setup--  
install raspios on an SD-card  
insert SD-card to raspberry pi

-configuration  
in terminal run:  
sudo raspi-config  
select 'Interface Options'  
SSH -> enable  
SPI -> enable

-change keyboard layout  
in GUI click:  
Preferences/'Keyboard and Mouse'/Keyboar/'Keyboard Layout'

-set static IP-address  
in terminal run:  
sudo nano /etc/dhcpcd.conf

change fields:  
interface NETWORK  
static ip_address=STATIC_IP/24  
static routers=ROUTER_IP  
static domain_name_servers=DNS_IP

example:
interface wlan0  
static ip_address=192.168.1.120/24  
static routers=192.168.1.254  
static domain_name_servers=192.168.1.254

#for changes to take effect  
sudo reboot

-ssh authentification  
generate ssh key according to below format  
in terminal window run  
(mkdir ~/.ssh folder)  
(cd ~/.ssh)  
ssh-keygen -t rsa -b 4096

#don't give any the key any name nor password  

cat id_rsa.pub  
add ssh key .pub to github

add ssh key to client laptop for smooth raspberry pi access without password  
scp olivers@192.168.0.10:~/.ssh/id_rsa.pub ~/Downloads

#if problem with authentification locate 'known_hosts' file and delete same ip adress key in ~/.ssh/known_hosts on local laptop (client)  
-required python packages  
#install all packages from file

pip install -r requirements.txt

pip install Adafruit-GPIO  
pip install Adafruit-MCP3008  
pip install spidev  