#!/bin/bash
sleep 15
sudo nmcli nm wifi off
sleep 2
sudo rfkill unblock wlan
sleep 2
sudo killall hostapd
sleep 2
sudo   ifconfig    wlan0   10.5.5.1    255.255.255.0 
sleep 2
sudo   dhcpd      wlan0     -pf        /var/run/dhcp-server/dhcpd.pid
sleep 2
sudo   iptables    -t      nat    -A    POSTROUTING     -o   eth0     -j   MASQUERADE
sleep 2
sudo hostapd /home/ubuntu/M100_RCphone/src/Onboard-SDK-ROS/wifi.conf &
