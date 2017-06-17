#!/bin/bash
sudo   ifconfig    wlan0   10.5.5.1    255.255.255.0
sudo   dhcpd      wlan0     -pf        /var/run/dhcp-server/dhcpd.pid
sudo   bash         -c     "echo       1>/proc/sys/net/ipv4/ip_forward"
sudo   iptables    -t      nat    -A    POSTROUTING     -o   eth0     -j   MASQUERADE
sudo   hostapd      ~/1.conf  &
