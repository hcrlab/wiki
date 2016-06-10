# Network setup
Network setup information for the PR2.

## iptables
Our iptables configuration is set up to block UDP traffic on port 111.

Block UDP port 111 with:
`sudo iptables -I INPUT 1 -p udp --dport 111 -j DROP`

Resources:
- [Reflection DDoS Attacks Abusing RPC Portmapper](https://threatpost.com/reflection-ddos-attacks-abusing-rpc-portmapper/114318/)
- [DigitalOcean introduction to iptables](https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-using-iptables-on-ubuntu-14-04)
