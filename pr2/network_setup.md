# Network setup
Network setup information for the PR2.

## iptables
Our iptables configuration is set up to block UDP traffic on port 111.

Block UDP port 111 with:
- Install `iptables-persistent` with `sudo apt-get install iptables-persistent`
- Add the rule, possibly to the first p`sudo iptables -I INPUT 1 -p udp --dport 111 -j DROP`
- Save the rules with `sudo invoke-rc.d iptables-persistent save`
- Check that UDP port 111 is closed with `nmap <hostname>`

Resources:
- [Reflection DDoS Attacks Abusing RPC Portmapper](https://threatpost.com/reflection-ddos-attacks-abusing-rpc-portmapper/114318/)
- [DigitalOcean introduction to iptables](https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-using-iptables-on-ubuntu-14-04)
