# Network setup
Network setup information for the PR2.

## dnsmasq
For some reason, c1 stopped serving DHCP one day.
This meant that c2 was unable to boot, since it does a netboot, and that plugging into the service port would not work.

An easy way to tell that c2 was not booting was by plugging in a monitor and watching it boot.
It showed this in a loop:
```
Intel Boot Agent
PXE-E51: No DHCP or proxyDHCP offers were received.
PXE-M0F: Exiting Intel Boot Agent.
```

**dnsmasq** is the service that provides DHCP.
We saw that it was not running:
```
/etc/init.d/dnsmasq status
(not running)
```

We resolved this problem by changing `/etc/dnsmasq.conf`.

First we got an error saying that dnsmasq couldn't be started with both `bind-interfaces` and `bind-dynamic`, so we made this change:
```diff
#bind-interfaces
- bind-dynamic
+ #bind-dynamic
```

Then we got an error saying that dnsmasq couldn't create a listening socket for 192.168.2.1:
```diff
- listen-address=192.168.2.1
+ #listen-address=192.168.2.1
```

Then we got an error saying "unknown interface wlan0"
```diff
- interface=wlan0
+ #interface=wlan0
- dhcp-range=wlan0,192.168.2.2,192.168.2.254,255.255.255.0,12h
+ #dhcp-range=wlan0,192.168.2.2,192.168.2.254,255.255.255.0,12h
```

Then, restarting dnsmasq worked:
```
sudo /etc/init.d/dnsmasq restart
```

Also, we changed these options to get access to the public internet while plugged into the service port:
```diff
- dhcp-option-force=option:router,192.168.2.1
+ dhcp-option-force=option:router,10.68.0.5
- dhcp-option-force=option:dns-server,192.168.2.1
+ dhcp-option-force=option:dns-server,<our university DNS>
```

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
