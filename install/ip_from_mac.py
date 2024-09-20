from scapy.all import ARP, Ether, srp
import sys

def get_ip_from_mac(mac_address):
    # Create an ARP request packet
    arp = ARP(pdst="192.168.1.0/24")
    ether = Ether(dst="ff:ff:ff:ff:ff:ff")
    packet = ether/arp

    result = srp(packet, timeout=3, verbose=0)[0]

    for sent, received in result:
        if received.hwsrc == mac_address:
            return received.psrc
    
    return None

# Example usage
mac_address = "b8:27:eb:01:e7:69"  # Replace with your target MAC address
ip = get_ip_from_mac(mac_address)
if ip:
    print(f"The IP address for MAC {mac_address} is {ip}")
else:
    print(f"No IP address found for MAC {mac_address}")