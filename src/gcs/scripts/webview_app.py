#!/usr/bin/env python3
# ros2 run rosbridge_server rosbridge_websocket
# /bin/python3 /home/benjamin/cc2/webview_app.py
# python3 -m http.server 3001
import webview
import socket
import netifaces
import nmap
import requests # Add this import
import os


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))  # Google's DNS server
local_ip = s.getsockname()[0]  # Get the local IP address
s.close()


# Initialize nmap scanner
nm = nmap.PortScanner()
# Construct the network range from the local IP
network_prefix = '.'.join(local_ip.split('.')[:3]) + '.0/24'
print(f"Scanning network: {network_prefix}")
# Scan the local network
nm.scan(hosts=network_prefix, arguments='-sn')  # -sn for ping scan (no port scan)

# Iterate through discovered hosts
discovered_hosts = []
for host in nm.all_hosts():
    if nm[host].state() == 'up':
        print(f"Device IP: {host}")
        hostname = nm[host].hostname() or "Unknown"
        print(f"Hostname: {hostname}")
        try:
            # Attempt to fetch index.html
            url = f"http://{host}:8080"
            response = requests.get(url, timeout=5) # Set a timeout
            if response.status_code == 200:
                print(f"Successfully fetched index.html from {host}")
                discovered_hosts.append(host)
            # You can add more specific error handling if needed
            # else:
            #     print(f"Failed to fetch index.html from {host}, status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            # print(f"Could not connect to {host} or fetch index.html: {e}")
            pass # Silently ignore hosts that don't respond or don't have index.html

script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, 'index.html')


with open(html_path, 'r', encoding='utf-8') as file:
    html_content = file.read()

print(f"HTML content loaded from {html_path}")
html_content = html_content.replace('ws://localhost:9090', f'ws://{local_ip}:9090')
html_content = html_content.replace('192.168.1.18', f'{discovered_hosts[0]}')  # Replace with the first discovered host
# Save the modified HTML content to index2.html
index2_path = os.path.join(script_dir, 'index2.html')
with open(index2_path, 'w', encoding='utf-8') as file:
    file.write(html_content)
print(f"Modified HTML content saved to {index2_path}")

if __name__ == '__main__':

    url = 'file://' + html_path
    webview.create_window('LeoRover UI', url=index2_path, width=1200, height=800)

    webview.start(http_port=3042)
