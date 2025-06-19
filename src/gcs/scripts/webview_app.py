#!/usr/bin/env python3
# ros2 run rosbridge_server rosbridge_websocket
# /bin/python3 /home/benjamin/cc2/webview_app.py
# python3 -m http.server 3001
import webview
import socket
import nmap
import requests # Add this import
import os


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))  # Google's DNS server
local_ip = s.getsockname()[0]  # Get the local IP address
s.close()
print(f"Local IP: {local_ip}") # Add this line


# Initialize nmap scanner
nm = nmap.PortScanner()
# Construct the network range from the local IP
network_prefix = '.'.join(local_ip.split('.')[:3]) + '.0/24'
print(f"Scanning network: {network_prefix}")
# Scan the local network
nm.scan(hosts=network_prefix, arguments='-sP')  # -sn for ping scan (no port scan)

# Iterate through discovered hosts
discovered_hosts = []
print("Hosts found by nmap scan (state 'up'):") # Add this line
for host in nm.all_hosts():
    if nm[host].state() == 'up':
        print(f"  Device IP: {host} is up. Checking HTTP on port 8080...") # Modified line
        hostname = nm[host].hostname() or "Unknown"
        # print(f"Hostname: {hostname}") # You can uncomment this if needed
        try:
            # Attempt to fetch index.html
            url = f"http://{host}:8080"
            response = requests.get(url, timeout=5) # Set a timeout
            print(f"    HTTP GET to {url} status: {response.status_code}") # Add this line
            if response.status_code == 200:
                print(f"    Successfully fetched index.html from {host}")
                discovered_hosts.append(host)
            else: # Add this else block
                print(f"    Failed to fetch index.html from {host}, status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"    Could not connect to {host}:8080 or fetch index.html: {e}") # Modified line to print error
            # pass # Silently ignore hosts that don't respond or don't have index.html

script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, 'index.html')
js_init_path = os.path.join(script_dir,"js/leo_init.js")

with open(js_init_path, 'r', encoding='utf-8') as file:
    js_content = file.read()

print(f"JS content loaded from {js_init_path}")
js_content = js_content.replace('ws://localhost:9090', f'ws://{local_ip}:9090')
try:
    js_content = js_content.replace('192.168.1.13', f'{discovered_hosts[0]}')  # Replace with the first discovered host
except IndexError as e:
    print("No hosts found, using default IP")

# Save the modified HTML content to index2.html
js_path = os.path.join(script_dir, 'js/leo.js')
with open(js_path, 'w', encoding='utf-8') as file:
    file.write(js_content)
print(f"Modified JS content saved to {js_path}")

if __name__ == '__main__':

    url = 'file://' + html_path
    primary_screen = webview.screens[0]
    webview.create_window('LeoRover UI', url=html_path, 
                          width=primary_screen.width // 2, 
                          height=primary_screen.height, 
                          x=0, y=0)

    webview.start()