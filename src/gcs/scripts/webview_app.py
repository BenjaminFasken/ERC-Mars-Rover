# ros2 run rosbridge_server rosbridge_websocket
# /bin/python3 /home/benjamin/cc2/webview_app.py
# python3 -m http.server 3001
import webview
import os

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, '/src/gcs/web/index.html')

if __name__ == '__main__':
    # option A) load by URL so all relative CSS/JS/img assets just work:
    url = 'file://' + html_path
    webview.create_window('LeoRover UI', url=url, width=1200, height=800)

    # --- OR, option B) load HTML string + base_url so relative paths resolve: ---
    # with open(html_path, 'r') as f:
    #     html = f.read()
    # webview.create_window('LeoRover UI', html=html, base_url=script_dir, width=1200, height=800)

    webview.start()
