import http.server
import socketserver
import threading
import requests
from bs4 import BeautifulSoup
import time
import os

PORT = 3000

# Ensure the initial image exists
initial_image_path = 'example.jpg'
uploaded_image_path = 'uploaded_image.jpg'

if not os.path.exists(initial_image_path):
    with open(initial_image_path, 'wb') as f:
        f.write(requests.get('https://via.placeholder.com/150').content)  # Placeholder image

# Initial HTML content with an upload form
html_template = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Test Page</title>
</head>
<body>
    <h1>Test Page</h1>
    <form method="POST" enctype="multipart/form-data" action="/">
        <input type="file" name="file" accept="image/*">
        <input type="submit" value="Upload Image">
    </form>
    <img id="uploaded_image" src="{image_src}" alt="Example Image">
</body>
</html>
"""

# Function to write HTML file with dynamic image source
def write_html_file(image_path):
    with open('index.html', 'w') as file:
        file.write(html_template.format(image_src=image_path))

# Save initial HTML file with the example image
write_html_file(initial_image_path)

# Create a request handler
class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        # Extract the file data
        boundary = self.headers['Content-Type'].split('=')[1].encode()
        parts = post_data.split(boundary)
        file_data = parts[1].split(b'\r\n\r\n')[1].split(b'\r\n--')[0]

        # Save the uploaded file
        with open(uploaded_image_path, 'wb') as f:
            f.write(file_data)

        # Update HTML file to display the uploaded image
        write_html_file(uploaded_image_path)

        # Respond with the HTML page showing the new image
        self.send_response(303)
        self.send_header('Location', '/')
        self.end_headers()

# Start the server in a separate thread
def start_server():
    with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
        print(f"Serving at port {PORT}")
        httpd.serve_forever()

server_thread = threading.Thread(target=start_server)
server_thread.daemon = True
server_thread.start()

# Allow the server some time to start
time.sleep(1)

def fetch_image(url, output_path='input.jpg'):
    # Send a GET request to the URL
    response = requests.get(url)
    response.raise_for_status()  # Check for request errors

    # Parse the HTML content
    soup = BeautifulSoup(response.content, 'html.parser')

    # Find the image tag with the id 'uploaded_image'
    img_tag = soup.find('img', {'id': 'uploaded_image'})

    if img_tag and 'src' in img_tag.attrs:
        # Get the image URL
        img_url = img_tag['src']

        # If the image URL is relative, make it absolute by joining with the base URL
        if not img_url.startswith(('http://', 'https://')):
            img_url = requests.compat.urljoin(url, img_url)

        try:
            # Send a GET request to the image URL
            img_response = requests.get(img_url)
            img_response.raise_for_status()  # Check for request errors

            # Save the image to the specified path
            with open(output_path, 'wb') as img_file:
                img_file.write(img_response.content)
            print(f"Image saved as {output_path}")
        except requests.exceptions.RequestException as e:
            print(f"Failed to fetch image: {e}")
    else:
        print("No image found on the webpage")

# Initial fetch of the image
webpage_url = f'http://localhost:{PORT}'

# Function to continuously check for the uploaded image
def check_for_uploaded_image():
    while True:
        try:
            fetch_image(webpage_url)
        except requests.exceptions.RequestException as e:
            print(f"Error fetching image: {e}")
        time.sleep(5)  # Check every 5 seconds

# Start the continuous check in a separate thread
image_check_thread = threading.Thread(target=check_for_uploaded_image)
image_check_thread.daemon = True
image_check_thread.start()

# Keep the main thread running to keep the server alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down server...")
