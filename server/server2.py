from flask import Flask, request, send_from_directory, jsonify
import os

app = Flask(__name__)

# configure the directory where the uploaded images will be stored
app.config['UPLOAD_FOLDER'] = 'images'

# define the POST method that will accept an image and save it to the server
@app.route('/upload', methods=['POST'])
def upload():
    # check if the request contains a file
    if 'file' not in request.files:
        return 'No file found in request', 400

    file = request.files['file']

    # check if the file has a name
    if file.filename == '':
        return 'No file selected', 400

    # save the file to the server
    file.save(os.path.join(app.config['UPLOAD_FOLDER'], file.filename))

    return 'File uploaded successfully', 200

# define the GET method that will download an image from the server to the device
@app.route('/download/<filename>', methods=['GET'])
def download(filename):
    # check if the file exists on the server
    if not os.path.exists(os.path.join(app.config['UPLOAD_FOLDER'], filename)):
        return 'File not found', 404

    # send the file to the client
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename, as_attachment=True)

# define the GET method that will serve the latest image URL
@app.route('/latest_image_url', methods=['GET'])
def latest_image_url():
    files = [f for f in os.listdir(app.config['UPLOAD_FOLDER']) if os.path.isfile(os.path.join(app.config['UPLOAD_FOLDER'], f))]
    if files:
        try:
            latest_file = max(files, key=lambda f: os.path.getctime(os.path.join(app.config['UPLOAD_FOLDER'], f)))
            image_url = f'http://{request.host}/download/{latest_file}'
            return jsonify({'image_url': image_url})
        except FileNotFoundError as e:
            return f'File not found: {e.filename}', 404
    else:
        return 'No images found', 404

if __name__ == '__main__':
    app.run(host='46.101.186.178', port=5001, debug = True)