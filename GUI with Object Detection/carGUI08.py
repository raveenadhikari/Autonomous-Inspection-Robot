import tkinter as tk
import serial
import time
import cv2
import urllib.request
import numpy as np
from PIL import Image, ImageTk
import threading

# === Global Settings ===
ESP32_CAM_URL = "http://10.21.89.132/capture"  # Update with your ESP32-CAM URL

# Initialize serial communication for LoRa commands
ser = None
lora_connected = False
try:
    ser = serial.Serial('COM7', 115200, timeout=1)  # Replace 'COM7' with your port
    time.sleep(2)  # Allow time for connection initialization
    lora_connected = True
    print("Serial connection established!")
except Exception as e:
    print(f"Failed to connect to the ESP32: {e}")

# === YOLO Setup (if using object detection) ===
whT = 320
confThreshold = 0.5
nmsThreshold = 0.3
classesfile = 'coco.names'
classNames = []
with open(classesfile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

modelConfig = 'yolov3.cfg'
modelWeights = 'yolov3.weights'
net = cv2.dnn.readNetFromDarknet(modelConfig, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def findObject(outputs, im):
    hT, wT, _ = im.shape
    bbox = []
    classIds = []
    confs = []
    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w, h = int(det[2] * wT), int(det[3] * hT)
                x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                bbox.append([x, y, w, h])
                classIds.append(classId)
                confs.append(float(confidence))
    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)
    if len(indices) > 0:
        for i in indices:
            if isinstance(i, (list, np.ndarray)):
                i = i[0]
            box = bbox[i]
            x, y, w, h = box
            cv2.rectangle(im, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(im, f'{classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    return im

def send_command(command):
    """Send a command via the serial connection."""
    if ser and ser.is_open:
        try:
            ser.write(f"{command}\n".encode('utf-8'))
            print(f"Command sent: {command}")
        except Exception as e:
            print(f"Failed to send command: {e}")
    else:
        print("Serial connection is not open.")

def update_image(imgtk):
    video_label.configure(image=imgtk, text="")
    video_label.image = imgtk

def fetch_frames():
    global ESP32_CAM_URL

    while True:
        try:
            img_resp = urllib.request.urlopen(ESP32_CAM_URL, timeout=2)  # Increased timeout
            img_array = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            frame = cv2.imdecode(img_array, -1)  # Decode the image

            if frame is None:
                print("Warning: Received empty frame")
                continue

            # --- Object Detection Processing ---
            blob = cv2.dnn.blobFromImage(frame, 1/255, (whT, whT), [0, 0, 0], 1, crop=False)
            net.setInput(blob)
            layerNames = net.getLayerNames()
            outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
            outputs = net.forward(outputNames)
            frame = findObject(outputs, frame)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img = img.resize((640, 480), Image.LANCZOS)
            imgtk = ImageTk.PhotoImage(image=img)
            
            # Update GUI in main thread
            video_label.after(0, update_image, imgtk)
        
        except Exception as e:
            print(f"Error fetching video frame: {e}")
            video_label.after(0, video_label.configure, {'text': "No Camera Feed", 'image': ''})
        
        time.sleep(0.2)  # Small delay to avoid overwhelming the ESP32

def on_key_press(event):
    car_key_mapping = {
        'w': "forward",
        'a': "left",
        's': "stop",
        'd': "right",
        'z': "backward"
    }
    camera_key_mapping = {
        'Up': "cam_up",
        'Down': "cam_down",
        'Left': "cam_left",
        'Right': "cam_right",
        'space': "cam_center"
    }
    command = car_key_mapping.get(event.keysym) or camera_key_mapping.get(event.keysym)
    if command:
        send_command(command)

def create_gui():
    global video_label, root

    root = tk.Tk()
    root.title("ESP32-CAM Car Controller")
    root.geometry("900x600")
    root.configure(bg="#2C3E50")
    root.bind('<KeyPress>', on_key_press)

    # --- Header and LoRa Status ---
    header_label = tk.Label(root, text="ESP32-CAM Car Controller", font=("Helvetica", 20, "bold"),
                            bg="#2C3E50", fg="#ECF0F1")
    header_label.grid(row=0, column=0, columnspan=2, pady=15, padx=20, sticky="w")
    lora_status = "LoRa Connected" if lora_connected else "LoRa Not Connected"
    lora_color = "#2ECC71" if lora_connected else "#E74C3C"
    lora_label = tk.Label(root, text=lora_status, font=("Helvetica", 12),
                          bg="#2C3E50", fg=lora_color)
    lora_label.grid(row=0, column=1, sticky="e", padx=20)

    # --- Car Control Section ---
    car_control_frame = tk.Frame(root, bg="#34495E", bd=2, relief="ridge")
    car_control_frame.grid(row=1, column=0, padx=20, pady=20, sticky="n")
    btn_config = {"width": 10, "height": 2, "bg": "#2980B9", "fg": "white",
                  "activebackground": "#3498DB", "relief": "flat", "bd": 0, "font": ("Helvetica", 12)}

    tk.Button(car_control_frame, text="Forward", command=lambda: send_command("forward"), **btn_config).grid(row=0, column=1, padx=10, pady=10)
    tk.Button(car_control_frame, text="Left", command=lambda: send_command("left"), **btn_config).grid(row=1, column=0, padx=10, pady=10)
    tk.Button(car_control_frame, text="Stop", command=lambda: send_command("stop"), **btn_config).grid(row=1, column=1, padx=10, pady=10)
    tk.Button(car_control_frame, text="Right", command=lambda: send_command("right"), **btn_config).grid(row=1, column=2, padx=10, pady=10)
    tk.Button(car_control_frame, text="Backward", command=lambda: send_command("backward"), **btn_config).grid(row=2, column=1, padx=10, pady=10)
    
    # Autonomous and Manual Mode buttons
    tk.Button(car_control_frame, text="Auto Mode", command=lambda: send_command("auto"), **btn_config).grid(row=3, column=0, padx=10, pady=10)
    tk.Button(car_control_frame, text="Manual Mode", command=lambda: send_command("manual"), **btn_config).grid(row=3, column=2, padx=10, pady=10)

    # --- Camera Display and Control Section ---
    camera_frame = tk.Frame(root, bg="#34495E", bd=2, relief="ridge")
    camera_frame.grid(row=1, column=1, padx=20, pady=20, sticky="n")
    global video_label
    video_label = tk.Label(camera_frame, text="Loading camera...", font=("Helvetica", 12),
                           bg="#1C1F26", fg="#ECF0F1", relief="sunken", bd=2)
    video_label.pack(pady=15, padx=15)
    camera_control_frame = tk.Frame(camera_frame, bg="#34495E")
    camera_control_frame.pack(pady=10)
    cam_btn_config = {"width": 10, "height": 2, "bg": "#27AE60", "fg": "white",
                      "activebackground": "#2ECC71", "relief": "flat", "bd": 0, "font": ("Helvetica", 12)}

    tk.Button(camera_control_frame, text="Cam Up", command=lambda: send_command("cam_up"), **cam_btn_config).grid(row=0, column=1, padx=10, pady=5)
    tk.Button(camera_control_frame, text="Cam Left", command=lambda: send_command("cam_left"), **cam_btn_config).grid(row=1, column=0, padx=10, pady=5)
    tk.Button(camera_control_frame, text="Cam Center", command=lambda: send_command("cam_center"), **cam_btn_config).grid(row=1, column=1, padx=10, pady=5)
    tk.Button(camera_control_frame, text="Cam Right", command=lambda: send_command("cam_right"), **cam_btn_config).grid(row=1, column=2, padx=10, pady=5)
    tk.Button(camera_control_frame, text="Cam Down", command=lambda: send_command("cam_down"), **cam_btn_config).grid(row=2, column=1, padx=10, pady=5)

    # Start frame fetching in a separate thread
    threading.Thread(target=fetch_frames, daemon=True).start()

    root.mainloop()

if __name__ == "__main__":
    create_gui()