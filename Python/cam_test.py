from picamera2 import *
import time

i = 0

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)

while True:

	#picam2.start_preview(Preview.QTGL)
	picam2.start()
	time.sleep(10.0)
	picam2.capture_file("image" + str(i) + ".jpg")
	i += 1
