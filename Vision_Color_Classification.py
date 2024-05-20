import cv2
import numpy as np
import pyzed.sl as sl
import math
import statistics
import paho.mqtt.client as mqtt
import time

# Define MQTT broker address and topic
# broker_address = "10.7.147.120"  # Replace with your broker address
broker_address = 'mqtt-dashboard.com'
# Create an MQTT client and connect to the broker
### Pub ###
client = mqtt.Client()
client.connect(broker_address)
def publish_command(topic, command):
    client.publish(topic, command)

# publish_command(pos2control,command)
## สำหรับส่งข้อมูลให้กับ control ในการเคลื่อนที่ไปยังตำแหน่ง
# publish_command(position,f"{position[i][0]},{position[i][1]}")
## สำหรับการส่งข้อมูลให้ VR ได้การบอกสิ่งที่ตรวจจับได้
# publish_command(object,f"{red[0]},{blue[0]},{green[0]}")

## sub ##
topic = ""
message = ''
desired_message = ''

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topic)
def on_message(client, userdata, msg):
    global topic, message, desired_message
    # print(msg.topic+" "+str(msg.payload.decode()))
    # เช็คว่าได้รับข้อมูลที่ต้องการหรือไม่
    if msg.topic == topic:
        if msg.payload.decode() == desired_message:
            print(msg.topic+" "+str(msg.payload.decode()))
            client.disconnect()  # หยุดการเชื่อมต่อ MQTT broker
            message = msg.payload.decode()
        elif msg.payload.decode() == 'red':
            print(msg.topic+" "+str(msg.payload.decode()))
            client.disconnect()  # หยุดการเชื่อมต่อ MQTT broker
            message = msg.payload.decode()
        elif msg.payload.decode() == 'green':
            print(msg.topic+" "+str(msg.payload.decode()))
            client.disconnect()  # หยุดการเชื่อมต่อ MQTT broker
            message = msg.payload.decode()
        elif msg.payload.decode() == 'blue':
            print(msg.topic+" "+str(msg.payload.decode()))
            client.disconnect()  # หยุดการเชื่อมต่อ MQTT broker
            message = msg.payload.decode()

        #print("Received stop command. Disconnecting from broker.")

## state robot ##
# desired_message = 'set_camera_already'
# topic = 'state_robot'
# client = mqtt.Client()
# client.on_connect = on_connect
# client.on_message = on_message
# client.connect(broker_address)
# client.loop_forever()

## state robot ##
# desired_message = 'red/green/blue'
# topic = 'choose_bottle'
# client = mqtt.Client()
# client.on_connect = on_connect
# client.on_message = on_message
# client.connect(broker_address)
# client.loop_forever()




def rotate_point(point, axis, angle):
    angle = np.radians(angle)
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    if axis == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, cos_angle, -sin_angle],
                                    [0, sin_angle, cos_angle]])
    elif axis == 'y':
        rotation_matrix = np.array([[cos_angle, 0, sin_angle],
                                    [0, 1, 0],
                                    [-sin_angle, 0, cos_angle]])
    elif axis == 'z':
        rotation_matrix = np.array([[cos_angle, -sin_angle, 0],
                                    [sin_angle, cos_angle, 0],
                                    [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")
    rotated_point = np.dot(rotation_matrix, point)
    return rotated_point

def checkColor(ori_img, limitx,limity):
   font = cv2.FONT_HERSHEY_SIMPLEX 
   fontScale = 0.5
   color_t = (255, 255, 255)  # สีข้อความในรูปแบบ BGR
   thickness = 2  # ความหนาของข้อความ
   img = ori_img
   n_red = 0
   n_blue = 0
   n_green = 0
   c_red = []
   c_blue = []
   c_green = []
   area_min = 4000
   ratio_x = 29.4/(370+327)
   ratio_y = 22/(223+298)
   # ratio_x = 1
   # ratio_y = 1
   hsv_frame = cv2.cvtColor(ori_img, cv2.COLOR_BGR2HSV)
   low_red1 = np.array([170, 70, 50])
   high_red1 = np.array([180, 255, 255])
   red_mask1 = cv2.inRange(hsv_frame, low_red1, high_red1)
   low_red2 = np.array([0, 70, 50])
   high_red2 = np.array([10, 255, 255])
   red_mask2 = cv2.inRange(hsv_frame, low_red2, high_red2)
   red_mask = red_mask1 | red_mask2
   red_contours, red_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   red = cv2.bitwise_and(ori_img, ori_img, mask=red_mask)
   sorted_red = sorted(red_contours, key=cv2.contourArea, reverse=True)
   if len(red_contours) != 0:
      for i in range(len(red_contours)):
         area = cv2.contourArea(sorted_red[i])
         if area >= area_min:
            # print(area)
            r = int((area/np.pi)**0.5)
            M = cv2.moments(sorted_red[i])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
               #  print(f'cx,cy {cx},{cy}')
               #  print(f'limit {limitx},{limity}')
                if cx >= limitx[0] and cx <= limitx[1] and cy >= limity[0] and cy <= limity[1]:
                  n_red += 1
                  x = round((((cy-360))*(ratio_y)) * 0.01  - 0.105 - 0.35538 - 0.05 - 0.03 ,5)
                  z = round(((cx-640)*(ratio_x)) * 0.01 + 0.22626 + 0.04,5)
                  point = np.array([round(((cx-640)*(ratio_x)),2), round((((cy-360))*(ratio_y)),2), 0])
                  # rotated_point = rotate_point(point, 'z', 90)
                  # x = round(rotated_point[0],2)
                  # y = round(rotated_point[1],2)
                  c_red.append([i,x,z])
                  # print(f'rotate_point Red => {point}')
                  img = cv2.circle(img, (cx, cy), radius=r , color=(0,0,255), thickness=5)
                  img = cv2.circle(img, (cx, cy), radius=10 , color=(255, 255, 255), thickness=-1)
                  img = cv2.putText(img, f'Red [x,z] {i} | {x},{z} m', (cx + 20,cy - 20), font, fontScale, (0,0,0), thickness) 
      img = cv2.putText(img, f'Number of Red => {n_red}', (2,20), font, fontScale, color_t, thickness) 
        # Blue color
   low_blue = np.array([94, 80, 2])
   high_blue = np.array([126, 255, 255])
   blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
   blue_contours, blue_hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   blue = cv2.bitwise_and(ori_img, ori_img, mask=blue_mask)
   sorted_blue = sorted(blue_contours, key=cv2.contourArea, reverse=True)
   if len(blue_contours) != 0:
      for i in range(len(blue_contours)):
         area = cv2.contourArea(sorted_blue[i])
         if area >= area_min:
            # print(area)
            r = int((area/np.pi)**0.5)
            M = cv2.moments(sorted_blue[i])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx >= limitx[0] and cx <= limitx[1] and cy >= limity[0] and cy <= limity[1]:
                  n_blue += 1
                  x = round((((cy-360))*(ratio_y)) * 0.01  - 0.105 - 0.35538 - 0.05 - 0.03,5)
                  z = round(((cx-640)*(ratio_x)) * 0.01 + 0.22626 + 0.04,5)
                  c_blue.append([i,x,z])
                  img = cv2.circle(img, (cx, cy), radius=r , color=(255,0,0), thickness=5)
                  img = cv2.circle(img, (cx, cy), radius=10 , color=(255, 255, 255), thickness=-1)
                  img = cv2.putText(img, f'Blue [x,z] {i} | {x},{z} m', (cx + 20,cy - 20), font, fontScale, (0,0,0), thickness) 
      img = cv2.putText(img, f'Number of blue => {n_blue}', (2,45), font, fontScale, color_t, thickness) 

    # Green color
   low_green1 = np.array([25, 52, 72])
   high_green1 = np.array([102, 255, 255])
   green_mask1 = cv2.inRange(hsv_frame, low_green1, high_green1)

   low_green2 = np.array([40, 100, 100])
   high_green2 = np.array([80, 255, 255])
   green_mask2 = cv2.inRange(hsv_frame, low_green2, high_green2)

   green_mask = green_mask1 | green_mask2

   green_contours, green_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   green = cv2.bitwise_and(ori_img, ori_img, mask=green_mask)
   sorted_green = sorted(green_contours, key=cv2.contourArea, reverse=True)
   if len(green_contours) != 0:
      for i in range(len(green_contours)):
         area = cv2.contourArea(sorted_green[i])
         if area >= area_min:
            # print(area)
            r = int((area/np.pi)**0.5)
            M = cv2.moments(sorted_green[i])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx >= limitx[0] and cx <= limitx[1] and cy >= limity[0] and cy <= limity[1]:
                  n_green += 1
                  x = round((((cy-360))*(ratio_y)) * 0.01  - 0.105 - 0.35538 - 0.05 - 0.03,5)
                  z = round(((cx-640)*(ratio_x)) * 0.01 + 0.22626 + 0.04,5)
                  c_green.append([i,x,z])
                  img = cv2.circle(img, (cx, cy), radius=r , color=(0,255,0), thickness=5)
                  img = cv2.circle(img, (cx, cy), radius=10 , color=(255, 255, 255), thickness=-1)
                  img = cv2.putText(img, f'Green [x,z] {i} | {x},{z} m', (cx + 20,cy - 20), font, fontScale, (0,0,0), thickness) 
      img = cv2.putText(img, f'Number of green => {n_green}', (2,70), font, fontScale, color_t, thickness) 
   red = [n_red, c_red, red_mask]
   blue = [n_blue, c_blue, blue_mask]
   green = [n_green, c_green, green_mask]
   return img, red, blue, green

# Create a Camera object
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 0.5
color_t = (255, 255, 255)  # สีข้อความในรูปแบบ BGR
thickness = 2  # ความหนาของข้อความ
zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

    # Open the camera
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
   print("Camera Open : "+repr(status)+". Exit program.")
   exit()

    # Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()

image_left = sl.Mat()
image_right = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()

mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
r = 0
limitx = 0
limity = 0
cal_cen = []

bottle_h = 180 #mm
center_avg = 0
n_frame = 0
while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        ## state robot ##
        zed.retrieve_image(image_left, sl.VIEW.LEFT)
        zed.retrieve_image(image_right, sl.VIEW.RIGHT)
                # Retrieve depth map. Depth is aligned on the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        img = image_left.get_data()

        cv2.imshow("Result image", img)
        key = cv2.waitKey(10)

        if r == 0:
            print('Begin state first times')
            # roi = cv2.selectROI('Select ROI', img, fromCenter=False, showCrosshair=True)
            desired_message = 'camera_position'
            topic = 'UR/control/state'
            client = mqtt.Client()
            client.on_connect = on_connect
            client.on_message = on_message
            client.connect(broker_address)
            client.loop_forever()
            if message == desired_message:
                print('robot on state in camera position')
            
            roi = [257, 67, 736, 472]
            img = img[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
            cv2.imshow("Result image", img)
            limity = [int(roi[1]),int(roi[1]+roi[3])]
            limitx = [int(roi[0]),int(roi[0]+roi[2])]
            # key = cv2.waitKey()
            # cv2.destroyAllWindows()
            cv2.imshow("Result image", img)
            key = cv2.waitKey(10)
            r += 3
        elif r == 1:
            print('r1')
            x = image_left.get_width() / 2
            y = image_left.get_height() / 2
            err, point_cloud_value = point_cloud.get_value(x, y)
            if math.isfinite(point_cloud_value[2]):
                distance_center = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                cal_cen.append(distance_center)
                if len(cal_cen) == 20:
                    center_avg = statistics.mean(cal_cen)
                    #print(f'distance center => {center_avg}')
                    r += 2
                cv2.imshow("Result image", img)
                key = cv2.waitKey(10)
            else : 
                r += 2
                center_avg = 46
                #print(f"The distance can not be computed at center {{{x};{y}}}")
        elif r == 2:
            width, height = 800, 600
            black_image = np.zeros((height, width, 3), dtype=np.uint8)  # 3 channels for color image
            black_image = cv2.putText(black_image,"Place the object on the work space.", (int(width/2) - 300,int(height/2) - 60), font, 1.0, (255,255,255), thickness)
            black_image = cv2.putText(black_image,"Please press key spacebar!", (int(width/2) - 250,int(height/2) + 100), font, 1.2, (255,255,255), thickness)
            
            cv2.imshow("Wait frame", black_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            r += 1
        else:
            # return img, red, blue, green
            # red = [n_red, c_red, red_mask]
            # c_red.append([i,x,z])
            print('Begin main loop')
            center_avg = 46
            img, red, blue, green = checkColor(img,limitx,limity)
            n = [red[0],green[0],blue[0]]
            check = []
            for i in range(len(n)):
                if n[i] > 0:
                    check.append(1)
                else:
                    check.append(0)
            time.sleep(2)
            client = mqtt.Client()
            client.connect(broker_address)
            print(f'Check {check}')
            publish_command('UR/vision/object',f"{check[0]},{check[1]},{check[2]}")
            #print(f"{check[0]},{check[1]},{check[2]}")
            distance_object = center_avg - bottle_h
            img = cv2.putText(img, f"distance form camera => {round(distance_object*0.1,2)} cm", (2,95), font, fontScale, color_t, thickness)
            w,h,c = img.shape
            start_point = (0, int(w/2))
            end_point = (int(h), int(w/2))
            start_point1 = (int(h/2), 0)
            end_point1 = (int(h/2), int(w))
            color = (0, 0, 0)
            thickness = 2
            client = mqtt.Client()
            client.connect(broker_address)
            publish_command('UR/vision/object',f"{check[0]},{check[1]},{check[2]}")
            # client = mqtt.Client()
            # client.connect(broker_address)
            # print(f'Check {check}')
            # publish_command('UR/vision/object',f"{check[0]},{check[1]},{check[2]}")
            cv2.line(img, start_point, end_point, color, thickness)
            cv2.line(img, start_point1, end_point1, color, thickness)
            cv2.imshow("Result image", img)
            key = cv2.waitKey(10)
            print('r6')
            topic = 'UR/unity/bottle'
            client = mqtt.Client()
            client.on_connect = on_connect
            client.on_message = on_message
            client.connect(broker_address)
            client.loop_forever()
            time.sleep(2)
            if message == 'red' and red[0] > 0:
                #print('red')
                client = mqtt.Client()
                client.connect(broker_address)
                publish_command('UR/vision/position',f"{red[1][0][1]},{red[1][0][2]}")
            elif message == 'blue' and blue[0] > 0:
                #print('blue')
                client = mqtt.Client()
                client.connect(broker_address)
                publish_command('UR/vision/position',f"{blue[1][0][1]},{blue[1][0][2]}")
            elif message == 'green' and green[0] > 0:
                #print('green')
                client = mqtt.Client()
                client.connect(broker_address)
                publish_command('UR/vision/position',f"{green[1][0][1]},{green[1][0][2]}")
            else:
                #print('There are currently no bottles of the color you selected.')
                client = mqtt.Client()
                client.connect(broker_address)
                publish_command('UR/vision/position',f"{-0.35538},{0.22626}")
            # n_frame += 1
            print('r7')
            r = 0
            key = cv2.waitKey(1)
            if key == 27:
                break
cv2.destroyAllWindows()
zed.close()

