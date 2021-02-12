import datetime, hashlib, hmac
import cv2
import requests
import json
import math
import getpass
import boto3
from botocore.exceptions import ClientError

import numpy as np
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy

from sensor_msgs.msg import CompressedImage, Image

from video_capture_ros import VideoCapture
from detector import Detector


class Controller(ConnectionBasedTransport):

    def __init__(self, env, video_device):
        rospy.init_node('image_to_label')
        rospy.set_param('~always_subscribe', True)
        super(Controller,self).__init__()
        self.cap = None
        print("new video coming")
        try:
            self.CROPPED_IMAGE_WIDTH = env['CroppedImageWidth']
            self.CROPPED_IMAGE_HEIGHT = env['CroppedImageHeight']
        except KeyError:
            print('Invalid config file')
            raise

        print('video_device:', video_device)
        #self.cap = cv2.VideoCapture(video_device)
        #if not self.cap.isOpened():
        #    raise RuntimeError('Cannot open camera')
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        try:
            self.API_ENDPOINT = env['ApiEndpoint']
            self.FACE_AREA_THRESHOLD = env['FaceAreaThreshold']
            self.NAME_TTL_SEC = env['NameTtlSec']
            self.FACE_SIMILARITY_THRESHOLD = env['FaceSimilarityThreshold']
            self.COGNITO_USERPOOL_ID = env['CognitoUserPoolId']
            self.COGNITO_USERPOOL_CLIENT_ID = env['CognitoUserPoolClientId']
            self.REGION = env['Region']
        except KeyError:
            print('Invalid config file')
            raise

        self.recent_name_list = []
        self.registered_name_set = set()
        print("name_set registered")
        #self.video_capture = VideoCapture(env, video_device)
        print("video capturing")
        self.detector = Detector(env)
        print("detector imported")

    def _update_name_list(self):
        limit_time = datetime.datetime.now() - datetime.timedelta(seconds=self.NAME_TTL_SEC)
        for d in self.recent_name_list[:]:
            if d.get('timestamp') < limit_time:
                self.recent_name_list.remove(d)

    def _sign(self, key, msg):
        return hmac.new(key, msg.encode("utf-8"), hashlib.sha256).digest()

    def _get_signature_key(self, key, date_stamp, region_name, service_name):
        date = self._sign(('AWS4' + key).encode('utf-8'), date_stamp)
        region = self._sign(date, region_name)
        service = self._sign(region, service_name)
        signing = self._sign(service, 'aws4_request')
        return signing

    def _get_id_token_by_cognito(self, username, password):
        client = boto3.client('cognito-idp', self.REGION)
        response = client.initiate_auth(
            ClientId=self.COGNITO_USERPOOL_CLIENT_ID,
            AuthFlow='USER_PASSWORD_AUTH',
            AuthParameters={
                'USERNAME': username,
                'PASSWORD': password
            }
        )
        return response['AuthenticationResult']['IdToken']


    def run(self):
        # input username and password
        credential_path = './.gitignore'
        try:
            with open(credential_path) as credential_json:
                credential = json.load(credential_json)
        except:
            print("credential missing. add credentials in .gitignore")

        username = credential['Username']
        password = credential['Password']

        try:
            self.id_token = self._get_id_token_by_cognito(username, password)
            print(self.id_token)
        except ClientError as e:
            if e.response['Error']['Code'] == 'UserNotFoundException':
                print('User does not exist')
                return
            elif e.response['Error']['Code'] == 'NotAuthorizedException':
                print('Invalid password')
                return
            raise

        self._sub = rospy.Subscriber('/head_camera/rgb/image_raw/compressed', CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)
        #self._sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_cb, queue_size=1, buff_size=2**26)
        rospy.spin()

        # Main loop
        
    def image_cb(self, msg):
        print(isinstance(msg, Image))
        if isinstance(msg, Image):
            bridge = cv_bridge.CvBridge()
            self.cap = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.cap = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.cap = self.cap[:, :, ::-1]
        #for i in range(5):
        #    self.cap.grab()
        #ret, frame_ = self.cap.read()
        #if ret is False or frame_ is None:
        #    return None

        frame_ = cv2.cvtColor(self.cap, cv2.COLOR_BGR2RGB)
        # frame_ = self.cap
        cv2.imshow("image_", frame_)
        cv2.waitKey(1)
        # crop
        cx = frame_.shape[1] / 2
        cy = frame_.shape[0] / 2
        # print("cx {}, cy {} {} {}\n", cx, cy, self.CROPPED_IMAGE_HEIGHT, self.CROPPED_IMAGE_WIDTH);
        frame = frame_[int(cy-self.CROPPED_IMAGE_HEIGHT/2):int(cy+self.CROPPED_IMAGE_HEIGHT/2),
                    int(cx-self.CROPPED_IMAGE_WIDTH/2):int(cx+self.CROPPED_IMAGE_WIDTH/2)]

        if frame is None:
            raise RuntimeError('A problem occurred with camera. Cannot read a new image.')

        #cv2.imshow("image", frame)
        #cv2.waitKey(1)
        face_exists, face_image = self.detector.detect(frame_)

        if face_exists:
            print("face founded")
            area = face_image.shape[0] * face_image.shape[1]
            if area > self.FACE_AREA_THRESHOLD * 2:
                # resize
                ratio = math.sqrt(area / (self.FACE_AREA_THRESHOLD * 2))
                face_image = cv2.resize(face_image, (int(
                    face_image.shape[1] / ratio), int(face_image.shape[0] / ratio)))
            _, encoded_face_image = cv2.imencode('.jpg', face_image)

            # Call API
            try:
                endpoint = 'https://' + self.API_ENDPOINT
                t = datetime.datetime.utcnow()
                amz_date = t.strftime('%Y%m%dT%H%M%SZ')
                headers = {
                    'Content-Type': 'image/jpg',
                    'X-Amz-Date':amz_date,
                    'Authorization': self.id_token
                }
                request_parameters = encoded_face_image.tostring()
                res = requests.post(endpoint, data=request_parameters, headers=headers).json()
                print(res)
                # renponse samples:
                #      {'result': 'OK', 'name': 'hoge', 'similarity': 95.15}
                #      {'result': 'NO_MATCH', 'name': '', 'similarity': 0}
                #      {'result': 'INVALID', 'name': '', 'similarity': 0}

                result = res['result']
            except Exception as e:
                print(e)

            else:
                if result == 'OK':
                    name = res['name']
                    similarity = res['similarity']
                    if similarity > self.FACE_SIMILARITY_THRESHOLD:
                        self._update_name_list()
                        if name not in [d.get('name') for d in self.recent_name_list]:
                            # OK! Go Ahead!
                            self.registered_name_set.add(name)
                            self.recent_name_list.append(
                                {'name': name, 'timestamp': datetime.datetime.now()})

        else:
            print("no face")

        key = cv2.waitKey(1)
        if key == ord('q'):
            raise RuntimeError("key 'q' is pressed")
