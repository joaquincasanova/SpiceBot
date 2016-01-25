from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import picamera
import picamera.array
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import os
import smbus
import logging
import Adafruit_GPIO.I2C as I2C
import curses

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)
# registers
TMP007_VOBJ             = 0x00
TMP007_TDIE             = 0x01
TMP007_CONFIG           = 0x02
TMP007_TOBJ             = 0x03
TMP007_STATUS           = 0x04
TMP007_STATMASK         = 0x05

# configure bytes
TMP007_CFG_RESET        = 0x8000
TMP007_CFG_MODEON       = 0x1000
TMP007_CFG_1SAMPLE      = 0x0000
TMP007_CFG_2SAMPLE      = 0x0200
TMP007_CFG_4SAMPLE      = 0x0400
TMP007_CFG_8SAMPLE      = 0x0600
TMP007_CFG_16SAMPLE     = 0x0800
TMP007_CFG_ALERTEN      = 0x0100
TMP007_CFG_ALERTF       = 0x0080
TMP007_CFG_TRANSC       = 0x0040

# interrupt configure
TMP007_STAT_ALERTEN     = 0x8000
TMP007_STAT_CRTEN       = 0x4000

# I2C address and device ID
TMP007_I2CADDR          = 0x40
TMP007_DEVID            = 0x1F

GPIO.setmode(GPIO.BOARD)

class TMP007(object):
        def __init__(self, mode=TMP007_CFG_16SAMPLE, address=TMP007_I2CADDR,
                                                         busnum=I2C.get_default_bus()):

                self._logger = logging.getLogger('TMP007')

                # Check that mode is valid.
                if mode not in [TMP007_CFG_1SAMPLE, TMP007_CFG_2SAMPLE, TMP007_CFG_4SAMPLE, TMP007_CFG_8SAMPLE, TMP007_CFG_16SAMPLE]:
                        raise ValueError('Unexpected mode value {0}.  Set mode to one of TMP007_CFG_1SAMPLE, TMP007_CFG_2SAMPLE, TMP007_CFG_4SAMPLE, TMP007_CFG_8SAMPLE or TMP007_CFG_16SAMPLE'.format(mode))
                self._mode = mode
                # Create I2C device.
                self._device = I2C.Device(address, busnum)
                # Load calibration values.
                self._load_calibration()

        # load calibration to sensor
        def _load_calibration(self):
                #load calibration               
                self._device.write16(TMP007_CONFIG, I2C.reverseByteOrder(TMP007_CFG_MODEON | TMP007_CFG_ALERTEN | TMP007_CFG_TRANSC | self._mode))
                #set alert status
                self._device.write16(TMP007_STATMASK, I2C.reverseByteOrder(TMP007_STAT_ALERTEN |TMP007_STAT_CRTEN))

        # read Die Temp in C
        def readDieTempC(self):
                raw = self._device.readU16BE(TMP007_TDIE)
                v = raw/4
                v *= 0.03125
                raw >>= 2
                Tdie = raw
                Tdie *= 0.03125 # convert to celsius
                self._logger.debug('Die temperature {0} C'.format(Tdie))
                return Tdie

        # read Obj Temp in C
        def readObjTempC(self):
                raw = self._device.readU16BE(TMP007_TOBJ)
                raw >>=2
                Tobj = raw
                Tobj *= 0.03125 # convert to celsius
                self._logger.debug('Obj temperature {0} C'.format(Tobj))
                return Tobj

        # read voltage
        def readVoltage(self):
                raw = self._device.readU16BE(TMP007_VOBJ);
                raw *= 156.25 # convert to nV
                raw /= 1000 # convert to uV
                self._logger.debug('Voltage {0} uV'.format(raw))
                return raw

class Bot:
	def __init__(self):
		self.Del = 0.01
		self.Imname = '/var/www/webcam.jpg'
		self.State = [0,0,0,0]
		self.FrontBumpPin = 18
		self.RearBumpPin = 7
		self.ResX = 800 
		self.ResY = 600
		GPIO.setup(self.FrontBumpPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.RearBumpPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

		# create a default object, no changes to I2C address or frequency
		mh = Adafruit_MotorHAT(addr=0x60)

		self.LFMotor = mh.getMotor(1)
		self.LRMotor = mh.getMotor(2)
		self.RRMotor = mh.getMotor(3)
		self.RFMotor = mh.getMotor(4)

		# turn on motor
		self.RFMotor.run(Adafruit_MotorHAT.RELEASE);
		self.LFMotor.run(Adafruit_MotorHAT.RELEASE);
		self.RRMotor.run(Adafruit_MotorHAT.RELEASE);
		self.LRMotor.run(Adafruit_MotorHAT.RELEASE);

		#self.Camera = picamera.PiCamera()
		#self.Camera.vflip=True
		#self.Camera.hflip=True
		#self.Camera.resolution = (self.ResX, self.ResY)
		#self.Camera.framerate = 30
		# Wait for the automatic gain control to settle
		#time.sleep(2)
		# Now fix the values
		#self.Camera.shutter_speed = self.Camera.exposure_speed
		#self.Camera.exposure_mode = 'off'
		#g = self.Camera.awb_gains
		#self.Camera.awb_mode = 'off'
		#self.Camera.awb_gains = g

	def StateToSpeed(self):
		if self.State[0]<0:
			self.LFMotor.run(Adafruit_MotorHAT.BACKWARD)
		else:
			self.LFMotor.run(Adafruit_MotorHAT.FORWARD)

		if self.State[3]<0:
			self.RFMotor.run(Adafruit_MotorHAT.BACKWARD)
		else:
			self.RFMotor.run(Adafruit_MotorHAT.FORWARD)

		if self.State[1]<0:
			self.LRMotor.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.LRMotor.run(Adafruit_MotorHAT.BACKWARD)

		if self.State[2]<0:
			self.RRMotor.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.RRMotor.run(Adafruit_MotorHAT.BACKWARD)

		self.LFMotor.setSpeed(abs(self.State[0]))
		self.LRMotor.setSpeed(abs(self.State[1]))
		self.RRMotor.setSpeed(abs(self.State[2]))
		self.RFMotor.setSpeed(abs(self.State[3]))

	def TurnOffMotors(self):
		self.LFMotor.run(Adafruit_MotorHAT.RELEASE)
		self.LRMotor.run(Adafruit_MotorHAT.RELEASE)
		self.RRMotor.run(Adafruit_MotorHAT.RELEASE)
		self.RFMotor.run(Adafruit_MotorHAT.RELEASE)

	def Idle(self,Seconds):
		t=time.time()
		while time.time()-t<Seconds:
			if GPIO.input(self.FrontBumpPin)==0:
				self.BounceBack()
			elif GPIO.input(self.RearBumpPin)==0:
				self.BounceForward()
			else:
				pass

	def Forward(self, Duration, Speed):
		print "Forward! "
		self.State = [Speed, Speed, Speed, Speed]
		self.StateToSpeed()
		self.Idle(max(self.Del,Duration))			
		print "Release"
		self.TurnOffMotors()
	def Backward(self, Duration, Speed):
		print "Backward! "
		self.State = [-Speed, -Speed, -Speed, -Speed]
		self.StateToSpeed()
		self.Idle(max(self.Del,Duration))			
		print "Release"
		self.TurnOffMotors()
	def Left(self, Duration, Speed):
                print "Left! "
		self.State = [-Speed, -Speed, Speed, Speed]
		self.StateToSpeed()
		self.Idle(max(self.Del,Duration))			
		print "Release"
		self.TurnOffMotors()
	def Right(self, Duration, Speed):
                print "Right! "
		self.State = [Speed, Speed, -Speed, -Speed]
		self.StateToSpeed()
		self.Idle(max(self.Del,Duration))			
		print "Release"
		self.TurnOffMotors()

	def BounceBack(self):
		self.Backward(0.5, 100)

	def BounceForward(self):
		self.Forward(0.5, 100)

	def Timelapse(self):
		for filename in self.Camera.capture_continuous('img{counter:03d}.jpg'):
			print('Captured %s' % filename)
			command = 'sudo cp '+filename+' '+self.Imname
			os.system(command)
			self.Idle(900) 

	def Normalize(self,x):
		X=np.float32(x)
		y=(X-np.min(X))/(np.max(X)-np.min(X))*255
		return np.uint8(y)

	def CVMove(self):
		if self.cx==None or self.cx==0:
			self.RandomMove()
		else:
			nx=np.int(np.float32(-(self.cx-self.ResX/2))/self.ResX*255)
			ny=np.int(np.float32(-(self.cy-self.ResY/2))/self.ResY*255)
		
			self.State = [-nx+ny,-nx+ny,nx+ny,nx+ny]
			Duration = np.sqrt(nx*nx+ny*ny)/255/2 
			print self.cx, self.cy, nx, ny, self.State, Duration
			self.StateToSpeed()
			self.Idle(Duration)
			self.TurnOffMotors()
	def Drone(self):
		print "Drone!"
		while True:
			char=screen.getch()
			if char==113: break
			elif char== curses.KEY_RIGHT : self.Right(.1, 180)
			elif char== curses.KEY_LEFT : self.Left(.1, 180)
			elif char== curses.KEY_UP : self.Forward(.1, 180)
			elif char== curses.KEY_DOWN : self.Backward(.1, 180)
			else : pass
	def RandomMove(self):
                Direction = np.random.randint(0,4)
                Duration = np.random.rand()
                Speed = np.random.randint(100,256)
                if Direction==0:
                        self.Forward(Duration, Speed)
                elif Direction==1:
                        self.Backward(Duration, Speed)
                elif Direction==2:
                        self.Right(Duration, Speed)
                else:
                        self.Left(Duration, Speed)

	def RandomWalk(self):
		Sensor = TMP007()
		for filename in self.Camera.capture_continuous('img{counter:03d}.jpg'):
                        self.RandomMove()
			objTempC = Sensor.readObjTempC()
			print 'Obj Temp:        ' + str(objTempC) + ' C'
			print('Captured %s' % filename)
			command = 'sudo cp '+filename+' '+self.Imname
			os.system(command)
			self.Idle(10) 
	def Webcam(self):
		self.Camera.capture(self.Imname)
		self.Idle(10)
	
	def CV(self):
		with picamera.array.PiRGBArray(self.Camera) as stream:
			self.Camera.capture(stream, format='bgr')
                        # At this point the image is available as stream.array
			img = stream.array
			cv2.imwrite(self.Imname,img)
			print "Processing..."
			gthresh=15
			rthresh=65
			B,G,R = cv2.split(img)
			g = self.Normalize(G)
 			r = self.Normalize(R)

			retval,rt = cv2.threshold(r,rthresh,255,cv2.THRESH_BINARY_INV)
			retval,gt = cv2.threshold(g,gthresh,255,cv2.THRESH_BINARY)
			bt = 255*np.uint8(np.sqrt(gt*rt))

			no=4
			kernel =  cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*no+1,2*no+1))
			bo = cv2.morphologyEx(bt, cv2.MORPH_OPEN, kernel)

			contours, hierarchy = cv2.findContours(bo,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

			img2 = np.copy(img)

			if contours==None:
				self.cx = None
				self.cy = None
				self.iy = None
				self.ix = None				
			else:
				a=np.shape(contours)
				if np.size(a)==1:
					nn=np.size(contours)
				elif np.size(a)==4:
					nn=a[0]
				
				area = np.zeros([nn,1])
				self.cx = 0.0
				self.cy = 0.0
				self.ix = 0.0
				self.iy = 0.0
				loc = 0
				areamax = 0.0
				for i in range(0,nn,1):
					M = cv2.moments(contours[i])
					area[i] = cv2.contourArea(contours[i])
					cv2.drawContours(img2, contours, i, (0,255,0), 3)
					if area[i]>areamax:
						areamax=area[i]
						loc = i
						if M['m00']==0:
							self.cx = None
							self.cy = None
							self.ix = None
							self.iy = None
						else:
							self.cx = int(M['m10']/M['m00'])
							self.cy = int(M['m01']/M['m00'])
							self.ix = int(M['m20']/M['m00'])
							self.iy = int(M['m02']/M['m00'])
			cv2.imwrite('/var/www/post.jpg',img2)
			print "Done"

SpiceBot=Bot()

try:
	SpiceBot.Drone()
except KeyboardInterrupt:
	SpiceBot.TurnOffMotors()
