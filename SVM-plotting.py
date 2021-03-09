# A small an hacky script to determine the position of the rcu via a Support Vector Machine
# steal this code if it is of any value
# Drone outputs pitch and roll via both complementary filter and kalman filter

import serial as ser
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import threading
import numpy as np
import pandas
from sklearn import svm
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import train_test_split

# constants
window = 300 # data sample window to plot, adjust to sample rate
update_interval = 500 # update the plot every x ms
baudrate = 115200
max_measurement = 40000 # stops after n measurement
n_features = 6
n_data = 8

ylables = ["Degrees", "Degrees", "Degrees", "Degrees"]
titles  = ["Pitch Kalman","Roll Kalman", "Pitch Complementary", "Roll Complementary"]

fig = plt.figure(1)
states = ["moving", "piloting", "resting"]
colors = ["yellow", "red", "green"]


# data 
texts = []
splots = []
data = []

# aua
for i in range(n_data):
	data.append([])

for i in range(1,7):
	print(i)
	splots.append(fig.add_subplot(2,3,i))


time = []
features = np.array([[0,0,0,0,0,0]])


clf = svm.SVC(decision_function_shape='ovo', probability=True)

# Serial polling function. ugly but working
def poll_serial():

	for j in range(max_measurement):
		actually = str(serial_usb.readline())
		#print(actually)

		actually = actually.replace("b'","").replace("\\n'","")
		#print(actually)
		if actually in ("",None):
			continue
		if actually[0] == "#":
			texts.append(actually)
			continue
		za = actually.split(",")
		j+=1
		if len(za) == 9:
			a = 0

			try:
				time.append(int(za[0]))
				
			except ValueError:
				print("value was problem :(")
				continue
			for elem in data:
				# for real data
				
				try:
					elem.append(float(za[a+1]))
				except ValueError as e:
					print("sth wen wrong")
					print(e, " ", actually)
					del time[-1]
				except IndexError:
					print("index problem")
					del time[-1]
				# for testing
				#elem.append(random.random())
				a+=1

# no comment on this behemoth
def animate(i, time, data):
	data_copy = []
	for i in range(n_data):
		data_copy.append([])
		data_copy[i] = data[i][-window:]
	time_copy = time[-window:]
	
	for i in range(2):
		splots[i].clear()
		splots[i].plot(time_copy, data_copy[i])
		splots[i].set_title(titles[i])
		splots[i].set_ylabel(ylables[i])
		splots[i].set_xlabel("t in ms")
	
	acc = np.sum(np.array([
		np.absolute(data_copy[2][int(-window/12):]), 
		np.absolute(data_copy[3][int(-window/12):]), 
		np.absolute(data_copy[4][int(-window/12):])]), axis=0)
	gyro = np.sum(np.array([
		np.absolute(data_copy[5][int(-window/12):]), 
		np.absolute(data_copy[6][int(-window/12):]), 
		np.absolute(data_copy[7][int(-window/12):])]), axis=0)

	global features
	features = np.append(features, np.array([[
		# np.mean(data_copy[0][int(-window/12):]),
		np.std(data_copy[0][int(-window/12):]),
		# np.mean(data_copy[1][int(-window/12):]),
		np.std(data_copy[1][int(-window/12):]),
		np.std(acc),
		np.mean(acc),
		np.std(gyro),
		np.mean(gyro)
		]]),axis=0)


	splots[2].plot(range(len(features[-window:,2])), features[-window:,0])
	splots[2].plot(range(len(features[-window:,2])), features[-window:,1])
	splots[2].plot(range(len(features[-window:,2])), features[-window:,2])
	splots[2].plot(range(len(features[-window:,2])), features[-window:,3])

	splots[3].clear()
	splots[3].scatter(features[:,0], features[:,1])

	splots[4].clear()
	for i in range(len(texts)):
		splots[4].text(0.05, (i+1)*0.05, texts[-i+1])
		if i > 10:
			break

	splots[5].clear()
	proba = clf.predict_proba(features[-1,:].reshape(-1,n_features))

	pred = clf.predict(features[-1,:].reshape(-1,n_features))

	for i in range(len(states)):
		splots[5].text(0.1, (i+1) *0.1, states[i] + ": " + str(100*np.round(proba[0,i], decimals=3)).format("%.2f") + "%")
	splots[5].set_facecolor(colors[int(pred[0])])
	splots[5].text(0.7,0.7,states[int(pred[0])])


def main():
	n_samples = 50
	# train svm on start
	# load data
	# get 
	moving   = pandas.read_csv("transMoving.csv", header=None)
	piloting = pandas.read_csv("transPiloting.csv", header=None)
	resting  = pandas.read_csv("transResting.csv", header=None)

	mov_header = moving.iloc[0]
	#drop header
	moving = moving[1:]
	piloting = piloting[1:]
	resting = resting[1:]

	moving = moving.drop([0], axis=1)
	piloting = piloting.drop([0], axis=1)
	resting =  resting.drop([0], axis=1)

	# Drop means
	moving = moving.drop([1], axis=1)
	piloting = piloting.drop([1], axis=1)
	resting =  resting.drop([1], axis=1)

	moving = moving.drop([3], axis=1)
	piloting = piloting.drop([3], axis=1)
	resting =  resting.drop([3], axis=1)

	#drop nans
	moving = moving.dropna()
	piloting = piloting.dropna()
	resting = resting.dropna()

	moving = np.asarray(moving, dtype=float)
	piloting = np.asarray(piloting, dtype=float)
	resting = np.asarray(resting, dtype=float)

	moving = moving[0:19000]
	resting = resting[0:19000]
	piloting = piloting[0:19000]

	x = moving
	x = np.append(x, piloting, axis=0)
	x = np.append(x, resting, axis=0)


	y0 = np.empty(len(moving))
	y0.fill(0)

	y1 = np.empty(len(piloting))
	y1.fill(1)

	y2 = np.empty(len(resting))
	y2.fill(2)

	y = y0
	y = np.append(y,y1)
	y = np.append(y,y2)

	# train SVM
	X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, shuffle=True)


	clf.fit(X_train, y_train)

	serial_usb = ser.Serial("/dev/ttyUSB0", baudrate)
	print(serial_usb)

	ani = animation.FuncAnimation(fig, animate, fargs=(time, data), interval = update_interval)
	x = threading.Thread(target=poll_serial)
	x.start()
	plt.show()

main()
