import numpy as np # Import numpy
import matplotlib.pyplot as plt #import matplotlib library
import matplotlib.patches as mpatches

x=[]
meanX = []
y=[]
meanY = []
yErrormin = []
yErrormax = []
xErrormin = []
xErrormax = []
for i in range(10):
	with open("sortedCSV/out{}.csv".format(i)) as csv:
		csv = csv.read().split('\n')[:-1]
		csv = np.array([a.split(",") for a in csv])
		csvX,csvY = zip(*csv)
		csvX = np.array([float(a) for a in csvX])
		meanX.append(csvX.mean())
		xErrormin.append(csvX.mean()-csvX.min())
		xErrormax.append(csvX.max()-csvX.mean())
		csvY = np.array([float(a) for a in csvY])
		meanY.append(csvY.mean())
		yErrormin.append(csvY.mean()-csvY.min())
		yErrormax.append(csvY.max()-csvY.mean())

		x.extend(csvX)
		y.extend(csvY)

# = [a for _,a in sorted(zip(y,x))]
#y = sorted(y)
print(yErrormin)
print(yErrormax)
#meanX = [a for _,a in sorted(zip(meanY,meanX))]
#meanY = sorted(meanY)

def f(x):
	return x

x1 = np.arange(0., 6, 0.2)
y1 = f(x1)

x2 = np.arange(5.,7.1,0.2)
y2 = x2*0+5.555555555555555555

plt.figure(1)
plt.grid(True)
plt.plot(x, y, 'rx')
plt.plot(x1,y1,'b--')
plt.plot(x2,y2,'b--')
plt.xlabel('Frequency measured by hall sensor (Hz)')
plt.ylabel('Frequency measured by IMU (Hz)')
plt.title("All Freq by IMU against Freq by Hall")

yellow_patch = mpatches.Patch(label='Expected Data')
plt.legend(handles=[yellow_patch])

plt.figure(2)
plt.grid(True)
plt.plot(meanX, meanY,'bx')
plt.errorbar(meanX, meanY, xerr=[xErrormin,xErrormax], yerr=[yErrormin,yErrormax], fmt='x', ecolor='k', capthick=2)
plt.plot(x1,y1,'y--')
plt.plot(x2,y2,'y--')

yellow_patch = mpatches.Patch(color='y',label='Expected Data')
plt.legend(handles=[yellow_patch])


plt.xlabel('Frequency measured by hall sensor (Hz)')
plt.ylabel('Frequency measured by IMU (Hz)')
plt.title("Mean Freq by IMU against Freq by Hall")



plt.show()