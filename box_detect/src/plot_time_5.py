import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
import numpy as np
from matplotlib.animation import FuncAnimation

import csv
import numbers

x1 = []
y1 = []

x2 = []
y2 = []

x3 = []
y3 = []

row_count = 0
previousY = 0
currentSign = True
previousSign = False
period_counter = 0

with open('/home/rainbow/Desktop/com_compare_good5.csv','r') as csvfile:
	plots = csv.reader(csvfile, delimiter=',')

	for row in plots:
		row_count = row_count + 1

		if(row_count >1290 and row_count < 3330):

			x1.append(-1*float(row[1]))
			y1.append(-1*float(row[2]))

			x2.append(float(row[4]))
			y2.append(float(row[5]) )



			x3.append(float(row[4]))
			y3.append( (float(row[2])*-1) - float(float(row[5])) )

			#detect change in peak
			if(float(row[4]) < -0.05 and float(row[4]) > -2.55):
				currentY = float(row[5])
				deltaY = currentY - previousY

				if(deltaY>0):
					currentSign = True
				elif(deltaY<0):
					currentSign = False

				if(period_counter > 10):
					period_counter = 0

					if(currentSign is previousSign):
						print("detected peak at:", float(row[4]), float(row[5]))
						previousSign = (not previousSign)


				period_counter = period_counter + 1
				previousY = currentY



	  # if(row_count > 160 and row_count < 4800):
	  #   #print(float(row[4]))
	  #   x1.append(float(row[4]))
	  #   y1.append(float(row[5]))

	  #   x2.append(float(row[6]))
	  #   y2.append(float(row[7]))


	  #   x3.append(float(row[8]))
	  #   y3.append(float(row[9]))
		
	  
	  #if(isinstance(row[0],float)):
	  #    x.append(int(row[0]))
		#if(isinstance(row[1],float)):
		#   y.append(int(row[1]))

	print(row_count)


x1plot, y1plot = [], []
x2plot, y2plot = [], []
x3plot, y3plot = [], []

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
# ax = plt.axes(xlim=(-2.75, 0.25), ylim=(-0.35, 0.35))
ax = plt.axes()
line, = ax.plot([], [], lw=2)
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.title('Visual Odom vs Kinematic CoM ref',fontsize=20)

plotlays, plotcols = [3], ["blue","red", "black"]
lines = []
for index in range(3):
	
	if(index < 2):
	  lobj = ax.plot([],[],lw=1,color=plotcols[index])[0]
	else:
	  lobj = ax.plot([],[],lw=1,color=plotcols[index])[0]
	lines.append(lobj)


# initialization function: plot the background of each frame
def init():
	for line in lines:
		line.set_data([],[])
	return lines

# animation function.  This is called sequentially
def animate(i):
	x1plot.append(x1[i])
	y1plot.append(y1[i])
	x2plot.append(x2[i])
	y2plot.append(y2[i])
	x3plot.append(x3[i])
	y3plot.append(y3[i])

	xlist = [x1plot, x2plot, x3plot]
	ylist = [y1plot, y2plot, y3plot]


	 #for index in range(0,1):
	for lnum,line in enumerate(lines):
		line.set_data(xlist[lnum], ylist[lnum]) # set data for each line separately. 

	return lines

# anim = animation.FuncAnimation(fig, animate, init_func=init,
							   # frames=1300, interval=3, blit=True)

# plt.show()

y2array = np.array(y2)
x2array = np.array(x2)
dy = y2array[1:len(y2array)] - y2array[0:len(y2array)-1] 
dx = x2array[1:len(x2array)] - x2array[0:len(x2array)-1] 
print (len(dx))
print (len(dy))

dy = np.ma.masked_equal(dy,0)
dx = np.ma.masked_equal(dx,0)
print (len(dy))

x4 = x2[0:len(x2)-1]
# plt.scatter(x1,y1, label='visual odom', marker='x', color='b')
plt.scatter(x2,y2, label='kinematic reference', marker='.', color='r')
# plt.scatter(x3,y3, label='3', marker='x', color='b')
# plt.scatter(x4,dy, label='3', marker='.', color='k')
# plt.plot(y2, marker= ".")

# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('step position Graph')
# plt.legend()
plt.show()