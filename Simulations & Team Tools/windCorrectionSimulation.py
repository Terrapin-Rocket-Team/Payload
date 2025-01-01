'''
See documentation here: https://docs.google.com/document/d/1soUME8JDSpf028hsgl010TmuEHOHm2ZJCv7ecYDvrWE/edit
'''


import matplotlib.pyplot as plt
import math
import random as rand


mass=1 #kg
t=0 #s
tStep=0.01 #s

numSims=6

vs=6
vsAct=[vs/2,vs/1.5,vs,vs*1.5,vs*2,vs]
target=[0,10]

windMax=vsAct[0]*0.9
windMin=0
windAct=[windMax,0]
windVariationSpeed=0.1
weightFactor=.5



windEst=[]
windEstX=[]
windEstY=[]
windx=[0]
windy=[0]
times=[0]
headings=[]
x=[]
y=[]
vx=[]
vy=[]

for i in range(numSims):
	windEst.append([0,0])
	windEstX.append([0])
	windEstY.append([0])
	headings.append([0])
	x.append([0])
	y.append([0])
	vx.append([0])
	vy.append([0])




while t<10:

	#Update time
	t+=tStep
	times.append(t)

	#Update wind
	windAct[0]+=windVariationSpeed*(2*rand.random()-1)
	if abs(windAct[0])>windMax:
		windAct[0]=windMax*windAct[0]/abs(windAct[0])
	#windAct[1]+=windVariationSpeed*(2*rand.random()-1)
	#if abs(windAct[1])>windMax:
	#	windAct[1]=windMax*windAct[1]/abs(windAct[1])
	windx.append(windAct[0])
	windy.append(windAct[1])

	for i in range(numSims):

		if i<(numSims-1):
			#Calculate heading vector for smart system
			h=[target[0]-x[i][-1],target[1]-y[i][-1]]
			hMag=(h[0]**2+h[1]**2)**0.5
			h=[h[0]/hMag,h[1]/hMag]


			#Determine wp using estimated wind vector 
			wpconst=((windEst[i][0]*h[0]+windEst[i][1]*h[1])/(h[0]**2+h[1]**2))
			wp=[h[0]*wpconst,h[1]*wpconst]

			#Determine from wp and wind estimate
			wc=[windEst[i][0]-wp[0],windEst[i][1]-wp[1]]

			#Create g based on wind strength
			if (windEst[i][0]**2+windEst[i][1]**2) < vs**2:
				hPrimeMag=(vs**2-(wc[0]**2+wc[1]**2))**0.5
				g=[h[0]*hPrimeMag-wc[0],h[1]*hPrimeMag-wc[1]]
			else: #This is primarily a saftey thing, not much control theory behind it
				g=[-windEst[i][0],-windEst[i][1]]


			#Calculate heading angle from g
			heading=math.atan2(g[1],g[0])

		else:
			h=[target[0]-x[i][-1],target[1]-y[i][-1]]
			heading=math.atan2(h[1],h[0])

		headings[i].append(heading)

		#Update velocities
		vx[i].append(windAct[0]+vsAct[i]*math.cos(heading))
		vy[i].append(windAct[1]+vsAct[i]*math.sin(heading))

		#Update positions
		x[i].append(x[i][-1]+vx[i][-1]*tStep)
		y[i].append(y[i][-1]+vy[i][-1]*tStep)

		if i<(numSims-1):
			#Update the wind estimate using 
			newWindEst=[((x[i][-1]-x[i][-2])/tStep)-vs*math.cos(heading),((y[i][-1]-y[i][-2])/tStep)-vs*math.sin(heading)]
			windEst[i]=[(1-weightFactor)*windEst[i][0]+weightFactor*newWindEst[0],(1-weightFactor)*windEst[i][1]+weightFactor*newWindEst[1]]
			windEstX[i].append(windEst[i][0])
			windEstY[i].append(windEst[i][1])
		



#Plot wind canceling and dumb trajectories
plt.figure()
for i in range(numSims-1):
	plt.plot(x[i],y[i], ls=":",label='A/E: '+str(round(vsAct[i]/vs,2)))
plt.plot(x[-1],y[-1], ls=":",label='No wind correction')
plt.legend(loc=4)


plt.figure()
plt.subplot(2, 1, 1)
#for i in range(numSims-1):
#	plt.plot(times,windEstX[i], ls=":",label='A/E: '+str(round(vsAct[i]/vs,2)))
plt.plot(times,windx, ls=":",label='Wind')
plt.legend()

plt.subplot(2, 1, 2)
#for i in range(numSims-1):
#	plt.plot(times,windEstY[i], ls=":",label='A/E: '+str(round(vsAct[i]/vs,2)))
plt.plot(times,windy, ls=":",label='Wind')
plt.legend()


plt.show()