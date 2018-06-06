import numpy as np
for i in range(10):
	with open("testData/IMU{}.CSV".format(i)) as rFile:
		raw_datas = rFile.read()
		raw_datas = raw_datas.split('\n')[0:-1]
		time = [[],[],[],[],[],[],[],[],[],[],[]]
		data = [[],[],[],[],[],[],[],[],[],[],[]]
		for raw_data in raw_datas:
			raw_data = raw_data.split(",")
			time[i].append(float(raw_data[0]))
			data[i].append(float(raw_data[1]))

		time[i] = np.array(time[i])
		data[i] = np.array(data[i])
		data[i] /= 360 # data to Hz
		with open("5-6-2018Test/filtered/0{}".format(i+1)) as dataFile:
			raw = dataFile.read()
			raw = np.array([float(a) for a in raw.split(", ")])
			mean = np.mean(raw)
			logic_array = []
			if mean < 5.5:
				logic_array = np.logical_and(data[i]<mean+0.3,data[i]>mean-0.3)
			else: 
				logic_array = data[i]>5.4
			filteredData = []
			filteredTime = []
			for z in range(len(logic_array)):
				if logic_array[z]:
					filteredData.append(data[i][z])
					filteredTime.append(time[i][z])
			data[i] = filteredData[:]
			time[i] = filteredTime[:]
			for z in range(len(data[i])):
				with open("sortedCSV/out"+str(i)+".csv","a+") as output:
					randomIndex = np.random.randint(0,len(raw))
					value = raw[randomIndex]
					np.delete(raw,randomIndex)
					output.write("{},{}\n".format(value,data[i][z]))
			with open("sortedCSV/README.txt","a+") as File:
				File.write("{}: {} Hz\n".format("sortedCSV/out"+str(i)+".csv",raw.mean()))


