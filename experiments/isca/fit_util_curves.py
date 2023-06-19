import numpy as np
import matplotlib.pyplot as plt

# fproc, bproc, intermediate_values
iiwa_luts_total = 580809
iiwa_luts = [273656, 111702, 51106]
iiwa_dsps = [2848, 2612, 0]
#iiwa_dsps = [2848, 2612-(7*196)+(7*144), 0]
iiwa_resources = [7,7,7]
iiwa_len_fproc_sched = 8
hyq_luts_total = 509875
hyq_luts = [164449, 78702, 154047]
hyq_dsps = [1512, 1064, 0]
hyq_resources = [3,3,6]
hyq_len_fproc_sched = 13
baxter_luts_total = 815941
baxter_luts = [305983, 121682, 248131]#213875]
baxter_dsps = [2034, 1308, 0]
baxter_resources = [4,4,6]
baxter_len_fproc_sched = 18

robs_luts = [iiwa_luts, hyq_luts, baxter_luts]
robs_dsps = [iiwa_dsps, hyq_dsps, baxter_dsps]
robs_resources = [iiwa_resources, hyq_resources, baxter_resources]

fproc_luts = []
bproc_luts = []
intermediate_values_luts = []
fproc_dsps = []
bproc_dsps = []
fproc_resources = []
bproc_resources = []
block_size_resources = []
# iiwa, hyq, baxter
for i in range(3):
    fproc_luts.append(robs_luts[i][0])
    bproc_luts.append(robs_luts[i][1])
    intermediate_values_luts.append(robs_luts[i][2])
    fproc_dsps.append(robs_dsps[i][0])
    bproc_dsps.append(robs_dsps[i][1])
    fproc_resources.append(robs_resources[i][0])
    bproc_resources.append(robs_resources[i][1])
    block_size_resources.append(robs_resources[i][2])

print("iiwa lut delta: " + str(iiwa_luts_total - sum(iiwa_luts)))
print("hyq lut delta: " + str(hyq_luts_total - sum(hyq_luts)))
print("baxter lut delta: " + str(baxter_luts_total - sum(baxter_luts)))
print("")

print("fproc luts/PE:")
print(np.divide(np.array(fproc_luts), 2*np.array(fproc_resources)))
print("")

print("bproc luts/PE:")
print(np.divide(np.array(bproc_luts), 2*np.array(bproc_resources)))
print("")

print("fproc dsps/PE:")
print(np.divide(np.array(fproc_dsps), 2*np.array(fproc_resources)))
print("")

print("bproc dsps/PE:")
print(np.divide(np.array(bproc_dsps), 2*np.array(bproc_resources)))
print("")

##
y = np.divide(np.array(fproc_luts), 2*np.array(fproc_resources))
x = [7, 12, 15]
z = np.polyfit(x, y, 1)
print(z)
f = np.poly1d(z)

x_new = np.linspace(x[0], x[-1], 50)
y_new = f(x_new)

plt.plot(x,y,'o', x_new, y_new)
plt.xlim([x[0]-1, x[-1] + 1 ])
plt.show()

##
y = np.divide(np.array(bproc_luts), 2*np.array(bproc_resources))
x = [7, 12, 15]
z = np.polyfit(x, y, 1)
print(z)
f = np.poly1d(z)

x_new = np.linspace(x[0], x[-1], 50)
y_new = f(x_new)

plt.plot(x,y,'o', x_new, y_new)
plt.xlim([x[0]-1, x[-1] + 1 ])
plt.show()

##
y = intermediate_values_luts
x = [7, 12, 15]
z = np.polyfit(x, y, 2)
print(z)
f = np.poly1d(z)

x_new = np.linspace(x[0], x[-1], 50)
y_new = f(x_new)

plt.plot(x,y,'o', x_new, y_new)
plt.xlim([x[0]-1, x[-1] + 1 ])
plt.show()

##
y = np.divide(np.array(bproc_dsps), 2*np.array(bproc_resources))
x = [7, 12, 15]
z = np.polyfit(x, y, 1)
print(z)
f = np.poly1d(z)

x_new = np.linspace(x[0], x[-1], 50)
y_new = f(x_new)

plt.plot(x,y,'o', x_new, y_new)
plt.xlim([x[0]-1, x[-1] + 1 ])
plt.show()

