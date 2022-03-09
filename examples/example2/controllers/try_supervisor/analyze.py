import numpy as np

f = open("rta_percentages.txt", "r")
lines = f.readlines()
f.close()

run_lines = []
lowbatteryrun_lines = []
for i in range(1, len(lines)):
	if "INFO: robot: Starting:" in lines[i]:
		run_lines.append(lines[i - 2])
		lowbatteryrun_lines.append(lines[i - 1])

untrusted_run = np.array(list(map(lambda x: int(x.split()[2]), run_lines)))
trusted_run = np.array(list(map(lambda x: int(x.split()[3]), run_lines)))
col_run = np.array(list(map(lambda x: int(x.split()[4]), run_lines)))

untrusted_lowbatteryrun = np.array(list(map(lambda x: int(x.split()[2]), lowbatteryrun_lines)))
trusted_lowbatteryrun = np.array(list(map(lambda x: int(x.split()[3]), lowbatteryrun_lines)))
col_lowbatteryrun = np.array(list(map(lambda x: int(x.split()[4]), lowbatteryrun_lines)))

print("Untrusted Run:", round(np.mean(untrusted_run/(untrusted_run + trusted_run + col_run)), 2))
print("Trusted Run:", round(np.mean(trusted_run/(untrusted_run + trusted_run + col_run)), 2))
print("Col Run:", round(np.mean(col_run/(untrusted_run + trusted_run + col_run)), 2))

print("Untrusted LowBatteryRun:", round(np.mean(untrusted_lowbatteryrun/(untrusted_lowbatteryrun + trusted_lowbatteryrun + col_lowbatteryrun)), 2))
print("Trusted LowBatteryRun:", round(np.mean(trusted_lowbatteryrun/(untrusted_lowbatteryrun + trusted_lowbatteryrun + col_lowbatteryrun)), 2))
print("Col LowBatteryRun:", round(np.mean(col_lowbatteryrun/(untrusted_lowbatteryrun + trusted_lowbatteryrun + col_lowbatteryrun)), 2))