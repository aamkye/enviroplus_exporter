factor = 1.25
raw_temp = 34
cpu_temps = [50] * 5
cpu_temp = 50
# Smooth out with some averaging to decrease jitter
cpu_temps = cpu_temps[1:] + [cpu_temp]
avg_cpu_temp = sum(cpu_temps) / float(len(cpu_temps))
temperature = raw_temp - ((avg_cpu_temp - raw_temp) / factor)

print(temperature)
