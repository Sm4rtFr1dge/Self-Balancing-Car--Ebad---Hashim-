import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
ser = serial.Serial('/dev/ttyACM0', 115200)  # Change if needed
plt.ion()

accX_vals = []
gyroX_vals = []
angle_vals = []
time_ms = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Live accX, gyroX & angle')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Degree')
    #plt.ylim(-100,100)
    plt.plot(time_ms, accX_vals, 'r.-', label='accX')
    plt.plot(time_ms, gyroX_vals, 'g.-', label='gyroX')
    plt.plot(time_ms, angle_vals, 'b.-', label='angle')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while ser.inWaiting() == 0:
        pass  # Wait for data

    try:
        line = ser.readline().decode().strip()
        values = line.split(',')

        if len(values) == 3:
            accX = float(values[0])
            gyroX = float(values[1])
            angle = float(values[2])
            
            accX_vals.append(accX)
            gyroX_vals.append(gyroX)
            angle_vals.append(angle)
            time_ms.append(cnt)
            cnt += 10  # assuming STM32 sends every 10ms

            drawnow(makeFig)
            plt.pause(0.0001)

            # Keep only last 500 samples
            if len(accX_vals) > 500:
                accX_vals.pop(0)
                gyroX_vals.pop(0)
                angle_vals.pop(0)
                time_ms.pop(0)

    except Exception as e:
        print("Error:", e)
