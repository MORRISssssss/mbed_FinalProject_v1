import serial
import time
import sys

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Usage: python bbcar_test_client.py <serial port to use>")
        exit()

    serdev = serial.Serial(sys.argv[1])

    
    print("Key 0: Start/stop line tracking.")
    print("Key 1: Get current distance.")
    print("Key 2: Get current speed.")
    print("Key w: Go straight.")
    print("Key a: Turn left.")
    print("Key s: Go backward.")
    print("Key d: Turn right.")
    print("Key e: Spin.")
    print("Key q: Leave task.")
    print("SPACE: Stop car.")

    while (True):
        line = input(">")
        if (line == "q"):
            serdev.close()
            print("Leave task.")
            break
        else: 
            if (line == "0" or line == "1" or line == "2" or line == " " or line == "w" or line == "a" or line == "s" or line == "d" or line == "e"):
                serdev.write(line.encode())
                time.sleep(0.001)
                line=serdev.readline()
                print(line.decode())


    
    