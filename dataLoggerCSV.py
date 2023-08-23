import datetime
import serial
import csv

#main
if __name__ == '__main__' :
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
    while True :
        if ser.in_waiting > 0 :
            # timee = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            timee = datetime.datetime.now().strftime('%H:%M:%S.%f')
            data = ser.readline().decode('utf-8').rstrip()
            print(data)
            datasplit = data.split(';')
            RAW  = datasplit[0]
            KALMAN = datasplit[1]
            MA_filter = datasplit[2]
            hh_stat = datasplit[3]
            ll_stat = datasplit[4] 
            with open ("/home/aryawisnu/datapraktikumSII.csv","a") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow([timee, RAW, KALMAN, MA_filter, hh_stat, ll_stat])
            # time.sleep(1)