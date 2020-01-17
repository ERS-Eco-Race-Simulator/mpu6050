from mpu6050 import mpu6050
from threading import Thread
import time, math

class Accel():

    def __init__(self, mpu, dt = 0.05, calibration_meas = 10000, out = True):

        self.DT = dt
        self.CALIBRATION_MEAS = calibration_meas
        self.MPU = mpu
        self.OUT = out
        
        self.COLLECTING_DATA = False

        self.err = (0, 0, 0)
        self.speeds = (0, 0, 0)
        self.speed = 0

    def calibration(self):
        
        errors = []
        start = time.time()
        
        for i in range(self.CALIBRATION_MEAS):
            if self.OUT: print(f'Calibration {i*100/self.CALIBRATION_MEAS}%', end='\r')
            data = self.MPU.get_accel_data()
            errors.append((data['x'], data['y'], data['z']))

        ex, ey, ez = 0, 0, 0
        for x, y, z in errors:
            ex += x
            ey += y
            ez += z  

        ex /= len(errors)
        ey /= len(errors)
        ez /= len(errors)

        self.err = (ex, ey, ez)
        if self.OUT: print(f'Calibration (x: {ex}, y: {ey}, z: {ez}) done in {round(time.time() - start, 2)}s')
        
    def start(self):
        self.COLLECTING_DATA = True
        self.collector = Thread(target=self.collect_data)
        self.collector.start()
    
    def stop(self):
        self.COLLECTING_DATA = False
        self.collector.join()

    def collect_data(self):
        while self.COLLECTING_DATA:
            data = self.MPU.get_accel_data()
            speeds = [ v + (a - e) * self.DT for v, a, e in zip(list(self.speeds), [data['x'], data['y'], data['z']], list(self.err)) ]
            self.speed = math.sqrt( data['y']**2 + math.sqrt( data['x']**2 + data['y']**2 )**2 )
            self.speeds = tuple(speeds)
            time.sleep(self.DT)

if __name__ == '__main__':

    mpu = mpu6050(0x68)
    acc = Accel(mpu)

    acc.calibration()
    acc.start()
    time.sleep(5)
    acc.stop()
    
    print(acc.speed)