import smbus
from time import sleep
from registriiMPU9250 import *

bus = smbus.SMBus(1)


class IMU:
    def __init__(self, address=mpu_adresa):
        self.address = address

    def config_moduri(self, afs, gfs, mfs):
        if afs == afs_2g:
            self.ares = 2.0 / 32768.0
        elif afs == afs_4g:
            self.ares = 4.0 / 32768.0
        elif afs == afs_8g:
            self.ares = 8.0 / 32768.0
        elif afs == afs_16g:
            self.ares = 16.0 / 32768.0

        if gfs == gfs_250:
            self.gres = 250.0 / 32768.0
        elif afs == gfs_500:
            self.gres = 500.0 / 32768.0
        elif afs == gfs_1000:
            self.gres = 1000.0 / 32768.0
        elif afs == gfs_2000:
            self.gres = 2000.0 / 32768.0

        if mfs == magne_14b:
            self.mres = 4912.0 / 8190.0
        elif mfs == magne_16b:
            self.mres = 4912.0 / 32760.0

    def MPU_initializare(self):
        bus.write_byte_data(self.address, pwr_mgmt1, 0x00)
        sleep(0.1)
        bus.write_byte_data(self.address, pwr_mgmt1, 0x01)
        sleep(0.2)
        bus.write_byte_data(self.address, config, 0x03)
        bus.write_byte_data(self.address, smplrt_div, 0x00)
        bus.write_byte_data(self.address, gyro_config, 0x18)
        bus.write_byte_data(self.address, accel_config, 0x18)
        bus.write_byte_data(self.address, accel_config2, 0x03)
        bus.write_byte_data(self.address, int_pin_cfg, 0x10)
        bus.write_byte_data(self.address, int_enable, 0x01)
        sleep(0.1)
        bus.write_byte_data(self.address, user_ctrl, 0x20)
        bus.write_byte_data(self.address, i2c_mst_ctrl, 0x1D)
        bus.write_byte_data(self.address, i2c_mst_delay_ctrl, 0x81)
        bus.write_byte_data(self.address, i2c_slv4_ctrl, 0x01)
        sleep(0.1)

    def initMagne(self):
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl2)
        bus.write_byte_data(self.address, i2c_slv0_do, 0x01)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl1)
        bus.write_byte_data(self.address, i2c_slv0_do, 0x00)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl1)
        bus.write_byte_data(self.address, i2c_slv0_do, 0x0F)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa | 0x80)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_asax)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x83)
        sleep(0.05)
        data = bus.read_i2c_block_data(self.address, ext_sens_data_00, 3)
        self.mXcoef = (data[0] - 128.0) / 256.0 + 1
        self.mYcoef = (data[1] - 128.0) / 256.0 + 1
        self.mZcoef = (data[2] - 128.0) / 256.0 + 1
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl1)
        bus.write_byte_data(self.address, i2c_slv0_do, 0x00)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl1)
        bus.write_byte_data(self.address, i2c_slv0_do, 0x16)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa | 0x80)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_cntl1)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x81)
        sleep(0.05)

    def reset(self):
        bus.write_byte_data(self.address, pwr_mgmt1, 0x80)

    def check_newdata(self):
        data_rdy = bus.read_byte_data(self.address, int_status)
        while data_rdy == 0:
            data_rdy = bus.read_byte_data(self.address, int_status)

    def citire_accel(self):
        data = bus.read_i2c_block_data(self.address, accel_out, 6)
        ax = self.concatenare(data[1], data[0])
        ay = self.concatenare(data[3], data[2])
        az = self.concatenare(data[5], data[4])
        ax = round(ax * self.ares, 7)
        ay = round(ay * self.ares, 7)
        az = round(az * self.ares, 7)
        return 9.8 * ax, 9.8 * ay, 9.8 * az

    def citire_gyro(self):
        data = bus.read_i2c_block_data(self.address, gyro_out, 6)
        gx = self.concatenare(data[1], data[0])
        gy = self.concatenare(data[3], data[2])
        gz = self.concatenare(data[5], data[4])
        gx = round(gx * self.gres, 7)
        gy = round(gy * self.gres, 7)
        gz = round(gz * self.gres, 7)
        return gx, gy, gz

    def citire_magne(self):
        bus.write_byte_data(self.address, i2c_slv0_addr, magne_adresa | 0x80)
        bus.write_byte_data(self.address, i2c_slv0_reg, magne_out)
        bus.write_byte_data(self.address, i2c_slv0_ctrl, 0x87)

        while (True):
            data = bus.read_i2c_block_data(self.address, ext_sens_data_00, 7)
            if data[6] == 16:
                mx = self.concatenare(data[0], data[1])
                my = self.concatenare(data[2], data[3])
                mz = self.concatenare(data[4], data[5])

                mx = mx * self.mres * self.mXcoef
                my = my * self.mres * self.mYcoef
                mz = mz * self.mres * self.mZcoef
                return mx, my, mz
            else:
                continue

    def concatenare(self, data1, data2):
        val = data1 | (data2 << 8)
        if (val & (1 << 16 - 1)):
            val -= (1 << 16)
        return val

    def calibrare_accel(self):
        print("Calibrare accelerometru")
        offsetX, offsetY, offsetZ = 0, 0, 0
        i = 0
        while i != 500:
            ax, ay, az = self.citire_accel()
            offsetX += ax
            offsetY += ay
            offsetZ += az
            i += 1
        return offsetX / 500, offsetY / 500, offsetZ / 500

    def calibrare_gyro(self):
        print("Calibrare giroscop")
        offsetX, offsetY, offsetZ = 0, 0, 0
        i = 0
        while i != 500:
            gx, gy, gz = self.citire_gyro()
            offsetX += gx
            offsetY += gy
            offsetZ += gz
            i += 1
        return offsetX / 500, offsetY / 500, offsetZ / 500

    def calibrare_magne(self):
        print("Calibrare magnetometru")
        sleep(2)
        print("Roteste in jurul lui Z")
        offsetX, offsetY, offsetZ = 0, 0, 0
        i = 60000
        mx, my, mz = self.citire_magne()
        sleep(0.5)
        mx, my, mz = self.citire_magne()  # citesc de doua ori cu pauza intre deoarece uneori este overflow; probabil de la transferul la master???
        maxX = minX = mx
        maxY = minY = my
        maxZ = minZ = mz
        sleep(0.2)
        print(mx, my, mz)
        while i > 1:
            if i == 39999:
                print("Roteste in jurul lui Y")
                sleep(0.5)
            if i == 19999:
                print("Roteste in jurul lui x")
                sleep(0.5)
            mx, my, mz = self.citire_magne()
            minX = min(minX, mx)
            maxX = max(maxX, mx)
            minY = min(minY, my)
            maxY = max(maxY, my)
            minZ = min(minZ, mz)
            maxZ = max(maxZ, mz)
            i -= 1

        offsetX = (minX + maxX) / 2
        offsetY = (minY + maxY) / 2
        offsetZ = (minZ + maxZ) / 2
        x = (maxX - minX) / 2
        y = (maxY - minY) / 2
        z = (maxZ - minZ) / 2
        avg = (x + y + z) / 3
        scaleX = avg / x
        scaleY = avg / y
        scaleZ = avg / z
        print(offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ)
        return offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ
