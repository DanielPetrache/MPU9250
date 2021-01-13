#MPU9250 used registers
mpu_adresa = 0x68
magne_adresa = 0x0C

smplrt_div = 0x19
config = 0x1A
gyro_config = 0x1B
accel_config = 0x1C
accel_config2 = 0x1D
i2c_mst_ctrl = 0x24
i2c_slv4_ctrl = 0x34
i2c_slv0_addr = 0x25
i2c_slv0_reg = 0x26
i2c_slv0_do = 0x63
i2c_slv0_ctrl = 0x27
int_pin_cfg = 0x37
int_enable = 0x38
int_status = 0x3A
accel_out = 0x3B
gyro_out = 0x43
ext_sens_data_00 = 0x49
i2c_mst_delay_ctrl = 0x67
user_ctrl = 0x6A
pwr_mgmt1 = 0x6B
pwr_mgmt2 = 0x6c

#registrii magne
magne_st1 = 0x02
magne_out = 0x03
magne_cntl1 = 0x0A
magne_cntl2 = 0x0B
magne_asax = 0x10

#scale accelerometru
afs_2g = 0x00
afs_4g = 0x01
afs_8g = 0x02
afs_16g = 0x03

#scale giroscop
gfs_250 = 0x00
gfs_500 = 0x01
gfs_1000 = 0x02
gfs_2000 = 0x03

#scale magnetometru
magne_14b = 0x00
magne_16b = 0x01
