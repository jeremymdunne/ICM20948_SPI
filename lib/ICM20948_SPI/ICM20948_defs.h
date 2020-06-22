#ifndef _ICM20948_DEFS_H_
#define _ICM20948_DEFS_H_

#define ICM20948_MAGNETOMETER_ADDR              0x0C

// Register definitions per the datasheet 
// Bank 0
#define ICM20948_WHO_AM_I_REG                   (0x00)
#define ICM20948_USER_CTRL_REG                  (0x03)
#define ICM20948_LP_CONFIG_REG                  (0x05)
#define ICM20948_PWR_MGMT_1_REG                 (0x06)
#define ICM20948_PWR_MGMT_2_REG                 (0x07)
#define ICM20948_INT_PIN_CFG_REG                (0x0F)
#define ICM20948_INT_ENABLE_REG                 (0x01)
#define ICM20948_INT_ENABLE_1_REG               (0x11)
#define ICM20948_INT_ENABLE_2_REG               (0x12)
#define ICM20948_INT_ENABLE_3_REG               (0x13)
#define ICM20948_I2C_MST_STATUE_REG             (0x17)
#define ICM20948_INT_STATUS_REG                 (0x19)
#define ICM20948_INT_STATUS_1_REG               (0x1A)
#define ICM20948_INT_STATUS_2_REG               (0x1B)
#define ICM20948_INT_STATUS_3_REG               (0x1C)
#define ICM20948_DELAY_TIMEH_REG                (0x28)
#define ICM20948_ACCEL_XOUT_H_REG               (0x2D)
#define ICM20948_EXT_SLV_SENS_DATA_00_REG       (0x3B)
#define ICM20948_FIFO_EN_1_REG                  (0x66)
#define ICM20948_FIFO_EN_2_REG                  (0x67)
#define ICM20948_FIFO_RST_REG                   (0x68)
#define ICM20948_FIFO_MODE_REG                  (0x69)
#define ICM20948_FIFO_COUNTH_REG                (0x70)
#define ICM20948_FIFO_R_W_REG                   (0x72)
#define ICM20948_DATA_RDY_STATUS_REG            (0x74)
#define ICM20948_FIFO_CFG_REG                   (0x76)
#define ICM20948_REG_BANK_SEL                   (0x7F)


// Bank 1
#define ICM20948_SELF_TEST_X_GYRO_REG           (0x02)
#define ICM20948_SELF_TEST_Y_GYRO_REG           (0x03)
#define ICM20948_SLEF_TEST_Z_GYRO_REG           (0x04)
#define ICM20948_SELF_TEST_X_ACCEL_REG          (0x0E)
#define ICM20948_SELF_TEST_Y_ACCEL_REG          (0x0F)
#define ICM20948_SELF_TEST_Z_ACCEL_REG          (0x10)
#define ICM20948_XA_OFFS_H_REG                  (0x14)
#define ICM20948_TIMEBASE_CORRECTION_PLL_REG    (0x28)

// Bank 2
#define ICM20948_GYRO_SMPLRT_DIV_REG            (0x00)
#define ICM20948_GYRO_CONFIG_1_REG              (0x01)
#define ICM20948_GYRO_CONFIG_2_REG              (0x02)
#define ICM20948_XG_OFFS_USRH_REG               (0x03)
#define ICM20948_ODR_ALGIN_EN_REG               (0x09)
#define ICM20948_ACCEL_SMPLRT_DIV_1_REG         (0x10)
#define ICM20948_ACCEL_SMPLRT_DIV_2_REG         (0x11)
#define ICM20948_ACCEL_INTEL_CTRL_REG           (0x12)
#define ICM20948_ACCEL_WOM_THR_REG              (0x13)
#define ICM20948_ACCEL_CONFIG_REG               (0x14)
#define ICM20948_ACCEL_CONFIG_2_REG             (0x15)
#define ICM20948_FSYNC_CONFIG_REG               (0x52)
#define ICM20948_TEMP_CONFIG_REG                (0x53)
#define ICM20948_MOD_CTRL_USR_REG               (0x54)

// Bank 3
#define ICM20948_I2C_MST_ODR_CONFIG_REG         (0x00)
#define ICM20948_I2C_MST_CTRL_REG               (0x01)
#define ICM20948_I2C_MST_DELAY_CTRL_REG         (0x02)
#define ICM20948_I2C_SLV0_ADDR_REG              (0x03)
#define ICM20948_I2C_SLV0_REG_REG               (0x04)
#define ICM20948_I2C_SLV0_CTRL_REG              (0x05)
#define ICM20948_I2C_SLV0_DO_REG                (0x06)




// bank IDs for the program 
#define ICM20948_BANK_0                         (0)
#define ICM20948_BANK_1                         (1)
#define ICM20948_BANK_2                         (2)
#define ICM20948_BANK_3                         (3)



// AK09916 Registers 
#define AK09916_DEVICE_ID_REG                   (0x01)
#define AK09916_STATUS_REG                      (0x10)
#define AK09916_HXL_REG                         (0x11)
#define AK09916_STATUS_2_REG                    (0x18)
#define AK09916_CONTROL_2_REG                   (0x31)
#define AK09916_CONTROL_3_REG                   (0x32)
#define AK09916_TEST_1_REG                      (0x33)
#define AK09916_TEST_2_REG                      (0x34)



#endif 