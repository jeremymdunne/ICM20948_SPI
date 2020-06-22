#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <Arduino.h> 
#include <SPI.h> 
#include <ICM20948_defs.h>

class ICM20948{
public: 
    /**
     * Struct to organize data from sensor 
     */ 
    struct ICM20948_raw_data{
        float gyro[3]; 
        float accel[3]; 
        float mag[3]; 
        float temperature; 
        unsigned long time_stamp; 
    }; 

    
    /**
     * gyro range select 
     */
    enum ICM20948_GYRO_FS_SEL{
        ICM20948_GYRO_250_DPS = 0x00,
        ICM20948_GYRO_500_DPS = 0x01, 
        ICM20948_GYRO_1000_DPS = 0x02,
        ICM20948_GYRO_2000_DPS = 0x03  
    };

    /**
     *  accel range select 
     */ 
    enum ICM20948_ACCEL_FS_SEL{ 
        ICM20948_ACCEL_2_G = 0x00,
        ICM20948_ACCEL_4_G = 0x01,
        ICM20948_ACCEL_8_G = 0x02,
        ICM20948_ACCEL_16_G = 0x03
    }; 

    // functions 

    /**
     * initialize the sensor and check connections 
     * @param cs_pin pin the cs is attatched to 
     * @return 0 for success, -1 for failure 
     */ 
    int begin(int cs_pin, ICM20948_ACCEL_FS_SEL accel_scale = ICM20948_ACCEL_2_G, ICM20948_GYRO_FS_SEL gyro_scale = ICM20948_GYRO_250_DPS ); 

    
    /**
     * update the sensor and return data 
     * @return 0 for success, -1 for failure 
     */ 
    int get_data(ICM20948_raw_data *data); 


    // scaling/calibration data 
    /**
     * set the mag scale for each axis 
     * @param scale pointer to three values between 0 and 1 
     */ 
    void set_mag_scale(float *scale); 

    /**
     * set the mag offset for each axis 
     * @param offset pointer to three offset values
     */ 
    void set_mag_offset(float *offset); 

    /**
     * set the accel offset for each axis 
     * @param offset pointer to three offset values
     */ 
    void set_accel_offset(float *offset); 

    

private: 
    // private variables 
    ICM20948_raw_data raw_data; 
    uint8_t cur_bank; // bank selector, this must be initialized properly 
    int cs_pin; 
    SPISettings spi_settings; 
    int sensor_data_length; // used to account for extra sensors 
    // scale and offset data 
    float gyro_scale; 
    float gyro_offset[3] = {0}; 
    float accel_scale; 
    float accel_offset[3] = {0}; // in G 
    float mag_resolution_scale = 4912;  
    float mag_scale[3] = {1,1,1}; 
    float mag_offset[3] = {0}; 
    float temp_scale = 0.00299517776f; // per the data sheet 
    float temp_offset = 21; // per the data sheet 



    // functions 


    /**
     * initialize the magnetometer 
     * @return 0 for success, -1 for failure 
     */ 
    int init_magnetometer(); 


    /**
     * update all sensor data 
     * @return 0 for success, -1 for failure
     */ 
    int update_sensors();

    /**
     * disable the I2C mode on the chip
     * @return 0 for success, -1 for failure
     */ 
    int disable_I2C(); 

    /**
     * scale the gyro data 
     * @param data raw data from the device 
     * @return 0 for success, -1 for failure 
     */ 
    int scale_gyro_data(int16_t *data); 

    /**
     * scale the accel data 
     * @param data raw data from the device 
     * @return 0 for success, -1 for failure 
     */
    int scale_accel_data(int16_t *data); 

    /**
     * scale the temperature data 
     * @param data raw data from the device 
     * @return 0 for sucess, -1 for failure 
     */ 
    int scale_temp_data(int16_t *data); 

    /**
     * read all sensor data 
     * @return 0 for success 
     */ 
    int read_sensor_data(uint8_t *buffer, uint8_t len); 

    /**
     * set the gyro full scale select 
     * @param scale gyro scale 
     * @return 0 for success
     */ 
    int set_gyro_scale(ICM20948_GYRO_FS_SEL scale); 

    /**
     * set the accel full scale select 
     * @param scale accel scale 
     * @return 0 for success
     */ 
    int set_accel_scale(ICM20948_ACCEL_FS_SEL scale); 

    /**
     * enable all sensors (accel, gyro, temp)
     * @return 0 for success
     */ 
    int enable_accel_gyro_temp();

    /** enable the accel and gyro sensors 
     * @return 0 for success 
     */ 
    int enable_accel_gyro(); 

    /** enable the temperature sensor 
     * @return 0 for success 
     */ 
    int enable_temp(); 

    /**
     * check the chip id 
     * currently doesn't care the value, just that it has one 
     * @return 0 for success, -1 for failure 
     */ 
    int check_chip_id(); 

    /**
     * reset the chip 
     * @return 0 for success 
     */ 
    int reset_chip(); 

    /**
     * wake the chip up from sleep 
     * @return 0 for success 
     */ 
    int wake_chip(); 

    /**
     * set the clock source for the chip 
     * default to the suggested mode per the data sheet 
     * @return 0 for success 
     */ 
    int set_clock_source(); 


    /**
     * set the register bank
     * @param bank desired bank to switch to 
     * @param override option to force writing
     * @return 0 for success, -1 for error
     */ 
    int set_bank(uint8_t bank, bool override = false); 

    /**
     * simple read function 
     * @param reg register to read from 
     * @param buffer buffer to read into 
     * @param len number of bytes to read 
     * @return 0 for success, -1 for failure 
     */ 
    int read_reg(uint8_t reg, uint8_t *buffer, uint8_t len); 

    /**
     * simple write function 
     * @param reg register to write to 
     * @param value value to write 
     * @return 0 for success, -1 for failure 
     */ 
    int write_reg(uint8_t reg, uint8_t value); 


    /**
     * scale the magnetometer data 
     * @return 0 for success, -1 for failure 
     */ 
    int scale_mag_data(int16_t *data);

    /**
     * read from the attatched ak09916 magnetometer
     * @param reg register to read from  
     * @return value read 
     */ 
    uint8_t read_ak(uint8_t reg); 


    /**
     * write to the attatched ak09916 magnetometer
     * @param reg register to write to  
     * @param value value to write 
     * @return 0 for success, -1 for failure 
     */ 
    int write_ak(uint8_t reg, uint8_t value); 

    
    /**
     * set the mag to a write state 
     */ 
    void set_mag_write(); 

    /**
     * set the mag to a read state 
     */ 
    void set_mag_read(); 

    /**
     * enable I2C master mode 
     */ 
    void enable_i2c_master(); 

    /**
     * set the slave I2C frequency 
     * currently maintain a 400 khz rate (mag rate)
     */ 
    void set_i2c_bus_frequency(); 

    /**
     * reset the mag 
     * @return 0 for success, -1 for failure
     */ 
    int reset_mag(); 


}; 


#endif 