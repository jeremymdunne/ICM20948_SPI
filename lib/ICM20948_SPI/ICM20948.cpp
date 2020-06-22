#include <ICM20948.h> 

//#define ICM20948_DEBUG 
//#define ICM20948_DEBUG_CS PA9

int ICM20948::begin(int cs_pin, ICM20948_ACCEL_FS_SEL accel_range, ICM20948_GYRO_FS_SEL gyro_range){
    // initialize the sensor 
    this->cs_pin = cs_pin; 
    pinMode(cs_pin, OUTPUT); 
    digitalWrite(cs_pin, HIGH); 
    #ifdef ICM20948_DEBUG_CS
        pinMode(ICM20948_DEBUG_CS, OUTPUT); 
        digitalWrite(ICM20948_DEBUG_CS, HIGH); 
    #endif 

    spi_settings = SPISettings(2000000, MSBFIRST, SPI_MODE0); 
    /* order of operations: 
    Check chip ID 
    Reset the chip 
    Check for errors 
    Initialize base settings 
    */   
    // reset the chip 
    uint8_t status = check_chip_id(); 
    if(status < 0){
        #ifdef ICM20948_DEBUG
            Serial.println("ICM20948 ERROR: CHIP ID FAILED"); 
        #endif 
        return status; 
    }
    reset_chip(); 
    // disable I2C 
    disable_I2C(); 
    // check_chip_id();
    wake_chip();
    // set the base settings 
    // enable_accel_gyro_temp(); 
    set_accel_scale(accel_range); 
    set_gyro_scale(gyro_range); 
    set_clock_source();
    // init the ak 
    init_magnetometer(); 
    delay(10); 
    set_bank(ICM20948_BANK_0); 

   return 0; 
}

void ICM20948::set_mag_scale(float *scale){
    // set the scales 
    memcpy(&mag_scale, scale, sizeof(float)*3); 
}

void ICM20948::set_mag_offset(float *offset){
    // set the offset 
    memcpy(&mag_offset, offset, sizeof(float)*3); 
}

void ICM20948::set_accel_offset(float *offset){
    // set the offset 
    memcpy(&accel_offset, offset, sizeof(float)*3); 
}


int ICM20948::disable_I2C(){
    uint8_t disable; 
    read_reg(ICM20948_USER_CTRL_REG, &disable, 1); 
    //scrub it 
    disable &= 0b11101111;
    disable |= 1 << 4; 
    write_reg(ICM20948_USER_CTRL_REG, disable); 
    return 0; 
}

int ICM20948::get_data(ICM20948::ICM20948_raw_data *data){
    // get new data 
    update_sensors();
    // copy over the data struct 
    memcpy(data,&raw_data,sizeof(raw_data)); 
    return 0; 
}

int ICM20948::update_sensors(){
    // get the sensor data 
    uint8_t buffer[20]; 
    read_sensor_data(buffer, 20); 
    // snag the time 
    raw_data.time_stamp = millis(); 
    // combine the data and compensate 
    int16_t temp_buffer[10]; 
    for(int i = 0; i < 7; i ++){
        temp_buffer[i] = (int16_t)(buffer[i*2] << 8 | buffer[i*2 + 1]); 
    }
    // mag data is LSB for some reason... 
    for(int i = 0; i < 3; i ++){
        temp_buffer[7+i] = (int16_t)(buffer[i*2 + 14 + 1] << 8 | buffer[i*2 + 14]); 
    }
    // scale and convert the data 
    scale_accel_data(temp_buffer);
    scale_gyro_data(&temp_buffer[3]); 
    scale_temp_data(&temp_buffer[6]); 
    scale_mag_data(&temp_buffer[7]); 
    // report the results 
    #ifdef ICM20948_DEBUG
        Serial.println("Gyro: " + String(raw_data.gyro[0]) + "; " + String(raw_data.gyro[1]) + "; " + String(raw_data.gyro[2]));
        Serial.println("Accel: " + String(raw_data.accel[0]) + "; " + String(raw_data.accel[1]) + "; " + String(raw_data.accel[2]));
        Serial.println("Temp: " + String(raw_data.temperature)); 
        Serial.println("Mag: " + String(raw_data.mag[0]) + "; " + String(raw_data.mag[1]) + "; " + String(raw_data.mag[2]));
    #endif 
    return 0; 
}

int ICM20948::scale_gyro_data(int16_t *data){
    for(int i = 0; i < 3; i ++){
        raw_data.gyro[i] = data[i] * gyro_scale + gyro_offset[i]; 
    }
    return 0; 
}

int ICM20948::scale_accel_data(int16_t *data){
    for(int i = 0; i < 3; i ++){
        raw_data.accel[i] = data[i] * accel_scale + accel_offset[i]; 
    }
    return 0; 
}

int ICM20948::scale_temp_data(int16_t *data){
    raw_data.temperature = (*data - 21) * temp_scale + temp_offset; 
    return 0; 
}

int ICM20948::read_sensor_data(uint8_t *buffer, uint8_t len){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 read_all_sensor_data function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // call a read on the appropriate data 
    // TODO add an appropriate DRDY check here 
    return read_reg(ICM20948_ACCEL_XOUT_H_REG, buffer, len);    
}

int ICM20948::set_gyro_scale(ICM20948_GYRO_FS_SEL scale){ 
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 set_gyro_scale function call"); 
    #endif 
    set_bank(ICM20948_BANK_2);
    // set by the GYRO_FS_SEL bits per the data sheet 
    uint8_t current; 
    read_reg(ICM20948_GYRO_CONFIG_1_REG, &current, 1); 
    // scrub the value 
    current &= 0b11111001; 
    current |= scale << 1;  
    // write it 
    write_reg(ICM20948_GYRO_CONFIG_1_REG, current); 
    // set the scale used for calculations 
    switch(scale){
        case(ICM20948_GYRO_250_DPS):
            gyro_scale = 1/131.0; 
            break; 
        case(ICM20948_GYRO_500_DPS):
            gyro_scale = 1/65.5; 
            break; 
        case(ICM20948_GYRO_1000_DPS):
            gyro_scale = 1/32.8; 
            break; 
        case(ICM20948_GYRO_2000_DPS):
            gyro_scale = 1/15.4; 
            break; 
    }
    read_reg(ICM20948_GYRO_CONFIG_1_REG, &current, 1); 
    return 0; 
}


int ICM20948::set_accel_scale(ICM20948_ACCEL_FS_SEL scale){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 set_accel_scale function call"); 
    #endif 
    set_bank(ICM20948_BANK_2);
    // set by the GYRO_FS_SEL bits per the data sheet 
    uint8_t current; 
    read_reg(ICM20948_ACCEL_CONFIG_REG, &current, 1); 
    // scrub the value 
    current &= 0b11111001; 
    current |= scale << 1;  
    // write it 
    write_reg(ICM20948_ACCEL_CONFIG_REG, current); 
    // set the scale used for calculations 
    switch(scale){
        case(ICM20948_ACCEL_2_G):
            accel_scale = 1/16384.0; 
            break; 
        case(ICM20948_ACCEL_4_G):
            accel_scale = 1/8192.0; 
            break; 
        case(ICM20948_ACCEL_8_G):
            accel_scale = 1/4096.0; 
            break; 
        case(ICM20948_ACCEL_16_G):
            accel_scale = 1/2048.0; 
            break; 
    }

    read_reg(ICM20948_ACCEL_CONFIG_REG, &current, 1); 
    
    return 0; 
}

int ICM20948::enable_accel_gyro_temp(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 enable_accel_gyro_temp function call"); 
    #endif 
    enable_temp();
    return enable_accel_gyro(); 
}

int ICM20948::enable_temp(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 enable_temp function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // enable by clearnign the TEMP_DIS bit per the data sheet 
    uint8_t current; 
    read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    // scrub the value 
    current &= 0b11110111; 
    return write_reg(ICM20948_PWR_MGMT_1_REG, current); 
}

int ICM20948::enable_accel_gyro(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 enable_accel_gyro function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // enable by the PWR_MGMT_2 register per the data sheet 
    uint8_t current; 
    read_reg(ICM20948_PWR_MGMT_2_REG, &current, 1); 
    // scrub the value 
    current &= 0b11000000;  
    return write_reg(ICM20948_PWR_MGMT_2_REG, current); 
}


int ICM20948::check_chip_id(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 check_chip_id function call"); 
    #endif 
    set_bank(ICM20948_BANK_0, true); // force the bank set as this is usually the first write performed to the chip 
    // get the chip id 
    uint8_t chip_id; 
    int status = read_reg(ICM20948_WHO_AM_I_REG,&chip_id,1); 
    if(status < 0){
        #ifdef ICM20948_DEBUG
            Serial.println("ICM20948 READ FAILED"); 
        #endif 
        return status; 
    }
    else if(chip_id != 0 && chip_id != 255){
        return 0; 
    }
    return -1; 
}

int ICM20948::reset_chip(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 reset_chip function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // reset by setting the DEVICE_RESET bit  
    uint8_t current; 
    read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    // scrub the value 
    current &= 0b01111111; 
    current |= 0b10000000; 
    write_reg(ICM20948_PWR_MGMT_1_REG, current); 
    //read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1);
    // wait a bit 
    delay(25); 
    // wait until that bit is cleared or too much time passes  
    read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    unsigned long start = millis(); 
    while((current & 0b10000000) != 0 && (millis() - start < 1000)){
        delay(1); 
        read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    }
    // check if the bit has been cleared 
    if((current & 0b10000000) != 0){
        // otherwise, assume a timeout 
        #ifdef ICM20948_DEBUG
            Serial.println("ICM20948 RESET TIMEOUT"); 
        #endif 
        return -1; 
    }
    return 0;
}

int ICM20948::wake_chip(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 wake_chip function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // wake the chip by clearing the SLEEP bit 
    uint8_t current; 
    read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    // scrub the value 
    current &= 0b10111111; 
    // current |= 0b01000000; 
    return write_reg(ICM20948_PWR_MGMT_1_REG, current); 
}

int ICM20948::set_clock_source(){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 set_clock_source function call"); 
    #endif 
    set_bank(ICM20948_BANK_0);
    // set the clock to the default 
    // auto selection (1 - 5) in bit 2:0 per the data sheet 
    uint8_t current; 
    read_reg(ICM20948_PWR_MGMT_1_REG, &current, 1); 
    // scrub the value 
    current &= 0b11111000; 
    current |= 1; 
    return write_reg(ICM20948_PWR_MGMT_1_REG, current); 
}

int ICM20948::set_bank(uint8_t bank, bool override){
    #ifdef ICM20948_DEBUG
        Serial.println("ICM20948 set_bank function call"); 
    #endif 
    // check for an override 
    if(override | cur_bank != bank){
        // write anyways 
        int status = write_reg(ICM20948_REG_BANK_SEL, bank<<4); 
        if(status < 0){
            #ifdef ICM20948_DEBUG
                Serial.println("ICM20948 WRITE FAILED"); 
            #endif 
            return status; 
        }
        cur_bank = bank;
        
    }
    return 0;
}

int ICM20948::read_reg(uint8_t reg, uint8_t *buffer, uint8_t len){
    #ifdef ICM20948_DEBUG 
        // print out the register reading from 
        Serial.print("\tICM20948 SPI$"); 
        Serial.print(reg, HEX); 
        Serial.print(": <== "); 
    #endif 
    reg = reg | 0x80; 

    SPI.beginTransaction(spi_settings); 

    digitalWrite(cs_pin, LOW); 
    #ifdef ICM20948_DEBUG_CS
        digitalWrite(ICM20948_DEBUG_CS, LOW);
    #endif 


    //delay(1);
    // send the address 
    SPI.transfer(reg); 
    while(len--){
        //delay(1);
        *buffer = SPI.transfer(0x00); 
        #ifdef ICM20948_DEBUG
            Serial.print(*buffer,HEX); 
            Serial.print(","); 
        #endif 
        buffer ++; 
    }
    digitalWrite(cs_pin, HIGH); 
    #ifdef ICM20948_DEBUG_CS
        digitalWrite(ICM20948_DEBUG_CS, HIGH);
    #endif 
    SPI.endTransaction(); 
    #ifdef ICM20948_DEBUG
        Serial.print("\n"); 
    #endif 
    return len; 
}

int ICM20948::write_reg(uint8_t reg, uint8_t value){
    #ifdef ICM20948_DEBUG 
        // print out the register reading from 
        Serial.print("\tICM20948 SPI$"); 
        Serial.print(reg, HEX); 
        Serial.print(": ==> "); 
        Serial.print(value, HEX); 
        Serial.print("\n"); 
    #endif 

    SPI.beginTransaction(spi_settings);  
    digitalWrite(cs_pin, LOW); 
    #ifdef ICM20948_DEBUG_CS
        digitalWrite(ICM20948_DEBUG_CS, LOW);
    #endif
    // send the address 
    SPI.transfer(reg); 
    SPI.transfer(value); 

    digitalWrite(cs_pin, HIGH); 
    #ifdef ICM20948_DEBUG_CS
        digitalWrite(ICM20948_DEBUG_CS, HIGH);
    #endif 
    SPI.endTransaction(); 
    return 0; 
}

// AK09916 Magnetometer Functions 

int ICM20948::init_magnetometer(){
    #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 init_magnetometer function call"); 
    #endif 
    // go through the initalizing of the magnetometer 
    // first, set up the necessary communication lanes 
    // isolate the communication 
    set_i2c_bus_frequency(); 
    // do this, idk 
    set_bank(ICM20948_BANK_0); 
    write_reg(ICM20948_INT_PIN_CFG_REG, 0x02); 
    enable_i2c_master(); 
    // set the slave address
    uint8_t slv0_addr = ICM20948_MAGNETOMETER_ADDR; 
    // write to I2C_SLV0_ADDR
    set_bank(ICM20948_BANK_3); 
    write_reg(ICM20948_I2C_SLV0_ADDR_REG, slv0_addr); 
    // attempt to read the device ID 
    uint8_t ak_id = read_ak(0x01); 
    Serial.println("AK ID: " + String(ak_id)); 
    if(ak_id == 0 || ak_id == 255){
        // something went wrong 
        #ifdef ICM20948_DEBUG 
            Serial.println("ICM20948 ERROR, MAG INCORRECT ID"); 
        #endif 
        return -1; 
    }
    // try to reset the mag 
    set_bank(ICM20948_BANK_3); 
    reset_mag(); 
    delay(100); 
    
    // set it to a normal run mode 
    write_ak(AK09916_CONTROL_2_REG, 0b01000); // set to normal measurement mode 4 
    set_mag_read(); 
    // set up the continuous read cycle 
    write_reg(ICM20948_I2C_SLV0_REG_REG, AK09916_HXL_REG); // set the start register 
    // set the length 
    write_reg(ICM20948_I2C_SLV0_CTRL_REG, 1 << 7 | 8); // set up to read 8 data bytes 
    delay(10); 
    return 0; 
}


int ICM20948::reset_mag(){
    #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 reset_mag function call"); 
    #endif 
    // tell the mag to reset 
    /*
    set_bank(ICM20948_BANK_0); 
    uint8_t base; 
    read_reg(ICM20948_USER_CTRL_REG, &base, 1);
    base &= 0b11111101;
    base |= 1 << 1;
    write_reg(ICM20948_USER_CTRL_REG, base);
    delay(1); 
    set_bank(ICM20948_BANK_0); 
    */
    write_ak(AK09916_CONTROL_3_REG, 1); // set the soft reset bit 
    delay(10); 
    return 0; // should get a better reset method... 
}

int ICM20948::scale_mag_data(int16_t *data){
     #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 scale_mag_data function call"); 
    #endif 
    for(int i = 0; i < 3; i ++){
        raw_data.mag[i] = data[i] / mag_resolution_scale; 
        raw_data.mag[i] *= mag_scale[i]; 
        raw_data.mag[i] += mag_offset[i]; 
    }
    return 0; 
}

void ICM20948::set_mag_write(){
    #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 set_mag_write function call"); 
    #endif 
    // read the SLVO_ADDR
    uint8_t base; 
    set_bank(ICM20948_BANK_3); 
    read_reg(ICM20948_I2C_SLV0_ADDR_REG, &base, 1); 
    // scrub it 
    base &= 0b01111111; 
    write_reg(ICM20948_I2C_SLV0_ADDR_REG, base); 
}

void ICM20948::set_mag_read(){
    #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 set_mag_read function call"); 
    #endif 
    // read the SLVO_ADDR
    uint8_t base; 
    set_bank(ICM20948_BANK_3); 
    read_reg(ICM20948_I2C_SLV0_ADDR_REG, &base, 1); 
    // scrub it 
    base &= 0b01111110;
    base |= 1<<7;   
    write_reg(ICM20948_I2C_SLV0_ADDR_REG, base); 
}

void ICM20948::set_i2c_bus_frequency(){
    #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 set_i2c_bus_frequency function call"); 
    #endif 
    set_bank(ICM20948_BANK_3); 
    write_reg(ICM20948_I2C_MST_CTRL_REG, 0x07);
}

void ICM20948::enable_i2c_master(){
     #ifdef ICM20948_DEBUG 
        Serial.println("ICM20948 enable_i2c_master function call"); 
    #endif 
    uint8_t base; 
    set_bank(ICM20948_BANK_0); 
    read_reg(ICM20948_USER_CTRL_REG, &base, 1); 
    //scrub it 
    base &= 0b11011111; 
    base |= 1 << 5; 
    write_reg(ICM20948_USER_CTRL_REG, base); 
}

uint8_t ICM20948::read_ak(uint8_t reg){
    #ifdef ICM20948_DEBUG 
        // print out the register reading from 
        Serial.print("\tICM20948 MAG READ$"); 
        Serial.print(reg, HEX); 
        Serial.print(": ==> "); 
        
    #endif 
    // read through the I2C passthrough
    // first, set up to read 
    set_mag_read(); 
    // set the start address 
    set_bank(ICM20948_BANK_3); 
    write_reg(ICM20948_I2C_SLV0_REG_REG, reg); 
    uint8_t value; 
    read_reg(ICM20948_I2C_SLV0_CTRL_REG, &value, 1);
    // scrub 
    value &= 0b01110000; 
    value |= 1; // only support a read len of 1 for now 
    value |= 1 << 7; 
    // write it 
    write_reg(ICM20948_I2C_SLV0_CTRL_REG, value); 
    delay(10); 
    // read the value 
    set_bank(ICM20948_BANK_0); 
    read_reg(ICM20948_EXT_SLV_SENS_DATA_00_REG, &value, 1); 
    #ifdef ICM20948_DEBUG 
        Serial.print(value, HEX); 
        Serial.print("\n"); 
    #endif 
    return value;  
}


int ICM20948::write_ak(uint8_t reg, uint8_t value){
    #ifdef ICM20948_DEBUG 
        // print out the register reading from 
        Serial.print("\tICM20948 MAG READ$"); 
        Serial.print(reg, HEX); 
        Serial.print(": ==> "); 
        Serial.print("\n"); 
    #endif 
    // read through the I2C passthrough
    // first, set up to read 
    set_bank(ICM20948_BANK_3); 
    set_mag_write(); 
    write_reg(ICM20948_I2C_SLV0_REG_REG, reg); 
    write_reg(ICM20948_I2C_SLV0_DO_REG, value); 
    write_reg(ICM20948_I2C_SLV0_CTRL_REG, 1 << 7 | 1); 
    return 0; 
}