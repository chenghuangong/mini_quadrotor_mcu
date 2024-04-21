#include "sensor.h"

sensor_bmp280::sensor_bmp280(i2c_communicator* communicator)
    : communicator_(communicator)
{
    init_sensor();
}


void sensor_bmp280::init_sensor()
{
    communicator_->default_i2c_init();  // init communicator
    init_bmp280();
    is_initialized_ = true;
}


void sensor_bmp280::init_bmp280()
{
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];
    // set sampling time 500ms
    // 地址为0xf5的config寄存器, 我们现在要用的有效bit位是bit7-bit2，所以使用1111 1100
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;  // 100 101 00 & 1111 1100, 100表示500ms
    buf[0] = static_cast<uint8_t>(BMP280_REG::REG_CONFIG);
    buf[1] = reg_config_val;
    i2c_write_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), buf, 2, false);

    // set oversampling control
    // osrs_t control oversampling of temperature data, osrs_4 control oversampling of pressure data
    // osrs_t x1, osrs_p x4, normal mode operation(power mode)
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = static_cast<uint8_t>(BMP280_REG::REG_CTRL_MEAS);
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), buf, 2, false);

    // get calibration parameters
    get_calib_params();
}


void sensor_bmp280::get_calib_params()
{
    uint8_t buf[static_cast<uint8_t>(BMP280_REG::NUM_CALIB_PARAMS)];
    uint8_t reg = static_cast<uint8_t>(BMP280_REG::REG_DIG_T1_LSB);

    i2c_write_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), &reg, 1, true);
    i2c_read_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), buf, static_cast<uint8_t>(BMP280_REG::NUM_CALIB_PARAMS), false);

    calib_param_.dig_t1 = (uint16_t)(buf[1] << 8) | buf[0]; // 温度校准参数
    calib_param_.dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    calib_param_.dig_t3 = (int16_t)(buf[5] << 8) | buf[4];
    calib_param_.dig_p1 = (uint16_t)(buf[7] << 8) | buf[6]; // 气压校准参数
    calib_param_.dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    calib_param_.dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    calib_param_.dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    calib_param_.dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    calib_param_.dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    calib_param_.dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    calib_param_.dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    calib_param_.dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}


double* sensor_bmp280::get_sensor_data()
{
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes
    // 0xfc是 temp_xlsb, 0xf7 press_msb, 这之间一共有六个数，我们可以一次性读取
    uint8_t buf[6];
    uint8_t reg = static_cast<uint8_t>(BMP280_REG::REG_PRESSURE_MSB);
    int32_t pressure = 0;
    int32_t temp = 0;
    
    // 读取寄存器时，需要写入起始读取的寄存器
    i2c_write_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), &reg, 1, true);
    i2c_read_blocking(communicator_->get_i2c_inst(), static_cast<uint8_t>(BMP280_REG::ADDR), buf, 6, false);

    pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    sensor_data[0] = convert_temp(temp) / 100.f;                  // temp
    sensor_data[1] = convert_pressure(pressure, temp) / 1000.f;   // pressure, unit kPa
    return sensor_data;
}


int32_t sensor_bmp280::convert(int32_t temp)
{
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)calib_param_.dig_t1 << 1))) * ((int32_t)calib_param_.dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)calib_param_.dig_t1)) * ((temp >> 4) - ((int32_t)calib_param_.dig_t1))) >> 12) * ((int32_t)calib_param_.dig_t3)) >> 14;
    return var1 + var2;
}

int32_t sensor_bmp280::convert_temp(int32_t temp) 
{
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = convert(temp);
    return (t_fine * 5 + 128) >> 8;
}

int32_t sensor_bmp280::convert_pressure(int32_t pressure, int32_t temp) 
{
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers
    int32_t t_fine = convert(temp);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_param_.dig_p6);
    var2 += ((var1 * ((int32_t)calib_param_.dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_param_.dig_p4) << 16);
    var1 = (((calib_param_.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_param_.dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_param_.dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)calib_param_.dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)calib_param_.dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + calib_param_.dig_p7) >> 4));
    return converted;
}


sensor_mpu6050::sensor_mpu6050(i2c_communicator* communicator)
    : communicator_(communicator)
{
    init_sensor();
}


void sensor_mpu6050::init_sensor()
{
    communicator_->default_i2c_init();  // init communicator
    i2c_inst_ = communicator_->get_i2c_inst();
    init_mpu6050();
    is_initialized_ = true;  
}

void sensor_mpu6050::init_mpu6050()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_inst_, MPU6050_ADDR, buf, 2, false);

    sleep_ms(100);

    mpu6050_set_low_pass_filter();
}




void sensor_mpu6050::mpu6050_set_low_pass_filter()
{
    // enable low pass filter, bandwidth = 10hz
    uint8_t buf1[] = {0x1A, 0x05};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf1, 2, false);

    // set sensitivity to 65.5
    // 256Hz
    uint8_t buf2[] = {0x1B, 0x08};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf2, 2, false);  
}


void sensor_mpu6050::mpu6050_read_raw() 
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_inst_, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_inst_, MPU6050_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel_[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_inst_, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_inst_, MPU6050_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro_[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_inst_, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_inst_, MPU6050_ADDR, buffer, 2, false);  // False - finished with bus

    temp_ = buffer[0] << 8 | buffer[1];
}

// before get data from sensor, should call this function
// used to sampling data, and apply kalman filter to it
void sensor_mpu6050::mpu6050_read_kalman()
{
    mpu6050_read_raw();

    if (!perform_zero_point_calibration())
    {
        return;
    }

    apply_kalman_filter();

    save_data_to_rate_control();
}

void sensor_mpu6050::apply_kalman_filter()
{
    // calculate angle by acc value
    kalman_filter_.acc_angle[0] = atan(accel_[1] / pow(accel_[0] * accel_[0] + accel_[2] * accel_[2], 0.5)) * (180 / M_PI);
    kalman_filter_.acc_angle[1] = atan(-accel_[0] / pow(accel_[1] * accel_[1] + accel_[2] * accel_[2], 0.5)) * (180 / M_PI);

    for (size_t i = 0; i < 2; i++)
    {
        // 1. predict the current state of the system
        kalman_filter_.kalman_angle[i] = kalman_filter_.kalman_angle[i] + (gyro_[i] / GYRO_SENSITIVITY) * kalman_filter_.ctrl_matrix;

        // 2. calculate the uncertainty of the prediction
        kalman_filter_.prediction_uncertainty[i] = kalman_filter_.prediction_uncertainty[i] + pow(MPU6050_SAMPLING_TIME * kalman_filter_.gyro_std_deviation, 2);
        
        // 3. calculate kalman gain
        kalman_filter_.kalman_gain[i] = kalman_filter_.prediction_uncertainty[i] / (kalman_filter_.prediction_uncertainty[i] + pow(kalman_filter_.acc_std_deviation, 2));
        
        // 4. update the kalman theta
        kalman_filter_.kalman_angle[i] = kalman_filter_.kalman_angle[i] + kalman_filter_.kalman_gain[i] * (kalman_filter_.acc_angle[i] - kalman_filter_.kalman_angle[i]);
        
        // 5. update uncertainty of the predicted state
        kalman_filter_.prediction_uncertainty[i] = (1 - kalman_filter_.kalman_gain[i]) * kalman_filter_.prediction_uncertainty[i];
    }

    // get yaw speed
    kalman_filter_.yaw_speed = gyro_[2] / GYRO_SENSITIVITY;
    kalman_filter_.yaw_angle = kalman_filter_.yaw_angle + MPU6050_SAMPLING_TIME * kalman_filter_.yaw_speed;

    kalman_filter_.temperature = (temp_ / 340.0) + 36.53;

    // set roll and pitch speed
    for (size_t i = 0; i < 2; i++)
    {
        kalman_filter_.kalman_previous_angel_speed[i] = kalman_filter_.kalman_angel_speed[i];
        kalman_filter_.kalman_angel_speed[i] = (kalman_filter_.kalman_angle[i] - kalman_filter_.kalman_previous_angel[i]) / MPU6050_SAMPLING_TIME;
        kalman_filter_.kalman_previous_angel[i] = kalman_filter_.kalman_angle[i];
    } 
}


void sensor_mpu6050::save_data_to_rate_control()
{
    rate_ctrl.prev_roll = rate_ctrl.roll;
    rate_ctrl.prev_pitch = rate_ctrl.pitch;
    rate_ctrl.prev_yaw = rate_ctrl.yaw;

    rate_ctrl.roll = gyro_[0] / GYRO_SENSITIVITY;
    rate_ctrl.pitch = gyro_[1] / GYRO_SENSITIVITY;
    rate_ctrl.yaw = gyro_[2] / GYRO_SENSITIVITY;
}


// [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temperature], raw data with offset
double* sensor_mpu6050::get_sensor_raw_data()
{
    for (size_t i = 0; i < 3; i++)
    {
        xyz_acc_gyro_data_[i] = accel_[i];
        xyz_acc_gyro_data_[i+3] = gyro_[i];
    }
    xyz_acc_gyro_data_[6] = temp_;
    return xyz_acc_gyro_data_;
}


// [roll with kalman filter, pitch with kalman filter, yaw(integrate gyro_z), temperature, kalman roll speed, kalman pitch speed]
double* sensor_mpu6050::get_sensor_kalman_data()
{
    rpy_kalman_data_[0] = kalman_filter_.kalman_angle[0];
    rpy_kalman_data_[1] = kalman_filter_.kalman_angle[1];
    rpy_kalman_data_[2] = kalman_filter_.yaw_angle;
    rpy_kalman_data_[3] = kalman_filter_.temperature;

    rpy_kalman_data_[4] = kalman_filter_.kalman_angel_speed[0];
    rpy_kalman_data_[5] = kalman_filter_.kalman_angel_speed[1];
    rpy_kalman_data_[6] = kalman_filter_.yaw_speed;

    rpy_kalman_data_[7] = kalman_filter_.kalman_previous_angel_speed[0];
    rpy_kalman_data_[8] = kalman_filter_.kalman_previous_angel_speed[1];

    return rpy_kalman_data_;
}

// [gyro_x, gyro_y, gyro_z], directly calculate xyz rotation speed, uint °/s
double* sensor_mpu6050::get_sensor_gyro_speed()
{
    for (size_t i = 0; i < 3; i++)
    {
        xyz_gyro_speed_[i] = gyro_[i] / GYRO_SENSITIVITY; 
    }     
    return xyz_gyro_speed_;
}

// sampling 1000 points, takes 10 seconds, 
// return false if calibration unfinished
// return true if calibration finihed
bool sensor_mpu6050::perform_zero_point_calibration()
{
    kalman_filter_.sampling_count++;
    
    if (kalman_filter_.sampling_count <= 1000)
    {
        for (size_t i = 0; i < 3; i++)
        {
            kalman_filter_.raw_data_sum[i] = kalman_filter_.raw_data_sum[i] + accel_[i];
            kalman_filter_.raw_data_sum[i + 3] = kalman_filter_.raw_data_sum[i + 3] + gyro_[i];
        }

        if (kalman_filter_.sampling_count == 1000)
        {
            for (size_t i = 0; i < 6; i++)
            {
                kalman_filter_.raw_data_offset[i] = int(kalman_filter_.raw_data_sum[i] / 1000.0);
            }

            // acc_z is different, acc_z = g, 1g = 16384
            kalman_filter_.raw_data_offset[2] = int(kalman_filter_.raw_data_sum[2] / 1000.0) - ACC_SENSITIVITY;
        }
        return false;
    }
   
    // zero point calibration
    for (size_t i = 0; i < 3; i++)
    {
        // save original data
        accel_no_offset_[i] = accel_[i];
        gyro_no_offset_[i] = gyro_[i];
        // apply offset to original data
        accel_[i] = accel_[i] - kalman_filter_.raw_data_offset[i];
        gyro_[i] = gyro_[i] - kalman_filter_.raw_data_offset[i + 3];
    }

    return true;
}



