#include "communicator.h"

void perform_motor_pid_controll(communicator* comm)
{
    // TODO 
    // PID controll
}

bool push_sensor_data_callback(repeating_timer_t* rt)
{
    auto comm = static_cast<communicator*>(rt->user_data);
    if (rt->alarm_id == comm->data_timer_.alarm_id)
    {
        auto msg = comm->get_sensor_data();
        comm->esp_.write(msg);
    }
    return true;
}

bool collect_sensor_data_callback(repeating_timer_t* rt)
{
    auto comm = static_cast<communicator*>(rt->user_data);
    if (rt->alarm_id == comm->sensor_timer_.alarm_id)
    {
        comm->sensor_gyro_.mpu6050_read_kalman();

        // apply PID to motor output
        if (comm->motor_->pid_on)
        {
            perform_motor_pid_controll(comm);
        }
    }
    return true;
}

communicator::communicator(const std::string& addr, const uint16_t& port)
    : esp_(addr, port)
    , i2c_communicator_(I2C_INST, SDA_PIN, SCL_PIN)
    , sensor_pressure_(&i2c_communicator_)
    , sensor_gyro_(&i2c_communicator_)
{
    esp_.connect_to_server();
}

void communicator::start_push_sensor_data()
{
    add_repeating_timer_ms(1000, push_sensor_data_callback, this, &data_timer_);
    add_repeating_timer_ms(10, collect_sensor_data_callback, this, &sensor_timer_);
}

std::string communicator::get_sensor_data()
{
    auto pressure_data = sensor_pressure_.get_sensor_data();
    auto gyro_data = sensor_gyro_.get_sensor_kalman_data();
    auto gyro_raw_data = sensor_gyro_.get_sensor_raw_data();

    // std::string msg = "Temperture = " + std::to_string(pressure_data[0]) + ", Pressure = " + std::to_string(pressure_data[1]) + 
    //                   "Accel_X = " + std::to_string(gyro_data[0]) + " , Accel_Y = " + std::to_string(gyro_data[1]) + " , Accel_Z = " + std::to_string(gyro_data[2]) + 
    //                   "Gyro_X = " + std::to_string(gyro_data[3]) + " , Gyro_Y = " + std::to_string(gyro_data[4]) + " , Gyro_Z = " + std::to_string(gyro_data[5]) + 
    //                   "TEMP = " + std::to_string(gyro_data[6]);
    
    // std::string msg = "Temperture = " + std::to_string(pressure_data[0]) + ", Pressure = " + std::to_string(pressure_data[1]) + 
    //                  ", roll = " + std::to_string(gyro_data[0]) + ", pitch = " + std::to_string(gyro_data[1]) + ", temperature = " + std::to_string(gyro_data[3]);

    std::string msg = "{\"msg_type\":\"data\",\"dev_type\":\"drone\",\"source\":\"gyro\",\"value\":[" + 
                        std::to_string(gyro_data[0]) + "," + 
                        std::to_string(gyro_data[1]) + "," + 
                        std::to_string(gyro_data[2]) + "," +
                        std::to_string(gyro_raw_data[0]) + "," + 
                        std::to_string(gyro_raw_data[1]) + "," +
                        std::to_string(gyro_raw_data[2]) + "," +
                        std::to_string(gyro_raw_data[3]) + "," +
                        std::to_string(gyro_raw_data[4]) + "," +
                        std::to_string(gyro_raw_data[5]) + "," +
                        std::to_string(gyro_data[3]) + "," +
                        std::to_string(pressure_data[1]) + "]}";
    return msg;
}