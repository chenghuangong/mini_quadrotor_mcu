#include "communicator.h"

void pid_equation(float error, float p, float i, float d, float prev_error, float prev_iterm)
{

}


// perform rate control
void perform_motor_pid_control(communicator* comm)
{
    quad_motor* motor = comm->motor_;

    if (!motor->pid_on)
    {
        return;
    }

    auto gyro_raw_data = comm->sensor_gyro_.get_sensor_raw_data();
    auto gyro_data = comm->sensor_gyro_.get_sensor_kalman_data();

    double yaw_value = gyro_data[2];
    double gyro_z = gyro_raw_data[5] / GYRO_SENSITIVITY;

    // start PID controller, yaw_set_deg is offset
    double e_roll =  motor->p_roll * gyro_data[4] + motor->i_roll * (motor->roll_set_deg - gyro_data[0]) + motor->d_roll * (gyro_data[4] - gyro_data[7]) / MPU6050_SAMPLING_TIME;
    
    double e_pitch = motor->p_pitch * gyro_data[5] + motor->i_pitch * (motor->pitch_set_deg - gyro_data[1]) + motor->d_pitch * (gyro_data[5] - gyro_data[8]) / MPU6050_SAMPLING_TIME;

    double e_yaw = motor->p_yaw * gyro_z + motor->i_yaw * (yaw_value - motor->yaw_set_deg);

    motor->update_input_error(e_roll, e_pitch, e_yaw);

    comm->sampling_count++;
}


// perform rate control2
void perform_motor_rate_control(communicator* comm, double roll_target = 0, double pitch_target = 0, double yaw_target = 0)
{
    quad_motor* motor = comm->motor_;

    if (!motor->pid_on)
    {
        return;
    }

    auto rc = comm->sensor_gyro_.rate_ctrl;

    // save previous error
    rc.prev_err_roll = rc.err_roll;
    rc.prev_err_pitch = rc.err_pitch;
    rc.prev_err_yaw = rc.err_yaw;

    // set target rate speed is zero
    rc.err_roll = roll_target - rc.roll;
    rc.err_pitch = pitch_target - rc.pitch;
    rc.err_yaw = yaw_target - rc.yaw;


    // start PID controller, yaw_set_deg is offset
    double e_roll =  motor->p_roll * rc.err_roll + 
                     motor->i_roll * ((rc.err_roll + rc.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME + rc.prev_integral_roll + 
                     motor->d_roll * (rc.err_roll - rc.prev_err_roll) / MPU6050_SAMPLING_TIME;
    
    double e_pitch = motor->p_pitch * rc.err_pitch + 
                     motor->i_pitch * ((rc.err_pitch + rc.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME + rc.prev_integral_pitch + 
                     motor->d_pitch * (rc.err_pitch - rc.prev_err_pitch) / MPU6050_SAMPLING_TIME;

    double e_yaw = motor->p_yaw * rc.err_yaw;

    motor->update_input_error(e_roll, e_pitch, e_yaw);


    // save previous integral value
    rc.prev_integral_roll += motor->i_roll * ((rc.err_roll + rc.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME;
    rc.prev_integral_pitch += motor->i_pitch * ((rc.err_pitch + rc.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME;

    comm->sampling_count++;
}



void perform_motor_angle_control(communicator* comm, double roll_target = 0, double pitch_target = 0, double yaw_target = 0)
{
    quad_motor* motor = comm->motor_;

    if (!motor->pid_on)
    {
        return;
    }

    auto gyro_data = comm->sensor_gyro_.get_sensor_kalman_data();

    auto ac = comm->sensor_gyro_.angle_ctrl;

    ac.prev_err_roll = ac.err_roll;
    ac.prev_err_pitch = ac.err_pitch;

    ac.err_roll = roll_target - gyro_data[0];
    ac.err_pitch = pitch_target - gyro_data[1];

    double roll_rate_target = motor->p_angle_roll * ac.err_roll + 
                              motor->i_angle_roll * ((ac.prev_err_roll + ac.err_roll) / 2.0) * MPU6050_SAMPLING_TIME + ac.prev_integral_roll;

    double pitch_rate_target = motor->p_angle_pitch * ac.err_pitch + 
                               motor->i_angle_pitch * ((ac.prev_err_pitch + ac.err_pitch) / 2.0) * MPU6050_SAMPLING_TIME + ac.prev_integral_pitch;

    ac.prev_integral_roll += motor->i_angle_roll * ((ac.err_roll + ac.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME;
    ac.prev_integral_pitch += motor->i_angle_pitch * ((ac.err_pitch + ac.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME;

    perform_motor_rate_control(comm, roll_rate_target, pitch_rate_target);  
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
            // perform_motor_pid_control(comm);
            // perform_motor_rate_control(comm);
            perform_motor_angle_control(comm);
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
    add_repeating_timer_ms(MPU6050_SAMPLING_TIME * 1000, collect_sensor_data_callback, this, &sensor_timer_);
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