#include "motor_controller_test.h"



// ========================================================================================
// motor default constructor
Motor::Motor()
{
    id = 0;
    Error = 0;
    // rx_packet = new uint8_t [6];
    // tx_packet = new uint8_t [9];
    // motor_params = new uint16_t [5];
    // control_limits = new int8_t [4];
    // control_params = new uint8_t [5];
    // states = new float [4];
}

// motor Parameterized constructor
Motor::Motor(uint8_t id_, MotorParams motor_params_, MotorControlLimits control_limits_)
{
    id = id_;
    Error = 0;
    params = motor_params_;
    control_limits = control_limits_;
    // rx_packet = new uint8_t [6];
    // tx_packet = new uint8_t [9];
    // control_limits = new int8_t [4];
    // motor_params = new uint16_t [5];
    // control_params = new uint8_t [5];
    // states = new float [4];
}

Motor::~Motor()
{   
    printf("closing motor id-%i \n", id);
    // delete [] rx_packet;
    // delete [] tx_packet;
    // delete [] motor_params;
    // delete [] control_limits;
    // delete [] control_params;
    // delete [] states;
    // rx_packet = nullptr;
    // tx_packet = nullptr;
    // motor_params = nullptr;
    // control_limits = nullptr;
    // control_params = nullptr; 
    // states = nullptr;
}


// =======================================================================================



// Motor Module Parameterized constructor
MotorModule::MotorModule(uint8_t com_interface, uint8_t num_of_motors)
{
  _com_interface = com_interface;
  _num_of_motors = num_of_motors;
  _num_of_available_motors = 0;
  motor = new Motor[_num_of_motors]; // allocating motor instance
}

// Motor Module deconstructor
MotorModule::~MotorModule()
{
    // delete allocated memory
    printf("deleting motor alloc\n");
    delete [] motor;
    motor = nullptr;

    // close communication interface
    if (_com_interface == 0){

        /* TODO: close serial here */

        printf("Closing Serial\n");
    }
    else if (_com_interface == 1){

        /* TODO: close CAN here */

        printf("Closing CAN\n");
    }
}

Motor* MotorModule::add_motor(uint8_t id)
{
    uint8_t m_index = _num_of_available_motors;
    Motor *m = &motor[m_index];
    m->id = id;
    m->config_status[0] = false;
    m->config_status[1] = false;

    _num_of_available_motors +=1; // increase _num_of_available_motors by one

    return m;
}

void MotorModule::set_motor_params(Motor* motor_, float max_p, float max_v, float max_kp, float max_kd, float max_iff)
{
    motor_->params.max_p = max_p;
    motor_->params.max_v = max_v;
    motor_->params.max_kd = max_kd;
    motor_->params.max_kp = max_kp;
    motor_->params.max_iff = max_iff;
    motor_->config_status[0] = true;
}

void MotorModule::set_control_limits(Motor* motor_, float min_p, float max_p, float max_v, float max_i)
{
    motor_->control_limits.min_p = min_p;
    motor_->control_limits.max_p = max_p;
    motor_->control_limits.max_v = max_v;
    motor_->control_limits.max_i = max_i;
    motor_->config_status[1] = true;
}

void MotorModule::enable_motor(Motor* motor_)
{
     
    uint8_t tx[9] = {motor_->id, 0xFF, 0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    /* TODO: send this cmd over selected com_interface here: */

    // debug print
    printf("motor id-%i enable cmd: ", motor_->id);
    for (int i=0; i<sizeof(tx); i++){
        printf("| %i |", tx[i]);
    }
    printf("\n");

    /* TODO: make is_enabled flag true only if motor send back feedback */
    motor_->states.is_enabled = true;
}

void MotorModule::enable_all_motors()
{
    for(int i = 0; i<_num_of_available_motors; i++){
        uint8_t tx[9] = {motor[i].id, 0xFF, 0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

        /* TODO: send cmd over selected com_interface here: */

        // debug print
        printf("motor id-%i enable cmd: ", motor[i].id);
        for (int i=0; i<sizeof(tx); i++){
            printf("| %i |", tx[i]);
        }
        printf("\n");

        /* TODO: make is_enabled flag true only if motor send back feedback */
        motor[i].states.is_enabled = true;
    } 
}

void MotorModule::disable_motor(Motor* motor_)
{
    uint8_t tx[9] = {motor_->id, 0xFF, 0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    /* TODO: send this cmd over selected com_interface here: */

    // debug print
    printf("motor id-%i disable cmd: ", motor_->id);
    for (int i=0; i<sizeof(tx); i++){
        printf("| %i |", tx[i]);
    }
    printf("\n");
    /* TODO: make is_enabled flag false only if motor send back feedback */
    motor_->states.is_enabled = false;
}

void MotorModule::disable_all_motors()
{
    for(int i = 0; i<_num_of_available_motors; i++){
        uint8_t tx[9] = {motor[i].id, 0xFF, 0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        /* TODO: send cmd over selected com_interface here: */

        // debug print
        printf("motor id-%i disable cmd: ", motor[i].id);
        for (int i=0; i<sizeof(tx); i++){
            printf("| %i |", tx[i]);
        }
        printf("\n");
        /* TODO: make is_enabled flag false only if motor send back feedback */
        motor[i].states.is_enabled = false;
    } 
}

void MotorModule::set_motor_zero(Motor* motor_)
{
    uint8_t tx[9] = {motor_->id, 0xFF, 0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

    /* TODO: send this cmd over selected com_interface here: */

    // debug print
    printf("motor id-%i set_zero cmd: ", motor_->id);
    for (int i=0; i<sizeof(tx); i++){
        printf("| %i |", tx[i]);
    }
    printf("\n");
}


MotorStates MotorModule::get_motor_states(Motor* motor_)
{
    if (motor_->config_status[0]){
        // send previous cmd to the motor and read the feedback
    
        /* TODO: 1. read 6 bytes of serial or can and save the into rx_packet only if rx_packet was sent by motor id: _id */

        uint8_t rx_data[] = {motor_->id,2,3,4,5,6}; // TODO: remove this temp line

        // unpack rx_packet
        unpack_rx_packet(rx_data);

    }
    else{
        // printf("Error: motor %i paramters are not set.\n", motor_->id);
        printf("Please set parameters of motor id-%i.\n", motor_->id);
        throw "Motor Parameters are NOT set";
        // abort();
    }

    return motor_->states;
}


void MotorModule::set_motor_position(Motor* motor_, ControlCmds cmd)
{
    // set motor commands
    motor_->control_cmd.p_des = cmd.p_des;
    motor_->control_cmd.v_des = cmd.v_des;
    motor_->control_cmd.kp = cmd.kp;
    motor_->control_cmd.kd = cmd.kd;
    motor_->control_cmd.i_ff = cmd.i_ff;
    // pack motor command
    pack_tx_packet(motor_);

    /*  TODO: Send motor command over selected com_interface */
    //

    // debug print
    printf("[setting motor position] cmd: ");
    for(int i=0; i<sizeof(motor_->tx_packet); i++){
        printf(" %i ", motor_->tx_packet[i]);
    }
}

// pack tx_packet
void MotorModule::pack_tx_packet(Motor * m)
{
    // limit data to be within bounds
    float p_des = fminf(fmaxf(-m->params.max_p, m->control_cmd.p_des), m->params.max_p);
    float v_des = fminf(fmaxf(-m->params.max_v, m->control_cmd.v_des), m->params.max_v);
    float kp = fminf(fmaxf(0, m->control_cmd.kp), m->params.max_kp);
    float kd = fminf(fmaxf(0, m->control_cmd.kd), m->params.max_kd);
    float iff = fminf(fmaxf(-m->params.max_iff, m->control_cmd.i_ff), m->params.max_iff);
    // convert floats to uints
    int p_int = float2uint(m->control_cmd.p_des, -m->params.max_p, m->params.max_p, 16);
    int v_int = float2uint(m->control_cmd.v_des, -m->params.max_v, m->params.max_v, 12);
    int kp_int = float2uint(m->control_cmd.kp, 0, m->params.max_kp, 12);
    int kd_int = float2uint(m->control_cmd.kd, 0, m->params.max_kd, 12);
    int iff_int = float2uint(m->control_cmd.i_ff, -m->params.max_iff, m->params.max_iff, 12);
    // pack data
    m->tx_packet[0] = m->id;
    m->tx_packet[1] = p_int>>8;
    m->tx_packet[2] = p_int&&0xFF;
    m->tx_packet[3] = v_int >> 4;
    m->tx_packet[4] = ((v_int&0xF)<<4) | (kp_int>>8);
    m->tx_packet[5] = kp_int&&0xFF;
    m->tx_packet[6] = kd_int>>4;
    m->tx_packet[7] = ((kd_int&0xF)<<4) | (iff_int>>8);
    m->tx_packet[8] = iff_int&0xFF;
}

// unpack rx_data from the motor
void MotorModule::unpack_rx_packet(uint8_t rx_data[6])
{

    // TODO: here unpack rx_packet into feedback

    
}


float MotorModule::fmaxf(float x, float y){
    return (((x)>(y)) ? (x) : (y));
}

float MotorModule::fminf(float x, float y){
    return (((x) < (y)) ? (x) : (y));
}

int MotorModule::float2uint(float x, float x_min, float x_max, int bits){
    float span = x_max = x_min;
    float offset = x_min;
    return (int) ((x-offset) * ((float)((1<<bits)-1))/span);
}

float MotorModule::uint2float(int x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x)*span/((float)((1<<bits)-1)) + offset;
}
