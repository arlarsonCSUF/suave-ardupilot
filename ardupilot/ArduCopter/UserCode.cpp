/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT

#if HAL_OS_POSIX_IO
    #include <stdio.h>
#endif

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
    uart->printf("1");
}

void Copter::userhook_init()
{
    setup_uart(hal.uartE, "uartE");
    
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP

static void send_uart(AP_HAL::UARTDriver *uart, uint8_t *data, uint16_t numberOfBytes){
    if (uart == NULL){
        return;
    }
    for(int i = 0; i < numberOfBytes; i++){
        uart->write(*(data + i));  
    }
}

void sendVelocity(AP_HAL::UARTDriver *uart,float v){
    uint8_t header[] ={0x02};
    send_uart(uart,header,sizeof(header));
    send_uart(uart,(uint8_t*)&v, sizeof(v));    
}

int readInt(AP_HAL::UARTDriver *uart){
    //MSB = most significant Byte, LSB = least significant Byte
    uint8_t MSB = uart->read();    
    uint8_t LSB = uart->read();
    return LSB | MSB << 8;    
}
    
void Copter::userhook_50Hz()
{
    AP_HAL::UARTDriver *uartSensor = hal.uartE;
    //
    const Vector3f &vel = inertial_nav.get_velocity();
    float vX = vel.x;
    sendVelocity(uartSensor,vX);
     
    while(uartSensor->available()){
        uint8_t messageType  = uartSensor->read();       //determine message type
        //hal.uartA->printf("messageType:%d",messageType);
        switch (messageType){
            
            case 0x00:{
                while(uartSensor->available() < 4);      //wait until four bytes are available
                //We read 4 bytes to create 2 16-bit ints      
                avoidancePitchForce = readInt(uartSensor);   //combine MSB and LSB to make 16bit int
                avoidanceRollForce = readInt(uartSensor);  
                hal.uartA->printf("pitch:%d\n",avoidancePitchForce); 
                hal.uartA->printf("roll:%d\n",avoidanceRollForce);          
                break;
            }
            
            case 0x01:{
                while(uartSensor->available() < 4);
                
                float x = 1;
                char *f = (char*)&x;
                for(uint8_t i=0; i < sizeof(x); i++){
                    char c = uartSensor->read();
                    f[i] = c;              
                }
                constrain_float(x,0,1);
                magnitudeUserInput = x;
                hal.uartA->printf("userInput:%f\n",magnitudeUserInput);
                break;
            }
            
            default:{
                while(uartSensor->available())
                    uartSensor->read();
                //If we do not recognize the message type print to error console
                #if HAL_OS_POSIX_IO
                    ::printf("Avoidance Sensor Message Type Not Valid");
                #endif
                break;
            }
            
        }      
    } 
    
    
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
