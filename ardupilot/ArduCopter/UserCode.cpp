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
}

void Copter::userhook_init()
{
    setup_uart(hal.uartE, "uartE");
    hal.uartE->printf("\n");
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

int readInt(AP_HAL::UARTDriver *uart){
    uint8_t MSB = uart->read();    
    uint8_t LSB = uart->read();
    return LSB1 | MSB1 << 8;    
}
    
void Copter::userhook_50Hz()
{
    AP_HAL::UARTDriver *uartSensor = hal.uartE;
 
    while(uartSensor->available()){
        uint8_t messageType  = uartSensor->read();       //determine message type
        
        switch (messageType){
            case 0x00:{              	                //message for avoidance forces
                while(uartSensor->available() < 4);      //wait until four bytes are available
                //We read 4 bytes to create 2 16-bit ints 
                //MSB = most significant Byte, LSB = least significant Byte
                uint8_t MSB1 = uartSensor->read();    
                uint8_t LSB1 = uartSensor->read();
                uint8_t MSB2 = uartSensor->read();    
                uint8_t LSB2 = uartSensor->read();
                
                avoidancePitchForce = LSB1 | MSB1 << 8;   //combine MSB and LSB to make 16bit int
                avoidanceRollForce = LSB2 | MSB2 << 8;  
                hal.uartA->printf("pitch:%d\n",avoidancePitchForce); 
                hal.uartA->printf("roll:%d\n",avoidanceRollForce);          
                break;
            }
            
            case 0x01:{
                //uartSensor->read((char*)&x, sizeof(x));
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
