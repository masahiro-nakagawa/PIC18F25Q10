/**
 * Lin Client Application
 * 
 * @file lin_client.c
 * 
 * @ingroup  linclient
 * 
 * @brief This source file provides the implementation of the API for the Lin Client driver.
 *
 * @version Lin Client Driver Version 1.0.0
 */

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/

#include "../lin_client.h"
#include "../../uart/eusart1.h"
#include "../../timer/tmr2.h"

#define READ_TIMEOUT    15  //ms

static void (*LIN_processData)(void);

linclient_packet_t LIN_packet;
bool LIN_rxInProgress = false;
const linclient_rx_cmd_t* LIN_rxCommand;
uint8_t LIN_rxCommandLength;

static uint8_t LIN_timeout = 10; //TODO: Make dependent on Baudrate
static bool LIN_timerRunning = false;
static volatile uint8_t CountCallBack = 0;

void LinClient_init(uint8_t tableLength, const linclient_rx_cmd_t* const command, void (*processData)(void)){
    LIN_rxCommand = command;
    LIN_rxCommandLength = tableLength;
    LIN_processData = processData;
    LinClient_stopTimer();
    LinClient_enableRx();
    LinClient_setTimerHandler();
}

void LinClient_queuePacket(uint8_t cmd){
    const linclient_rx_cmd_t* tempSchedule = LIN_rxCommand;    //copy table pointer so we can modify it
    
    cmd &= 0x3F;    //clear possible parity bits
    for(uint8_t i = 0; i < LIN_rxCommandLength; i++){
        if(cmd == tempSchedule->cmd){
            break;
        }
        tempSchedule++;    //go to next entry
    }
    
    LIN_packet.type = tempSchedule->type;
    LIN_packet.length = tempSchedule->length;
    
    //Build Packet - User defined data
    //add data
    memcpy(LIN_packet.data, tempSchedule->data, LIN_packet.length);
    
    //Add Checksum
    LIN_packet.checksum = LinClient_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data);
    LinClient_sendPacket(LIN_packet.length, LIN_packet.PID, LIN_packet.data);

    
}

linclient_rx_state_t LinClient_handler(void){
    static linclient_rx_state_t LIN_rxState = LIN_RX_IDLE;
    static uint8_t rxDataIndex = 0;

    if(LIN_rxInProgress == true){
        if(LIN_timerRunning == false){
            //Timeout
            LIN_rxState = LIN_RX_ERROR;
        }
    }

    switch(LIN_rxState){
        case LIN_RX_IDLE:
            if(EUSART1_IsRxReady() > 0){
                //Start Timer
                LinClient_startTimer(READ_TIMEOUT); 
                LIN_rxInProgress = true;
                LIN_rxState = LIN_RX_BREAK;
            }
            break;
        case LIN_RX_BREAK:
            if(EUSART1_IsRxReady() > 0){
                if(LinClient_breakCheck() == true){  //Read Break
                    LIN_rxState = LIN_RX_SYNC;
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                }
            }
            break;
        case LIN_RX_SYNC:
            if(EUSART1_IsRxReady() > 0){
                if(EUSART1_Read() == 0x55){  //Read sync - discard
                    LIN_rxState = LIN_RX_PID;
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                }
            }
            break;
        case LIN_RX_PID:
            if(EUSART1_IsRxReady() > 0){
                LIN_packet.PID = EUSART1_Read();

                //check LIN Parity bits
                if(LinClient_checkPID(LIN_packet.PID) == false){
                    LIN_rxState = LIN_RX_ERROR;
                    break;
                }
                LIN_packet.type = LinClient_getFromTable(LIN_packet.PID, TYPE);
                if(LIN_packet.type == RECEIVE){
                    LIN_packet.length = LinClient_getFromTable(LIN_packet.PID, LENGTH);
                    LIN_rxState = LIN_RX_DATA;
                }
                else{
                    LinClient_disableRx();
                    LIN_rxState = LIN_RX_TX_DATA;
                }
            }
            break;
        case LIN_RX_DATA:
            if(EUSART1_IsRxReady() > 0){
                LIN_packet.data[rxDataIndex] = EUSART1_Read();
                if(++rxDataIndex >= LIN_packet.length){
                    //received all data bytes
                    rxDataIndex = 0;
                    LIN_rxState = LIN_RX_CHECKSUM;
                }
            }
            break;
        case LIN_RX_CHECKSUM:
            if(EUSART1_IsRxReady() > 0){
                LIN_packet.checksum = EUSART1_Read();
                if(LIN_packet.checksum != LinClient_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data)) {
                    LIN_rxState = LIN_RX_ERROR;
                }
                else {
                    LIN_rxState = LIN_RX_RDY;
                }
            }
            break;
        case LIN_RX_TX_DATA:
            LinClient_queuePacket(LIN_packet.PID); //Send response automatically
            LIN_rxState = LIN_RX_RDY;
        case LIN_RX_RDY:
            LIN_processData();
        case LIN_RX_ERROR:
            LinClient_stopTimer();
            rxDataIndex = 0;
            LIN_rxInProgress = false;
            memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));  //clear receive data
        case LIN_RX_WAIT:
            if(EUSART1_IsTxDone()){
                LinClient_enableRx();
                LIN_rxState = LIN_RX_IDLE;
            } else {
                LIN_rxState = LIN_RX_WAIT;
            }
            break;
    }
    return LIN_rxState;
}

void LinClient_sendPacket(uint8_t length, uint8_t pid, uint8_t* data){

    //Write data    
    for(uint8_t i = 0; i < length; i++){
        EUSART1_Write(*(data + i));
    }
    //Add Checksum
    EUSART1_Write(LinClient_getChecksum(length, pid, data));
}

uint8_t LinClient_getPacket(uint8_t* data){
    uint8_t cmd = LIN_packet.PID & 0x3F;
    
    memcpy(data, LIN_packet.data, sizeof(LIN_packet.data));
    memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));
    
    return cmd;
}

uint8_t LinClient_getFromTable(uint8_t cmd, linclient_sch_param_t param){
    const linclient_rx_cmd_t* rxCommand = LIN_rxCommand;    //copy table pointer so we can modify it
    
    cmd &= 0x3F;    //clear possible parity bits
    //check table
    for(uint8_t length = 0; length < LIN_rxCommandLength; length++){
        if(cmd == rxCommand->cmd){
            break;
        }
        rxCommand++;    //go to next entry

        if(length == (LIN_rxCommandLength-1)){
            return ERROR;   //command not in schedule table
        }
    }
    
    switch(param){
        case CMD:
            return rxCommand->cmd;
        case TYPE:
            return rxCommand->type;
        case LENGTH:
            return rxCommand->length;
        default:
            break;
    }
    
    return ERROR;
}

bool LinClient_checkPID(uint8_t pid){
    if(LinClient_getFromTable(pid, TYPE) == ERROR)
        return false;   //PID not in schedule table
    
    if(pid == LinClient_calcParity(pid & 0x3F))
        return true;  
    
    return false; //Parity Error

}

uint8_t LinClient_calcParity(uint8_t CMD){
    linclient_pid_t PID;
    PID.rawPID = CMD;

    //Workaround for compiler bug - CAE_MCU8-200:
//    PID.P0 = PID.ID0 ^ PID.ID1 ^ PID.ID2 ^ PID.ID4;
//    PID.P1 = ~(PID.ID1 ^ PID.ID3 ^ PID.ID4 ^ PID.ID5);
    PID.P0 = PID.ID0 ^ PID.ID1;
    PID.P0 = PID.P0 ^ PID.ID2;
    PID.P0 = PID.P0 ^ PID.ID4;
    PID.P1 = PID.ID1 ^ PID.ID3;
    PID.P1 = PID.P1 ^ PID.ID4;
    PID.P1 = PID.P1 ^ PID.ID5;
    PID.P1 = ~PID.P1;
    
    return PID.rawPID;
}

uint8_t LinClient_getChecksum(uint8_t length, uint8_t pid, uint8_t* data){
    uint16_t checksum = pid;
    
    for (uint8_t i = 0; i < length; i++){
        checksum = checksum + *data++;
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;
    
    return (uint8_t)checksum;
}

void LinClient_startTimer(uint8_t timeout){
    LIN_timeout = timeout;
    Timer2_Write(0);
    Timer2_Start();
    LIN_timerRunning = true;
}

void LinClient_timerHandler(void){

    // callback function
    if (++CountCallBack >= LIN_timeout)
    {
        // ticker function call
        LinClient_stopTimer();
    }
}

void LinClient_setTimerHandler(void){
    Timer2_OverflowCallbackRegister(LinClient_timerHandler);
}

void LinClient_stopTimer(void){
    Timer2_Stop();
    // reset ticker counter
    CountCallBack = 0;
    LIN_timerRunning = false;
}

void LinClient_enableRx(void){

    EUSART1_ReceiveEnable();
    EUSART1_ReceiveInterruptEnable();
}

void LinClient_disableRx(void){
    EUSART1_ReceiveDisable();
    EUSART1_ReceiveInterruptDisable();
}

bool LinClient_breakCheck(void){
    
    if(EUSART1_Read()==0x00){
        return true;
    }
    
    return false;
}