/**
 * Lin Client Application
 * 
 * @file lin_client_app.c
 * 
 * @ingroup linclient
 * 
 * @brief This source file provides the implementation of the API for the Lin Client app driver.
 *
 * @version Lin Client App Driver Version 1.0.0
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

#include "../lin_client_app.h"
#include "../../adc/adcc.h"
void LinClient_Initialize(void){

    LinClient_init(TABLE_SIZE, scheduleTable, processLinClient);
    
}

void processLinClient(void){
    uint8_t tempRxData[8];
    uint8_t cmd;

    cmd = LinClient_getPacket(tempRxData);
    switch(cmd){
      case Serial_Analyzer:
            break; 
      case Serial_Pod:
            Serial_Pod_Data[0] = ADCC_GetSingleConversion(0);
            break; 
        default:
            break;
    }
}