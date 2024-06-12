/**
 * Lin Client App Header file
 *
 * @file lin_client_app.h
 * 
 * @ingroup linclient
 * 
 * @brief This header file provides the API for the Lin Client App.
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

#ifndef LIN_APP_H
#define	LIN_APP_H

/**
  Section: Included Files
*/

#include "lin_client.h"

/**
 * @ingroup linclient
 * @enum linclient_cmd_t.
 * @brief Enumeration of signalName and ID value.
*/

typedef enum {
    Serial_Analyzer=0,
    Serial_Pod=1
}linclient_cmd_t;

uint8_t Serial_Analyzer_Data[1];
uint8_t Serial_Pod_Data[1];

const linclient_rx_cmd_t scheduleTable[] = {
    //Command, Type, TX/RX Length, Data Address
   {Serial_Analyzer,RECEIVE,1,Serial_Analyzer_Data},
   {Serial_Pod,TRANSMIT,1,Serial_Pod_Data}
}; 
#define TABLE_SIZE  (sizeof(scheduleTable)/sizeof(linclient_rx_cmd_t))
/**
 * @ingroup linclient
 * @brief Initializes the linclient module
 *        This routine must be called before any other linclient routine
 * @param None.
 * @return None. 
 */
void LinClient_Initialize(void);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called from Initializes.
 * @param None.
 * @return None. 
 */

void processLinClient(void);


#endif	/* LIN_APP_H */

