/**
 * Lin Client Header file
 *
 * @file lin_client.h
 * 
 * @defgroup linclient LIN Client Application
 *
 * @brief This header file provides the API for the Lin Client.
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

#ifndef LIN_H
#define	LIN_H

/**
  Section: Included Files
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @ingroup linclient
 * @enum receive state.
 * @brief Enumeration of receive state.
*/
typedef enum {
    LIN_RX_IDLE,
    LIN_RX_BREAK,
    LIN_RX_SYNC,
    LIN_RX_PID,
    LIN_RX_DATA,
    LIN_RX_CHECKSUM,
    LIN_RX_TX_DATA,
    LIN_RX_RDY,
    LIN_RX_ERROR,
    LIN_RX_WAIT
}linclient_rx_state_t;

/**
 * @ingroup linclient
 * @enum Type mode.
 * @brief Enumeration of receive state.
*/
typedef enum {
    TRANSMIT,
    RECEIVE,
    ERROR
}linclient_packet_type_t;

/**
 * @ingroup linclient
 * @enum cmd ,Type mode and Data Length.
 * @brief Enumeration of param.
*/
typedef enum {
    CMD,
    TYPE,
    LENGTH
}linclient_sch_param_t;

typedef union {
    struct {
        uint8_t PID;
        uint8_t data[8];
        uint8_t checksum;
        linclient_packet_type_t type;
        int8_t length;
    };
    uint8_t rawPacket[13];
}linclient_packet_t;

typedef struct {
    uint8_t cmd;
    linclient_packet_type_t type;
    uint8_t length;
    uint8_t* data;
}linclient_rx_cmd_t;

typedef union {
    struct {
        unsigned ID0: 1;
        unsigned ID1: 1;
        unsigned ID2: 1;
        unsigned ID3: 1;
        unsigned ID4: 1;
        unsigned ID5: 1;
        unsigned P0: 1;
        unsigned P1: 1;
    };
    uint8_t rawPID;
}linclient_pid_t;

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called from Initializes.
 * @param tableLength:unsigned 8-bit integer , Callback function of receive , Callback function of processData.
 * @return None. 
 */
void LinClient_init(uint8_t tableLength, const linclient_rx_cmd_t* const command, void (*processData)(void));

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes.
 * @param cmd:unsigned 8-bit integer.
 * @return None. 
 */
void LinClient_queuePacket(uint8_t cmd);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes.
 * @param length::unsigned 8-bit integer and dataPointer.
 * @return None. 
 */
void LinClient_sendPacket(uint8_t length, uint8_t pid, uint8_t* data);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes and return data Packet.
 * @param dataPointer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LinClient_getPacket(uint8_t* data);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes.
 * @param cmd:unsigned 8-bit integer , dataParam.
 * @return unsigned 8-bit integer. 
 */
uint8_t LinClient_getFromTable(uint8_t cmd, linclient_sch_param_t param);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes.
 * @param None.
 * @return linclient_rx_state_t struct. 
 */
linclient_rx_state_t LinClient_handler(void);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This routine must be called  Initializes.
 * @param pid:unsigned 8-bit integer.
 * @return bool. 
 */
bool LinClient_checkPID(uint8_t pid);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This return the checksum value.
 * @param length:unsigned 8-bit integer, pid:unsigned 8-bit integer , dataPointer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LinClient_getChecksum(uint8_t length, uint8_t pid, uint8_t* data);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This return the calculated parity value.
 * @param CMD:unsigned 8-bit integer.
 * @return unsigned 8-bit integer. 
 */
uint8_t LinClient_calcParity(uint8_t CMD);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This start the timer.
 * @param timeout:unsigned 8-bit integer.
 * @return None. 
 */
void LinClient_startTimer(uint8_t timeout);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This handle the timer.
 * @param  None.
 * @return None. 
 */
void LinClient_timerHandler(void);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This set the timer.
 * @param  None.
 * @return None. 
 */
void LinClient_setTimerHandler(void);
/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This stop the timer.
 * @param  None.
 * @return None. 
 */
void LinClient_stopTimer(void);
/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This enable the receive.
 * @param  None.
 * @return None. 
 */
void LinClient_enableRx(void);
/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This disable the receive.
 * @param  None.
 * @return None. 
 */
void LinClient_disableRx(void);

/**
 * @ingroup linclient
 * @brief process the linclient module
 *        This break the check.
 * @param  None.
 * @return bool. 
 */
bool LinClient_breakCheck(void);

#endif	/* LIN_H */

