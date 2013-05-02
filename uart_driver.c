/*
* Copyright (c) 2013, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
* of its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  originally by Austin Buchan (?)
* modified to better handle mis formed packets, or bad starts R. Fearing April, 25, 2013
*
*
*
*/

#include "uart_driver.h"
#include "uart.h"
#include "mac_packet.h"
#include "payload.h"
#include "ppool.h"
#include "utils.h"

static MacPacket tx_packet = NULL;
static Payload tx_payload = NULL;
static unsigned char tx_idx;
static unsigned char tx_checksum;
static unsigned char tx_mode;  // use separate state variable, don't overload

static MacPacket rx_packet = NULL;
static Payload rx_payload = NULL;
static unsigned char rx_idx;
static unsigned char rx_checksum;
static unsigned char rx_mode;

static packet_callback rx_callback = NULL;

void uartInit(packet_callback rx_cb) {
    /// UART2 for RS-232 w/PC @ 230400, 8bit, No parity, 1 stop bit
    unsigned int U2MODEvalue, U2STAvalue, U2BRGvalue;
    U2MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                  UART_MODE_SIMPLEX & UART_UEN_00 & UART_DIS_WAKE &
                  UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                  UART_BRGH_FOUR & UART_NO_PAR_8BIT & UART_1STOPBIT;
    U2STAvalue  = UART_INT_TX & UART_INT_RX_CHAR &UART_SYNC_BREAK_DISABLED &
                  UART_TX_ENABLE & UART_ADR_DETECT_DIS &
                  UART_IrDA_POL_INV_ZERO; // If not, whole output inverted.
    U2BRGvalue  = 9; // =3 for 2.5M Baud
    //U2BRGvalue  = 43; // =43 for 230500Baud (Fcy / ({16|4} * baudrate)) - 1
    //U2BRGvalue  = 86; // =86 for 115200 Baud
    //U2BRGvalue  = 1041; // =1041 for 9600 Baud
    

    OpenUART2(U2MODEvalue, U2STAvalue, U2BRGvalue);

    	tx_mode = UART_TX_IDLE;
  	tx_idx = 0;
    	rx_idx = 0;
    	rx_mode = UART_RX_IDLE;
    	rx_callback = rx_cb;
    
    ConfigIntUART2(UART_TX_INT_EN & UART_TX_INT_PR4 & UART_RX_INT_EN & UART_RX_INT_PR4);
    EnableIntU2TX;
    EnableIntU2RX;
}

//General blocking UART send function, appends basic checksum
unsigned char uartSend(unsigned char length, unsigned char *frame) {
    int i;
    unsigned char checksum = 0;

    while(BusyUART2());
    WriteUART2(length);
    while(BusyUART2());
    WriteUART2(~length);

    checksum = 0xFF;

    //send payload data
    for (i = 0; i < length; i++) {
        checksum += frame[i];
        while(BusyUART2());
        WriteUART2(frame[i]);
    }

    //Send Checksum Data
    while(BusyUART2());
    WriteUART2(checksum);
    return 1;
}

unsigned char uartSendPayload(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    MacPacket packet;
    Payload pld;

    packet = ppoolRequestFullPacket(length);
    if(packet == NULL)
        return 0;

    pld = packet->payload;
    paySetType(pld, type);
    paySetStatus(pld, status);
    paySetData(pld, length, frame);
    if(uartSendPacket(packet)) {
        return 1;
    } else {
        ppoolReturnFullPacket(packet);
        return 0;
    }
}

unsigned char uartSendPacket(MacPacket packet) {
    CRITICAL_SECTION_START
    LED_3 = 1;
    if(tx_packet != NULL) {
        ppoolReturnFullPacket(tx_packet);
        tx_packet = NULL;
        tx_mode = UART_TX_IDLE;
    }

    if(tx_mode == UART_TX_IDLE && packet != NULL && packet->payload_length < UART_MAX_SIZE) {
        tx_packet = packet;
        tx_payload = packet->payload;
        tx_checksum = packet->payload_length + 3; // add three for size, size check, and checksum
        tx_mode = UART_TX_SEND_SIZE;
        WriteUART2(tx_checksum);  // sending size byte here, rest sent from interrupt.
        CRITICAL_SECTION_END
        LED_3 = 0;
        return 1;
    } else {
        LED_3 = 0;
        CRITICAL_SECTION_END
        return 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    unsigned char tx_byte;

    CRITICAL_SECTION_START
    _U2TXIF = 0;
    LED_3 = 1;
    if(tx_mode != UART_TX_IDLE) {
        if(tx_mode == UART_TX_SEND_SIZE)  // send complement of size byte
	 {  tx_idx = 0;
            tx_byte = ~tx_checksum; // send size check
	     tx_mode = UART_TX_PACKET; // transmit packet mode
        }
	 else 
		if(tx_idx == tx_payload->data_length + PAYLOAD_HEADER_LENGTH) // finished
		{  ppoolReturnFullPacket(tx_packet);
            	    tx_packet = NULL;
            	    tx_mode = UART_TX_IDLE;
                  tx_byte = tx_checksum;
        	}
		else {  tx_byte = tx_payload->pld_data[tx_idx++];  }  // get next byte to transmit
        tx_checksum += tx_byte;
        WriteUART2(tx_byte);
    }
    LED_3 = 0;
    CRITICAL_SECTION_END
}

static char temp_buffer[255]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // temporary to see if serial is being read correctly

// Packet format: length, ~length, status, type, data, check sum
//read data from the UART, and call the proper function based on the Xbee code
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    unsigned char rx_byte;

    CRITICAL_SECTION_START
    LED_3 = 1;
    while(U2STAbits.URXDA) 
    {
        rx_byte = U2RXREG;
	// State machine on rx_idx:
	// state IDLE (0xff): init checksum to byte length, set state CHECK_SIZE
	// state CHECK_SIZE: check that second byte is complement of first byte, and < 200 bytes length
       //  					 request packet in queue of size length
	//					initilize index, set state 0...
	// state 0...0xfd: store data in packet, check for max rx_idx

	switch(rx_mode){
		case UART_RX_IDLE:
			rx_checksum = rx_byte; // first byte should be length
			if (rx_byte < UART_MAX_SIZE) { rx_mode = UART_RX_CHECK_SIZE;} // if packet size legal, go to next state
			break;

		case UART_RX_CHECK_SIZE:
			 if((rx_checksum ^ rx_byte) == 0xFF) // second byte is complement of first ==> valid packet
	      		 {   rx_packet = ppoolRequestFullPacket(rx_checksum - (PAYLOAD_HEADER_LENGTH+3));
                  		rx_payload = rx_packet->payload;
                  		rx_checksum += rx_byte;
                  		rx_idx = 0;
				rx_mode = UART_RX_PACKET; 	}										
	        	else // either parity error on packet length, or length > 200, 
	       	{  rx_checksum = rx_byte; }  // continue CHECK_SIZE until get consecutive complementary bytes
			break;
	
		case UART_RX_PACKET:
      			if (rx_idx == rx_payload->data_length + PAYLOAD_HEADER_LENGTH) // got all bytes?
			{	if(rx_checksum == rx_byte && rx_callback != NULL)  // checksum ok?
				{  (rx_callback)(rx_packet); }  // if yes, push command function
				else 
				{  ppoolReturnFullPacket(rx_packet); }  // else return packet to pool
            			rx_mode = UART_RX_IDLE; } // get next packet
        		else // need more bytes
			{	rx_checksum += rx_byte;
            			rx_payload->pld_data[rx_idx++] = rx_byte;
				temp_buffer[rx_idx] = rx_byte; }  // temp store message locally
        		break;
		default:
			rx_mode = UART_RX_IDLE; // should not get to this case
			break;
	     }  // switch
	} // while
    if(U2STAbits.OERR) {
        U2STAbits.OERR = 0;
    }
    
    _U2RXIF = 0;
    LED_3 = 0;
    CRITICAL_SECTION_END
}
