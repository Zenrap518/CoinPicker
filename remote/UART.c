#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "UART.h"

// Needed to use scanf() and gets(), even if we don't directly use them in the code
int _mon_getc(int canblock)
{
    char c;

    if (canblock)
    {
        while (!U2STAbits.URXDA); // wait (block) until data available in RX buffer
        c = U2RXREG;
        if (c == '\r') c = '\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
        return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
            c = U2RXREG;
            if (c == '\r') c = '\n';
            return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

void putc1(char c)
{
    while (U1STAbits.UTXBF);   // wait while TX buffer full
    U1TXREG = c;               // send single character to transmit buffer
}

int SerialTransmit1(const char* buffer)
{
    unsigned int size = strlen(buffer);
    while (size)
    {
        while (U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }

    while (!U1STAbits.TRMT);        // wait for last transmission to finish

    return 0;
}

unsigned int SerialReceive1(char* buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    while (num_char < max_size)
    {
        while (!U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

        // insert nul character to indicate end of string
        if (*buffer == '\n')
        {
            *buffer = '\0';
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
}

unsigned int SerialReceive1_timeout(char* buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout_cnt;

    while (num_char < max_size)
    {
        timeout_cnt = 0;
        while (1)
        {
            if (U1STAbits.URXDA) // check if data is available in RX buffer
            {
                *buffer = U1RXREG; // copy RX buffer into *buffer pointer
                break;
            }
            if (++timeout_cnt == 100) // 100 * 100us = 10 ms
            {
                *buffer = '\n';
                break;
            }
            delayus(100);
        }

        // insert nul character to indicate end of string
        if (*buffer == '\n')
        {
            *buffer = '\0';
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
}

void ClearFIFO(void)
{
    unsigned char c;
    U1STA = 0x1400;     // enable TX and RX, clear FIFO
    while (U1STAbits.URXDA) c = U1RXREG;
}

void SendATCommand(char* s)
{
    char buff[40];
    printf("Command: %s", s);
    LATB &= ~(1 << 14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
    waitms(10);
    SerialTransmit1(s);
    U1STA = 0x1400;     // enable TX and RX, clear FIFO
    SerialReceive1(buff, sizeof(buff) - 1);
    LATB |= 1 << 14; // 'SET' pin of JDY40 to 1 is normal operation mode.
    waitms(10);
    printf("Response: %s\n", buff);
}

void ReceptionOff(void)
{
    LATB &= ~(1 << 14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
    waitms(10);
    SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
    waitms(10);
    ClearFIFO();
    LATB |= 1 << 14; // 'SET' pin of JDY40 to 1 is normal operation mode.
}


