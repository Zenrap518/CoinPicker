
// Function prototypes for UART communication
int _mon_getc(int canblock);
void putc1(char c);
int SerialTransmit1(const char* buffer);
unsigned int SerialReceive1(char* buffer, unsigned int max_size);
unsigned int SerialReceive1_timeout(char* buffer, unsigned int max_size);
void ClearFIFO(void);
void SendATCommand(char* s);
void ReceptionOff(void);