

extern int txf;
extern int rxf;

void baudrateInit(unsigned long int baudrate);
void uart_putchar(unsigned char c);
void uart_puts(unsigned char* s);

unsigned char uart_getchar(void);
void uart_putuint(unsigned int num, unsigned int digit);
void uart_putint(int num, unsigned int digit);

void uartInit(void);

extern int operation;
extern int operationIOHex;
extern int initiation;
extern int UpMode;
extern int Kattsun;
extern int sinWave;
extern int sinpos;

extern unsigned int INtargetM0Edge;
extern unsigned int INtargetM1Edge;
extern unsigned int INtargetM2Edge;

int KattsunM0duty;
int KattsunM1duty;
int KattsunM2duty;
