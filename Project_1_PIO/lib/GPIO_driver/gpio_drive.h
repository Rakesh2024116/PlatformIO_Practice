
#define RCC_APB2ENR		(*((volatile unsigned long *)	0x40021018))

#define GPIO_A					(*((volatile unsigned long *)	0x40010800))
#define GPIO_B					(*((volatile unsigned long *)	0x40010C00))
#define GPIO_C					(*((volatile unsigned long *)	0x40011000))

/*Port names*/

#define POA	1
#define POB	2
#define POC	3


/* Port Modes(directions)*/
#define	IN 		0
#define OUT10	1
#define OUT2	2
#define OUT50	3

/*Port modes in Input mode (dir options)*/
#define IN_AN	0		//Input Analog Mode
#define IN_F	1		//Floatin
#define IN_PP	2		//Input with pull-up/pull down

/*output mode (dir options)*/
#define OP_GP_PP	0	// output with push/pull
#define	OP_GP_OD	1	// output with open drain
#define	OP_AF_PP	2	// Alternate function with open drain
#define OP_AF_OD	3	// Alternate function with open drain

/*Status Definitions*/
#define LOW 	0
#define HIGH 	1



void init_GPIO(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt);

int Read_GP(unsigned short port, unsigned short pin);

void Write_GP(unsigned short port, unsigned short pin, unsigned short state); 

void toggle_GP(unsigned short port, unsigned short pin);
