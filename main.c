//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************
void UART0_init(void);

#include "msp.h"

volatile int ADC_val = 0;
volatile int flag = 0;

int main(void) {
    volatile unsigned int i;
    static char list[4];
    int count = 0;
    int digit = 0;
    static int voltage;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
                 WDT_A_CTL_HOLD;

    // GPIO Setup
    P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC
    P5->SEL0 |= BIT4;

    // Enable global interrupt
    UART0_init();
    __enable_irq();

    //UART0_init();

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_3;         // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

    while (1)
    {
        for (i = 20000; i > 0; i--);        // Delay
        // Start sampling/conversion

        if(flag == 1){
            EUSCI_A0->TXBUF = '\r';
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

            voltage = 277 * ADC_val + 150170;
            voltage /= 1000;

            count = 0;
            while(count < 4 && ADC_val >= 0){
                digit = voltage % 10;
                voltage /= 10;
                list[count] = (digit + '0');
                count++;
            }

//        while (!(EUSCI_A0->IFG & 0x02)) { }  // wait for transmit buffer empty
        for(count = count; count >= 0; count--){
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF = list[count];

        }
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
        flag = 0;// For debugger

        }
    }
}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    ADC_val = ADC14->MEM[0];
    flag = 1;
}

void UART0_init(void) {
    EUSCI_A0->CTLW0 |= 1;     /* put in reset mode for config */
    EUSCI_A0->MCTLW = 0;      /* disable oversampling */
    EUSCI_A0->CTLW0 = 0x0081; /* 1 stop bit, no parity, SMCLK, 8-bit data */
    EUSCI_A0->BRW = 26;       /* 3000000 / 115200 = 26 */
    P1->SEL0 |= 0x0C;         /* P1.3, P1.2 for UART */
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;    /* take UART out of reset mode */
    EUSCI_A0->IE |= 1;        /* enable receive interrupt */
}
