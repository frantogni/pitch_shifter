//=============================================================================
// TPFF.c
//=============================================================================
// Copyright (...)
// Módulo: TP_01
// Autor: Francisco Togni, 2023
//
// Este firmware samplea una señal analogica en 16 bits y
// produce una salida analogica a traves de 2 señales PWM
// Se procesa la señal para convertir a un intervalo
// comprendido entre -1 octava y +1 octava, en escala justa y temperada

//
// Target: Tiva C Launchpad
// Tool chain: GCC
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

//#include "tm4c123gh6pm.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sysctl.h"
#include "hw_sysctl.h"
#include "gpio.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_nvic.h"
#include "hw_timer.h"
#include "timer.h"
#include "hw_uart.h"
#include "uart.h"
#include "adc.h"
#include "hw_adc.h"
#include "pwm.h"
#include "hw_pwm.h"
#include "comp.h"
#include "hw_comp.h"

#include "numeros.h"

//-----------------------------------------------------------------------------
// 32-bit Definitions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

//#define led_R HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA+((GPIO_PIN_1)<<2)))
//#define led_B HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA+((GPIO_PIN_2)<<2)))
//#define led_G HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA+((GPIO_PIN_3)<<2)))


#define r2r_bit15 HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA+(GPIO_PIN_3)<<2))
#define r2r_bit14 HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA+(GPIO_PIN_2)<<2))
#define r2r_bit13 HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA+(GPIO_PIN_1)<<2))
#define r2r_bit12 HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA+(GPIO_PIN_0)<<2))

#define ENCODER_A (HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA+((GPIO_PIN_0)<<2))))
#define ENCODER_B (HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA+((GPIO_PIN_1)<<2))))>>1


//#define r2r_bits HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA+(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)<<2))

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void _delay_ms(uint32_t);
void _delay_us(uint32_t);

void init_CLOCK(void);
void init_GPIO(void);
void init_ADC0(void);
void init_CMP(void);
void init_TIMER0(void);
void init_TIMER2(void);
void init_PWM1(void);

void check_encoder(void);

void EnableInterrupts(void);
void DisableInterrupts(void);

extern interrupt void ISR_PORTB(void);
extern interrupt void ISR_PORTD(void);
extern interrupt void ISR_PORTF(void);
extern interrupt void ISR_TIMER0(void);
extern interrupt void ISR_UART0(void);
extern interrupt void ISR_ADC_SS1(void);
extern interrupt void ISR_TIMER2(void);

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------


uint16_t in[0x3AFF],out,f_adc_base,f_pwm_base, aux;
uint16_t i,j;
uint8_t encoder;

bool rate;


//=============================================================================
// MAIN Routine
//=============================================================================
int main(void)
{
    init_CLOCK();
    init_GPIO();
    init_PWM1();
    init_ADC0();
    //init_CMP();

    f_adc_base = 0x2D0; // 111111 [Hz] 5A0 168 2D0  FOR DWN, BAJO EL ADC
    f_pwm_base = 0x2D0; // 111111 [Hz]
    i = 0;
    j = 0;
    rate = 0;
    encoder = 13;

    //r2r_bit15 = 0; //GPIO_PIN_3;
    //r2r_bit14 = 0; //GPIO_PIN_2;
    //r2r_bit13 = 0; //GPIO_PIN_1;
    //r2r_bit12 = 0; //GPIO_PIN_0;

    init_TIMER0();
    init_TIMER2();

    EnableInterrupts();  // habilita las interrupciones globales del core

    HWREG(TIMER0_BASE+TIMER_O_CTL) |= TIMER_CTL_TAEN; // timer0 GO
    HWREG(TIMER2_BASE+TIMER_O_CTL) |= TIMER_CTL_TAEN; // timer2 GO

    for(;;)
    {
        check_encoder();

        while(!rate);
        HWREG(PWM1_BASE + PWM_O_3_CMPB) = (in[j] & 0x00FF);
        HWREG(PWM1_BASE + PWM_O_3_CMPA) = (in[j] >> 8);
        (j==0x3AFE)? j=0 : j++;
        rate = 0;
    }
}

//=============================================================================
// Initialization Subroutines
//=============================================================================

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
void init_CLOCK(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Configure the GPIO ports
//
void init_GPIO(void)
{
    HWREG(SYSCTL_RCGC2) |= (SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOD);

    //HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) &= (~(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)); // TEST LEDS ON-BOARD
    //HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);      // DIG EN PF
    //HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= (GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);      // SALIDA PF
    //HWREG(GPIO_PORTF_BASE + GPIO_O_DR8R) |= (GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);     // 4mA.

    //HWREG(GPIO_PORTB_BASE + GPIO_O_AFSEL) &= (~(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)); // DESHABILITO ANALOG
    //HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // HABILITO DIGITAL
    //HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // SALIDAS DE 4 A 7 (MOTOR)
    //HWREG(GPIO_PORTB_BASE + GPIO_O_DR8R) |= (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // 8mA.


    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= (~(GPIO_PIN_0|GPIO_PIN_1)); // DESHABILITO ANALOG
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (GPIO_PIN_0|GPIO_PIN_1); // HABILITO DIGITAL
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= ~(GPIO_PIN_0|GPIO_PIN_1); // ENTRADAS DE 0 A 3
    HWREG(GPIO_PORTD_BASE + GPIO_O_PUR) |= (GPIO_PIN_0|GPIO_PIN_1); // PULL UP EN ENTRADAS

    HWREG(GPIO_PORTD_BASE + GPIO_O_IM) = 0;                             // GLOBAL INT ACT (1) DES (0)
    HWREG(GPIO_PORTD_BASE + GPIO_O_IS) &= ~(GPIO_PIN_0);     // INT POR FLANCO(0) O NIVEL (1)
    HWREG(GPIO_PORTD_BASE + GPIO_O_IBE) &= ~(GPIO_PIN_0);    // INT POR UN FLANCO (0) O AMBOS (1)
    HWREG(GPIO_PORTD_BASE + GPIO_O_IEV) &= ~(GPIO_PIN_0);    // INT FLANCO ASC (1) O DES (0)
    HWREG(GPIO_PORTD_BASE + GPIO_O_ICR) = 0XFF;                         // DESACTIVO CON 1 CUALQUIER INT
    HWREG(GPIO_PORTD_BASE + GPIO_O_IM) = (GPIO_PIN_0);                  // ACTIVO PINES DE INT
    HWREG(NVIC_EN0) |= 1<<3;                                            // HABILITO NVIC PUERTO B

}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//
// Configure A/D converter to use Timer3 overflows as conversion source, to
// generate an interrupt on conversion complete, and to use right-justified
// output mode. Enables ADC end of conversion interrupt. Leaves ADC disabled.
//
void init_ADC0(void)
{
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOE;
    HWREG(SYSCTL_RCGCADC) = SYSCTL_RCGCADC_R0; // CLK ADC0
    HWREG(GPIO_PORTE_BASE + GPIO_O_AFSEL) |= (GPIO_PIN_2|GPIO_PIN_3);                // Funcion alternativa en PE3 para AIN0
    HWREG(GPIO_PORTE_BASE + GPIO_O_DEN) &= ~(GPIO_PIN_2|GPIO_PIN_3);                 // Deshabilitamos la función digital para que quede como analógico en PE3, para AIN0
    HWREG(GPIO_PORTE_BASE + GPIO_O_AMSEL) = (GPIO_PIN_2|GPIO_PIN_3);                 // AIN0

    // Sample sequencer 1 configuration
    HWREG(ADC0_BASE + ADC_O_ACTSS ) &= ~ADC_ACTSS_ASEN1; //
    HWREG(ADC0_BASE + ADC_O_EMUX) = ADC_EMUX_EM1_PROCESSOR; //
    HWREG(ADC0_BASE + ADC_O_SSMUX1) = 0x00; //The ADCSSMUXn fields select the input pin
    HWREG(ADC0_BASE + ADC_O_SSCTL1) |= ADC_SSCTL1_END0 ; //los demás en cero.
    //HWREG(ADC0_BASE + ADC_O_IM)= ADC_IM_MASK1; //activamos la interrupcion por secuenciador 1 para que vaya al NVIC
    HWREG(ADC0_BASE + ADC_O_ACTSS ) |= ADC_ACTSS_ASEN1;  //habilitamos el ss1 tomadno toda la configuracion que seteamos
    //Nos falta activar el canal del NVIC por donde entra interrupcion del ADC0, y vectorizar la ISR, y habilitar interrupciones globaleS
    //HWREG(NVIC_EN0) |= 1 << 15;  //ADC0 SS1
    HWREG(ADC0_BASE + ADC_O_PC) = ADC_PC_SR_1M ;
}

//-----------------------------------------------------------------------------
// COMPARATOR_Init
//-----------------------------------------------------------------------------
//
//
//
//
void init_CMP(void)
{
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOC;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7); // ALTERNATE FUNCTION
    HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) &= ~(GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7); // Deshabilitar la función digital
    HWREG(GPIO_PORTC_BASE + GPIO_O_AMSEL) = (GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7); // analog mode

    HWREG(SYSCTL_RCGCACMP) = SYSCTL_RCGCACMP_R0; // CLK CMPS

    HWREG(SYSCTL_RCGC1) |= (SYSCTL_RCGC1_COMP0 | SYSCTL_RCGC1_COMP1); // CLK COMPARADORES
    HWREG(COMP_BASE + COMP_O_ACREFCTL) = 0;
    HWREG(COMP_BASE + COMP_O_ACCTL0) = 0;
    HWREG(COMP_BASE + COMP_O_ACCTL1) = 0;
    HWREG(COMP_BASE + COMP_O_PP) = (COMP_PP_CMP0); // | COMP_PP_CMP1); // Comparators GO
}


//-----------------------------------------------------------------------------
// TIMER_Init
//-----------------------------------------------------------------------------
//
// Configure Timer to auto-reload at interval specified by <counts> (no
// interrupt generated) using SYSCLK as its time base.
//
void init_TIMER0(void)
{
    HWREG(SYSCTL_RCGC1) |= (SYSCTL_RCGC1_TIMER0); //Habilitar el TIMER
    HWREG(TIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN; // APAGO TIMER0
    HWREG(TIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_32_BIT_TIMER;
    HWREG(TIMER0_BASE+TIMER_O_TAMR) = 0x0000;
    HWREG(TIMER0_BASE+TIMER_O_TAMR) |= TIMER_TAMR_TAMR_PERIOD;
    HWREG(TIMER0_BASE+TIMER_O_TAMR) &= ~TIMER_TAMR_TACDIR;
    HWREG(TIMER0_BASE+TIMER_O_TAILR) = f_adc_base; //   111111 Hz
    HWREG(TIMER0_BASE+TIMER_O_IMR) = TIMER_IMR_TATOIM; //GPTM Timer A Time-Out Interrupt
    HWREG(NVIC_EN0) |= 1<<19; // habilito INT por timer0
}

void init_TIMER2(void)
{
    HWREG(SYSCTL_RCGC1) |= (SYSCTL_RCGC1_TIMER2); //Habilitar el TIMER
    HWREG(TIMER2_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN; // APAGO TIMER0
    HWREG(TIMER2_BASE+TIMER_O_CFG) = TIMER_CFG_32_BIT_TIMER;
    HWREG(TIMER2_BASE+TIMER_O_TAMR) = 0x0000;
    HWREG(TIMER2_BASE+TIMER_O_TAMR) |= TIMER_TAMR_TAMR_PERIOD;
    HWREG(TIMER2_BASE+TIMER_O_TAMR) &= ~TIMER_TAMR_TACDIR;
    HWREG(TIMER2_BASE+TIMER_O_TAILR) = f_pwm_base; //
    HWREG(TIMER2_BASE+TIMER_O_IMR) = TIMER_IMR_TATOIM; //GPTM Timer A Time-Out Interrupt
    HWREG(NVIC_EN0) |= 1<<23; // habilito INT por timer0
}


//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
void init_UART0(void)
{
    HWREG(UART0_BASE + UART_O_CTL) = 0;                                             // DESACT USART0 BEFORE SETUP
    HWREG(UART0_BASE + UART_O_IBRD) = 43;                                          // 115200
    HWREG(UART0_BASE + UART_O_FBRD) = 26;
    HWREG(UART0_BASE + UART_O_LCRH) = UART_LCRH_WLEN_8 ;                           // 8-N-1, no FIFO
    HWREG(UART0_BASE + UART_O_CC) = UART_CC_CS_SYSCLK;                              // 80Mhz
    HWREG(UART0_BASE + UART_O_IM) = UART_IM_RXIM;                                  // ACT ISR(RX) UART0
    HWREG(UART0_BASE + UART_O_ECR) = 0xFF;                                           // CLEAR FLAGS ISR
    HWREG(UART0_BASE + UART_O_CTL) = (UART_CTL_UARTEN|UART_CTL_RXE|UART_CTL_TXE);  // ACT UART0, TXE, RXE
    HWREG(NVIC_EN0) |= 0x01 << 5;                                                    // INT POR UART0
}

//-----------------------------------------------------------------------------
// PWM1_Init
//-----------------------------------------------------------------------------
void init_PWM1(void)
{
    HWREG(SYSCTL_RCGC2) |= SYSCTL_RCGC2_GPIOF;
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    HWREG(SYSCTL_RCGCPWM) = SYSCTL_RCGCPWM_R1;
    HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= (GPIO_PIN_2|GPIO_PIN_3);    // ALTERNATE FUNCTION
    HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (GPIO_PIN_2|GPIO_PIN_3);      // DIG EN PF
    HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= (GPIO_PIN_2|GPIO_PIN_3);      // SALIDA PF
    HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) = 0x5500;                       // FUNCION PWM
    HWREG(PWM1_BASE + PWM_O_3_LOAD) = 0xFF; // 2 x 8 bits
    HWREG(PWM1_BASE + PWM_O_3_CMPA) = 0x00;
    HWREG(PWM1_BASE + PWM_O_3_CMPB) = 0x00;
    HWREG(PWM1_BASE + PWM_O_3_GENA) = 0b000010001100;
    HWREG(PWM1_BASE + PWM_O_3_GENB) = 0b100000001100;
    HWREG(PWM1_BASE + PWM_O_ENABLE) = (GPIO_PIN_7|GPIO_PIN_6); // Enable PWM outputs
    HWREG(PWM1_BASE + PWM_O_3_CTL) = 0x01; // Start the timers
}


//-----------------------------------------------------------------------------
// Delay Routine (ms)
//-----------------------------------------------------------------------------
void _delay_ms(uint32_t ui32ms)
{
    SysCtlDelay(ui32ms * (SysCtlClockGet() / 3000));
}

//-----------------------------------------------------------------------------
// Delay Routine (us)
//-----------------------------------------------------------------------------
void _delay_us(uint32_t ui32us)
{
    SysCtlDelay(ui32us * (SysCtlClockGet() / 3000000));
}


void check_encoder(void)
{
    if(encoder == 0)
    {
        encoder = 1;
    }
    if(encoder == 26)
    {
        encoder = 25;
    }

    switch(encoder)
        {
        case 1 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x168;
            break;
        case 2 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x180;
            break;
        case 3 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x190;
            break;
        case 4 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x1B0;
            break;
        case 5 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x1C2;
            break;
        case 6 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x1E0;
            break;
        case 7 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x1F4;
            break;
        case 8 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x21C;
            break;
        case 9 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x240;
            break;
        case 10 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x258;
            break;
        case 11 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x280;
            break;
        case 12 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x2A3;
            break;
        case 13 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x2D0;
            break;
        case 14 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x300;
            break;
        case 15 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x32A;
            break;
        case 16 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x360;
            break;
        case 17 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x384;
            break;
        case 18 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x3BF;
            break;
        case 19 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x40C;
            break;
        case 20 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x438;
            break;
        case 21 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x480;
            break;
        case 22 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x4B0;
            break;
        case 23 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x510;
            break;
        case 24 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x546;
            break;
        case 25 :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x5A0;
            break;
        default :
            HWREG(TIMER2_BASE+TIMER_O_TAILR) = 0x2D0;
            break;
        }
}

//=============================================================================
// Interrupt Service Routines
//=============================================================================

//-----------------------------------------------------------------------------
// Global E
//-----------------------------------------------------------------------------
void EnableInterrupts(void)
{
    __asm(" cpsie   i\n");
}

//-----------------------------------------------------------------------------
// Global D
//-----------------------------------------------------------------------------
void DisableInterrupts(void)
{
    __asm(" cpsid  i\n");
}

//-----------------------------------------------------------------------------
// PORTB_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_PORTB(void)
{
    return;
}

//-----------------------------------------------------------------------------
// PORTB_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_PORTD(void)
{
    HWREG(GPIO_PORTD_BASE + GPIO_O_ICR) = GPIO_PIN_0;
    HWREG(GPIO_PORTD_BASE + GPIO_O_IEV) ^= GPIO_PIN_0;
    (!(ENCODER_B) == !(ENCODER_A))? encoder++ : encoder--;
    return;
}

//-----------------------------------------------------------------------------
// PORTF_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_PORTF(void)
{
    return;
}

//-----------------------------------------------------------------------------
// ADC_SS1_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_ADC_SS1()
{
    return;
}

//-----------------------------------------------------------------------------
// TIMER0_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_TIMER0(void)
{
    HWREG(TIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_TATOCINT; // ISR TIMER0
    in[i] = HWREG(ADC0_BASE + ADC_O_SSFIFO1); //Lectura de las muestras que estan en la fifo
    (i==0x3AFE)? i=0 : i++;
    HWREG(ADC0_BASE + ADC_O_PSSI) |= ADC_PSSI_SS1 ; //inicio conversion del SS1
    return;
}

//-----------------------------------------------------------------------------
// TIMER2_ISR
//-----------------------------------------------------------------------------
interrupt void ISR_TIMER2(void)
{
    HWREG(TIMER2_BASE+TIMER_O_ICR) = TIMER_ICR_TATOCINT; // ISR TIMER0
    rate = 1;
    //HWREG(PWM1_BASE + PWM_O_3_CMPB) = (sample_AN01[j] & 0x00FF);
    //HWREG(PWM1_BASE + PWM_O_3_CMPA) = (sample_AN01[j] >> 8);
    return;
}

//-----------------------------------------------------------------------------
// UART0_ISR
//-----------------------------------------------------------------------------
void ISR_UART0(void)
{
    return;
}





