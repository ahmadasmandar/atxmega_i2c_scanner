#include <avr/io.h>

#include "Clock.h"

void Clock_Init()
{
    // OSC.CTRL |= OSC_RC32MEN_bm;
    // while(!(OSC.STATUS & OSC_RC32MRDY_bm));
    // CCP = CCP_IOREG_gc;
    // CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

    /*** PLL to more acurate Clock **/

    OSC.CTRL |= OSC_RC32MEN_bm;
    while (!(OSC.STATUS & OSC_RC32MRDY_bm))
        ;

    OSC.PLLCTRL = OSC_PLLSRC_RC32M_gc | 0x04;

    OSC.CTRL |= OSC_PLLEN_bm;
    while (!(OSC.STATUS & OSC_PLLRDY_bm))
        ;

    CCP      = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}