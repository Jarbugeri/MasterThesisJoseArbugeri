#include "F28x_Project.h"
#include "F2837xD_Peripheral_Setup.h"

#ifdef CPU1
    extern void mainCPU1();
#endif

#ifdef CPU2
    extern void mainCPU2();
#endif

int main(void)
{

    InitSysCtrl();

    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    #ifdef CPU1
        InitGpio();
    #endif


    Peripheral_Setup_InitLEDsGpio();

    Peripheral_Setup_Management();

    Peripheral_Setup_configClaMemory();

    Peripheral_Setup_Memory_shared();

    EINT;
    ERTM;

    #ifdef CPU1
        mainCPU1();
    #endif

    #ifdef CPU2
        mainCPU2();
    #endif

    return(0);
}
