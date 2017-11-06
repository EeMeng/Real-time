#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <process.h>
#include <pthread.h>
#include <math.h>

#define	INTERRUPT		iobase[1] + 0			// Badr1 + 0 : also ADC register
#define	MUXCHAN			iobase[1] + 2			// Badr1 + 2
#define	TRIGGER			iobase[1] + 4			// Badr1 + 4
#define	AUTOCAL			iobase[1] + 6			// Badr1 + 6
#define DA_CTLREG		iobase[1] + 8		    // Badr1 + 8

#define	AD_DATA		    iobase[2] + 0			// Badr2 + 0
#define	AD_FIFOCLR		iobase[2] + 2			// Badr2 + 2

#define	TIMER0			iobase[3] + 0			// Badr3 + 0
#define	TIMER1			iobase[3] + 1			// Badr3 + 1
#define	TIMER2			iobase[3] + 2			// Badr3 + 2
#define	COUNTCTL		iobase[3] + 3			// Badr3 + 3
#define	DIO_PORTA		iobase[3] + 4			// Badr3 + 4
#define	DIO_PORTB		iobase[3] + 5			// Badr3 + 5
#define	DIO_PORTC		iobase[3] + 6			// Badr3 + 6
#define	DIO_CTLREG		iobase[3] + 7			// Badr3 + 7
#define	PACER1			iobase[3] + 8			// Badr3 + 8
#define	PACER2			iobase[3] + 9			// Badr3 + 9
#define	PACER3			iobase[3] + 0xa			// Badr3 + a
#define	PACERCTL		iobase[3] + 0xb			// Badr3 + b

#define DA_Data		    iobase[4] + 0			// Badr4 + 0
#define	DA_FIFOCLR		iobase[4] + 2			// Badr4 + 2

#define PI              acos(-1)
//Number of maximum possible changes in state in 1 sec for all DACs
#define FBEST           360000


//global variable struct for waveform
typedef struct {
    bool resetWave;
    bool isOn;
    unsigned short waveform_type;
    const unsigned short identity;
    unsigned short data[1000];
    unsigned short plus;
    unsigned short DAC_mode;
    int samples_per_period;
    float output_res;
    float period;
    float mean;
    float freq;
    float amp;
}DACField ;

int badr[5];						// PCI 2.2 assigns 6 IO base addresses
char* clearScreen="clear"

bool isOperating=true;

DACField DAC[2]={   {true, false, 2, 0,{}, 0, 1, 100, 0, 0, 6, 10, 2},
                    {true, false, 3 ,1,{}, 0, 1, 100, 0, 0, 0, 10, 1}};

DACField temp={true, false, 2,0,{}, 0, 1, 100, 0, 0, 0, 10, 1};

int f_available=FBEST;

pthread_mutex_t MainMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PushMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

//*************************************************************//
// DAC parts
//*************************************************************//
// Supporting function: check whether data array has negative elements
bool hasNegative(DACField* Curr){
    if( ((*Curr).mean-(*Curr).amp) < 0)
        return true;
    return false;
}
// Supporting function: find absolute maximum value
float absMaximum(DACField* Curr){
    if(fabs((*Curr).mean+(*Curr).amp)>fabs((*Curr).mean-(*Curr).amp))
        return fabs((*Curr).mean+(*Curr).amp);
    else
        return fabs((*Curr).mean-(*Curr).amp);
}
// Change the bipolar/unipolar mode to give best resolution
void chooseBestRes(DACField* Curr){
    if(hasNegative(Curr))       //Only DAC0 is used
        if(absMaximum(Curr)<5){
            (*Curr).plus=(0x0000<<(*Curr).identity);
            (*Curr).DAC_mode = 0;
            (*Curr).output_res=152.59;
        }
        else{
            (*Curr).plus=(0x0100<<(*Curr).identity);
            (*Curr).DAC_mode = 1;
            (*Curr).output_res=305.14;
        }
    else
        if(absMaximum(Curr)<5){
            (*Curr).plus=(0x0200<<(*Curr).identity);
            (*Curr).DAC_mode = 2;
            (*Curr).output_res=76.29;
        }
        else{
            (*Curr).plus=(0x0300<<(*Curr).identity);
            (*Curr).DAC_mode = 3;
            (*Curr).output_res=152.59;
        }
}
// Waveform generator
void WaveformGen (DACField* Curr){
    int i=0;
    double delta_incr, dummy, res;
    unsigned short offset=0;
    /*
    value = mean + x
        sine wave       : x= amp*sin(2*PI*freq*t)
        triangular wave : x= (4*amp/T)*t        (0<t<T/4)
                          x= 2*amp-(4*amp/T)*t  (T/4<t<3*T/4)
                          x= -4*amp+(4*amp/T)*t (3*T/4<t<T)
        square wave     : x= amp                (0<t<T/2)
                          x= -amp               (T/2<t<T)
    */
    chooseBestRes(Curr);
    (*Curr).period=1/(*Curr).freq;
    res = (*Curr).output_res/1000000;
    if((*Curr).DAC_mode<2)
        offset+=0x7FFF;
    switch ((*Curr).waveform_type){
        case 1: {// sine wave waveform creation
                 printf("\nSetting Sine Wave\n");
                 delta_incr=2.0*PI/(*Curr).samples_per_period;	// increment
                 for(i=0;i<(*Curr).samples_per_period;i++) {
                     dummy= (sinf((float)(i*delta_incr)))* (*Curr).amp + (*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 2: {// triangular wave waveform creation
                 printf("\nSetting Triangular Wave\n");
                 delta_incr=4*(*Curr).amp/(*Curr).samples_per_period;	// increment
                 for(i=0;i<(*Curr).samples_per_period/4;i++) {
                     dummy= delta_incr*i +(*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                 for(;i<(3*(*Curr).samples_per_period/4);i++) {
                     dummy= 2*(*Curr).amp-delta_incr*i +(*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                 for(;i<(*Curr).samples_per_period;i++) {
                     dummy= -4*(*Curr).amp+delta_incr*i +(*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 3: {// square wave waveform creation
                 printf("\nSetting Square Wave\n");
                 for(i=0;i<(*Curr).samples_per_period/2;i++) {
                     dummy= (*Curr).amp + (*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                 for(;i<(*Curr).samples_per_period;i++) {
                     dummy= -(*Curr).amp + (*Curr).mean;
                     dummy= offset + dummy/res;
                     (*Curr).data[i]= (unsigned short) dummy;
                 }
                break;
                }
        }
    (*Curr).resetWave==false;
}
// Function to push-out data to DAC(thread function)
void* PushDAC (void* Curr){
    DACField* Current = (DACField*) Curr;
    unsigned short CTLREG_content;
    int w=(int)f_available/(*Current).freq/(*Current).samples_per_period;
    int i=0,j=0;
    CTLREG_content=(unsigned short)((*Current).plus+((*Current).identity+0x1)*0x20+0x3;
    while (1){
        for(i=0;i<(*Current).samples_per_period;i++) {
            //does not exit only when isOn==true and resetWave==false
            if(!((*Current).resetWave==false && (*Current).isOn==true))
                pthread_exit(NULL);
            for(j=0;j<w;j++){
            //allow different DACs to write in turn
                pthread_mutex_lock(&PushMutex);
                out16(DA_CTLREG, CTLREG_content);       // write setting to DAC CTLREG
                out16(DA_FIFOCLR, 0);					// Clear DA FIFO buffer
                out16(DA_Data, (*Current).data[i]);     // Output data
                pthread_mutex_unlock(&PushMutex);
            }
        }
    }
    return (0);
}
// Wave generator manager (thread function)
void* WaveGenManager (void){
    int i=0;
    pthread_t tid[2];
    while(1){
        if(!isOperating)
            pthread_exit(NULL);
        pthread_mutex_lock(&MainMutex);
        f_available = FBEST/(((int)DAC[0].isOn)+DAC[1].isOn);
        for(i=0; i<2; i++){
            if(DAC[i].isOn==false)
                continue;
            else {
                if(DAC[i].resetWave==false)
                    continue;
                else{
                    WaveformGen(&DAC[i]);
                    pthread_create(&tid[i], NULL, PushDAC, (void *)&DAC[i]);
                }
            }
        }
        pthread_mutex_unlock(&MainMutex);
    }
}


//*************************************************************//
// Input manager for switches and analogue inputs
//*************************************************************//
void change(bool onSignal, int DACID, int wvty, float f, float m, float a){
    DAC[DACID].waveform_type=wvty;
    DAC[DACID].freq=f;
    DAC[DACID].mean=m;
    DAC[DACID].amp=a;
    DAC[DACID].resetWave=true;
    DAC[DACID].isOn=onSignal;
}

void* PeripheralInputs (void){
}

//*************************************************************//
// Primary input(keyboard)/output manager
//*************************************************************//
void displayHelp() {
    printf("\n*************\n");
	printf("Help menu\n");
	printf("*************\n");
	printf("%*s\t\t%s", 6, "1", "Show current devices' configurations.\n");
    printf("%*s\t\t%s", 6, "2" ,"Show current devices' statuses.\n");
    printf("%*s\t\t%s", 6, "3", "Change the configurations.\n");
    printf("%*s\t\t%s", 6, "4", "Enable or disable components.\n");
    printf("%*s\t\t%s", 6, "5", "Export configurations or waveforms.\n");
    printf("%*s\t\t%s", 6, "6", "Import configurations.\n");
    printf("%*s\t\t%s", 6, "7", "Halt all operations.\n");
    printf("%*s\t\t%s", 6, "8", "Exit program\n");
}
void getInput(char* in) {
	printf("Please enter your command: ");
	scanf("%s", in);
	// clear input buffer after each acquisition of input
	fflush(stdin);
}
int checkInput(char* in) {
    int temp=0;
    char* endptr;
    temp = strtol(in, &endptr);
    // check if valid integer is inputted
    if(*endptr == '\0'){
        if(temp>0 && temp <8)
            return temp;
        else if(temp==8)
            return 66;
    }
    else if(strcmp(in,"help")==0)
        return 8;
    else
        printf("Invalid input! Inputted:%s\n", in);
}
void showConfig(void){
    int i=0;
    printf("%*s\t\t%*s", 38, "DAC0", 15, "DAC1\n");
    printf("%*s%*d\t%*d\n", 25, "Running?", 15, DAC[0].isOn, 15,
           DAC[1].isOn);
    printf("%*s", 25, "Waveform type");
    for(i=0;i<2;i++){
        switch(DAC[i].waveform_type){
            case 1: { printf("%*s", 15, "Sinusoidal"); break;}
            case 2: { printf("%*s", 15, "Triangular"); break;}
            case 3: { printf("%*s", 15, "Square"); break;}
        }
        printf("        ");
    }
    printf("\n");
    printf("%*s%*d\t%*d\n", 25,"Samples per period", 15,
           DAC[0].samples_per_period, 15, DAC[1].samples_per_period);
    printf("%*s%*d\t%*d\n", 25, "Output resolution (V)", 15,
           DAC[0].output_res, 15, DAC[1].output_res);
    printf("%*s%*.2f\t%*.2f\n", 25, "Frequency (Hz)", 15,
           DAC[0].freq, 15, DAC[1].freq);
    printf("%*s%*.2f\t%*.2f\n", 25, "Amplitude (V)", 15,
           DAC[0].amp, 15, DAC[1].amp);
    printf("%*s%*.2f\t%*.2f\n", 25, "Mean (V)", 15,
           DAC[0].mean, 15, DAC[1].mean);
}

void* MainIO (void){
    displayHelp();
    char* input;
    while (1) {
		getInput(input);
		switch (checkInput(input)) {
		    // case 1 - print current devices' configurations.
			case 1: {   showConfig();       break; }
            // case 2 - print current devices' statuses.
			case 2: {   showStatus();       break; }
			// case 3 - change the configurations.
			case 3: {   changeConfig();       break; }
            // case 4 - enable or disable components
			case 4: {   compSelect();  break; }
			// case 5 - export configurations or waveforms.
			case 5: {   exportConfig();  break; }
			// case 6 - import configurations.
			case 6: {   importConfig();  break; }
			// case 7 - halt all operations.
			case 7: {   stopOps(); break; }
            // case 8 - display help
			case 8: {   displayHelp(); break; }
            // case 66 - execute order 66: quit the program
			case 66: {  isOperating=false; break; }
			//show error in input
			default:{   printf("Invalid character. Please reenter. \n");
                        printf("To display help, enter 'help'.\n");
                    }
		}
		if(!isOperating)
            pthread_exit(NULL);
    }
}
//*************************************************************//
// Command line argument manager (Called once only)
//*************************************************************//
void CLManager (int argc, char **argv){
    int counter, i, j, temp2;
    int waveform_type=0;    // counter for argument
    float temp;    // temporary storage for argument values
    char* endptr;
    counter=1;
    j=0;
    /*
    Check for argument flags, and then proceed to check for values in the next argument
    If argument are true, then check for next argument to see if it has a valid value.
    If values are valid, then move on to the next argument.
    If argument are false, then argument is ignored, and an error message is displayed.
    */
    for(;counter<argc-1;counter++){
        if(strcmp(argv[counter],"-sin") == 0)
            waveform_type=1;
        else if(strcmp(argv[counter],"-tri") == 0)
            waveform_type=2;
        else if(strcmp(argv[counter],"-squ") == 0)
            waveform_type=3;
        else
            printf("Invalid waveform.\n");
        for(i=0;i<3;i++){
            temp = strtod(argv[++counter], &endptr);
            if(*endptr == '\0'){
                switch(counter%5){
                    case 2: {
                        if(temp>0 && temp <1800) //a bit low
                            freq = temp;
                        else
                            printf("Frequency must be in the range (0, 1800) Hz\n");
                        break;
                    }
                    case 3: {
                        if(fabs(temp)=<10)
                            mean = temp;
                        else
                            printf("Mean value must be in the range [-10,10]\n");
                        break;
                    }
                    case 4: {
                        if(temp < 10 && temp>=0)
                            amp = temp;
                        else
                            printf("Amplitude must be in the range [0,5]\n");
                        break;
                    }
                }
            }
        else
            printf("Invalid input: %s\n", argv[counter]);
        }
        if((counter%5) ==0){
            temp2 = strtol(argv[++counter], &endptr);
            if(temp2==1)
                change(true, j++, waveform_type, freq, mean, amp);
            else
                change(false, j++, waveform_type, freq, mean, amp);
        }
    }
}

//*************************************************************//
// Main function
//*************************************************************//
int main(int argc, char** argv) {
    //Call command line manager
    CLManager (argc, argv);

    //PCI device variable
    struct pci_dev_info info;
    void *hdl;

    uintptr_t iobase[5];
    uintptr_t dio_in;
    uint16_t adc_in;

    unsigned int i,count;
    unsigned short chan;

    pthread_attr_t attr;
    pthread_t thread[3];

    printf("\fSet-up Routine for PCI-DAS 1602\n\n");
    memset(&info,0,sizeof(info));
    if(pci_attach(0)<0) {
      perror("pci_attach");
      exit(EXIT_FAILURE);
    }

    // Vendor and Device ID
    info.VendorId=0x1307;
    info.DeviceId=0x01;

    if ((hdl=pci_attach_device(0, PCI_SHARE|PCI_INIT_ALL, 0, &info))==0) {
      perror("pci_attach_device");
      exit(EXIT_FAILURE);
      }

    // Determine assigned BADRn IO addresses for PCI-DAS1602
    printf("\nDAS 1602 Base addresses:\n\n");
    for(i=0;i<5;i++) {
      badr[i]=PCI_IO_ADDR(info.CpuBaseAddress[i]);
      printf("Badr[%d] : %x\n", i, badr[i]);
      }

    // map I/O base address to user space
    printf("\nReconfirm Iobase:\n");
    for(i=0;i<5;i++) {
      // expect CPU Base Address to be the same as IO Base for PC
      iobase[i]=mmap_device_io(0x0f,badr[i]);
      printf("Index %d : Address : %x ", i,badr[i]);
      printf("IOBASE  : %x \n",iobase[i]);
      }

    // Modify thread control privity
    if(ThreadCtl(_NTO_TCTL_IO,0)==-1) {
      perror("Thread Control");
      exit(1);
      }

    //Create joinable attribute
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /*
    Create three threads:
    1. Waveform Manager
    2. Keyboard I/O
    3. ADC and Switches
    */
    for(i=0;i<3;i++){
        switch (i){
            case 1:{rc = pthread_create(&thread[i], &attr, WaveGenManager, NULL);
                    break;}
            case 2:{rc = pthread_create(&thread[i], &attr, MainIO, NULL);
                    break;}
            case 3:{rc = pthread_create(&thread[i], &attr, PeripheralInputs, NULL);
                    break;}
        }
        if (rc){
            printf("pthread_create() #%d return error! Code %d\n", rc);
            exit(-1);
        }
    }
    pthread_attr_destroy(&attr);

    //Joining MainIO input
    for(i=0;i<3;i++){
        rc = pthread_join(thread[i], &status);
        if (rc){
            printf("pthread_join() #%d return error! Code %d\n", i, rc);
            exit(-1);
        }
    }

    pci_detach_device(hdl);
    return 0;
}
