#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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

#define HIGHESTFREQ 1800
#define FIFO_DELAY 6500
#define THRESHOLD 50

//struct for DAC waveform
typedef struct {
    bool resetWave;
    bool isOn;
    const short	identity;
    unsigned short waveform_type;
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
//
typedef struct {
    int waveform_type;
    float freq;
    float mean;
    float amp;
    bool isOn;
}ChangeField ;
// PCI 2.2 assigns 6 IO base addresses
// PCI device global variables
int badr[5];
uintptr_t iobase[6];

struct pci_dev_info info;
void *hdl;

// Program global variables
bool isOperating = true;
bool keyboard_enable = false;
uintptr_t dio_in;                                       // digital input output
uint16_t adc_in;                                        // reading potentialmeter
bool ADC_Refresh;

// DACField struct global variables
DACField DAC={true, false, 0, 1, {}, 0, 1, 100, 0, 0, 0, 10, 2};

//Mutexes and conditional variables
pthread_mutex_t MainMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PushMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

//Function Declaration for Housekeeping

//ADC & GPIO & Manager
void switches();
void check_potentiometer(ChangeField* CField);
void ADC_Help();

//DAC
void chooseBestRes(int z);
void WaveformGen (int z);
void* PushDAC (void* Curr);
void* WaveGenManager (void * pointer);

//Keyboard & Manager
void CLManager (int argc, char **argv);
void displayHelp();
void getInput(char* in); 
int checkInput(char* in);
void showConfig();
void importConfig();
void exportConfig();
void changeParam();
void stopOps();
void* MainIO (void *pointer);

//Signal
void signal_handler(int signum);

//Utilities
void change(bool onSignal, int wvty, float f, float m, float a);
bool hasNegative();
float absMaximum();
unsigned int old_to_new(unsigned int old_val, unsigned int new_val);
float checkValidFloat();
int checkValidInt();
void setChangeField(ChangeField* CF);
void init();

//*************************************************************//
//                      Initialization
//*************************************************************//

void init()
{
    struct pci_dev_info info;
    void *hdl;
    int rc, i;

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
}

//*************************************************************//
//                      Main function
//*************************************************************//
int main(int argc, char** argv) {


	int rc, i;
    pthread_attr_t attr;
    pthread_t thread[2];

    //Create joinable attribute
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    signal(SIGINT, signal_handler);
    init();

    //Call command line manager
    CLManager (argc, argv);

    /*
    Create three threads:
    1. Waveform Manager
    2. Keyboard I/O
    3. ADC and Switches
    */

 for(i=0;i<2;i++){
        switch (i){
            case 0:{rc = pthread_create(&thread[i], &attr, &WaveGenManager, NULL);
                    break;}
            case 1:{rc = pthread_create(&thread[i], &attr, &MainIO, NULL);
                    break;}
        }
        if (rc){
            printf("pthread_create() #%d return error! Code %d\n", rc);
            exit(-1);
        }
    }
    pthread_attr_destroy(&attr);

    //Joining MainIO input
    for(i=0;i<2;i++){
        rc = pthread_join(thread[i], NULL);
        if (rc){
            printf("pthread_join() #%d return error! Code %d\n", i, rc);
            exit(-1);
        }
    }

    pci_detach_device(hdl);
    return 0;

}

//*************************************************************//
//                          Functions
//*************************************************************//
// Function to directly change the parameters of the DAC
void change(bool onSignal, int wvty, float f, float m, float a){
    DAC.waveform_type=wvty;
    DAC.freq=f;
    DAC.mean=m;
    DAC.amp=a;
    DAC.resetWave=true;
    DAC.isOn=onSignal;
}

//*************************************************************//
//                          Signal
//*************************************************************//
void signal_handler(int signum)
{
	isOperating = false;
	pthread_mutex_lock(&PushMutex);
	DAC.isOn = false;
	pthread_mutex_unlock(&PushMutex);
	printf("\f");
	printf("Exiting...\n");
	delay(1000);
	exit(0);
}

//*************************************************************//
//                Command line argument manager
//*************************************************************//
void CLManager (int argc, char **argv){
    int counter, i, temp2;
    ChangeField CField;
    int waveform_type=0;    // counter for argument
    float temp,freq,mean,amp;    // temporary storage for argument values
    char* endptr;
    setChangeField(&CField);
    for(counter=1;counter < argc;counter++){
        if((counter%6) ==1){
            if(strcmp(argv[counter],"-sin") == 0)
                CField.waveform_type=1;
            else if(strcmp(argv[counter],"-tri") == 0)
                CField.waveform_type=2;
            else if(strcmp(argv[counter],"-squ") == 0)
                CField.waveform_type=3;
            else
                printf("Invalid waveform.\n");
        }
        else  if ((counter%5) !=0){
            temp = strtod(argv[counter], &endptr);
            if(*endptr == '\0'){
                switch(counter%5){
                    case 2: {
                        if(temp>0 && temp <1000) //a bit low
                            CField.freq = temp;
                        else
                            printf("Frequency must be in the range (0, 1000) Hz\n");
                        break;
                    }
                    case 3: {
                        if(fabs(temp)<=5)
                            CField.mean = temp;
                        else
                            printf("Mean value must be in the range [-5,5]\n");
                        break;
                    }
                    case 4: {
                        if(temp < 5 && temp>=0)
                            CField.amp = temp;
                        else
                            printf("Amplitude must be in the range [0,5]\n");
                        break;
                    }
                }
            }
            else
                printf("Invalid input: %s\n", argv[counter]);

        }
         else {
            CField.isOn = strtol(argv[counter], &endptr, 2);
            if(!(CField.isOn == 0 || CField.isOn == 1))
            {
            	printf("On/Off must be in the range 0 or 1\n");
            	printf("Setting Off as default.\n");
            	CField.isOn = 0;
            }
            pthread_mutex_lock(&PushMutex);
    		change(CField.isOn, CField.waveform_type, CField.freq, CField.mean, CField.amp);
    		pthread_mutex_unlock(&PushMutex);
      
        }
    }
	          
    
    printf("Ending command line manager function...\n");
    sleep(1);
    return;
}


//*************************************************************//
//                           DAC parts
//*************************************************************//
// Supporting function: check whether DAC will have negative data
bool hasNegative(){
    if( (DAC.mean-DAC.amp) < 0)
        return true;
    return false;
}
// Supporting function: find absolute maximum value
float absMaximum(){
    if(fabs(DAC.mean+DAC.amp)>fabs(DAC.mean-DAC.amp))
        return fabs(DAC.mean+DAC.amp);
    else
        return fabs(DAC.mean-DAC.amp);
}
// Change the bipolar/unipolar mode based on mean and amplitude
// to give best resolution
void chooseBestRes(int z){
    if(hasNegative(z))       //Only DAC0 is used
        if(absMaximum(z)<5){
            DAC.plus=(0x0000<<DAC.identity);
            DAC.DAC_mode = 0;
            DAC.output_res=152.59;
        }
        else{
            DAC.plus=(0x0100<<DAC.identity);
            DAC.DAC_mode = 1;
            DAC.output_res=305.14;
        }
    else
        if(absMaximum(z)<5){
            DAC.plus=(0x0200<<DAC.identity);
            DAC.DAC_mode = 2;
            DAC.output_res=76.29;
        }
        else{
            DAC.plus=(0x0300<<DAC.identity);
            DAC.DAC_mode = 3;
            DAC.output_res=152.59;
        }
}
// Waveform generator
void WaveformGen (int z){
    int i=0;
    double delta_incr, dummy, res;
    unsigned short offset=0;
    chooseBestRes(z);
    DAC.period=1/DAC.freq;
    res = DAC.output_res/1000000;
    if(DAC.DAC_mode<2)
        offset+=0x7FFF;
    switch (DAC.waveform_type){
        case 1: {// sine wave waveform creation
                 //printf("\nSetting Sine Wave for DAC[%d]\n", z);
                 delta_incr=2.0*PI/DAC.samples_per_period;	// increment
                 for(i=0;i<DAC.samples_per_period;i++) {
                     dummy= (sinf((float)(i*delta_incr)))* DAC.amp + DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 2: {// triangular wave waveform creation
                 //printf("\nSetting Triangular Wave for DAC[%d]\n", z);
                 delta_incr=4*DAC.amp/DAC.samples_per_period;	// increment
                 for(i=0;i<DAC.samples_per_period/4;i++) {
                     dummy= delta_incr*i +DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 for(;i<(3*DAC.samples_per_period/4);i++) {
                     dummy= 2*DAC.amp-delta_incr*i +DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 for(;i<DAC.samples_per_period;i++) {
                     dummy= -4*DAC.amp+delta_incr*i +DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 3: {// square wave waveform creation
                 //printf("\nSetting Square Wave for DAC[%d]\n", z);
                 for(i=0;i<DAC.samples_per_period/2;i++) {
                     dummy= DAC.amp + DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 for(;i<DAC.samples_per_period;i++) {
                     dummy= -DAC.amp + DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                break;
                }
        }
    DAC.resetWave=false;
    return;
}

// Function to push-out data to DAC(thread function)
void* PushDAC (void* Curr){
    DACField* Current = (DACField*) Curr;
    int i = 0;
    unsigned short CTLREG_content;
    struct timespec sleep;
    CTLREG_content=(unsigned short)((*Current).plus+((*Current).identity+0x1)*0x20+0x3);
	sleep.tv_sec = 0;
	sleep.tv_nsec = (long)(1000000000.0/(Current->freq*Current->samples_per_period));
    while (1){
        for(i=0;i<(*Current).samples_per_period;i++) {
            //does not exit only when isOn==true and resetWave==false
            if(!((*Current).resetWave==false && (*Current).isOn==true))
                pthread_exit(NULL);
                out16(DA_CTLREG, CTLREG_content);       // write setting to DAC CTLREG
                out16(DA_FIFOCLR, 0);					// Clear DA FIFO buffer
                out16(DA_Data, (*Current).data[i]);     // Output data
                nanospin_ns(sleep.tv_nsec - FIFO_DELAY);
        }
    }
    return (0);
}
// Wave generator manager (thread function)
void* WaveGenManager (void * pointer){int i=0;
    pthread_t tid[2];
    while(1){
        delay(100);
        if(!isOperating)
            pthread_exit(NULL);
        if(DAC.isOn==false)
            continue;
        else{
            if(DAC.resetWave==false)
                continue;
            else{
                pthread_mutex_lock(&MainMutex);
                WaveformGen(i);
                pthread_mutex_unlock(&MainMutex);
                pthread_create(&tid[i], NULL, &PushDAC, (void *)&DAC);
            }
        }
    }
}



//*************************************************************//
//        Input manager for switches and analogue inputs
//*************************************************************//

void ADC_Help(){
		printf("Waveform generation.\n");
        printf("Switches are labelled 1 to 4 from left to right.\n");
        printf("Switch 1 - Toggle switches on and off.\n");
        printf("Switch 2 - Toggle Amplitude(0) or Mean(1) Variation.\n");
        printf("Switch 3 & 4 - Toggle Wave Shape\n\n");
        printf("  Wave       Switch 3       Switch 4\n");
        printf("========     =========     =========\n");
        printf("  Sine          1              1\n");
        printf("Triangle        0              1\n");
        printf(" Square         1              0\n");
        printf("  Off           0              0\n");
        printf("====================================\n\n");
}

void switches()                              // thread to control the switch
{
ChangeField CField;
int truth = 0;
while(1)
{
  	if(!isOperating) return;
  	setChangeField(&CField);
    //printf("\nDIO Functions\n");
    out8(DIO_CTLREG,0x90);                  // Port A : Input,  Port B : Output,  Port C (upper | lower) : Output | Output

    dio_in=in8(DIO_PORTA);                  // Read Port A
    //printf("Port A : %02x\n", dio_in);

    out8(DIO_PORTB, dio_in);                    // output Port A value -> write to Port B

    if ((dio_in & 0x08)==0)
    {
        printf("The board is off.\n");
        return;
    }
    else if(truth != dio_in)
    {
        truth = dio_in;
        ADC_Refresh = true;
    }
    else ADC_Refresh = false;
    check_potentiometer(&CField);
     switch(dio_in & 0x03)                                                              //dio_in defines the mode below
        {
            case 0x00:    CField.isOn = 0;  break;
            case 0x01:    CField.waveform_type = 2; CField.isOn = 1; break;
            case 0x02:    CField.waveform_type = 3; CField.isOn = 1; break;
            case 0x03:    CField.waveform_type = 1; CField.isOn = 1; break;
        }
    if(ADC_Refresh)
    {
    pthread_mutex_lock(&PushMutex);
    change(CField.isOn, CField.waveform_type, CField.freq, CField.mean, CField.amp);
    pthread_mutex_unlock(&PushMutex);
    printf("\f");
    ADC_Help();
    switch(dio_in & 0x03)                                                              //dio_in defines the mode below
        {
            case 0: 	printf("Selection: Off\n"); break;
            case 1:		printf("Selection: Sine Wave\n"); break;
            case 2:    	printf("Selection: Triangle Wave\n"); break;
            case 3:    	printf("Selection: Square Wave\n"); break;
            default:    printf("Invalid Option!\n"); break;
        }
    switch(dio_in & 0x04)
    {
    	case 0x04: printf("Varying: Mean\n"); break;
    	case 0x00: printf("Varying: Amplitude\n"); break;
    }
    printf("Mean:%f, Amp: %f, Freq: %f\n", DAC.mean, DAC.amp, DAC.freq);
    ADC_Refresh = false;
    }
    
    delay(100);
}
    return;
}

unsigned int old_to_new(unsigned int  new_val,unsigned int old_val)                                                         // comparative function
{
   	if ((abs(new_val-old_val))< THRESHOLD)return old_val;
    else {
    ADC_Refresh = true;
    return new_val;
    }
}

void check_potentiometer(ChangeField* CField)                                               // pontentiometer thread
{
    unsigned short chan;
    unsigned int count;
    int ampmean;
    unsigned int i;

    out16(INTERRUPT,0x60c0);                // sets interrupts   - Clears
    out16(TRIGGER,0x2081);                  // sets trigger control: 10MHz, clear, Burst off,SW trig. default:20a0
    out16(AUTOCAL,0x007f);                  // sets automatic calibration : default

    out16(AD_FIFOCLR,0);                        // clear ADC buffer
    out16(MUXCHAN,0x0D00);              // Write to MUX register - SW trigger, UP, SE, 5v, ch 0-0
                                                        // x x 0 0 | 1  0  0 1  | 0x 7   0 | Diff - 8 channels
                                                        // SW trig |Diff-Uni 5v| scan 0-7| Single - 16 channels
    ampmean=dio_in;
    count=0x00;
    if (!(dio_in & 0x04))                                // Change Amp Value
    {
        while(count <0x02)
        {
            chan= ((count & 0x0f)<<4) | (0x0f & count);
            out16(MUXCHAN,0x0D00|chan);     // Set channel   - burst mode off.
            delay(1);                                           // allow mux to settle
            out16(AD_DATA,0);                           // start ADC
            while(!(in16(MUXCHAN) & 0x4000));
            adc_in=in16(AD_DATA);
            if (count == 0x00)CField->amp = old_to_new(adc_in, DAC.amp*65535.0/5.0)*5.0/65535.0;
	        if (count == 0x01)CField->freq = old_to_new(adc_in, DAC.freq*65535.0/1000.0)*1000.0/65535.0;                                // Da0 freq
           
            count++;
            //                                      // Write to MUX register - SW trigger, UP, DE, 5v, ch 0-7
        }
    }

    else if(dio_in & 0x04)                              // Change Mean Value
    {
        while(count <0x02)
        {
            chan= ((count & 0x0f)<<4) | (0x0f & count);
            out16(MUXCHAN,0x0D00|chan);     // Set channel   - burst mode off.
            delay(1);                                           // allow mux to settle
            out16(AD_DATA,0);                           // start ADC
            while(!(in16(MUXCHAN) & 0x4000));
            adc_in=in16(AD_DATA);
            if (count == 0x00)CField->mean = old_to_new(adc_in, (DAC.mean + 5)*65535.0/10.0)*10.0/65535.0 - 5;                        //Da0_mean mean mode base on switch 2
            if (count == 0x01)CField->freq = old_to_new(adc_in, DAC.freq*65535.0/1000.0)*1000.0/65535.0;                                     //compare free values for resolution
            count++;
            //                                      // Write to MUX register - SW trigger, UP, DE, 5v, ch 0-7
        }
    }
    if(CField->freq < 0.1)CField->freq = 0.1;
    return;
  }


//*************************************************************//
//            Primary input (keyboard) manager
//*************************************************************//
//Display help menu
void displayHelp() {
    printf("\n*************\n");
	printf("Help menu\n");
	printf("*************\n");
	printf("%*s\t\t%s", 6, "1", "Show current devices' configurations.\n");
    printf("%*s\t\t%s", 6, "2" ,"Use Potentiometer and Flip switches.\n");
    printf("%*s\t\t%s", 6, "3", "Change the configurations.\n");
    printf("%*s\t\t%s", 6, "4", "Export configurations or waveforms.\n");
    printf("%*s\t\t%s", 6, "5", "Import configurations.\n");
    printf("%*s\t\t%s", 6, "6", "Halt all operations.\n");
    printf("%*s\t\t%s", 6, "7", "Display help.\n");
    printf("%*s\t\t%s", 6, "8", "Exit program\n");

}
//Get input from keyboard
void getInput(char* in) {
	printf("\nPlease enter your command: ");
	fflush(stdout);
	scanf("%s", in);
	// clear input buffer after each acquisition of input
	fflush(stdin);
	printf("\f");
}
//Check the input validity in MainIO
int checkInput(char* in) {
    int temp=0;
    char* endptr;
    temp = strtol(in, &endptr, 10);
    // check if valid integer is inputted
    if(*endptr == '\0'){
        if(temp>0 && temp <9)
            return temp;
    }
    else if(strcmp(in,"help")==0)
        return 7;
    else
        printf("Invalid input! Inputted:%s\n", in);
}
//Show current DAC configuration
void showConfig(){
    printf("%*s\n", 38, "DAC0");
    printf("%*s%*d\n", 25, "Running? (0-OFF, 1-ON)", 15, DAC.isOn);
    printf("%*s", 25, "Waveform type");
	switch(DAC.waveform_type){
        case 1: { printf("%*s", 15, "Sinusoidal"); break;}
        case 2: { printf("%*s", 15, "Triangular"); break;}
        case 3: { printf("%*s", 15, "Square"); break;}
    }

    printf("\n");
    printf("%*s%*d\n", 25,"Samples per period", 15, DAC.samples_per_period);
    printf("%*s%*.2E\n", 25, "Output resolution (V)", 15, DAC.output_res);
    printf("%*s%*.2f\n", 25, "Frequency (Hz)", 15, DAC.freq);
    printf("%*s%*.2f\n", 25, "Amplitude (V)", 15, DAC.amp);
    printf("%*s%*.2f\n", 25, "Mean (V)", 15, DAC.mean);
    return;
}
//Import the configuration from .txt file
void importConfig(){
	int i=0;
	int j=1;
	bool isNull=false;
	FILE* fd;
	char filename[30];
	const char source[10]="imported";
	char buffer[100]={'\0'};
	char *input[12];
    printf("Please enter filename (\".txt\" is added at the end): ");
    fflush(stdout);
    scanf("%s" , filename);
    strcat(filename, ".txt");
    fd = fopen(filename,"r+");
    if(fd == NULL){
        printf("Failed to open %s.\n", filename);
        return;
    }
    input[0]=(char *) &source;
    input[j]=&buffer[0];
    while(1){
        if(isNull){
            input[++j]= &buffer[i];
            isNull=false;
            
        }
         if(fscanf(fd, "%c", &buffer[i])==EOF)
         {
         	input[++j]= &buffer[i];
         	break;
         }
        if(buffer[i]=='\n' || buffer[i]==' '){
            buffer[i]='\0';
            isNull=true;
        }
        i++;
    }
    pthread_mutex_lock(&MainMutex);
    CLManager(j, (char**) &input);
    pthread_mutex_unlock(&MainMutex);
    fflush(fd);
    close((int)fd);
    printf("Configuration from %s is loaded.\n", filename);
    sleep(1);
    return;
}
//Export the configuration to .txt file
void exportConfig(){
	FILE* fd;
	char filename[30];
    printf("Please enter filename (\".txt\" is added at the end): ");
    fflush(stdout);
    scanf("%s" , filename);
    strcat(filename, ".txt");
    fd = fopen(filename,"w+");
    if(fd == NULL){
        printf("Failed to open/create %s.\n", filename);
        return;
    }
        switch(DAC.waveform_type){
            case 1:{fprintf(fd, "%s ", "-sin"); break;}
            case 2:{fprintf(fd, "%s ", "-tri"); break;}
            case 3:{fprintf(fd, "%s ", "-squ"); break;}
        }
        fprintf(fd, "%.2f %.2f %.2f %d\n",
                DAC.freq, DAC.mean, DAC.amp, DAC.isOn);
	fflush(fd);
    close((int)fd);
    printf("Configuration saved to file.\n");
    return;
}
//Check validity of floating point number and integers
//- Return the number if valid
//- Return -1 if invalid
float checkValidFloat(){
    char input[12];
    char* pointer;
    float temp=0;
    fflush(stdout);
    scanf("%s", input);
    fflush(stdin);
    temp = strtod(input, &pointer);
    if(*pointer=='\0')
        return temp;
    else
        printf("Invalid floating point number!\n");
    return -1;
}
int checkValidInt(){
    char input[5];
    char* pointer;
    float temp=0;
    fflush(stdout);
    scanf("%s", input);
    fflush(stdin);
    temp = strtol(input, &pointer, 10);
    if(*pointer=='\0')
        return temp;
    return -1;
}
//Set the change field to be equal to initial DAC parameters
void setChangeField(ChangeField* CF){
    (*CF).waveform_type=DAC.waveform_type;
    (*CF).freq=DAC.freq;
    (*CF).mean=DAC.mean;
    (*CF).amp=DAC.amp;
    (*CF).isOn=DAC.isOn;
}
//Change the parameters of the DAC0 and DAC1
void changeParam(){
    ChangeField CField = {0, 0, 0, 0, false};
    bool isRepeat=false;
    char repeatchar;
    char input[5];
    int select1=0;
    int select2=0;
    float temp;
    char *pointer;
    do{
            printf("\f");
            printf("DAC0 selected.\n");
            setChangeField(&CField);
            printf("\nCurrent OFF/ON (0/1) status: %d\n", CField.isOn);
            printf("Select option:\n");
            printf("1 - Change the waveform type of DAC[%d]\n", 0);
            printf("2 - Change the frequency of DAC[%d]\n", 0);
            printf("3 - Change the mean of DAC[%d]\n", 0);
            printf("4 - Change the amplitude of DAC[%d]\n", 0);
            printf("5 - Change the OFF/ON (0/1) status of DAC[%d]\n", 0);
            printf("Enter option: ");
            if((select1=checkValidInt())>=0){
                switch (select1){
                    case 1: {
                        printf("\nChanging waveform type of DAC[%d]\n", 0);
                        printf("Enter 1 for sinusoidal waveform\n");
                        printf("Enter 2 for triangular waveform\n");
                        printf("Enter 3 for square waveform\n");
                        printf("Enter option: ");
                        if((select2=checkValidInt())>=0)
                            if(select2>0 && select2<4){
                                CField.waveform_type=select2;
                                printf("\nChanged waveform type of DAC[%d]\n", 0);
                            }
                            else
                                printf("Invalid waveform selection.\n");
                        else
                            printf("Invalid input for waveform. (Must be integer)");
                        break;
                        }
                    case 2: {
                        printf("\nChanging frequency (float) of DAC[%d]\n", 0);
                        printf("Enter frequency (Hz): ");
                        if((temp=checkValidFloat())>=0)
                        if(temp>0 && temp <HIGHESTFREQ) {
                            CField.freq=temp;
                            printf("\nChanged freq of DAC[%d]\n", 0);
                        }
                        else
                            printf("Frequency must be in the range (0, %d) Hz\n", HIGHESTFREQ);
                        break;
                        }
                    case 3: {
                        printf("\nChanging mean (float) of DAC[%d]\n", 0);
                        printf("Enter mean (V): ");
                        if((temp=checkValidFloat())>=0)
                        if(fabs(temp)<=10){
                            CField.mean=temp;
                            printf("\nChanged mean of DAC[%d]\n", 0);
                        }
                        else
                            printf("Mean value must be in the range [-10,10]\n");
                        break;
                        }
                    case 4: {
                        printf("\nChanging amplitude (float) of DAC[%d]\n", 0);
                        printf("Enter amplitude (V): ");
                        if((temp=checkValidFloat())>=0)
                        if(temp < 10 && temp>=0){
                            CField.amp=temp;
                            printf("\nChanged amplitude of DAC[%d]\n", 0);
                        }
                        else
                            printf("Amplitude must be in the range [0,5]\n");
                        break;
                        }
                    case 5: {
                        printf("\nChanging OFF/ON (0/1) state of the DAC[%d]\n", 0);
                        printf("Enter option (0/1): ");
                        if((select2=checkValidInt())>=0)
                            if(select2==0)
                                if (CField.isOn==false)
                                    printf("ADC[%d] is already off.\n", 0);
                                else
                                    CField.isOn=false;
                            else if(select2==1)
                                if (CField.isOn==false)
                                    CField.isOn=true;
                                else
                                    printf("ADC[%d] is already on.\n", 0);
                        else
                            printf("Invalid options for OFF/ON.\n");
                        break;
                    }
                    default: printf("\nInvalid choice.\n");
                }
            // should put mutex here
            change(CField.isOn, CField.waveform_type, CField.freq,
                   CField.mean, CField.amp);
            sleep(1);  //wait for WaveGenManager to finish its configuration
        }
        else
            printf("Invalid input. Please enter correct DAC number\n");
        printf("\nDo you want to change other parameters? (Y/N): ");
        fflush(stdout);
		scanf("%s", input);
		fflush(stdin);
        if(input[0]=='y' ||input[0]=='Y')
            isRepeat=true;
        else if (input[0]=='n' ||input[0]=='N')
            isRepeat=false;
        else{
            printf("Invalid choice. Returning to main IO.\n");
            isRepeat=false;
        }
    } while(isRepeat);
}
//Stop operation (will turn on again if the switches are on)
void stopOps(){
    pthread_mutex_lock(&MainMutex);
    DAC.isOn=false;
  	pthread_mutex_unlock(&MainMutex);
    return;
}
//Main function for keyboard thread
void* MainIO (void *pointer){
    char input10[10];
    char* input = input10;
    sleep(1);  //wait for WaveGenManager to finish its configuration
    printf("\f");
    while (1) {
            delay(500);
    	displayHelp();
		getInput(input);
		switch (checkInput(input)) {
		    // case 1 - print current devices' configurations.
			case 1: {   showConfig(); break; }
            // case 2 - print current devices' statuses.
			case 2: {   switches(); printf("\f"); break; }
			// case 3 - change the configurations.
			case 3: {   changeParam(); printf("\f"); break; }
			// case 4 - export configurations or waveforms.
			case 4: {   exportConfig(); printf("\f");  break; }
			// case 5 - import configurations.
			case 5: {   importConfig(); printf("\f"); break; }
			// case 6 - halt all operations.
			case 6: {   stopOps(); break; }
            // case 7 - Switch to switches
            case 7: 	displayHelp(); printf("\f"); break;
            // case 8 - Turns off program
			case 8: {   isOperating=false; break;}
			//show error in input
			default:{   printf("Invalid character. Please reenter. \n");
                        printf("To display help, enter 'help'.\n");
                    }
		}
		if(!isOperating)
            pthread_exit(NULL);
    }
}

