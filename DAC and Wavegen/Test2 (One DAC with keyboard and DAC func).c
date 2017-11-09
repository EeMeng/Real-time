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

#define HIGHESTFREQ     1750
#define FIFO_DELAY      6500

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

// struct for intermediary field for changing global variables
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
uintptr_t iobase[5];

// Program global variables
bool isOperating=true;

// DACField struct global variables
DACField DAC={true, true, 0, 1, {}, 0, 1, 100, 0, 0, 0, 1, 1};

//Mutexes and conditional variables
pthread_mutex_t MainMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PushMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

//*************************************************************//
//                          Functions
//*************************************************************//
// Supporting function: check whether DAC will have negative data
bool hasNegative(){
    if( (DAC.mean-DAC.amp) < 0)
        return true;
    return false;
}
// Supporting function: check absolute maximum value
short checkAbsMax(float mean, float amp){
    float absMax=0;
    if(fabs(mean+amp)>fabs(mean-amp))
        absMax = fabs(mean+amp);
    else
        absMax = fabs(mean-amp);
    if(absMax<5)
        return 1;
    else if(absMax<10)
        return 2;
    else {
        printf("\nError: The absolute maximum is more than 10V.\n");
        printf("Either reset the data field or select small absolute value.\n");
        return 0;
    }
}
// Function to directly change the parameters of the DAC
void change(bool onSignal, int wvty, float f, float m, float a){
    DAC.waveform_type=wvty;
    DAC.freq=f;
    DAC.mean=m;
    DAC.amp=a;
    DAC.resetWave=true;
    DAC.isOn=onSignal;
}
//Set the change field to be equal to initial DAC parameters
void setChangeField(ChangeField* CF){
    (*CF).waveform_type=DAC.waveform_type;
    (*CF).freq=DAC.freq;
    (*CF).mean=DAC.mean;
    (*CF).amp=DAC.amp;
    (*CF).isOn=DAC.isOn;
}


//*************************************************************//
//                Command line argument manager
//*************************************************************//
void CLManager (int argc, char **argv){
    int counter, i, j, temp2;
    float temp;    // temporary storage for argument values
    ChangeField CField;
    char* endptr;
    setChangeField(CField);
    for(counter=1;counter<argc;counter++){
        if((counter%5) ==1){
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
                        if(temp>0 && temp <HIGHESTFREQ) //a bit low
                            CField.freq = temp;
                        else
                            printf("Frequency must be in the range (0, %d) Hz\n", HIGHESTFREQ);
                        break;
                    }
                    case 3: {
                        if(checkAbsMax(temp, CField.amp)!=0)
                            CField.mean = temp;
                        else
                            printf("Error: Out of range mean value: %.2f V\n", temp);
                        break;
                    }
                    case 4: {
                        if(checkAbsMax(CField.amp, temp)!=0)
                            CField.amp = temp;
                        else
                            printf("Error: Out of range amplitude value: %.2f V\n", temp);
                        break;
                    }
                }
            }
            else
                printf("Invalid input #%d: %s\n", counter, argv[counter]);
        }
        else {
            temp2 = strtol(argv[counter], &endptr, 2);
            if(temp2==1)
                change(true, CField.waveform_type, CField.freq, CField.mean, CField.amp);
            else
                change(false, CField.waveform_type, CField.freq, CField.mean, CField.amp);
        }
    }
    printf("Ending command line manager function...\n");
    sleep(1);
    return;
}


//*************************************************************//
//                           DAC parts
//*************************************************************//
// Change the bipolar/unipolar mode based on mean and amplitude
// to give best resolution
void chooseBestRes(){
    if(hasNegative())
        if(checkAbsMax(DAC.mean, DAC.amp)==1){
            DAC.plus=(0x0000<<DAC.identity);
            DAC.DAC_mode = 0;
            DAC.output_res=152.59;
        }
        else if (checkAbsMax(DAC.mean, DAC.amp)==2){
            DAC.plus=(0x0100<<DAC.identity);
            DAC.DAC_mode = 1;
            DAC.output_res=305.14;
        }
    else
        if(checkAbsMax(DAC.mean, DAC.amp)==1){
            DAC.plus=(0x0200<<DAC.identity);
            DAC.DAC_mode = 2;
            DAC.output_res=76.29;
        }
         else if (checkAbsMax(DAC.mean, DAC.amp)==2){
            DAC.plus=(0x0300<<DAC.identity);
            DAC.DAC_mode = 3;
            DAC.output_res=152.59;
        }
    return;
}
// Waveform generator
void WaveformGen (){
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
    chooseBestRes();
    DAC.period=1/DAC.freq;
    res = DAC.output_res/1000000;
    if(DAC.DAC_mode<2)
        offset+=0x7FFF;
    switch (DAC.waveform_type){
        case 1: {// sine wave waveform creation
                 printf("\nSetting Sine Wave for DAC[%d]\n", z);
                 delta_incr=2.0*PI/DAC.samples_per_period;	// increment
                 for(i=0;i<DAC.samples_per_period;i++) {
                     dummy= (sinf((float)(i*delta_incr)))* DAC.amp + DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 2: {// triangular wave waveform creation
                 printf("\nSetting Triangular Wave for DAC[%d]\n", z);
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
                 printf("\nSetting Square Wave for DAC[%d]\n", z);
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
void* WaveGenManager (void * pointer){
    pthread_t tid;
    while(1){
        delay(100);
        if(!isOperating)
            pthread_exit(NULL);
        pthread_mutex_lock(&MainMutex);
        if(DAC.isOn==false)
            continue;
        else {
            if(DAC.resetWave==false)
                continue;
            else{
                WaveformGen();
                pthread_create(&tid, NULL, &PushDAC, (void *)&DAC);
            }
        }
        pthread_mutex_unlock(&MainMutex);
    }
}


//*************************************************************//
//        Input manager for switches and analogue inputs
//*************************************************************//
void* PeripheralInputs (void *pointer){
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
    printf("%*s\t\t%s", 6, "2" ,"Show current devices' statuses.\n");
    printf("%*s\t\t%s", 6, "3", "Change the configurations.\n");
    printf("%*s\t\t%s", 6, "4", "Export configurations or waveforms.\n");
    printf("%*s\t\t%s", 6, "5", "Import configurations.\n");
    printf("%*s\t\t%s", 6, "6", "Halt all operations.\n");
    printf("%*s\t\t%s", 6, "7", "Exit program\n");
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
        if(temp>0 && temp <7)
            return temp;
        else if(temp==7)
            return 66;
    }
    else if(strcmp(in,"help")==0)
        return 7;
    else
        printf("Invalid input! Inputted:%s\n", in);
    return 0;
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
	char input100[100]={'\0'};
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
    input[0]=(char *)&source;
    input[j]=&input100[0];
    while(1){
        if(isNull){
            input[++j]=&input100[i];
            isNull=false;
        }
        if(fscanf(fd, "%c", &input100[i])==EOF)
            break;
        if(input100[i]=='\n' || input100[i]==' '){
            input100[i]='\0';
            isNull=true;
        }
        i++;
    }
    pthread_mutex_lock(&MainMutex);
    CLManager(j, (char**) &input);
    pthread_mutex_unlock(&MainMutex);
    fflush(fd);
    fclose(fd);
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
    fclose(fd);
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
    return -100;
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
//Change the parameters of the DAC0 and DAC1
void changeParam(){
    ChangeField CField;
    bool isRepeat=false;
    char repeatchar;
    char input[5];
    int select1=0;
    int select2=0;
    float temp;
    do{
        setChangeField(&CField);
        printf("\nCurrent OFF/ON (0/1) status: %d\n", CField.isOn);
        printf("Select option:\n");
        printf("1 - Change the waveform type of DAC[0]\n");
        printf("2 - Change the frequency of DAC[0]\n");
        printf("3 - Change the mean of DAC[0]\n");
        printf("4 - Change the amplitude of DAC[0]\n");
        printf("5 - Change the OFF/ON (0/1) status of DAC[0]\n");
        printf("Enter option: ");
        if((select1=checkValidInt())>=0){
            switch (select1){
                case 1: {
                    printf("\nChanging waveform type of DAC[0]\n");
                    printf("Enter 1 for sinusoidal waveform\n");
                    printf("Enter 2 for triangular waveform\n");
                    printf("Enter 3 for square waveform\n");
                    printf("Enter option: ");
                    if((select2=checkValidInt())>=0)
                        if(select2>0 && select2<4){
                            CField.waveform_type=select2;
                            printf("\nChanged waveform type of DAC[0]\n");
                        }
                        else
                            printf("Invalid waveform selection.\n");
                    else
                        printf("Invalid input for waveform. (Must be integer)");
                    break;
                    }
                case 2: {
                    printf("\nChanging frequency (float) of DAC[0]\n");
                    printf("Frequency must be in the range (0,%d) Hz\n", HIGHESTFREQ);
                    printf("Enter frequency (Hz): ");
                    if((temp=checkValidFloat())>=0)
                        if(temp>0 && temp <HIGHESTFREQ) {
                            CField.freq=temp;
                            printf("\nChanged freq of DAC[0]\n");
                        }
                    else
                        printf("Error: Frequency is not changed.\n", HIGHESTFREQ);
                    break;
                    }
                case 3: {
                    printf("\nChanging mean (float) of DAC[0]\n");
                    printf("Mean value must be in the range (%.2f,%.2f)\n",
                           -(10-CField.amp), 10-CField.amp);
                    printf("Enter mean (V): ");
                    if((temp=checkValidFloat())>= -10)
                        if(checkAbsMax(temp, CField.amp)!=0){
                            CField.mean=temp;
                            printf("\nChanged mean of DAC[0]\n");
                        }
                        else
                            printf("Mean value is not changed.\n");
                    break;
                    }
                case 4: {
                    printf("\nChanging amplitude (float) of DAC[0]\n");
                    printf("Amplitude value must be in the range (0,%.2f)\n"
                           , 10-CField.mean);
                    printf("Enter amplitude (V): ");
                    if((temp=checkValidFloat())>= 0)
                        if(checkAbsMax(temp, CField.amp)!=0){
                            CField.amp=temp;
                            printf("\nChanged amplitude of DAC[0]\n");
                        }
                        else
                            printf("Amplitude value is not changed.\n");
                    break;
                    }
                case 5: {
                    printf("\nChanging OFF/ON (0/1) state of the DAC[0]\n");
                    printf("Enter option (0/1): ");
                    if((select2=checkValidInt())>=0)
                        if(select2==0)
                            if (CField.isOn==false)
                                printf("DAC[0] is already off.\n");
                            else
                                CField.isOn=false;
                        else if(select2==1)
                            if (CField.isOn==false)
                                CField.isOn=true;
                            else
                                printf("DAC[0] is already on.\n");
                    else
                        printf("Invalid options for OFF/ON.\n");
                    break;
                }
                default: printf("\nInvalid choice.\n");
            }
        // change the parameters with mutex
            if(select1<6 && select1>0){
                pthread_mutex_lock(&MainMutex);
                change(CField.isOn, CField.waveform_type,
                CField.freq, CField.mean, CField.amp);
                pthread_mutex_unlock(&MainMutex);
            }
        //wait for WaveGenManager to finish its configuration
        sleep(1);
        }
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
    char input[10];
    sleep(1);  //wait for WaveGenManager to finish its configuration
    while (1) {
    	displayHelp();
		getInput(&input[0]);
		switch (checkInput(&input[0])) {
		    // case 1 - print current devices' configurations.
			case 1: {   showConfig(); break; }
            // case 2 - print current devices' statuses.
			//case 2: {   showStatus();       break; }
			// case 3 - change the configurations.
			case 3: {   changeParam();       break; }
			// case 4 - export configurations or waveforms.
			case 4: {   exportConfig();  break; }
			// case 5 - import configurations.
			case 5: {   importConfig();  break; }
			// case 6 - halt all operations.
			case 6: {   stopOps(); break; }
            // case 7 - display help
			case 7: {   displayHelp(); break; }
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
//                      Main function
//*************************************************************//
int main(int argc, char** argv) {
    //PCI device variable
    struct pci_dev_info info;
    void *hdl;
	int rc;

    uintptr_t dio_in;
    uint16_t adc_in;

    unsigned int i, count;
    unsigned short chan;

    pthread_attr_t attr;
    pthread_t thread[3];

    //Call command line manager
    CLManager (argc, argv);

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
    for(i=0;i<2;i++){
        switch (i){
            case 0:{rc = pthread_create(&thread[i], &attr, &WaveGenManager, NULL);
                    break;}
            case 1:{rc = pthread_create(&thread[i], &attr, &MainIO, NULL);
                    break;}
            case 2:{rc = pthread_create(&thread[i], &attr, &PeripheralInputs, NULL);
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
