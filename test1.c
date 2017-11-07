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
#define FBEST           360000


//struct for DAC waveform
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
uintptr_t iobase[5];

// Program global variables
bool isOperating=true;
int f_available=FBEST;

// DACField struct global variables
DACField DAC[2]={   {true, true, 3, 0,{}, 0, 1, 200, 0, 0, 6, 10, 2},
                    {true, true, 3 ,1,{}, 0, 1, 100, 0, 0, 0, 10, 1}};

//Mutexes and conditional variables
pthread_mutex_t MainMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PushMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

//*************************************************************//
//                          Functions
//*************************************************************//
// Function to directly change the parameters of the DAC
void change(bool onSignal, int DACID, int wvty, float f, float m, float a){
    DAC[DACID].waveform_type=wvty;
    DAC[DACID].freq=f;
    DAC[DACID].mean=m;
    DAC[DACID].amp=a;
    DAC[DACID].resetWave=true;
    DAC[DACID].isOn=onSignal;
}


//*************************************************************//
//                Command line argument manager
//*************************************************************//
void CLManager (int argc, char **argv){
    int counter, i, j, temp2;
    int waveform_type=0;    // counter for argument
    float temp,freq,mean,amp;    // temporary storage for argument values
    char* endptr;
    j=0;
    for(counter=1;counter<argc;counter++){
        if((counter%5) ==1){
            if(strcmp(argv[counter],"-sin") == 0)
                waveform_type=1;
            else if(strcmp(argv[counter],"-tri") == 0)
                waveform_type=2;
            else if(strcmp(argv[counter],"-squ") == 0)
                waveform_type=3;
            else
                printf("Invalid waveform.\n");
        }
        else  if ((counter%5) !=0){
            temp = strtod(argv[counter], &endptr);
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
                        if(fabs(temp)<=10)
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
        else {
            temp2 = strtol(argv[counter], &endptr, 2);
            if(temp2==1)
                change(true, j++, waveform_type, freq, mean, amp);
            else
                change(false, j++, waveform_type, freq, mean, amp);
        }
    }
    return;
}


//*************************************************************//
//                           DAC parts
//*************************************************************//
// Supporting function: check whether DAC will have negative data
bool hasNegative(int z){
    if( (DAC[z].mean-DAC[z].amp) < 0)
        return true;
    return false;
}
// Supporting function: find absolute maximum value
float absMaximum(int z){
    if(fabs(DAC[z].mean+DAC[z].amp)>fabs(DAC[z].mean-DAC[z].amp))
        return fabs(DAC[z].mean+DAC[z].amp);
    else
        return fabs(DAC[z].mean-DAC[z].amp);
}
// Change the bipolar/unipolar mode based on mean and amplitude
// to give best resolution
void chooseBestRes(int z){
    if(hasNegative(z))       //Only DAC0 is used
        if(absMaximum(z)<5){
            DAC[z].plus=(0x0000<<DAC[z].identity);
            DAC[z].DAC_mode = 0;
            DAC[z].output_res=152.59;
        }
        else{
            DAC[z].plus=(0x0100<<DAC[z].identity);
            DAC[z].DAC_mode = 1;
            DAC[z].output_res=305.14;
        }
    else
        if(absMaximum(z)<5){
            DAC[z].plus=(0x0200<<DAC[z].identity);
            DAC[z].DAC_mode = 2;
            DAC[z].output_res=76.29;
        }
        else{
            DAC[z].plus=(0x0300<<DAC[z].identity);
            DAC[z].DAC_mode = 3;
            DAC[z].output_res=152.59;
        }
}
// Waveform generator
void WaveformGen (int z){
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
    chooseBestRes(z);
    DAC[z].period=1/DAC[z].freq;
    res = DAC[z].output_res/1000000;
    if(DAC[z].DAC_mode<2)
        offset+=0x7FFF;
    switch (DAC[z].waveform_type){
        case 1: {// sine wave waveform creation
                 printf("\nSetting Sine Wave\n");
                 delta_incr=2.0*PI/DAC[z].samples_per_period;	// increment
                 for(i=0;i<DAC[z].samples_per_period;i++) {
                     dummy= (sinf((float)(i*delta_incr)))* DAC[z].amp + DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 2: {// triangular wave waveform creation
                 printf("\nSetting Triangular Wave\n");
                 delta_incr=4*DAC[z].amp/DAC[z].samples_per_period;	// increment
                 for(i=0;i<DAC[z].samples_per_period/4;i++) {
                     dummy= delta_incr*i +DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                 for(;i<(3*DAC[z].samples_per_period/4);i++) {
                     dummy= 2*DAC[z].amp-delta_incr*i +DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                 for(;i<DAC[z].samples_per_period;i++) {
                     dummy= -4*DAC[z].amp+delta_incr*i +DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 3: {// square wave waveform creation
                 printf("\nSetting Square Wave\n");
                 for(i=0;i<DAC[z].samples_per_period/2;i++) {
                     dummy= DAC[z].amp + DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                 for(;i<DAC[z].samples_per_period;i++) {
                     dummy= -DAC[z].amp + DAC[z].mean;
                     dummy= offset + dummy/res;
                     DAC[z].data[i]= (unsigned short) dummy;
                 }
                break;
                }
        }
    DAC[z].resetWave==false;
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
void* WaveGenManager (void *){
    int i=0;
    pthread_t tid[2];
    while(1){
        if(!isOperating)
            pthread_exit(NULL);
        pthread_mutex_lock(&MainMutex);
        if(DAC[0].isOn && DAC[1].isOn)
            f_available = FBEST/2;
        else
            f_available = FBEST;
        for(i=0; i<2; i++){
            if(DAC[i].isOn==false)
                continue;
            else {
                if(DAC[i].resetWave==false)
                    continue;
                else{
                    WaveformGen(i);
                    pthread_create(&tid[i], NULL, &PushDAC, (void *)&DAC[i]);
                }
            }
        }
        pthread_mutex_unlock(&MainMutex);
    }
}


//*************************************************************//
//        Input manager for switches and analogue inputs
//*************************************************************//
void* PeripheralInputs (void *){
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
	printf("Please enter your command: ");
	scanf("%s", in);
	// clear input buffer after each acquisition of input
	fflush(stdin);
}
//Check the input validity in MainIO
int checkInput(char* in) {
    int temp=0;
    char* endptr;
    temp = strtol(in, &endptr);
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
}
//Show current DAC configuration
void showConfig(){
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
    printf("%*s%*.2E\t%*.2E\n", 25, "Output resolution (V)", 15,
           DAC[0].output_res, 15, DAC[1].output_res);
    printf("%*s%*.2f\t%*.2f\n", 25, "Frequency (Hz)", 15,
           DAC[0].freq, 15, DAC[1].freq);
    printf("%*s%*.2f\t%*.2f\n", 25, "Amplitude (V)", 15,
           DAC[0].amp, 15, DAC[1].amp);
    printf("%*s%*.2f\t%*.2f\n", 25, "Mean (V)", 15,
           DAC[0].mean, 15, DAC[1].mean);
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
    scanf("%s" , filename);
    strcat(filename, ".txt");
    fd = fopen(filename,"r+");
    if(fd == NULL){
        printf("Failed to open %s.\n", filename);
        return;
    }
    input[0]=&source;
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
    CLManager(j, &input);
    pthread_mutex_unlock(&MainMutex);
    fflush(fd);
    close(fd);
    printf("Configuration from %s is loaded.\n", filename);
    return;
}
//Export the configuration to .txt file
void exportConfig(){
	int i = 0;
	FILE* fd;
	char filename[30];
    printf("Please enter filename (\".txt\" is added at the end): ");
    scanf("%s" , filename);
    strcat(filename, ".txt");
    fd = fopen(filename,"w+");
    if(fd == NULL){
        printf("Failed to open/create %s.\n", filename);
        return;
    }
    for(i=0;i<2;i++){
        switch(DAC[i].waveform_type){
            case 1:{fprintf(fd, "%s ", "-sin"); break;}
            case 2:{fprintf(fd, "%s ", "-tri"); break;}
            case 3:{fprintf(fd, "%s ", "-squ"); break;}
        }
        fprintf(fd, "%.2f %.2f %.2f %d\n",
                DAC[i].freq, DAC[i].mean, DAC[i].amp, DAC[i].isOn);
    }
	fflush(fd);
    close(fd);
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
    scanf("%s", input);
    fflush(stdin);
    temp = strtol(input, &pointer, 10);
    if(*pointer=='\0')
        return temp;
    return -1;
}
//Set the change field to be equal to initial DAC parameters
void setChangeField(ChangeField* CF, int i){
    (*CF).waveform_type=DAC[i].waveform_type;
    (*CF).freq=DAC[i].freq;
    (*CF).mean=DAC[i].mean;
    (*CF).amp=DAC[i].amp;
    (*CF).isOn=DAC[i].isOn;
}
//Change the parameters of the DAC0 and DAC1
void changeParam(){
    ChangeField CField = {0, 0, 0, 0, false};
    bool isRepeat=false;
    char input[5];
    int select=0;
    int select1=0;
    int select2=0;
    float temp;
    char *pointer;
    do{
        printf("\nPlease select the DAC (0/1): ");
        if((select=checkValidInt())>=0){
            if(select==0){
                printf("DAC0 selected.\n");
                setChangeField(&CField, 0);
            }
            else if (select==1){
                printf("DAC1 selected.\n");
                setChangeField(&CField, 1);
            }
            else {
                printf("Invalid DAC.\n");
            }
            if(select==0 || select==1){
            printf("\nCurrent OFF/ON (0/1) status: %d\n", CField.isOn);
            printf("Select option:\n");
            printf("1 - Change the waveform type of DAC[%d]\n", select);
            printf("2 - Change the frequency of DAC[%d]\n", select);
            printf("3 - Change the mean of DAC[%d]\n", select);
            printf("4 - Change the amplitude of DAC[%d]\n", select);
            printf("5 - Change the OFF/ON (0/1) status of DAC[%d]\n", select);
            printf("Enter option: ");
            if((select1=checkValidInt())>=0){
                switch (select1){
                    case 1: {
                        printf("\nChanging waveform type of DAC[%d]\n", select);
                        printf("Enter 1 for sinusoidal waveform\n");
                        printf("Enter 2 for triangular waveform\n");
                        printf("Enter 3 for square waveform\n");
                        printf("Enter option: ");
                        if((select2=checkValidInt())>=0)
                            if(select2>0 && select2<4){
                                CField.waveform_type=select2;
                                printf("\nChanged waveform type of DAC[%d]\n",select);
                            }
                            else
                                printf("Invalid waveform selection.\n");
                        else
                            printf("Invalid input for waveform. (Must be integer)");
                        break;
                        }
                    case 2: {
                        printf("\nChanging frequency (float) of DAC[%d]\n", select);
                        printf("Enter frequency (Hz): ");
                        if((temp=checkValidFloat())>=0)
                        if(temp>0 && temp <HIGHESTFREQ) {
                            CField.freq=temp;
                            printf("\nChanged freq of DAC[%d]\n",select);
                        }
                        else
                            printf("Frequency must be in the range (0, %d) Hz\n", HIGHESTFREQ);
                        break;
                        }
                    case 3: {
                        printf("\nChanging mean (float) of DAC[%d]\n", select);
                        printf("Enter mean (V): ");
                        if((temp=checkValidFloat())>=0)
                        if(fabs(temp)<=10){
                            CField.mean=temp;
                            printf("\nChanged mean of DAC[%d]\n",select);
                        }
                        else
                            printf("Mean value must be in the range [-10,10]\n");
                        break;
                        }
                    case 4: {
                        printf("\nChanging amplitude (float) of DAC[%d]\n", select);
                        printf("Enter amplitude (V): ");
                        if((temp=checkValidFloat())>=0)
                        if(temp < 10 && temp>=0){
                            CField.mean=temp;
                            printf("\nChanged amplitude of DAC[%d]\n",select);
                        }
                        else
                            printf("Amplitude must be in the range [0,5]\n");
                        break;
                        }
                    case 5: {
                        printf("\nChanging OFF/ON (0/1) state of the DAC[%d]\n", select);
                        printf("Enter option (0/1): ");
                        if((select2=checkValidInt())>=0)
                            if(select2==0)
                                if (CField.isOn==false)
                                    printf("ADC[%d] is already off.\n", select);
                                else
                                    CField.isOn=false;
                            else if(select2==1)
                                if (CField.isOn==false)
                                    CField.isOn=true;
                                else
                                    printf("ADC[%d] is already on.\n", select);
                        else
                            printf("Invalid options for OFF/ON.\n");
                        break;
                    }
                    default: printf("\nInvalid choice.\n");
                }
            // should put mutex here
            change(CField.isOn, select, CField.waveform_type, CField.freq,
                   CField.mean, CField.amp);
            }
            else
                printf("Invalid number choice for DAC[%d] (Must be integer)\n", select);
            }
        }
        else
            printf("Invalid input. Please enter correct DAC number\n");
        printf("\nDo you want to change other parameters? (Y/N): ");
        scanf("%c", &input[0]);
        fflush(stdin);
        if(input[0]=='y' || input[0]=='Y')
            isRepeat=true;
        else if (input[0]=='n' || input[0]=='N')
            isRepeat=false;
        else{
            printf("Invalid choice. Returning to main IO.\n");
            isRepeat=false;
        }
    } while(isRepeat);
}
//Stop operation (will turn on again if the switches are on)
void stopOp(){
    pthread_mutex_lock(&MainMutex);
    DAC[0].isOn=false;
    DAC[1].isOn=false;
    pthread_mutex_unlock(&MainMutex);
    return;
}
//Main function for keyboard thread
void* MainIO (void *){
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
// Main function
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

    printf("\n\nSet-up Routine for PCI-DAS 1602\n\n");
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
            case 1:{rc = pthread_create(&thread[i], &attr, &WaveGenManager, NULL);
                    break;}
            case 2:{rc = pthread_create(&thread[i], &attr, &MainIO, NULL);
                    break;}
            case 3:{rc = pthread_create(&thread[i], &attr, &PeripheralInputs, NULL);
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
        rc = pthread_join(thread[i], NULL);
        if (rc){
            printf("pthread_join() #%d return error! Code %d\n", i, rc);
            exit(-1);
        }
    }

    pci_detach_device(hdl);
    return 0;
}
