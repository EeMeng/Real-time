/* Program name: MA4830 Waveform Generator.c

 * Author:  Khoo Kar Keng (U1420494C)
			Ng Ee Meng (U1420377G)
			Joyce Lim Xin Yan (U1521972L)
			Tan Jin Cai (U1421759E)
			Joseph Ng Rong En (U1422346L)
			Phillip Puan Jun Kit (U1420409F)

 * Date of creation     :    26/10/2017
 * Updated              :    17/11/2017
 * Version              :    1.3

 * This program generates waveform (sine, triangular, square) wave on PCI-DAS 1602
 * board. Maximum frequency achievable is 1750Hz while minimum and maximum output
 * voltages are -10 and 10V respectively.

 * Input variables: 1. Waveform type
 *                  2. Frequency
 *                  3. Mean
 *                  4. Amplitude
 *                  5. isOn flag

 * To change variables through arguments, the following order must be met:
 * Format:  <program_name.exe> <waveform type> <frequency> <mean> <amplitude> <isOn>
 * Waveform type: -sin =sine, -tri =triangular, -squ = square
 * e.g:  ./wavegen -sin 1000.3 2.4 5 1

 * The user can change the DAC parameters (waveform properties) from keyboard
 * (MainIO) and switches & potentiometer (PeripheralInput). However, only one
 * device can change parameters at a time. PCI-DAS 1602's switches and ADC inputs
 * must be switched off before changing parameters through keyboard.

 * The ADC input ranges are:
 * 1. frequency - [1.22E-3, 1750]Hz
 * 2. amplitude - [0,9.8)V
 * 3. mean - (-9.8, 9,8)V

 * The keyboard input ranges are:
 * 1. frequency - (0, 1750)Hz
 * 2. amplitude - [0, 10)V
 * 3. mean - (-10, 10)V

 * However, in range checking, if the combined parameters gives
 * data not in range of (0, 1750)Hz and (-10,10)V (keyboard)/(-9.8, 9,8)V(ADC),
 * error message will show (keyboard) and parameter is not changed.

 * Description of each thread:
 * 1. Main thread - Initialise communications with PCI-DAS 1602 board,
 *                  create thread 2-4 and wait for them to finish
 *                  (pthread_join) before quitting the program.
 * 2. MainIO thread -   Interface with users with keyboard and display,
 *                      display real-time DAC and ADC status,
 *                      change DAC parameters from keyboard,
 *                      import and export DAC configuration, and
 *                      reset the isOperating flag if user quits
 * 3. PeripheralInput - Receive switches and potentiometers inputs,
 *                      convert data to correct forms,
 *                      changes the DAC parameters with range checking
 * 4. WaveGenManager -  Supervises the isOn and resetWave flag to reconfigure
 *                      DAC data arrays and create thread
 * 5. PushDAC - Dedicated thread to output the data continuously,
 *              thread exits when isOperating/isOn==false or resetWave==true

 * User can import and export the DAC configuration from and to .txt file
 * in command line argument format.
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>        //for boolean data type
#include <termios.h>        //for tcischars();
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

#define THRESHOLD		100

// Struct for DAC waveform
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
    float mean;
    float freq;
    float amp;
}DACField ;

// Struct for intermediary field for changing global variables
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
bool isOperating=true;			//boolean for program operation. False shutsdown the program.
bool ctrlc_pressed=false;		//boolean for SIGNINT. Used in checkQuit.
bool toReturn = false;			//boolean for returning to MainIO(thread) after scanf. Used with Signal.
bool ADC_Refresh = true;		//boolean for refreshing ADC/GPIO display

// DACField struct global variables
DACField DAC={true, true, 0, 1, {}, 0, 1, 100, 0, 5, 100, 1};

// ADC global variables
uintptr_t digital_in;
uint16_t adc_in[2] = {};
uint16_t old_adc_in[2] = {};


// Mutex (only one to change DAC variables)
pthread_mutex_t MainMutex = PTHREAD_MUTEX_INITIALIZER;

// Function Declaration for Housekeeping

// Program functionalities
void CLManager (int argc, char **argv);		//Manages Command line
void* MainIO (void *pointer);				//General purpose thread for inputting from keyboard
void displayHelp();							//Display instructions for MainIO
void showDACConfig();						//Show current DAC configuration
void showADCStatus();						//Show ADC status
void importConfig();						//Import the configuration from .txt file
void exportConfig();						//Export the configuration to .txt file
void changeParam();							//Change the parameters of the DAC0
void stopOps();								//Stop operation of DAC (will turn on again if the switches are on)
void getInput(char* in);					//Get input from keyboard
int checkInput(char* in);					//Check the input validity in MainIO
float checkValidFloat();					//Check validity of floating point number

// Quit signal

void checkQuit(char ch);					//Reconfirm with user about quitting after SIGINT (Refer to function for more description)
void INThandler(int sig);					//Signal handler for SIGINT

//Utilities
bool hasNegative();
short checkAbsMax(float mean, float amp);	// Supporting function: check absolute maximum value
void change(bool onSignal, int wvty,
	float f, float m, float a);				// Function to directly change the parameters of the DAC
void setChangeField(ChangeField* CF);		//Set the change field to be equal to initial DAC parameters

/******* Thread functions declaration *******/
// Peripherals (GPIO and ADC)
void* PeripheralInputs(void *pointer);		//Thread for GPIO and ADC

// DAC
void* WaveGenManager (void * pointer);		//Thread for managing waveform generating capabilities
void WaveformGen ();						//Generate data for waveform
void chooseBestRes();						/*Change the bipolar/unipolar mode based on mean and amplitude
											to give best resolution*/
void* PushDAC (void* Curr);					//Thread to push-out data to DAC asynchronously


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

    // Invoke Signal
	signal(SIGINT, INThandler);

    // Call command line manager
    CLManager (argc, argv);

	// Set up the PCI
    printf("\fSet-up Routine for PCI-DAS 1602\n\n");
    memset(&info,0,sizeof(info));
    if(pci_attach(0)<0) {
      perror("pci_attach");
      exit(EXIT_FAILURE);
    }

    // Vendor and Device ID
    info.VendorId=0x1307;
    info.DeviceId=0x01;

	// Attempt to attach to PCI device and populate info struct
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

    // Map I/O base address to user space
    printf("\nReconfirm Iobase:\n");
    for(i=0;i<5;i++) {
      // Expect CPU Base Address to be the same as IO Base for PC
      iobase[i]=mmap_device_io(0x0f,badr[i]);
      printf("Index %d : Address : %x ", i,badr[i]);
      printf("IOBASE  : %x \n",iobase[i]);
      }

    // Modify thread control privity
    if(ThreadCtl(_NTO_TCTL_IO,0)==-1) {
      perror("Thread Control");
      exit(1);
      }

    // ADC write register
    out16(INTERRUPT, 0x60c0);
    out16(TRIGGER, 0x2081);
    out16(AUTOCAL, 0x007f);
    out16(AD_FIFOCLR, 0);
    out16(MUXCHAN, 0x0D00);

    // Create joinable attribute
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /*
    Create three threads:
    1. Waveform Manager
    2. Keyboard I/O
    3. ADC and Switches
    */
    delay(1500);
    system("clear");

    for(i=0;i<3;i++){
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

    // Joining MainIO input
    for(i=0;i<3;i++){
        rc = pthread_join(thread[i], NULL);
        if (rc){
            printf("pthread_join() #%d return error! Code %d\n", i, rc);
            exit(-1);
        }
    }

	// Exit message and clean screen
    printf("Process quitting in 1 second...");
    fflush(stdout);
	sleep(1);
	system("clear");
    pci_detach_device(hdl);
    return 0;
}

//*************************************************************//
//            Main Program Functionalities
//*************************************************************//
//Display help menu
void displayHelp() {
    printf("\n*************\n");
	printf("Help menu\n");
	printf("*************\n");
	printf("%*s\t\t%s", 6, "1", "Show current DAC0 configurations.\n");
    printf("%*s\t\t%s", 6, "2" ,"Show current ADCs' statuses.\n");
    printf("%*s\t\t%s", 6, "3", "Change the configurations.\n");
    printf("%*s\t\t%s", 6, "4", "Export configurations or waveforms.\n");
    printf("%*s\t\t%s", 6, "5", "Import configurations.\n");
    printf("%*s\t\t%s", 6, "6", "Halt DAC operation "
    	"(must turn off peripheral input).\n");
    printf("%*s\t\t%s", 6, "7", "Exit program\n");
    printf("\nFriendly reminder: please turn off peripheral input\n"
    "before changing any variable through keyboard.\n");
    printf("\nPlease enter your command: ");
}
//Get input from keyboard
void getInput(char* in) {
	// Flush out any output buffer (printf)
	fflush(stdout);
	// Flush to ensure input buffer is cleared
	fflush(stdin);
	scanf("%s", in);
    // Clear input buffer as a safe practice
	fflush(stdin);
	// Check whether CTRL+C is pressed
    if(toReturn) {
  		checkQuit(in[0]);
    }
    return;
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
    else
        printf("Invalid input! Inputted:%s\n", in);
    return 0;
}
//Show current DAC configuration
void showDACConfig(){
    printf("%*s\n", 38, "DAC0");
    printf("%*s%*d\n", 25,
           "Running? (0-OFF, 1-ON)", 15, DAC.isOn);
    printf("%*s", 25, "Waveform type");
	switch(DAC.waveform_type){
        case 1: { printf("%*s", 15, "Sinusoidal"); break;}
        case 2: { printf("%*s", 15, "Triangular"); break;}
        case 3: { printf("%*s", 15, "Square"); break;}
    }
    printf("\n");
    printf("%*s%*d\n", 25,
           "Samples per period", 15, DAC.samples_per_period);
    printf("%*s%*.2E\n", 25,
           "DAC Output resolution (V)", 15, DAC.output_res/1000000);
    printf("%*s%*.2f\n", 25, "Frequency (Hz)", 15, DAC.freq);
    printf("%*s%*.2f\n", 25, "Amplitude (V)", 15, DAC.amp);
    printf("%*s%*.2f\n", 25, "Mean (V)", 15, DAC.mean);
    return;
}
//Show ADC status
void showADCStatus(){
	bool showDAC = false;
	char input[5];
	char in, key;
	/* Ask user whether to show DAC configuration to
	supervise real-time changes on DAC  */
	printf("\nDo you want to show DAC config? (Y/N): ");
    getInput(&input[0]);
	/* Return to MainIO if CTRL+C is pressed (same applies for
	subsequent similar statements)   */
    if(toReturn)
        return;
	if (input[0] == 'y' || input[0] == 'Y') {
		showDAC = true;
	}
	else if (!(input[0] == 'n' || input[0] == 'N')) {
		printf("Invalid choice. DAC config will not show.\n");
	}
	// While to show ADC settings (optional: DAC settings)
	while (1){
		/* Break loop if any key is pressed (tcischars detects number
		of characters waiting to be read from the therminal    */
        if(tcischars(1)>0){
            fflush(stdin);
            printf("\f");
            break;
        }
		delay(1000);
		/* Get user's confirmation character and return to
		MainIO if CTRL+C is pressed  */
        if(toReturn){
            getInput(&input[0]);
            return;
   		}
   		if(ADC_Refresh){
			// Switches input and descriptions
			printf("\f%*s\nDescriptions\n", 50, "Digital input");
			printf("Bit 3     --> 0 = Keyboard control only ");
			printf("1 = Keyboard and peripheral inputs\n");
			printf("Bit 2     --> P0/ADC0 changes:  0 = mean  1 = amplitude\n");
			printf("%*s", 14, "Bit 1 & 0 --> ");
			printf("%-*s\n", 26, "0x00 - DAC0 is off");
			printf("%*s%-*s\n", 14, "", 40, "0x01 - Sine wave");
			printf("%*s%-*s\n", 14, "", 40, "0x02 - Triangular wave");
			printf("%*s%-*s\n", 14, "", 40, "0x03 - Square wave");
			printf("\nPort A Bit\t\t3\t\t2\t\t1\t\t0\n");
			printf("\t\t\t%1d", (digital_in&0x08) >> 3);
			printf("\t\t%1d", (digital_in&0x04)>> 2);
			printf("\t\t%1d", (digital_in&0x02) >> 1);
			printf("\t\t%1d\n", (digital_in&0x01));
			// ADC input
			printf("\n\n%*s\n", 38, "ADC Value (16 bit - Hex)");
			printf("%*s\t\t%04X\n", 10, "ADC0 ", adc_in[0]);
			printf("%*s\t\t%04X\n", 10, "ADC1 ", adc_in[1]);
			// DAC settings
			if (showDAC){
				printf("\n\n\n");
				showDACConfig();
			}
			printf("Press any key to quit the function\n");
			ADC_Refresh = false;
        }
	}
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
	printf("Warning: Please turn off peripheral control before importing!\n");
    printf("Please enter filename (\".txt\" is added at the end): ");
    getInput(&filename[0]);
    if(toReturn)
        return;
    strcat(filename, ".txt");
	// Open file to read, return if fails
    fd = fopen(filename,"r+");
    if(fd == NULL){
        printf("Failed to open %s.\n", filename);
        return;
    }
	/* Read the file character by character
	If " " or "\n" is found, replace it by EOF.
	Input the address of first character of each word
	to char pointer array    */
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
	/* Use CLManager to read imported settings

	input[0] = "imported"
	input[i] = respective string

	Use of mutex when changing shared global variables
	*/
    pthread_mutex_lock(&MainMutex);
    CLManager(j, (char**) &input);
    pthread_mutex_unlock(&MainMutex);
	// Close file and print confirmation message
    fflush(fd);
    fclose(fd);
    printf("\nConfiguration from %s is loaded.\n", filename);
    sleep(1);
    return;
}
//Export the configuration to .txt file
void exportConfig(){
    FILE* fd;
	bool printConfig =false, printData = false;
	char filename[30];
	int i;
    printf("Please enter filename (\".txt\" is added at the end): ");
    getInput(&filename[0]);
    if(toReturn)  return;
    strcat(filename, ".txt");
	// Open file (buffered write)
    fd = fopen(filename,"w+");
    if(fd == NULL){
        printf("Failed to open/create %s.\n", filename);
        return;
    }
    // Display filename and export options
    printf("\fFilename: %s\n", filename);
    printf("Options:\n");
    printf("1 - Export DAC configuration only (importable)\n");
    printf("2 - Export waveform data only (unimportable)\n");
    printf("3 - Export both (unimportable)\n");
    printf("Enter option: ");
    // Check options
    switch(checkValidInt()){
        case 1: {printConfig = true; break;}
        case 2: {printData = true; break;}
        case 3: {printConfig = true; printData = true; break;}
        default:{if(toReturn)  return;
                 printf("Invalid choice. File is not written.\n");}
    }
    // Printing configuration only
    if(printConfig){
        // Print setting in command line format
        switch(DAC.waveform_type){
            case 1:{fprintf(fd, "%s ", "-sin"); break;}
            case 2:{fprintf(fd, "%s ", "-tri"); break;}
            case 3:{fprintf(fd, "%s ", "-squ"); break;}
        }
        fprintf(fd, "%.2f %.2f %.2f %d\n",
                DAC.freq, DAC.mean, DAC.amp, DAC.isOn);
    }
    // Printing waveform data only
    if(printData){
        fprintf(fd, "\n Data number\t\tDAC value(Hex)\t\tReal value(V)\n\n");
        for(i=0;i<DAC.samples_per_period;i++)
            fprintf(fd, "\t%d\t\t%04x\t\t\t%.4f\n", i+1, DAC.data[i],
                (DAC.data[i]-(int)(DAC.DAC_mode/2)*(0x7FFF))*(DAC.output_res/1000000));
    }
	// Close file and print confirmation message
	fflush(fd);
    fclose(fd);
    printf("Configuration saved to file.\n");
    return;
}
//Check validity of floating point numbers
float checkValidFloat(){
    char input[12];
    char* pointer;
    float temp=0;
    getInput(&input[0]);
    if(toReturn)
        return -100;
	// Convert string to double
    temp = strtod(input, &pointer);
	// Return the number if valid
    if(*pointer=='\0')
        return temp;
	// Return -100 if invalid
    else
        printf("Invalid floating point number!\n");
    return -100;
}
// Check validity of integer
int checkValidInt(){
    char input[5];
    char* pointer;
    float temp=0;
    getInput(&input[0]);
    if(toReturn)
        return -1;
	// Convert string to long
    temp = strtol(input, &pointer, 10);
	// Return the number if valid, else return -1
    if(*pointer=='\0')
        return temp;
    return -1;
}
//Change the parameters of the DAC0
void changeParam(){
    ChangeField CField;
    bool isRepeat=false;
    bool hasChanged = false;
    char repeatchar;
    char input[5];
    int select1=0;
    int select2=0;
    float temp;
    do{
		// Load CField with current DAC values
        setChangeField(&CField);
		// Display options
        printf("\nCurrent OFF/ON (0/1) status: %d\n", CField.isOn);
        printf("Select option:\n");
        printf("1 - Change the waveform type of DAC[0]\n");
        printf("2 - Change the frequency of DAC[0]\n");
        printf("3 - Change the mean of DAC[0]\n");
        printf("4 - Change the amplitude of DAC[0]\n");
        printf("5 - Change the OFF/ON (0/1) status of DAC[0]\n");
        printf("6 - Reset the settings to default\n");
        printf("Enter option: ");
		// Integer validity check
        if((select1=checkValidInt())>=0){
            switch (select1){
				// Option 1: change waveform
                case 1: {
                    printf("\nChanging waveform type of DAC[0]\n");
                    printf("Enter 1 for sinusoidal waveform\n");
                    printf("Enter 2 for triangular waveform\n");
                    printf("Enter 3 for square waveform\n");
                    switch(DAC.waveform_type){
						case 1: printf("Current waveform (V): sinusoidal\n"); break;
						case 2: printf("Current waveform (V): triangular\n"); break;
						case 3: printf("Current waveform (V): square\n"); break;
                    }
                    printf("Enter option: ");
					// Integer validity check
                    if((select2=checkValidInt())>=0)
						// Selection validity check
                        if(select2>0 && select2<4){
                        	hasChanged = true;
                            CField.waveform_type=select2;
                            printf("\nChanged waveform type of DAC[0]\n");
                        }
                        else
                            printf("Invalid waveform selection.\n");
                    else{
                        if(toReturn) return;
                        printf("Invalid input for waveform. (Must be integer)");
                        }
                    break;
                    }
				// Option 2: change frequency
                case 2: {
                    printf("\nChanging frequency (float) of DAC[0]\n");
                    printf("Frequency must be in the range (0,%d) Hz\n", HIGHESTFREQ);
                    printf("Current frequency (Hz): %.2f\n", DAC.freq);
                    printf("Enter frequency (Hz): ");
					// Range checking
                    if((temp=checkValidFloat())>0 && temp < HIGHESTFREQ){
                            hasChanged = true;
                            CField.freq=temp;
                            printf("\nChanged freq of DAC[0]\n");
                    }
                    else{
                    	if(toReturn) return;
                        printf("Error: Frequency is not changed.\n");
                    }
                    break;
                    }
				// Option 3: change mean
                case 3: {
                    printf("\nChanging mean (float) of DAC[0]\n");
                    printf("Mean value must be in the range (%.2f,%.2f)\n",
                           -(10-CField.amp), 10-CField.amp);
                    printf("Current mean (V): %.2f\n", DAC.mean);
                    printf("Enter mean (V): ");
                    if((temp=checkValidFloat())>= -10)
						// Range checking
                        if(checkAbsMax(temp, CField.amp)!=0){
                            hasChanged = true;
                            CField.mean=temp;
                            printf("\nChanged mean of DAC[0]\n");
                        }
                        else{
                        	if(toReturn) return;
                            printf("Mean value is not changed.\n");
                        }
                    break;
                    }
				// Option 4: change amplitude
                case 4: {
                    printf("\nChanging amplitude (float) of DAC[0]\n");
                    printf("Amplitude value must be in the range (0,%.2f)\n"
                           , 10-CField.mean);
                    printf("Current amplitude (V): %.2f\n", DAC.amp);
                    printf("Enter amplitude (V): ");
                    if((temp=checkValidFloat())>= 0)
						// Range checking
                        if(checkAbsMax(temp, CField.amp)!=0){
                            hasChanged = true;
                            CField.amp=temp;
                            printf("\nChanged amplitude of DAC[0]\n");
                        }
                        else{
                        	if(toReturn) return;
                            printf("Amplitude value is not changed.\n");
                        }
                    break;
                    }
				// Option 5: change isOn flag
                case 5: {
                    printf("\nChanging OFF/ON (0/1) state of the DAC[0]\n");
                    printf("DAC is currently:");
                    DAC.isOn ? printf("On\n") : printf("Off\n");
                    printf("Enter option (0/1): ");
                    if((select2=checkValidInt())>=0)
                        if(select2==0)
                            if (CField.isOn==false)
                                printf("DAC[0] is already off.\n");
                            else
 {
                                hasChanged = true;
                                CField.isOn=false;
                            }
                        else if(select2==1)
                            if (CField.isOn==false)
{
                                hasChanged = true;
                                CField.isOn=true;
                            }
                            else
                                printf("DAC[0] is already on.\n");
                    else{
                        if(toReturn) return;
                        printf("Invalid options for OFF/ON.\n");
                    }
                    break;
                }
				// Option 6: restore default values
                case 6: {
                    printf("\nResetting values to default values.\n");
                    printf("Sine wave, freq = 1 Hz, mean = 0V, amplitude = 1V, isOn = 0\n");
                    hasChanged = true;
                    CField.waveform_type=1;
                    CField.freq=1;
                    CField.mean=0;
                    CField.amp=1;
                    CField.isOn=false;
                    break;
                }
                default: {
                	if(toReturn) return;
                    printf("\nInvalid choice.\n");
                }
                }
			// Use of mutex when changing shared global variables
        	if(hasChanged){
            	if(select1<7 && select1>0){
                	pthread_mutex_lock(&MainMutex);
                	change(CField.isOn, CField.waveform_type,
                	CField.freq, CField.mean, CField.amp);
                	pthread_mutex_unlock(&MainMutex);
					// Reset hasChanged flag
                	hasChanged = false;
            	}
             	// Wait for WaveGenManger to finish
       			delay(50);
        	}
        }
        if(toReturn) return;
		// Ask user whether to repeat changeParam
        printf("\nDo you want to change other parameters? (Y/N): ");
        getInput(&input[0]);
        if(toReturn) return;
        if(input[0]=='y' ||input[0]=='Y')
            isRepeat=true;
        else if (input[0]=='n' ||input[0]=='N')
            isRepeat=false;
        else{
            printf("Invalid choice. Returning to main IO.\n");
            isRepeat=false;
        }
        printf("\f");
    } while(isRepeat);
    return;
}
//Stop operation of DAC(will turn on again if the switches are on)
void stopOps(){
	// Use of mutex when changing shared global variables
    pthread_mutex_lock(&MainMutex);
    DAC.isOn=false;
  	pthread_mutex_unlock(&MainMutex);
    return;
}
//The general purpose thread for inputting from keyboard
void* MainIO (void *pointer){
    char input[10];
    sleep(1);  // Wait for WaveGenManager to finish its configuration
    while (1) {
		// Exit thread if isOperating is false
        if(!isOperating)
            pthread_exit(NULL);
		// Reset toReturn flag only when program reaches MainIO
        if(ctrlc_pressed==false)
            toReturn = false;
        delay(100);
        displayHelp();
		getInput(&input[0]);
		// Continue loop if CTRL+C is pressed
        if(toReturn)
            continue;
       	printf("\f");
		switch (checkInput(&input[0])) {
		    // case 1 - print current devices' configurations.
			case 1: {   showDACConfig(); break; }
            // case 2 - print current devices' statuses.
			case 2: {   showADCStatus();       break; }
			// case 3 - change the configurations.
			case 3: {   changeParam();       break; }
			// case 4 - export configurations or waveforms.
			case 4: {   exportConfig();  break; }
			// case 5 - import configurations.
			case 5: {   importConfig();  break; }
			// case 6 - halt all operations.
			case 6: {   stopOps(); break; }
            // case 66 - execute order 66: quit the program
			case 66: {  isOperating = false; break; }
			//show error in input
			default:{   printf("Invalid character. Please reenter. \n");}
		}
    }
     return 0;
}

// Check if scanf has pickup a quit confirmation after signal
void checkQuit(char ch){
    ctrlc_pressed=false;
    if (ch == 'y' || ch== 'Y'){
        isOperating=false;
    }
    else printf("\f");
    return;
}
/* SIGINT handler function

When CTRL+C is pressed, INThandler is started.
INThandler sets toReturn and ctrlc_pressed flags.
Confirmation to quit is asked and INTHandler quits.

Meanwhile, MainIO stops at getInput (waits user
input).

getInput will receive input regardless of said
flags settings. After receiving input, toReturn
flag is checked.

If true, getInput will run checkQuit for the
inputted character. If not, getInput progresses
as per normal.
*/
void  INThandler(int sig){
    ctrlc_pressed=true;
    toReturn = true;
    printf("\nYou pressed CTRL+C. "
             "Do you really want to quit? (Y/y to quit): \n");
}

// Manages arguments
void CLManager (int argc, char **argv){
    int counter, temp2;
    float temp;
    ChangeField CField;
    char* endptr;
	// Load CField with default DAC values
    setChangeField(&CField);
	// Argument checking (from argv[1])
    for(counter=1;counter<argc;counter++){
		// Check argv[1] - waveform type
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
		// Check argv[2] to argc[4]
        else  if ((counter%5) !=0){
			// Convert string to double
            temp = strtod(argv[counter], &endptr);
			// Continue if valid
            if(*endptr == '\0'){
				/* Check argv[2] - freq, argv[3] - mean
				and argv[4] - amplitude */
                switch(counter%5){
                    case 2: {
						// Range checking for all parameters
                        if(temp>0 && temp <HIGHESTFREQ)
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
				// Show invalid input (not double), if any
                printf("Invalid input #%d: %s\n", counter, argv[counter]);
        }
		// Check argv[5] - isOn status
        else {
            temp2 = strtol(argv[counter], &endptr, 2);
            if(temp2==1)
                change(true, CField.waveform_type, CField.freq, CField.mean, CField.amp);
            else
                change(false, CField.waveform_type, CField.freq, CField.mean, CField.amp);
        }
    }
	// CLManager ending message
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
	// Data has negative value(s)
    if(hasNegative()){
		// Absolute maximum <5V
        if(checkAbsMax(DAC.mean, DAC.amp)==1){
            DAC.plus=(0x0000<<DAC.identity);
            DAC.DAC_mode = 0;
            DAC.output_res=152.59;
        }
		// Absolute maximum <10V
        else if (checkAbsMax(DAC.mean, DAC.amp)==2){
            DAC.plus=(0x0100<<DAC.identity);
            DAC.DAC_mode = 1;
            DAC.output_res=305.14;
        }
    }
	// Data has no negative value
    else{
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
	}
    return;
}
// Generate data for waveform
void WaveformGen (){
    int i=0;
    int z=0;
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
	// Choose unipolar/bipolar DAC mode
    chooseBestRes();
	// Convert resolution to unit of V
    res = DAC.output_res/1000000;
	// Add offset for bipolar mode
    if(DAC.DAC_mode<2)
        offset+=0x7FFF;
    switch (DAC.waveform_type){
        case 1: {// Sine wave waveform creation
                 delta_incr=2.0*PI/DAC.samples_per_period;	// increment
                 for(i=0;i<DAC.samples_per_period;i++) {
                     dummy= (sinf((float)(i*delta_incr)))* DAC.amp + DAC.mean;
                     dummy= offset + dummy/res;
                     DAC.data[i]= (unsigned short) dummy;
                 }
                 break;
                }
        case 2: {// Triangular wave waveform creation
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
        case 3: {// Square wave waveform creation
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
	// Reset resetWave flag after finishing configuration
    DAC.resetWave=false;
    return;
}
// Function to push-out data to DAC(thread function)
void* PushDAC (void* Curr){
	// Obtained struct pointer from pthread_create
    DACField* Current = (DACField*) Curr;
    int i = 0;
    unsigned short CTLREG_content;
	long nanospin_time;
	// Configure the DAC CTRL register data values
    CTLREG_content=(unsigned short)((*Current).plus+((*Current).identity+0x1)*0x20+0x3);
	nanospin_time = (long)(1000000000.0/(Current->freq*Current->samples_per_period));
	// While loop to push out data
    while (1){
        for(i=0;i<(*Current).samples_per_period;i++) {
			// Exit thread if isOperating or isOn ==false, or when resetWave==true              */
            if(!((*Current).resetWave==false && (*Current).isOn==true) || !isOperating)
                pthread_exit(NULL);
            out16(DA_CTLREG, CTLREG_content);       // Write setting to DAC CTLREG
            out16(DA_FIFOCLR, 0);					// Clear DA FIFO buffer
            out16(DA_Data, (*Current).data[i]);     // Output data
            nanospin_ns(nanospin_time - FIFO_DELAY);// Busy wait
        }
    }
    return (0);
}
//Thread for managing waveform generating capabilities
void* WaveGenManager (void * pointer){
    pthread_t tid;
    while(1){
        delay(100);
        if(!isOperating){
            pthread_exit(NULL);
            DAC.isOn = false;
        }
		// Continue checking if PushDAC needs no change
        if(DAC.isOn==false)
            continue;
        else {
            if(DAC.resetWave==false)
                continue;
			/* Create a thread if DAC setting is changed, PushDAC
			automatically dies (see above descriptions)*/
            else{
				// Use to Mutex when changing shared global variables
                pthread_mutex_lock(&MainMutex);
				// Set up data field for DAC
                WaveformGen();
                pthread_mutex_unlock(&MainMutex);
				// Create thread
                pthread_create(&tid, NULL, &PushDAC, (void *)&DAC);
            }
        }
    }
}

//*************************************************************//
//        Input manager for switches and analogue inputs
//*************************************************************//
void * PeripheralInputs(void *pointer){
	bool isOn = false;
	bool hasChanged = false;
	unsigned short mean_amp=0;
	unsigned short count = 0x00;
	unsigned short chan = 0x00;
	unsigned short wavef = 1;
	float temp;
	ChangeField CField;
	ADC_Refresh = true;
	while(1){
		// Exit thread if isOperating is false
        if(!isOperating)
            pthread_exit(NULL);
		// Load the CField with default DAC values
		setChangeField(&CField);
		temp=0;
		count = 0x00;
		// Delay to allow DAC to work better
		delay(100);
		// Port A : Input,  Port B : Output,  Port C (upper | lower) : Output | Output
		out8(DIO_CTLREG,0x90);

		// Read Port A
		digital_in =in8(DIO_PORTA);
		// Output Port A value -> write to Port B (LEDs)
		out8(DIO_PORTB, digital_in);

		// Read potentiometers
		while (count < 0x02) {
			chan = ((count & 0x0f) << 4) | (0x0f & count);
			out16(MUXCHAN, 0x0D00 | chan);			// Set channel	 - burst mode off
			delay(1);								// Allow mux to settle
			out16(AD_DATA, 0); 						// Start ADC
			while (!(in16(MUXCHAN) & 0x4000));		// Wait until the data is filled
			adc_in[count] = in16(AD_DATA);
			count++;
		}

		// Remove unneeded bits
		digital_in = digital_in & 0x0f;

		// Continue if the peripheral input is turned off
		if (!(digital_in & 0x08))
			continue;
		else {
			// Assume the DAC to be on
			isOn = true;
			switch ((digital_in & 0x03)) {
				case 1: { wavef = 1; break; }
				case 2: { wavef = 2; break; }
				case 3: { wavef = 3; break; }
				// Set isOn to be off is bit 0 and 1 are off
				default:   { isOn = false; break; }
			}
			/* Only set the hasChanged flag if the current
			configuration is different from the previous one */
			if (CField.waveform_type != wavef) {
				CField.waveform_type = wavef;
				hasChanged = true;
			}
			if (CField.isOn != isOn) {
				CField.isOn = isOn;
				hasChanged = true;
			}
			// Get the bit value of bit 2
			mean_amp = ((digital_in & 0x04) >> 2);
			/*
			ADC[0] - for mean and amplitude
			- check whether the difference in new & old values is
			more than threshold before trying to change
			*/
			if (abs((int)adc_in[0] - (int)old_adc_in[0]) > THRESHOLD) {
				// Change amplitude if bit 2 is set
				if (mean_amp == 1) {
					temp = (float)(adc_in[0]) * 10 / 65535;
					// Range checking
					if (fabs(DAC.mean + temp) < 9.8 && fabs(DAC.mean - temp) < 9.8) {
						CField.amp = temp;
						hasChanged = true;
					}
				}
				// Change mean if bit 2 is not set
				else {
					temp = (float)(adc_in[0]) * 10 / 32767 - 10;
					// Range checking
					if (fabs(DAC.amp + temp) < 9.8 && fabs(DAC.amp - temp) < 9.8) {
						CField.mean = temp;
						hasChanged = true;
					}
				}
			}
			/*
			ADC[1] - dedicated for frequency
			- check whether the difference in new & old values is
			more than threshold before trying to change
			*/
			if (abs((int)adc_in[1] - (int)old_adc_in[1])>THRESHOLD) {
				// Minimum input required (> 0x50)
				if (adc_in[1] > 0x0050) {
					CField.freq = (float)(adc_in[1]) * HIGHESTFREQ / 65535;
					hasChanged = true;
				}
			}

			// Change the value(s) if hasChanged flag is set
			if (hasChanged) {
			    ADC_Refresh = true;
				// Use of mutex when changing shared global variables
				pthread_mutex_lock(&MainMutex);
				change(CField.isOn, wavef, CField.freq, CField.mean, CField.amp);
				pthread_mutex_unlock(&MainMutex);
                hasChanged = false;
				// Load new values to old values array
                old_adc_in[0] = adc_in[0];
                old_adc_in[1] = adc_in[1];
			}
		}
	}
	return(0);
}



//*************************************************************//
//            Supporting Programs
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
