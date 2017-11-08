// 8 November 2017
//******************************************************************************************************
// Performs basic I/O for the Omega PCI-DAS1602 
// 
//
// 
//******************************************************************************************************

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/pci.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <pthread.h>
#include <process.h>
																
#define	INTERRUPT			iobase[1] + 0				// Badr1 + 0 : also ADC register
#define	MUXCHAN				iobase[1] + 2				// Badr1 + 2
#define	TRIGGER				iobase[1] + 4				// Badr1 + 4
#define	AUTOCAL				iobase[1] + 6				// Badr1 + 6
#define	DA_CTLREG			iobase[1] + 8				// Badr1 + 8

#define	AD_DATA				iobase[2] + 0				// Badr2 + 0
#define	AD_FIFOCLR			iobase[2] + 2				// Badr2 + 2

#define	TIMER0				iobase[3] + 0				// Badr3 + 0
#define	TIMER1				iobase[3] + 1				// Badr3 + 1
#define	TIMER2				iobase[3] + 2				// Badr3 + 2
#define	COUNTCTL			iobase[3] + 3				// Badr3 + 3
#define	DIO_PORTA			iobase[3] + 4				// Badr3 + 4
#define	DIO_PORTB			iobase[3] + 5				// Badr3 + 5
#define	DIO_PORTC			iobase[3] + 6				// Badr3 + 6
#define	DIO_CTLREG			iobase[3] + 7				// Badr3 + 7
#define	PACER1				iobase[3] + 8				// Badr3 + 8
#define	PACER2				iobase[3] + 9				// Badr3 + 9
#define	PACER3				iobase[3] + a				// Badr3 + a
#define	PACERCTL			iobase[3] + b				// Badr3 + b

#define 	DA_Data			iobase[4] + 0				// Badr4 + 0
#define 	DA_FIFOCLR		iobase[4] + 2				// Badr4 + 2

#define	DEBUG						1
 	
int badr[5];											// PCI 2.2 assigns 6 IO base addresses
float DA0_amp, DA0_mean, DA0_freq;						// global variables to be put in wave equation
uintptr_t iobase[6];									// base address
uintptr_t dio_in;										// digital input output 
uint16_t adc_in;										// reading potentialmeter

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void * switches(void *arg)								// thread to control the switch
{
	struct pci_dev_info info;
	void *hdl;
	uintptr_t truth;
	unsigned int data[100];
	unsigned int i;
	
	/* Setting up PCI-DAS 1602 */
	memset(&info,0,sizeof(info));
	if(pci_attach(0)<0) 
	{
  		perror("pci_attach");
  		exit(EXIT_FAILURE);
  	}

	/* Vendor and Device ID */
	info.VendorId=0x1307;
	info.DeviceId=0x01;

	if ((hdl=pci_attach_device(0, PCI_SHARE|PCI_INIT_ALL, 0, &info))==0) {
  	perror("pci_attach_device");
  	exit(EXIT_FAILURE);
  	}
  
// Assign BADRn IO addresses for PCI-DAS1602			
if(DEBUG) {
//printf("\nDAS 1602 Base addresses:\n\n");
for(i=0;i<5;i++) {
  badr[i]=PCI_IO_ADDR(info.CpuBaseAddress[i]);
  //if(DEBUG) printf("Badr[%d] : %x\n", i, badr[i]);
  }
 
	//printf("\nReconfirm Iobase:\n");  			// map I/O base address to user space						
for(i=0;i<5;i++) {								// expect CpuBaseAddress to be the same as iobase for PC
  iobase[i]=mmap_device_io(0x0f,badr[i]);	
  //printf("Index %d : Address : %x ", i,badr[i]);
  //printf("IOBASE  : %x \n",iobase[i]);
  }													
}
														// Modify thread control privity
if(ThreadCtl(_NTO_TCTL_IO,0)==-1) {
  perror("Thread Control");
  exit(1);
  }																											
truth = 0;


while(1)
{
//*****************************************************************************
//Digital Port Functions
//*****************************************************************************																																	

	//printf("\nDIO Functions\n");													  										
	out8(DIO_CTLREG,0x90);					// Port A : Input,  Port B : Output,  Port C (upper | lower) : Output | Output			

	dio_in=in8(DIO_PORTA); 					// Read Port A	
	//printf("Port A : %02x\n", dio_in);																												
																						
	out8(DIO_PORTB, dio_in);					// output Port A value -> write to Port B 		
	
	if ((dio_in & 0x08)==0)
	{
		printf("The board is off.\n");
		exit(0);
	}
	else if(truth != dio_in)
	{
		printf("Waveform generation.\n");
		printf("Switches are labelled 1 to 4 from left to right.\n");
		printf("Switch 2 - Toggle Amplitude(0) or Mean(1) Variation\n");
		printf("Switch 3 & 4 - Toggle Wave Shape\n\n");
		printf("  Wave       Switch 3       Switch 4\n");
		printf("========     =========     =========\n");
		printf("  Sine          1              1\n");
		printf("Triangle        0              1\n");
		printf(" Square         1              0\n");
		printf("====================================\n\n");
		
		switch(dio_in) 																//dio_in defines the mode below
		{
			case 0xf9:			printf("Selection: Triangle Wave\n");				// switch cases to define wave
								printf("Varying: Amplitude\n"); 			 		
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);
								break;
			case 0xfa: 			printf("Selection: Square Wave\n");
								printf("Varying: Amplitude\n"); 
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);	
								//funcADC0(iobase, dio_in);
								break;
			case 0xfb:			printf("Selection: Sine Wave\n");
								printf("Varying: Amplitude\n"); 
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);	
								//funcADC0(iobase, dio_in);
								break;	
			case 0xfd:			printf("Selection: Triangle Wave\n");
								printf("Varying: Mean\n");
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);
								//funcADC0(iobase, dio_in);
								break;			
			case 0xfe: 			printf("Selection: Square Wave\n");
								printf("Varying: Mean\n"); 
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);
								//funcADC0(iobase, dio_in);
								break;		
			case 0xff:			printf("Selection: Sine Wave\n");
								printf("Varying: Mean\n");
								printf("Amp: %f, Freq: %f\n", DA0_amp, DA0_freq);
								//funcADC0(iobase, dio_in);
								break;		
			default: 			printf("error\n");					 		
		}
		truth = dio_in;
	}
	else
	{		
	}

}

	
	return(0);
}


void * check_potentiometer(void *arg)												// pontentiometer thread
{
//******************************************************************************
// ADC Port Functions
//******************************************************************************
			
	
	unsigned short chan;
	unsigned int count;	
	int ampmean;
	
	struct pci_dev_info info;
	void *hdl;
	uintptr_t truth;
	unsigned int data[100];
	unsigned int i;
	
	float old_to_new();
	
	/* Setting up PCI-DAS 1602 */
	memset(&info,0,sizeof(info));
	if(pci_attach(0)<0) 
	{
  		perror("pci_attach");
  		exit(EXIT_FAILURE);
  	}

	/* Vendor and Device ID */
	info.VendorId=0x1307;
	info.DeviceId=0x01;

	if ((hdl=pci_attach_device(0, PCI_SHARE|PCI_INIT_ALL, 0, &info))==0) {
  	perror("pci_attach_device");
  	exit(EXIT_FAILURE);
  	}
  
/*  for(i=0;i<6;i++) {							// Another printf BUG ? - Break printf to two statements
    if(info.BaseAddressSize[i]>0) {
      printf("Aperture %d  Base 0x%x Length %d Type %s\n", i, 
        PCI_IS_MEM(info.CpuBaseAddress[i]) ?  (int)PCI_MEM_ADDR(info.CpuBaseAddress[i]) : 
        (int)PCI_IO_ADDR(info.CpuBaseAddress[i]),info.BaseAddressSize[i], 
      PCI_IS_MEM(info.CpuBaseAddress[i]) ? "MEM" : "IO");
      }
  }  
    														
 printf("IRQ %d\n",info.Irq); */
														// Assign BADRn IO addresses for PCI-DAS1602			
if(DEBUG) {
//printf("\nDAS 1602 Base addresses:\n\n");
for(i=0;i<5;i++) {
  badr[i]=PCI_IO_ADDR(info.CpuBaseAddress[i]);
  //if(DEBUG) printf("Badr[%d] : %x\n", i, badr[i]);
  }
 
	//printf("\nReconfirm Iobase:\n");  			// map I/O base address to user space						
for(i=0;i<5;i++) {								// expect CpuBaseAddress to be the same as iobase for PC
  iobase[i]=mmap_device_io(0x0f,badr[i]);	
  //printf("Index %d : Address : %x ", i,badr[i]);
  //printf("IOBASE  : %x \n",iobase[i]);
  }													
}
														// Modify thread control privity
if(ThreadCtl(_NTO_TCTL_IO,0)==-1) {
  perror("Thread Control");
  exit(1);
  }	
   
	out16(INTERRUPT,0x60c0);				// sets interrupts	 - Clears			
	out16(TRIGGER,0x2081);					// sets trigger control: 10MHz, clear, Burst off,SW trig. default:20a0
	out16(AUTOCAL,0x007f);					// sets automatic calibration : default

	out16(AD_FIFOCLR,0); 						// clear ADC buffer
	out16(MUXCHAN,0x0D00);				// Write to MUX register - SW trigger, UP, SE, 5v, ch 0-0 	
														// x x 0 0 | 1  0  0 1  | 0x 7   0 | Diff - 8 channels
														// SW trig |Diff-Uni 5v| scan 0-7| Single - 16 channels
	
	ampmean=dio_in;
	printf("\n\nRead multiple ADC\n");
	count=0x00;
	for ( ; ; )
	{
	if (ampmean == 0xf9) 								// Change Amp Value
	{
		while(count <0x02) 
		{
  			chan= ((count & 0x0f)<<4) | (0x0f & count);
		 	out16(MUXCHAN,0x0D00|chan);		// Set channel	 - burst mode off.
		 	delay(1);											// allow mux to settle
			out16(AD_DATA,0); 							// start ADC 
			while(!(in16(MUXCHAN) & 0x4000));
  			adc_in=in16(AD_DATA);   
  			printf("ADC Chan: %02x Data [%3d]: %4x \n", chan, (int)count, (unsigned int)adc_in);		// print ADC
  			
  			if (count == 0x00)
  			{
  				DA0_amp = (float)adc_in * 5/65535;									//Da0_amp amp mode base on switch 2 
  				printf("Amp is %.2f V\n", DA0_amp);
  			}
  			if (count == 0x01)
  			{
  				DA0_freq = (float)adc_in * 1000/65535;								// Da0 freq
  				DA0_freq = old_to_new();											//compare freq values for resolution
  				printf("Freq is %.2f Hz\n\n", DA0_freq);
  			}
  																
  			fflush( stdout );
 	 		count++;
			//delay(5);											// Write to MUX register - SW trigger, UP, DE, 5v, ch 0-7 	
  		}
  	}
  	
  	if (ampmean == 0xfd)								// Change Mean Value
	{
		while(count <0x02) 
		{
  			chan= ((count & 0x0f)<<4) | (0x0f & count);
		 	out16(MUXCHAN,0x0D00|chan);		// Set channel	 - burst mode off.
		 	delay(1);											// allow mux to settle
			out16(AD_DATA,0); 							// start ADC 
			while(!(in16(MUXCHAN) & 0x4000));
  			adc_in=in16(AD_DATA);   
  			printf("ADC Chan: %02x Data [%3d]: %4x \n", chan, (int)count, (unsigned int)adc_in);		// print ADC	
  		
  			if (count == 0x00)
  			{
  				DA0_mean = (float)adc_in * 5/65535 - 2.5;						//Da0_mean mean mode base on switch 2
  				printf("Mean is %.2f V\n", DA0_mean);
  			}
  			if (count == 0x01)
  			{
  				DA0_freq = (float)adc_in * 1000/65535;							//Da0 freq
  				Da0_freq = old_to_new();										//compare free values for resolution
  				printf("Freq is %.2f Hz\n\n", DA0_freq);
  			}
  															
  			fflush( stdout );
 	 		count++;
			//delay(5);											// Write to MUX register - SW trigger, UP, DE, 5v, ch 0-7 	
  		}
  	}
  	count = 0x00;
  	sleep(1);
  	}
  	return(0);
	}
	

int main(int argc, char *argv[])												// main function
{
	pthread_attr_t attr;								
	pthread_create( NULL, NULL, &switches, NULL);								// thread creation for switch
	sleep(1);
	pthread_create( NULL, NULL, &check_potentiometer, NULL);					// thread creation for potentionmeter
	sleep(1000);
	
	return(0);
	
}

float old_to_new(void)															// comparative function
{
	unsigned short chan;
	unsigned int count = 0x01;

	float old, new, ans;
	old = DA0_freq;
	
	sleep(2);		
	while(count <0x02) 
		{
  			chan= ((count & 0x0f)<<4) | (0x0f & count);
		 	out16(MUXCHAN,0x0D00|chan);		// Set channel	 - burst mode off.
		 	delay(1);											// allow mux to settle
			out16(AD_DATA,0); 							// start ADC 
			while(!(in16(MUXCHAN) & 0x4000));
  			adc_in=in16(AD_DATA);
	
			new = (float)adc_in*1000/65535;
			count++;
		}
		
	if ((abs(new-old))<50)
	{
		ans = new-old;
		
		printf("old: %.2f,     new: %.2f,    diff: %.2f\n", old, new, ans);
		DA0_freq=old;
	}
	
	return DA0_freq;
}