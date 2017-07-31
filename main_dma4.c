/*
*********************************************


 * Tao  Yang 14231112
 * Yang Yuan 14211079


 *2017/4/21
*********************************************
*/



#include <stdio.h>
#include <csl_mcbsp.h>
#include <csl_dma.h>
#include <csl_irq.h>
#include "i_cmplx.h"

//---------Global data definition---------

/* Constants for the buffered ping-pong transfer */
#define BUFFSIZE         1024
#define PING              0
#define PONG              1

COMPLEX lleftRcvPing[512];
COMPLEX rrightRcvPing[512];
COMPLEX lleftRcvPong[512];
COMPLEX rrightRcvPong[512];
Uint32 mod[512];

#define SAMPLELONG 2   // =2说明信号做512点FFT
Uint16 SampleLong;
/*
 * Data buffer declarations - the program uses four logical buffers of size
 * BUFFSIZE, one ping and one pong buffer on both receive and transmit sides.
 */
#pragma DATA_SECTION (gBufferXmtPing, "buffer_sect");
Int16 gBufferXmtPing[BUFFSIZE];  // Transmit PING buffer
#pragma DATA_SECTION (gBufferXmtPong, "buffer_sect");
Int16 gBufferXmtPong[BUFFSIZE];  // Transmit PONG buffer
#pragma DATA_SECTION (gBufferRcvPing, "buffer_sect");
Int16 gBufferRcvPing[BUFFSIZE];  // Receive PING buffer
#pragma DATA_SECTION (gBufferRcvPong, "buffer_sect");
Int16 gBufferRcvPong[BUFFSIZE];  // Receive PONG buffer


#pragma DATA_SECTION (leftXmtPing, "buffer_sect");
Int16 leftXmtPing[BUFFSIZE];  // Transmit PING buffer
#pragma DATA_SECTION (leftXmtPong, "buffer_sect");
Int16 leftXmtPong[BUFFSIZE];  // Transmit PONG buffer
#pragma DATA_SECTION (leftRcvPing, "buffer_sect");
Int16 leftRcvPing[BUFFSIZE];  // Receive PING buffer
#pragma DATA_SECTION (leftRcvPong, "buffer_sect");
Int16 leftRcvPong[BUFFSIZE];  // Receive PONG buffer


#pragma DATA_SECTION (rightXmtPing, "buffer_sect");
Int16 rightXmtPing[BUFFSIZE];  // Transmit PING buffer
#pragma DATA_SECTION (rightXmtPong, "buffer_sect");
Int16 rightXmtPong[BUFFSIZE];  // Transmit PONG buffer
#pragma DATA_SECTION (rightRcvPing, "buffer_sect");
Int16 rightRcvPing[BUFFSIZE];  // Receive PING buffer
#pragma DATA_SECTION (rightRcvPong, "buffer_sect");
Int16 rightRcvPong[BUFFSIZE];  // Receive PONG buffer



/*------------------------------------------------------------------------------------*/
//
// Config McBSP:  Use McBSP to send and receive the data between DSP and AIC23B
//
/*------------------------------------------------------------------------------------*/
MCBSP_Config Mcbsp1Config = {
  MCBSP_SPCR1_RMK(    
    MCBSP_SPCR1_DLB_OFF,  			// DLB    = 0 
    MCBSP_SPCR1_RJUST_LZF,          // RJUST  = 0,right justify the data and zero fill the MSBs
    MCBSP_SPCR1_CLKSTP_DISABLE,     // CLKSTP = 0 
    MCBSP_SPCR1_DXENA_ON,           // DXENA  = 1,DX delay enabler on 
    0,             				   	// Reserved   = 0 
    MCBSP_SPCR1_RINTM_RRDY,         // RINTM  = 0 
    MCBSP_SPCR1_RSYNCERR_NO,        // RSYNCER = 0 
//    MCBSP_SPCR1_RFULL_NO,           // RFULL = 0  
//    MCBSP_SPCR1_RRDY_NO,            // RRDY = 0  
    MCBSP_SPCR1_RRST_DISABLE 		// RRST   = 0; Disable receiver 
   ),
  MCBSP_SPCR2_RMK(  
    MCBSP_SPCR2_FREE_NO,            // FREE   = 0 
    MCBSP_SPCR2_SOFT_NO,            // SOFT   = 0 
    MCBSP_SPCR2_FRST_FSG,         	// FRST   = 1 ; Enable the frame-sync logic
    MCBSP_SPCR2_GRST_CLKG,         	// GRST   = 1 ; The sample rate generator is take out of its reset state 
    MCBSP_SPCR2_XINTM_XRDY,         // XINTM  = 0 
    MCBSP_SPCR2_XSYNCERR_NO,        // XSYNCER =0 
 //   MCBSP_SPCR2_XEMPTY_NO,          // XEMPTY = 0 
 //   MCBSP_SPCR2_XRDY_NO,            // XRDY   = 0             
    MCBSP_SPCR2_XRST_DISABLE 	    // XRST   = 0 Disable transimitter 
   ),
   // 单数据相，接受数据长度为16位,每相2个数据
  MCBSP_RCR1_RMK( 
  	MCBSP_RCR1_RFRLEN1_OF(1),       // RFRLEN1 = 1 
  	MCBSP_RCR1_RWDLEN1_16BIT        // RWDLEN1 = 2 
  ),
  MCBSP_RCR2_RMK(    
    MCBSP_RCR2_RPHASE_SINGLE,       // RPHASE  = 0 
    MCBSP_RCR2_RFRLEN2_OF(0),       // RFRLEN2 = 0 
    MCBSP_RCR2_RWDLEN2_8BIT,       	// RWDLEN2 = 0 
    MCBSP_RCR2_RCOMPAND_MSB,        // RCOMPAND = 0 No companding,any size data, MSB received first 
    MCBSP_RCR2_RFIG_YES,  		    // RFIG    = 1 Frame-sync ignore 
    MCBSP_RCR2_RDATDLY_1BIT  		// RDATDLY = 1 1-bit data delay 
    ),  
  MCBSP_XCR1_RMK(    
    MCBSP_XCR1_XFRLEN1_OF(1),       // XFRLEN1 = 1  
    MCBSP_XCR1_XWDLEN1_16BIT        // XWDLEN1 = 2   
 ),   
 MCBSP_XCR2_RMK(   
    MCBSP_XCR2_XPHASE_SINGLE,       // XPHASE  = 0 
    MCBSP_XCR2_XFRLEN2_OF(0),       // XFRLEN2 = 0 
    MCBSP_XCR2_XWDLEN2_8BIT,       	// XWDLEN2 = 0 
    MCBSP_XCR2_XCOMPAND_MSB,        // XCOMPAND = 0 
    MCBSP_XCR2_XFIG_YES,            // XFIG    = 1 Unexpected Frame-sync ignore 
    MCBSP_XCR2_XDATDLY_1BIT         // XDATDLY = 1 1-bit data delay 
  ),            
 MCBSP_SRGR1_DEFAULT,
 MCBSP_SRGR2_DEFAULT,				 
 MCBSP_MCR1_DEFAULT,
 MCBSP_MCR2_DEFAULT, 
 MCBSP_PCR_RMK(
  // MCBSP_PCR_IDLEEN_RESET,          // IDLEEN   = 0   
   MCBSP_PCR_XIOEN_SP,              // XIOEN    = 0   
   MCBSP_PCR_RIOEN_SP,              // RIOEN    = 0   
   MCBSP_PCR_FSXM_EXTERNAL,  		// FSXM     = 0 Tranmit frame-syn is provided by AIC23B 
   MCBSP_PCR_FSRM_EXTERNAL,         // FSRM     = 0 Receive frame-syn is provided by AIC23B 
   MCBSP_PCR_CLKXM_INPUT,   		// CLKR is input 
   MCBSP_PCR_CLKRM_INPUT,           // CLKX is input 
   MCBSP_PCR_SCLKME_NO,             // SCLKME=0 CLKG is taken from the McBSP internal input clock  
 //  MCBSP_PCR_CLKSSTAT_0,            // The signal on the CLKS pin is low   
   MCBSP_PCR_DXSTAT_0,              // Drive the signal on the DX pin low   
  // MCBSP_PCR_DRSTAT_0,              // The signal on the DR pin is low   
   MCBSP_PCR_FSXP_ACTIVEHIGH,  		// FSXP     = 1 Because a falling edge on LRCIN or LRCOUT starts data transfer  
   MCBSP_PCR_FSRP_ACTIVELOW,        // FSRP     = 1   
   MCBSP_PCR_CLKXP_FALLING,         // CLKXP    = 1   The falling edge of BCLK starts data transfer 
   MCBSP_PCR_CLKRP_RISING           // CLKRP    = 1   
 ),
 MCBSP_RCERA_DEFAULT, 
 MCBSP_RCERB_DEFAULT, 
 MCBSP_RCERC_DEFAULT, 
 MCBSP_RCERD_DEFAULT, 
 MCBSP_RCERE_DEFAULT, 
 MCBSP_RCERF_DEFAULT, 
 MCBSP_RCERG_DEFAULT, 
 MCBSP_RCERH_DEFAULT, 
 MCBSP_XCERA_DEFAULT,
 MCBSP_XCERB_DEFAULT,
 MCBSP_XCERC_DEFAULT,
 MCBSP_XCERD_DEFAULT,  
 MCBSP_XCERE_DEFAULT,
 MCBSP_XCERF_DEFAULT,  
 MCBSP_XCERG_DEFAULT,
 MCBSP_XCERH_DEFAULT
 }; 
 
DMA_Config  dmaRcvConfig = { 
  DMA_DMACSDP_RMK(
    DMA_DMACSDP_DSTBEN_NOBURST,
    DMA_DMACSDP_DSTPACK_OFF,
    DMA_DMACSDP_DST_DARAMPORT1,
    DMA_DMACSDP_SRCBEN_NOBURST,
    DMA_DMACSDP_SRCPACK_OFF,
    DMA_DMACSDP_SRC_PERIPH,
    DMA_DMACSDP_DATATYPE_16BIT
  ),                                       /* DMACSDP  */
  DMA_DMACCR_RMK(
    DMA_DMACCR_DSTAMODE_POSTINC,
    DMA_DMACCR_SRCAMODE_CONST,
    DMA_DMACCR_ENDPROG_OFF,				/* ENDPROG OFF */
    DMA_DMACCR_WP_DEFAULT,
    DMA_DMACCR_REPEAT_OFF,
    DMA_DMACCR_AUTOINIT_ON,				/* AUTOINIT ON */
    DMA_DMACCR_EN_STOP,
    DMA_DMACCR_PRIO_LOW,
    DMA_DMACCR_FS_DISABLE,
    DMA_DMACCR_SYNC_REVT1
  ),                                       /* DMACCR   */
  DMA_DMACICR_RMK(
    DMA_DMACICR_AERRIE_ON,
    DMA_DMACICR_BLOCKIE_OFF,
    DMA_DMACICR_LASTIE_OFF,
    DMA_DMACICR_FRAMEIE_ON,
    DMA_DMACICR_FIRSTHALFIE_OFF,
    DMA_DMACICR_DROPIE_OFF,
    DMA_DMACICR_TIMEOUTIE_OFF
  ),                                       /* DMACICR  */
    (DMA_AdrPtr)(MCBSP_ADDR(DRR11)),       /* DMACSSAL */
    0,                                     /* DMACSSAU */
	NULL,							       /* DMACDSAL, to be loaded by submit  */
    0,                                     /* DMACDSAU */
    BUFFSIZE,                              /* DMACEN   */
    1,                                     /* DMACFN   */
    0,                                     /* DMACFI   */
    0                                      /* DMACEI   */
};

DMA_Config  dmaXmtConfig = { 
  DMA_DMACSDP_RMK(
    DMA_DMACSDP_DSTBEN_NOBURST,
    DMA_DMACSDP_DSTPACK_OFF,
    DMA_DMACSDP_DST_PERIPH,
    DMA_DMACSDP_SRCBEN_NOBURST,
    DMA_DMACSDP_SRCPACK_OFF,
    DMA_DMACSDP_SRC_DARAMPORT0,
    DMA_DMACSDP_DATATYPE_16BIT
  ),                                       /* DMACSDP  */
  DMA_DMACCR_RMK(
    DMA_DMACCR_DSTAMODE_CONST,
    DMA_DMACCR_SRCAMODE_POSTINC,
    DMA_DMACCR_ENDPROG_ON,
    DMA_DMACCR_WP_DEFAULT,
    DMA_DMACCR_REPEAT_OFF,
    DMA_DMACCR_AUTOINIT_OFF,
    DMA_DMACCR_EN_STOP,
    DMA_DMACCR_PRIO_LOW,
    DMA_DMACCR_FS_DISABLE,
    DMA_DMACCR_SYNC_XEVT1
  ),                                       /* DMACCR   */
  DMA_DMACICR_RMK(
    DMA_DMACICR_AERRIE_ON,    
    DMA_DMACICR_BLOCKIE_OFF,
    DMA_DMACICR_LASTIE_OFF,
    DMA_DMACICR_FRAMEIE_ON,
    DMA_DMACICR_FIRSTHALFIE_OFF,
    DMA_DMACICR_DROPIE_OFF,
    DMA_DMACICR_TIMEOUTIE_OFF
  ),                                       /* DMACICR  */
	NULL,                					/* DMACDSAL, to be loaded by submit  */
    0,                                     /* DMACSSAU */
    (DMA_AdrPtr)(MCBSP_ADDR(DXR11)),       /* DMACDSAL */
    0,                                     /* DMACDSAU */
    BUFFSIZE,                              /* DMACEN   */
    1,                                     /* DMACFN   */
    0,                                     /* DMACFI   */
    0                                      /* DMACEI   */
};

/* Define a DMA_Handle object to be used with DMA_open function */
DMA_Handle hDmaRcv, hDmaXmt;

/* Define a MCBSP_Handle object to be used with MCBSP_open function */
MCBSP_Handle hMcbsp;

volatile Uint16 transferComplete = FALSE;
Uint16 err = 0;
Uint16 old_intm;
Uint16 xmtEventId, rcvEventId;

//---------Function prototypes---------

/* Reference the start of the interrupt vector table */
/* This symbol is defined in file vectors.s55        */
extern void VECSTART(void);

/* Protoype for interrupt functions */
interrupt void dmaXmtIsr(void);
interrupt void dmaRcvIsr(void);
void taskFxn(void);


/*
 *  copyData() - Copy one buffer with length elements to another.
 */
void copyData(Int16 *inbuf, Int16 *outbuf, Int16 length)
{
    Int16 i = 0;

    for (i = 0; i < length; i++) {
        outbuf[i]  = inbuf[i];
    }
}


//######################################################################################################
void copyleft(Int16 *inbuf, Int16 *outbuf, Int16 length)
          {
                  Int16 i = 0;
                  Int16 k = 0;

                   for (i = 0,k = 0; i < length; i=i+2,k++) {
                    outbuf[k]  = inbuf[i];
             
                   }
          }

//######################################################################################################
      void copyright(Int16 *inbuf, Int16 *outbuf, Int16 length)
          {
                  Int16 i = 1;
                  Int16 k = 0;
                     for (i = 1,k = 0; i < length; i=i+2,k++) {
                         outbuf[k]  = inbuf[i];
                                              }
            
                  }

//######################################################################################################
 
      void recoverl(Int16 *inbuf, Int16 *outbuf, Int16 length)
          {
                  Int16 i = 0;
                

                   for (i = 0; i < length; i++) {
                    outbuf[2*i]  = inbuf[i];
                   }
          }

//######################################################################################################
      void recoverr(Int16 *inbuf, Int16 *outbuf, Int16 length)
          {
                  Int16 i = 1;
               
                   for (i = 0; i < length; i++) {
                    outbuf[2*i+1]  = inbuf[i];
                   }
          }


/* ------------------------------- Threads ------------------------------ */

/*
 *  processBuffer() - Process audio data once it has been received.
 */
void processBuffer(void)
{
    Uint32 addr;
    static Int16 pingPong = PING;
	Uint16  i = 0;
    Uint16 m=0;
    long n;
	short p,q;

	while(DMA_FGETH (hDmaRcv, DMACCR, ENDPROG)){
        ;   
    }

	/* 修改DMA接收通道的目的地址 */

    // Determine which ping-pong state we're in
    if (pingPong == PING)
    {
        // Configure the receive channel for pong input data
        addr = ((Uint32)gBufferRcvPong) << 1;
        DMA_RSETH(hDmaRcv, DMACDSAL, addr & 0xffff);
        DMA_RSETH(hDmaRcv, DMACDSAU, (addr >> 16) & 0xffff);

		// Set new state to PONG
        pingPong = PONG;
    }
    else
    {
        // Configure the receive channel for ping input data
        addr = ((Uint32)gBufferRcvPing) << 1;
        DMA_RSETH(hDmaRcv, DMACDSAL, addr & 0xffff);
        DMA_RSETH(hDmaRcv, DMACDSAU, (addr >> 16) & 0xffff);

       // Set new state to PING
        pingPong = PING;
    }

	DMA_FSETH (hDmaRcv, DMACCR, ENDPROG, 1);
	DMA_FSETH (hDmaXmt, DMACCR, ENDPROG, 1);


	/* 修改DMA发送通道的源地址 */

    if (pingPong == PONG)
    {  
       // copyData(gBufferRcvPing, gBufferXmtPing, BUFFSIZE);
        copyleft(gBufferRcvPing, leftRcvPing, BUFFSIZE);
        copyright(gBufferRcvPing, rightRcvPing, BUFFSIZE);
    for(i=0;i<(512);i++)
	   {
	      lleftRcvPing[i].real=leftRcvPing[i];  //short int
          lleftRcvPing[i].imag=0;  //short int
      	}	

		 fft512(lleftRcvPing,512);                   //FFT

	//	for(i=0;i<512;i++)
    //  	{  
    //      	p= lleftRcvPing[i].real;
    //      	q= lleftRcvPing[i].imag;     
    //      	n=(long)p*(long)p+(long)q*(long)q;
    //      	n=(Uint32)(p*p+q*q);
    //    	mod[m]=sqrt(n);
    //      	m++;                           
    //   	}
  
           //频域置零
           lleftRcvPing[32].real=0;                                      //F=2000HZ
           lleftRcvPing[32].imag=0; 
		   lleftRcvPing[40].real=0;                                      //F=2500HZ
	       lleftRcvPing[40].imag=0;
		   lleftRcvPing[48].real=0;                                      //F=3000HZ
		   lleftRcvPing[48].imag=0;
           	for(i=0;i<512/2;i++)                                         //由于实信号的频谱是对称的，恢复出后一半频谱
			{
             lleftRcvPing[512-i].real=100*lleftRcvPing[i].real;
             lleftRcvPing[512-i].imag=100*lleftRcvPing[i].imag;
             lleftRcvPing[i].real=100*lleftRcvPing[i].real;
             lleftRcvPing[i].imag=-100*lleftRcvPing[i].imag;
            }

            fft512(lleftRcvPing,512);                                    //IFFT
            for(i=0;i<1024/2;i++)
	   {
	      leftXmtPing[i]=100*lleftRcvPing[i].real;  //short int
         // leftXmtPing[2*i+1]=lleftRcvPing[i].imag;  //short int
       }
          recoverl(leftXmtPing, gBufferXmtPing, BUFFSIZE/2);



 for(i=0;i<(512);i++)
	   {
	      rrightRcvPing[i].real=rightRcvPing[i];  //short int
          rrightRcvPing[i].imag=0;  //short int
      	}	
		 fft512(rrightRcvPing,512);
		
		
	//	for(i=0;i<512;i++)
    //   	{  
    //       	p= lleftRcvPing[i].real;
    //       	q= lleftRcvPing[i].imag;     
    //       	n=(long)p*(long)p+(long)q*(long)q;
    //       	//n=(Uint32)(p*p+q*q);
    //       	mod[m]=sqrt(n);
    //      	m++;                           
    //    	}
	


          rrightRcvPing[32].real=0;
          rrightRcvPing[32].imag=0;
		  rrightRcvPing[40].real=0;
	      rrightRcvPing[40].imag=0;
		  rrightRcvPing[48].real=0;
		  rrightRcvPing[48].imag=0;
           	for(i=0;i<512/2;i++)
			{
             rrightRcvPing[512-i].real=100*rrightRcvPing[i].real;
             rrightRcvPing[512-i].imag=100*rrightRcvPing[i].imag;
             rrightRcvPing[i].real=100*rrightRcvPing[i].real;
             rrightRcvPing[i].imag=-100*rrightRcvPing[i].imag;
            }

            fft512(rrightRcvPing,512);
            for(i=0;i<1024/2;i++)
	   {
	      rightXmtPing[i]=100*rrightRcvPing[i].real;  //short int
         // leftXmtPing[2*i+1]=lleftRcvPing[i].imag;  //short int
       }
          recoverr(rightXmtPing, gBufferXmtPing, BUFFSIZE/2);







        // Configure the transmit channel for ping output data
        addr = ((Uint32)gBufferXmtPing) << 1;
        DMA_RSETH(hDmaXmt, DMACSSAL, addr & 0xffff);
        DMA_RSETH(hDmaXmt, DMACSSAU, (addr >> 16) & 0xffff);

    }
    else
    {
		//Insert your application program here
      //  copyData(gBufferRcvPong, gBufferXmtPong, BUFFSIZE);


       copyleft(gBufferRcvPong, leftRcvPong, BUFFSIZE);
       copyright(gBufferRcvPong, rightRcvPong, BUFFSIZE);
    for(i=0;i<(512);i++)
	   {
	      lleftRcvPong[i].real=leftRcvPong[i];  //short int
          lleftRcvPong[i].imag=0;  //short int
      	}	
		 fft512(lleftRcvPong,512);
		
		
	//	for(i=0;i<512;i++)
    //   	{  
    //       	p= lleftRcvPing[i].real;
    //       	q= lleftRcvPing[i].imag;     
    //       	n=(long)p*(long)p+(long)q*(long)q;
    //       	//n=(Uint32)(p*p+q*q);
    //       	mod[m]=sqrt(n);
    //      	m++;                           
    //    	}
	





           lleftRcvPong[32].real=0;
           lleftRcvPong[32].imag=0;
		   lleftRcvPong[40].real=0;
	       lleftRcvPong[40].imag=0;
		   lleftRcvPong[48].real=0;
		   lleftRcvPong[48].imag=0;
           	for(i=0;i<512/2;i++)
			{
             lleftRcvPong[512-i].real=100*lleftRcvPong[i].real;
             lleftRcvPong[512-i].imag=100*lleftRcvPong[i].imag;
             lleftRcvPong[i].real=100*lleftRcvPong[i].real;
             lleftRcvPong[i].imag=-100*lleftRcvPong[i].imag;
            }

            fft512(lleftRcvPong,512);
            for(i=0;i<1024/2;i++)
	   {
	      leftXmtPong[i]=100*lleftRcvPong[i].real;  //short int
         // leftXmtPing[2*i+1]=lleftRcvPing[i].imag;  //short int
       }
          recoverl(leftXmtPong, gBufferXmtPong, BUFFSIZE/2);



 for(i=0;i<(512);i++)
	   {
	      rrightRcvPong[i].real=rightRcvPong[i];  //short int
          rrightRcvPong[i].imag=0;  //short int
      	}	
		 fft512(rrightRcvPong,512);
		
		
	//	for(i=0;i<512;i++)
    //   	{  
    //       	p= lleftRcvPing[i].real;
    //       	q= lleftRcvPing[i].imag;     
    //       	n=(long)p*(long)p+(long)q*(long)q;
    //       	//n=(Uint32)(p*p+q*q);
    //       	mod[m]=sqrt(n);
    //      	m++;                           
    //    	}








          rrightRcvPong[32].real=0;
          rrightRcvPong[32].imag=0;
		  rrightRcvPong[40].real=0;
	      rrightRcvPong[40].imag=0;
		  rrightRcvPong[48].real=0;
		  rrightRcvPong[48].imag=0;
           	for(i=0;i<512/2;i++)
			{
             rrightRcvPong[512-i].real=100*rrightRcvPong[i].real;
             rrightRcvPong[512-i].imag=100*rrightRcvPong[i].imag;
             rrightRcvPong[i].real=100*rrightRcvPong[i].real;
             rrightRcvPong[i].imag=-100*rrightRcvPong[i].imag;
            }

            fft512(rrightRcvPong,512);
            for(i=0;i<1024/2;i++)
	   {
	      rightXmtPong[i]=100*rrightRcvPong[i].real;  //short int
         // leftXmtPing[2*i+1]=lleftRcvPing[i].imag;  //short int
       }
          recoverr(rightXmtPong, gBufferXmtPong, BUFFSIZE/2);







        // Configure the transmit channel for pong output data
        addr = ((Uint32)gBufferXmtPong) << 1;
        DMA_RSETH(hDmaXmt, DMACSSAL, addr & 0xffff);
        DMA_RSETH(hDmaXmt, DMACSSAU, (addr >> 16) & 0xffff);
    }

	/* 启动DMA传输 */
    // Start the DMA
    DMA_start(hDmaRcv);
    DMA_start(hDmaXmt);

}


//---------main routine---------
void main(void)
{
    Uint16 i;

    /* Initialize CSL library - This is REQUIRED !!! */
    CSL_init();

// The main frequency of system is 240MHz
// 该频率是为了设置IIC模块的需要设置的,为了使用I2C_setup函数
    PLL_setFreq(1, 0xC, 0, 1, 3, 3, 0);

    //EMIF初始化
     Emif_Config(); 
  
// Open McBSP port 1 and get a McBSP type handle
	hMcbsp = MCBSP_open(MCBSP_PORT1,MCBSP_OPEN_RESET);

// Config McBSP	port 1 by use previously defined structure
    MCBSP_config(hMcbsp, &Mcbsp1Config); 
	
//I2C初始化
	I2C_cofig(); 
    
//CODEC寄存器初始化
	inti_AIC(); 

    /* Set IVPH/IVPD to start of interrupt vector table */
    IRQ_setVecs((Uint32)(&VECSTART));

    for (i = 0; i <= BUFFSIZE - 1; i++) {  
        gBufferXmtPing[i] =  0;
        gBufferXmtPong[i] = 0;
    }

    /* Call function to effect transfer */
    taskFxn();
}

void taskFxn(void)
{
    Uint16 srcAddrHi, srcAddrLo;
    Uint16 dstAddrHi, dstAddrLo;
 
    /* By default, the TMS320C55xx compiler assigns all data symbols word */
    /* addresses. The DMA however, expects all addresses to be byte       */
    /* addresses. Therefore, we must shift the address by 2 in order to   */
    /* change the word address to a byte address for the DMA transfer.    */ 

	/* DMA接收通道的源地址和目的地址 */

    srcAddrHi = (Uint16)(((Uint32)(MCBSP_ADDR(DRR11))) >> 15) & 0xFFFFu;
    srcAddrLo = (Uint16)(((Uint32)(MCBSP_ADDR(DRR11))) << 1) & 0xFFFFu;
    dstAddrHi = (Uint16)(((Uint32)(&gBufferRcvPing)) >> 15) & 0xFFFFu;
    dstAddrLo = (Uint16)(((Uint32)(&gBufferRcvPing)) << 1) & 0xFFFFu;

    dmaRcvConfig.dmacssal = (DMA_AdrPtr)srcAddrLo;
    dmaRcvConfig.dmacssau = srcAddrHi;
    dmaRcvConfig.dmacdsal = (DMA_AdrPtr)dstAddrLo;
    dmaRcvConfig.dmacdsau = dstAddrHi;

	/* DMA发送通道的源地址和目的地址 */

	srcAddrHi = (Uint16)(((Uint32)(&gBufferXmtPing)) >> 15) & 0xFFFFu;
    srcAddrLo = (Uint16)(((Uint32)(&gBufferXmtPing)) << 1) & 0xFFFFu;
    dstAddrHi = (Uint16)(((Uint32)(MCBSP_ADDR(DXR11))) >> 15) & 0xFFFFu;
    dstAddrLo = (Uint16)(((Uint32)(MCBSP_ADDR(DXR11))) << 1) & 0xFFFFu;

    dmaXmtConfig.dmacssal = (DMA_AdrPtr)srcAddrLo;
    dmaXmtConfig.dmacssau = srcAddrHi;
    dmaXmtConfig.dmacdsal = (DMA_AdrPtr)dstAddrLo;
    dmaXmtConfig.dmacdsau = dstAddrHi;


    /* Open MCBSP Port 1 and set registers to their power on defaults */
//    hMcbsp = MCBSP_open(MCBSP_PORT1, MCBSP_OPEN_RESET);
    
	/* 配置DMA通道及其中断 */

    /* Open DMA channels 4 & 5 and set regs to power on defaults */
    hDmaRcv = DMA_open(DMA_CHA4,DMA_OPEN_RESET);
    hDmaXmt = DMA_open(DMA_CHA5,DMA_OPEN_RESET);  

    /* Get interrupt event associated with DMA receive and transmit */
    xmtEventId = DMA_getEventId(hDmaXmt);
    rcvEventId = DMA_getEventId(hDmaRcv);
    
    /* Temporarily disable interrupts and clear any pending */
    /* interrupts for MCBSP transmit */
    old_intm = IRQ_globalDisable();
    
    /* Clear any pending interrupts for DMA channels */
    IRQ_clear(xmtEventId);
    IRQ_clear(rcvEventId);

    /* Enable DMA interrupt in IER register */
    IRQ_enable(xmtEventId);
    IRQ_enable(rcvEventId);

    /* Place DMA interrupt service addresses at associate vector */
    IRQ_plug(xmtEventId,&dmaXmtIsr);
    IRQ_plug(rcvEventId,&dmaRcvIsr);

    /* Write values from configuration structure to MCBSP control regs */
    MCBSP_config(hMcbsp, &Mcbsp1Config); 
    
    /* Write values from configuration structure to DMA control regs */
    DMA_config(hDmaRcv,&dmaRcvConfig);
    DMA_config(hDmaXmt,&dmaXmtConfig);
  
   /* Enable all maskable interrupts */
    IRQ_globalEnable();

    /* Start Sample Rate Generator and Enable Frame Sync */
    MCBSP_start(hMcbsp,
                MCBSP_SRGR_START | MCBSP_SRGR_FRAMESYNC,
                0x300u);

	DMA_FSETH (hDmaRcv, DMACCR, ENDPROG, 1);
	DMA_FSETH (hDmaXmt, DMACCR, ENDPROG, 1);

    /* Enable DMA */
    DMA_start(hDmaRcv);
    DMA_start(hDmaXmt);

    /* Take MCBSP transmit and receive out of reset */
    MCBSP_start(hMcbsp,
                MCBSP_XMIT_START | MCBSP_RCV_START,
                0u);
  
   /* Wait for DMA transfer to be complete */
    while (TRUE){
        ;   
    }
   
    /* Restore status of global interrupt enable flag */
    IRQ_globalRestore(old_intm);
        
    /* We're done with MCBSP and DMA , so close them */
    MCBSP_close(hMcbsp);
    DMA_close(hDmaRcv);
    DMA_close(hDmaXmt);                     
}

/* DMA发送中断服务程序 */
interrupt void dmaXmtIsr(void) {

	// Read the DMA status register to clear it so new interrupts will be seen
    DMA_RGETH(hDmaXmt, DMACSR);
	DMA_start(hDmaXmt);

//   DMA_stop(hDmaXmt);
//   IRQ_disable(xmtEventId);
}

/* DMA接收中断服务程序 */
interrupt void dmaRcvIsr(void) {
	// Process audio data once it has been received
	processBuffer();

	// Read the DMA status register to clear it so new interrupts will be seen
    DMA_RGETH(hDmaRcv, DMACSR);

//   IRQ_disable(rcvEventId);
//   transferComplete = TRUE;
}
