//------------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the Silicon Laboratories End User
// License Agreement which accompanies this distribution, and is available at
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
// Original content and implementation provided by Silicon Laboratories.
//------------------------------------------------------------------------------
//Supported Devices:
//  EFM8UB10F16G
//  EFM8UB10F16G
//  EFM8UB10F8G
//  EFM8UB11F16G

#ifndef SI_EFM8UB1_DEFS_H
#define SI_EFM8UB1_DEFS_H

//#include <si_toolchain.h>

//-----------------------------------------------------------------------------
// Register Definitions
//-----------------------------------------------------------------------------
#define ACC        *(uint8_t*) 0xE0 ///< Accumulator                            
#define ADC0AC     *(uint8_t*) 0xB3 ///< ADC0 Accumulator Configuration         
#define ADC0CF     *(uint8_t*) 0xBC ///< ADC0 Configuration                     
#define ADC0CN0    *(uint8_t*) 0xE8 ///< ADC0 Control 0                         
#define ADC0CN1    *(uint8_t*) 0xB2 ///< ADC0 Control 1                         
#define ADC0GTH    *(uint8_t*) 0xC4 ///< ADC0 Greater-Than High Byte            
#define ADC0GTL    *(uint8_t*) 0xC3 ///< ADC0 Greater-Than Low Byte             
#define ADC0H      *(uint8_t*) 0xBE ///< ADC0 Data Word High Byte               
#define ADC0L      *(uint8_t*) 0xBD ///< ADC0 Data Word Low Byte                
#define ADC0LTH    *(uint8_t*) 0xC6 ///< ADC0 Less-Than High Byte               
#define ADC0LTL    *(uint8_t*) 0xC5 ///< ADC0 Less-Than Low Byte                
#define ADC0MX     *(uint8_t*) 0xBB ///< ADC0 Multiplexer Selection             
#define ADC0PWR    *(uint8_t*) 0xDF ///< ADC0 Power Control                     
#define ADC0TK     *(uint8_t*) 0xB9 ///< ADC0 Burst Mode Track Time             
#define B          *(uint8_t*) 0xF0 ///< B Register                             
#define CKCON0     *(uint8_t*) 0x8E ///< Clock Control 0                        
#define CKCON1     *(uint8_t*) 0xA6 ///< Clock Control 1                        
#define CLKSEL     *(uint8_t*) 0xA9 ///< Clock Select                           
#define CMP0CN0    *(uint8_t*) 0x9B ///< Comparator 0 Control 0                 
#define CMP0CN1    *(uint8_t*) 0x99 ///< Comparator 0 Control 1                 
#define CMP0MD     *(uint8_t*) 0x9D ///< Comparator 0 Mode                      
#define CMP0MX     *(uint8_t*) 0x9F ///< Comparator 0 Multiplexer Selection     
#define CMP1CN0    *(uint8_t*) 0xBF ///< Comparator 1 Control 0                 
#define CMP1CN1    *(uint8_t*) 0xAC ///< Comparator 1 Control 1                 
#define CMP1MD     *(uint8_t*) 0xAB ///< Comparator 1 Mode                      
#define CMP1MX     *(uint8_t*) 0xAA ///< Comparator 1 Multiplexer Selection     
#define CRC0CN0    *(uint8_t*) 0xCE ///< CRC0 Control 0                         
#define CRC0CN1    *(uint8_t*) 0x86 ///< CRC0 Control 1                         
#define CRC0CNT    *(uint8_t*) 0xD3 ///< CRC0 Automatic Flash Sector Count      
#define CRC0DAT    *(uint8_t*) 0xDE ///< CRC0 Data Output                       
#define CRC0FLIP   *(uint8_t*) 0xCF ///< CRC0 Bit Flip                          
#define CRC0IN     *(uint8_t*) 0xDD ///< CRC0 Data Input                        
#define CRC0ST     *(uint8_t*) 0xD2 ///< CRC0 Automatic Flash Sector Start      
#define DERIVID    *(uint8_t*) 0xAD ///< Derivative Identification              
#define DEVICEID   *(uint8_t*) 0xB5 ///< Device Identification                  
#define DPH        *(uint8_t*) 0x83 ///< Data Pointer High                      
#define DPL        *(uint8_t*) 0x82 ///< Data Pointer Low                       
#define EIE1       *(uint8_t*) 0xE6 ///< Extended Interrupt Enable 1            
#define EIE2       *(uint8_t*) 0xCE ///< Extended Interrupt Enable 2            
#define EIP1       *(uint8_t*) 0xF3 ///< Extended Interrupt Priority 1 Low      
#define EIP1H      *(uint8_t*) 0xF5 ///< Extended Interrupt Priority 1 High     
#define EIP2       *(uint8_t*) 0xF4 ///< Extended Interrupt Priority 2          
#define EIP2H      *(uint8_t*) 0xF6 ///< Extended Interrupt Priority 2 High     
#define EMI0CN     *(uint8_t*) 0xE7 ///< External Memory Interface Control      
#define FLKEY      *(uint8_t*) 0xB7 ///< Flash Lock and Key                     
#define HFO0CAL    *(uint8_t*) 0xC7 ///< High Frequency Oscillator 0 Calibration
#define HFO1CAL    *(uint8_t*) 0xD6 ///< High Frequency Oscillator 1 Calibration
#define HFOCN      *(uint8_t*) 0xEF ///< High Frequency Oscillator Control      
#define I2C0CN0    *(uint8_t*) 0xBA ///< I2C0 Control                           
#define I2C0DIN    *(uint8_t*) 0xBC ///< I2C0 Received Data                     
#define I2C0DOUT   *(uint8_t*) 0xBB ///< I2C0 Transmit Data                     
#define I2C0FCN0   *(uint8_t*) 0xAD ///< I2C0 FIFO Control 0                    
#define I2C0FCN1   *(uint8_t*) 0xAB ///< I2C0 FIFO Control 1                    
#define I2C0FCT    *(uint8_t*) 0xF5 ///< I2C0 FIFO Count                        
#define I2C0SLAD   *(uint8_t*) 0xBD ///< I2C0 Slave Address                     
#define I2C0STAT   *(uint8_t*) 0xB9 ///< I2C0 Status                            
#define IE         *(uint8_t*) 0xA8 ///< Interrupt Enable                       
#define IP         *(uint8_t*) 0xB8 ///< Interrupt Priority                     
#define IPH        *(uint8_t*) 0xF2 ///< Interrupt Priority High                
#define IT01CF     *(uint8_t*) 0xE4 ///< INT0/INT1 Configuration                
#define LFO0CN     *(uint8_t*) 0xB1 ///< Low Frequency Oscillator Control       
#define P0         *(uint8_t*) 0x80 ///< Port 0 Pin Latch                       
#define P0MASK     *(uint8_t*) 0xFE ///< Port 0 Mask                            
#define P0MAT      *(uint8_t*) 0xFD ///< Port 0 Match                           
#define P0MDIN     *(uint8_t*) 0xF1 ///< Port 0 Input Mode                      
#define P0MDOUT    *(uint8_t*) 0xA4 ///< Port 0 Output Mode                     
#define P0SKIP     *(uint8_t*) 0xD4 ///< Port 0 Skip                            
#define P1         *(uint8_t*) 0x90 ///< Port 1 Pin Latch                       
#define P1MASK     *(uint8_t*) 0xEE ///< Port 1 Mask                            
#define P1MAT      *(uint8_t*) 0xED ///< Port 1 Match                           
#define P1MDIN     *(uint8_t*) 0xF2 ///< Port 1 Input Mode                      
#define P1MDOUT    *(uint8_t*) 0xA5 ///< Port 1 Output Mode                     
#define P1SKIP     *(uint8_t*) 0xD5 ///< Port 1 Skip                            
#define P2         *(uint8_t*) 0xA0 ///< Port 2 Pin Latch                       
#define P2MASK     *(uint8_t*) 0xFC ///< Port 2 Mask                            
#define P2MAT      *(uint8_t*) 0xFB ///< Port 2 Match                           
#define P2MDIN     *(uint8_t*) 0xF3 ///< Port 2 Input Mode                      
#define P2MDOUT    *(uint8_t*) 0xA6 ///< Port 2 Output Mode                     
#define P2SKIP     *(uint8_t*) 0xCC ///< Port 2 Skip                            
#define P3         *(uint8_t*) 0xB0 ///< Port 3 Pin Latch                       
#define P3MDIN     *(uint8_t*) 0xF4 ///< Port 3 Input Mode                      
#define P3MDOUT    *(uint8_t*) 0x9C ///< Port 3 Output Mode                     
#define PCA0CENT   *(uint8_t*) 0x9E ///< PCA Center Alignment Enable            
#define PCA0CLR    *(uint8_t*) 0x9C ///< PCA Comparator Clear Control           
#define PCA0CN0    *(uint8_t*) 0xD8 ///< PCA Control                            
#define PCA0CPH0   *(uint8_t*) 0xFC ///< PCA Channel 0 Capture Module High Byte 
#define PCA0CPH1   *(uint8_t*) 0xEA ///< PCA Channel 1 Capture Module High Byte 
#define PCA0CPH2   *(uint8_t*) 0xEC ///< PCA Channel 2 Capture Module High Byte 
#define PCA0CPL0   *(uint8_t*) 0xFB ///< PCA Channel 0 Capture Module Low Byte  
#define PCA0CPL1   *(uint8_t*) 0xE9 ///< PCA Channel 1 Capture Module Low Byte  
#define PCA0CPL2   *(uint8_t*) 0xEB ///< PCA Channel 2 Capture Module Low Byte  
#define PCA0CPM0   *(uint8_t*) 0xDA ///< PCA Channel 0 Capture/Compare Mode     
#define PCA0CPM1   *(uint8_t*) 0xDB ///< PCA Channel 1 Capture/Compare Mode     
#define PCA0CPM2   *(uint8_t*) 0xDC ///< PCA Channel 2 Capture/Compare Mode     
#define PCA0H      *(uint8_t*) 0xFA ///< PCA Counter/Timer High Byte            
#define PCA0L      *(uint8_t*) 0xF9 ///< PCA Counter/Timer Low Byte             
#define PCA0MD     *(uint8_t*) 0xD9 ///< PCA Mode                               
#define PCA0POL    *(uint8_t*) 0x96 ///< PCA Output Polarity                    
#define PCA0PWM    *(uint8_t*) 0xF7 ///< PCA PWM Configuration                  
#define PCON0      *(uint8_t*) 0x87 ///< Power Control                          
#define PCON1      *(uint8_t*) 0x9A ///< Power Control 1                        
#define PFE0CN     *(uint8_t*) 0xC1 ///< Prefetch Engine Control                
#define PRTDRV     *(uint8_t*) 0xF6 ///< Port Drive Strength                    
#define PSCTL      *(uint8_t*) 0x8F ///< Program Store Control                  
#define PSW        *(uint8_t*) 0xD0 ///< Program Status Word                    
#define REF0CN     *(uint8_t*) 0xD1 ///< Voltage Reference Control              
#define REG0CN     *(uint8_t*) 0xC9 ///< Voltage Regulator 0 Control            
#define REG1CN     *(uint8_t*) 0xC6 ///< Voltage Regulator 1 Control            
#define REVID      *(uint8_t*) 0xB6 ///< Revision Identifcation                 
#define RSTSRC     *(uint8_t*) 0xEF ///< Reset Source                           
#define SBCON1     *(uint8_t*) 0x94 ///< UART1 Baud Rate Generator Control      
#define SBRLH1     *(uint8_t*) 0x96 ///< UART1 Baud Rate Generator High Byte    
#define SBRLL1     *(uint8_t*) 0x95 ///< UART1 Baud Rate Generator Low Byte     
#define SBUF0      *(uint8_t*) 0x99 ///< UART0 Serial Port Data Buffer          
#define SBUF1      *(uint8_t*) 0x92 ///< UART1 Serial Port Data Buffer          
#define SCON0      *(uint8_t*) 0x98 ///< UART0 Serial Port Control              
#define SCON1      *(uint8_t*) 0xC8 ///< UART1 Serial Port Control              
#define SFRPAGE    *(uint8_t*) 0xA7 ///< SFR Page                               
#define SFRPGCN    *(uint8_t*) 0xCF ///< SFR Page Control                       
#define SFRSTACK   *(uint8_t*) 0xD7 ///< SFR Page Stack                         
#define SMB0ADM    *(uint8_t*) 0xD6 ///< SMBus 0 Slave Address Mask             
#define SMB0ADR    *(uint8_t*) 0xD7 ///< SMBus 0 Slave Address                  
#define SMB0CF     *(uint8_t*) 0xC1 ///< SMBus 0 Configuration                  
#define SMB0CN0    *(uint8_t*) 0xC0 ///< SMBus 0 Control                        
#define SMB0DAT    *(uint8_t*) 0xC2 ///< SMBus 0 Data                           
#define SMB0FCN0   *(uint8_t*) 0xC3 ///< SMBus0 FIFO Control 0                  
#define SMB0FCN1   *(uint8_t*) 0xC4 ///< SMBus0 FIFO Control 1                  
#define SMB0FCT    *(uint8_t*) 0xEF ///< SMBus0 FIFO Count                      
#define SMB0RXLN   *(uint8_t*) 0xC5 ///< SMBus0 Receive Length Counter          
#define SMB0TC     *(uint8_t*) 0xAC ///< SMBus 0 Timing and Pin Control         
#define SMOD1      *(uint8_t*) 0x93 ///< UART1 Mode                             
#define SP         *(uint8_t*) 0x81 ///< Stack Pointer                          
#define SPI0CFG    *(uint8_t*) 0xA1 ///< SPI0 Configuration                     
#define SPI0CKR    *(uint8_t*) 0xA2 ///< SPI0 Clock Rate                        
#define SPI0CN0    *(uint8_t*) 0xF8 ///< SPI0 Control                           
#define SPI0DAT    *(uint8_t*) 0xA3 ///< SPI0 Data                              
#define SPI0FCN0   *(uint8_t*) 0x9A ///< SPI0 FIFO Control 0                    
#define SPI0FCN1   *(uint8_t*) 0x9B ///< SPI0 FIFO Control 1                    
#define SPI0FCT    *(uint8_t*) 0xF7 ///< SPI0 FIFO Count                        
#define TCON       *(uint8_t*) 0x88 ///< Timer 0/1 Control                      
#define TH0        *(uint8_t*) 0x8C ///< Timer 0 High Byte                      
#define TH1        *(uint8_t*) 0x8D ///< Timer 1 High Byte                      
#define TL0        *(uint8_t*) 0x8A ///< Timer 0 Low Byte                       
#define TL1        *(uint8_t*) 0x8B ///< Timer 1 Low Byte                       
#define TMOD       *(uint8_t*) 0x89 ///< Timer 0/1 Mode                         
#define TMR2CN0    *(uint8_t*) 0xC8 ///< Timer 2 Control 0                      
#define TMR2CN1    *(uint8_t*) 0xFD ///< Timer 2 Control 1                      
#define TMR2H      *(uint8_t*) 0xCD ///< Timer 2 High Byte                      
#define TMR2L      *(uint8_t*) 0xCC ///< Timer 2 Low Byte                       
#define TMR2RLH    *(uint8_t*) 0xCB ///< Timer 2 Reload High Byte               
#define TMR2RLL    *(uint8_t*) 0xCA ///< Timer 2 Reload Low Byte                
#define TMR3CN0    *(uint8_t*) 0x91 ///< Timer 3 Control 0                      
#define TMR3CN1    *(uint8_t*) 0xFE ///< Timer 3 Control 1                      
#define TMR3H      *(uint8_t*) 0x95 ///< Timer 3 High Byte                      
#define TMR3L      *(uint8_t*) 0x94 ///< Timer 3 Low Byte                       
#define TMR3RLH    *(uint8_t*) 0x93 ///< Timer 3 Reload High Byte               
#define TMR3RLL    *(uint8_t*) 0x92 ///< Timer 3 Reload Low Byte                
#define TMR4CN0    *(uint8_t*) 0x98 ///< Timer 4 Control 0                      
#define TMR4CN1    *(uint8_t*) 0xFF ///< Timer 4 Control 1                      
#define TMR4H      *(uint8_t*) 0xA5 ///< Timer 4 High Byte                      
#define TMR4L      *(uint8_t*) 0xA4 ///< Timer 4 Low Byte                       
#define TMR4RLH    *(uint8_t*) 0xA3 ///< Timer 4 Reload High Byte               
#define TMR4RLL    *(uint8_t*) 0xA2 ///< Timer 4 Reload Low Byte                
#define UART1FCN0  *(uint8_t*) 0x9D ///< UART1 FIFO Control 0                   
#define UART1FCN1  *(uint8_t*) 0xD8 ///< UART1 FIFO Control 1                   
#define UART1FCT   *(uint8_t*) 0xFA ///< UART1 FIFO Count                       
#define UART1LIN   *(uint8_t*) 0x9E ///< UART1 LIN Configuration                
#define USB0ADR    *(uint8_t*) 0xAE ///< USB0 Indirect Address                  
#define USB0AEC    *(uint8_t*) 0xB2 ///< USB0 Advanced Energy Control           
#define USB0CDCF   *(uint8_t*) 0xB6 ///< USB0 Charger Detect Configuration      
#define USB0CDCN   *(uint8_t*) 0xBE ///< USB0 Charger Detect Control            
#define USB0CDSTA  *(uint8_t*) 0xBF ///< USB0 Charger Detect Status             
#define USB0CF     *(uint8_t*) 0xB5 ///< USB0 Configuration                     
#define USB0DAT    *(uint8_t*) 0xAF ///< USB0 Data                              
#define USB0XCN    *(uint8_t*) 0xB3 ///< USB0 Transceiver Control               
#define VDM0CN     *(uint8_t*) 0xFF ///< Supply Monitor Control                 
#define WDTCN      *(uint8_t*) 0x97 ///< Watchdog Timer Control                 
#define XBR0       *(uint8_t*) 0xE1 ///< Port I/O Crossbar 0                    
#define XBR1       *(uint8_t*) 0xE2 ///< Port I/O Crossbar 1                    
#define XBR2       *(uint8_t*) 0xE3 ///< Port I/O Crossbar 2                    
/*
//------------------------------------------------------------------------------
// 16-bit Register Definitions (may not work on all compilers)
//------------------------------------------------------------------------------
SI_SFR16 (ADC0GT,  0xC3); ///< ADC0 Greater-Than            
SI_SFR16 (ADC0,    0xBD); ///< ADC0 Data Word               
SI_SFR16 (ADC0LT,  0xC5); ///< ADC0 Less-Than               
SI_SFR16 (DP,      0x82); ///< Data Pointer                 
SI_SFR16 (PCA0CP0, 0xFB); ///< PCA Channel 0 Capture Module 
SI_SFR16 (PCA0CP1, 0xE9); ///< PCA Channel 1 Capture Module 
SI_SFR16 (PCA0CP2, 0xEB); ///< PCA Channel 2 Capture Module 
SI_SFR16 (PCA0,    0xF9); ///< PCA Counter/Timer            
SI_SFR16 (SBRL1,   0x95); ///< UART1 Baud Rate Generator    
SI_SFR16 (TMR2,    0xCC); ///< Timer 2                      
SI_SFR16 (TMR2RL,  0xCA); ///< Timer 2 Reload               
SI_SFR16 (TMR3,    0x94); ///< Timer 3                      
SI_SFR16 (TMR3RL,  0x92); ///< Timer 3 Reload               
SI_SFR16 (TMR4,    0xA4); ///< Timer 4                      
SI_SFR16 (TMR4RL,  0xA2); ///< Timer 4 Reload               

//------------------------------------------------------------------------------
// Indirect Register Definitions
//------------------------------------------------------------------------------
#define CLKREC   0x0F ///< USB0 Clock Recovery Control       
#define CMIE     0x0B ///< USB0 Common Interrupt Enable      
#define CMINT    0x06 ///< USB0 Common Interrupt             
#define E0CNT    0x16 ///< USB0 Endpoint0 Data Count         
#define E0CSR    0x11 ///< USB0 Endpoint0 Control            
#define EENABLE  0x1E ///< USB0 Endpoint Enable              
#define EINCSRH  0x12 ///< USB0 IN Endpoint Control High     
#define EINCSRL  0x11 ///< USB0 IN Endpoint Control          
#define EOUTCNTH 0x17 ///< USB0 OUT Endpoint Count High      
#define EOUTCNTL 0x16 ///< USB0 OUT Endpoint Count           
#define EOUTCSRH 0x15 ///< USB0 OUT Endpoint Control High    
#define EOUTCSRL 0x14 ///< USB0 OUT Endpoint Control         
#define FADDR    0x00 ///< USB0 Function Address             
#define FIFO0    0x20 ///< USB0 Endpoint 0 FIFO Access       
#define FIFO1    0x21 ///< USB0 Endpoint 1 FIFO Access       
#define FIFO2    0x22 ///< USB0 Endpoint 2 FIFO Access       
#define FIFO3    0x23 ///< USB0 Endpoint 3 FIFO Access       
#define FRAMEH   0x0D ///< USB0 Frame Number High            
#define FRAMEL   0x0C ///< USB0 Frame Number                 
#define IN1IE    0x07 ///< USB0 IN Endpoint Interrupt Enable 
#define IN1INT   0x02 ///< USB0 IN Endpoint Interrupt        
#define INDEX    0x0E ///< USB0 Endpoint Index               
#define OUT1IE   0x09 ///< USB0 OUT Endpoint Interrupt Enable
#define OUT1INT  0x04 ///< USB0 OUT Endpoint Interrupt       
#define POWER    0x01 ///< USB0 Power                        

//------------------------------------------------------------------------------
// Bit Definitions
//------------------------------------------------------------------------------

// ACC (Accumulator)
#define SFR_ACC 0xE0
SI_SBIT (ACC_ACC0, SFR_ACC, 0); ///< Accumulator Bit 0
SI_SBIT (ACC_ACC1, SFR_ACC, 1); ///< Accumulator Bit 1
SI_SBIT (ACC_ACC2, SFR_ACC, 2); ///< Accumulator Bit 2
SI_SBIT (ACC_ACC3, SFR_ACC, 3); ///< Accumulator Bit 3
SI_SBIT (ACC_ACC4, SFR_ACC, 4); ///< Accumulator Bit 4
SI_SBIT (ACC_ACC5, SFR_ACC, 5); ///< Accumulator Bit 5
SI_SBIT (ACC_ACC6, SFR_ACC, 6); ///< Accumulator Bit 6
SI_SBIT (ACC_ACC7, SFR_ACC, 7); ///< Accumulator Bit 7

// ADC0CN0 (ADC0 Control 0)
#define SFR_ADC0CN0 0xE8
SI_SBIT (ADC0CN0_ADCM0,  SFR_ADC0CN0, 0); ///< Start of Conversion Mode Select Bit 0
SI_SBIT (ADC0CN0_ADCM1,  SFR_ADC0CN0, 1); ///< Start of Conversion Mode Select Bit 1
SI_SBIT (ADC0CN0_ADCM2,  SFR_ADC0CN0, 2); ///< Start of Conversion Mode Select Bit 2
SI_SBIT (ADC0CN0_ADWINT, SFR_ADC0CN0, 3); ///< Window Compare Interrupt Flag        
SI_SBIT (ADC0CN0_ADBUSY, SFR_ADC0CN0, 4); ///< ADC Busy                             
SI_SBIT (ADC0CN0_ADINT,  SFR_ADC0CN0, 5); ///< Conversion Complete Interrupt Flag   
SI_SBIT (ADC0CN0_ADBMEN, SFR_ADC0CN0, 6); ///< Burst Mode Enable                    
SI_SBIT (ADC0CN0_ADEN,   SFR_ADC0CN0, 7); ///< ADC Enable                           

// B (B Register)
#define SFR_B 0xF0
SI_SBIT (B_B0, SFR_B, 0); ///< B Register Bit 0
SI_SBIT (B_B1, SFR_B, 1); ///< B Register Bit 1
SI_SBIT (B_B2, SFR_B, 2); ///< B Register Bit 2
SI_SBIT (B_B3, SFR_B, 3); ///< B Register Bit 3
SI_SBIT (B_B4, SFR_B, 4); ///< B Register Bit 4
SI_SBIT (B_B5, SFR_B, 5); ///< B Register Bit 5
SI_SBIT (B_B6, SFR_B, 6); ///< B Register Bit 6
SI_SBIT (B_B7, SFR_B, 7); ///< B Register Bit 7

// IE (Interrupt Enable)
#define SFR_IE 0xA8
SI_SBIT (IE_EX0,   SFR_IE, 0); ///< External Interrupt 0 Enable
SI_SBIT (IE_ET0,   SFR_IE, 1); ///< Timer 0 Interrupt Enable   
SI_SBIT (IE_EX1,   SFR_IE, 2); ///< External Interrupt 1 Enable
SI_SBIT (IE_ET1,   SFR_IE, 3); ///< Timer 1 Interrupt Enable   
SI_SBIT (IE_ES0,   SFR_IE, 4); ///< UART0 Interrupt Enable     
SI_SBIT (IE_ET2,   SFR_IE, 5); ///< Timer 2 Interrupt Enable   
SI_SBIT (IE_ESPI0, SFR_IE, 6); ///< SPI0 Interrupt Enable      
SI_SBIT (IE_EA,    SFR_IE, 7); ///< All Interrupts Enable      

// IP (Interrupt Priority)
#define SFR_IP 0xB8
SI_SBIT (IP_PX0,   SFR_IP, 0); ///< External Interrupt 0 Priority Control LSB                        
SI_SBIT (IP_PT0,   SFR_IP, 1); ///< Timer 0 Interrupt Priority Control LSB                           
SI_SBIT (IP_PX1,   SFR_IP, 2); ///< External Interrupt 1 Priority Control LSB                        
SI_SBIT (IP_PT1,   SFR_IP, 3); ///< Timer 1 Interrupt Priority Control LSB                           
SI_SBIT (IP_PS0,   SFR_IP, 4); ///< UART0 Interrupt Priority Control LSB                             
SI_SBIT (IP_PT2,   SFR_IP, 5); ///< Timer 2 Interrupt Priority Control LSB                           
SI_SBIT (IP_PSPI0, SFR_IP, 6); ///< Serial Peripheral Interface (SPI0) Interrupt Priority Control LSB

// P0 (Port 0 Pin Latch)
#define SFR_P0 0x80
SI_SBIT (P0_B0, SFR_P0, 0); ///< Port 0 Bit 0 Latch
SI_SBIT (P0_B1, SFR_P0, 1); ///< Port 0 Bit 1 Latch
SI_SBIT (P0_B2, SFR_P0, 2); ///< Port 0 Bit 2 Latch
SI_SBIT (P0_B3, SFR_P0, 3); ///< Port 0 Bit 3 Latch
SI_SBIT (P0_B4, SFR_P0, 4); ///< Port 0 Bit 4 Latch
SI_SBIT (P0_B5, SFR_P0, 5); ///< Port 0 Bit 5 Latch
SI_SBIT (P0_B6, SFR_P0, 6); ///< Port 0 Bit 6 Latch
SI_SBIT (P0_B7, SFR_P0, 7); ///< Port 0 Bit 7 Latch

// P1 (Port 1 Pin Latch)
#define SFR_P1 0x90
SI_SBIT (P1_B0, SFR_P1, 0); ///< Port 1 Bit 0 Latch
SI_SBIT (P1_B1, SFR_P1, 1); ///< Port 1 Bit 1 Latch
SI_SBIT (P1_B2, SFR_P1, 2); ///< Port 1 Bit 2 Latch
SI_SBIT (P1_B3, SFR_P1, 3); ///< Port 1 Bit 3 Latch
SI_SBIT (P1_B4, SFR_P1, 4); ///< Port 1 Bit 4 Latch
SI_SBIT (P1_B5, SFR_P1, 5); ///< Port 1 Bit 5 Latch
SI_SBIT (P1_B6, SFR_P1, 6); ///< Port 1 Bit 6 Latch
SI_SBIT (P1_B7, SFR_P1, 7); ///< Port 1 Bit 7 Latch

// P2 (Port 2 Pin Latch)
#define SFR_P2 0xA0
SI_SBIT (P2_B0, SFR_P2, 0); ///< Port 2 Bit 0 Latch
SI_SBIT (P2_B1, SFR_P2, 1); ///< Port 2 Bit 1 Latch
SI_SBIT (P2_B2, SFR_P2, 2); ///< Port 2 Bit 2 Latch
SI_SBIT (P2_B3, SFR_P2, 3); ///< Port 2 Bit 3 Latch

// P3 (Port 3 Pin Latch)
#define SFR_P3 0xB0
SI_SBIT (P3_B0, SFR_P3, 0); ///< Port 3 Bit 0 Latch
SI_SBIT (P3_B1, SFR_P3, 1); ///< Port 3 Bit 1 Latch

// PCA0CN0 (PCA Control)
#define SFR_PCA0CN0 0xD8
SI_SBIT (PCA0CN0_CCF0, SFR_PCA0CN0, 0); ///< PCA Module 0 Capture/Compare Flag
SI_SBIT (PCA0CN0_CCF1, SFR_PCA0CN0, 1); ///< PCA Module 1 Capture/Compare Flag
SI_SBIT (PCA0CN0_CCF2, SFR_PCA0CN0, 2); ///< PCA Module 2 Capture/Compare Flag
SI_SBIT (PCA0CN0_CR,   SFR_PCA0CN0, 6); ///< PCA Counter/Timer Run Control    
SI_SBIT (PCA0CN0_CF,   SFR_PCA0CN0, 7); ///< PCA Counter/Timer Overflow Flag  

// PSW (Program Status Word)
#define SFR_PSW 0xD0
SI_SBIT (PSW_PARITY, SFR_PSW, 0); ///< Parity Flag               
SI_SBIT (PSW_F1,     SFR_PSW, 1); ///< User Flag 1               
SI_SBIT (PSW_OV,     SFR_PSW, 2); ///< Overflow Flag             
SI_SBIT (PSW_RS0,    SFR_PSW, 3); ///< Register Bank Select Bit 0
SI_SBIT (PSW_RS1,    SFR_PSW, 4); ///< Register Bank Select Bit 1
SI_SBIT (PSW_F0,     SFR_PSW, 5); ///< User Flag 0               
SI_SBIT (PSW_AC,     SFR_PSW, 6); ///< Auxiliary Carry Flag      
SI_SBIT (PSW_CY,     SFR_PSW, 7); ///< Carry Flag                

// SCON0 (UART0 Serial Port Control)
#define SFR_SCON0 0x98
SI_SBIT (SCON0_RI,    SFR_SCON0, 0); ///< Receive Interrupt Flag             
SI_SBIT (SCON0_TI,    SFR_SCON0, 1); ///< Transmit Interrupt Flag            
SI_SBIT (SCON0_RB8,   SFR_SCON0, 2); ///< Ninth Receive Bit                  
SI_SBIT (SCON0_TB8,   SFR_SCON0, 3); ///< Ninth Transmission Bit             
SI_SBIT (SCON0_REN,   SFR_SCON0, 4); ///< Receive Enable                     
SI_SBIT (SCON0_MCE,   SFR_SCON0, 5); ///< Multiprocessor Communication Enable
SI_SBIT (SCON0_SMODE, SFR_SCON0, 7); ///< Serial Port 0 Operation Mode       

// SCON1 (UART1 Serial Port Control)
#define SFR_SCON1 0xC8
SI_SBIT (SCON1_RI,   SFR_SCON1, 0); ///< Receive Interrupt Flag   
SI_SBIT (SCON1_TI,   SFR_SCON1, 1); ///< Transmit Interrupt Flag  
SI_SBIT (SCON1_RBX,  SFR_SCON1, 2); ///< Extra Receive Bit        
SI_SBIT (SCON1_TBX,  SFR_SCON1, 3); ///< Extra Transmission Bit   
SI_SBIT (SCON1_REN,  SFR_SCON1, 4); ///< Receive Enable           
SI_SBIT (SCON1_PERR, SFR_SCON1, 6); ///< Parity Error Flag        
SI_SBIT (SCON1_OVR,  SFR_SCON1, 7); ///< Receive FIFO Overrun Flag

// SMB0CN0 (SMBus 0 Control)
#define SFR_SMB0CN0 0xC0
SI_SBIT (SMB0CN0_SI,      SFR_SMB0CN0, 0); ///< SMBus Interrupt Flag            
SI_SBIT (SMB0CN0_ACK,     SFR_SMB0CN0, 1); ///< SMBus Acknowledge               
SI_SBIT (SMB0CN0_ARBLOST, SFR_SMB0CN0, 2); ///< SMBus Arbitration Lost Indicator
SI_SBIT (SMB0CN0_ACKRQ,   SFR_SMB0CN0, 3); ///< SMBus Acknowledge Request       
SI_SBIT (SMB0CN0_STO,     SFR_SMB0CN0, 4); ///< SMBus Stop Flag                 
SI_SBIT (SMB0CN0_STA,     SFR_SMB0CN0, 5); ///< SMBus Start Flag                
SI_SBIT (SMB0CN0_TXMODE,  SFR_SMB0CN0, 6); ///< SMBus Transmit Mode Indicator   
SI_SBIT (SMB0CN0_MASTER,  SFR_SMB0CN0, 7); ///< SMBus Master/Slave Indicator    

// SPI0CN0 (SPI0 Control)
#define SFR_SPI0CN0 0xF8
SI_SBIT (SPI0CN0_SPIEN,  SFR_SPI0CN0, 0); ///< SPI0 Enable            
SI_SBIT (SPI0CN0_TXNF,   SFR_SPI0CN0, 1); ///< TX FIFO Not Full       
SI_SBIT (SPI0CN0_NSSMD0, SFR_SPI0CN0, 2); ///< Slave Select Mode Bit 0
SI_SBIT (SPI0CN0_NSSMD1, SFR_SPI0CN0, 3); ///< Slave Select Mode Bit 1
SI_SBIT (SPI0CN0_RXOVRN, SFR_SPI0CN0, 4); ///< Receive Overrun Flag   
SI_SBIT (SPI0CN0_MODF,   SFR_SPI0CN0, 5); ///< Mode Fault Flag        
SI_SBIT (SPI0CN0_WCOL,   SFR_SPI0CN0, 6); ///< Write Collision Flag   
SI_SBIT (SPI0CN0_SPIF,   SFR_SPI0CN0, 7); ///< SPI0 Interrupt Flag    

// TCON (Timer 0/1 Control)
#define SFR_TCON 0x88
SI_SBIT (TCON_IT0, SFR_TCON, 0); ///< Interrupt 0 Type Select
SI_SBIT (TCON_IE0, SFR_TCON, 1); ///< External Interrupt 0   
SI_SBIT (TCON_IT1, SFR_TCON, 2); ///< Interrupt 1 Type Select
SI_SBIT (TCON_IE1, SFR_TCON, 3); ///< External Interrupt 1   
SI_SBIT (TCON_TR0, SFR_TCON, 4); ///< Timer 0 Run Control    
SI_SBIT (TCON_TF0, SFR_TCON, 5); ///< Timer 0 Overflow Flag  
SI_SBIT (TCON_TR1, SFR_TCON, 6); ///< Timer 1 Run Control    
SI_SBIT (TCON_TF1, SFR_TCON, 7); ///< Timer 1 Overflow Flag  

// TMR2CN0 (Timer 2 Control 0)
#define SFR_TMR2CN0 0xC8
SI_SBIT (TMR2CN0_T2XCLK0, SFR_TMR2CN0, 0); ///< Timer 2 External Clock Select Bit 0
SI_SBIT (TMR2CN0_T2XCLK1, SFR_TMR2CN0, 1); ///< Timer 2 External Clock Select Bit 1
SI_SBIT (TMR2CN0_TR2,     SFR_TMR2CN0, 2); ///< Timer 2 Run Control                
SI_SBIT (TMR2CN0_T2SPLIT, SFR_TMR2CN0, 3); ///< Timer 2 Split Mode Enable          
SI_SBIT (TMR2CN0_TF2CEN,  SFR_TMR2CN0, 4); ///< Timer 2 Capture Enable             
SI_SBIT (TMR2CN0_TF2LEN,  SFR_TMR2CN0, 5); ///< Timer 2 Low Byte Interrupt Enable  
SI_SBIT (TMR2CN0_TF2L,    SFR_TMR2CN0, 6); ///< Timer 2 Low Byte Overflow Flag     
SI_SBIT (TMR2CN0_TF2H,    SFR_TMR2CN0, 7); ///< Timer 2 High Byte Overflow Flag    

// TMR4CN0 (Timer 4 Control 0)
#define SFR_TMR4CN0 0x98
SI_SBIT (TMR4CN0_T4XCLK0, SFR_TMR4CN0, 0); ///< Timer 4 External Clock Select Bit 0
SI_SBIT (TMR4CN0_T4XCLK1, SFR_TMR4CN0, 1); ///< Timer 4 External Clock Select Bit 1
SI_SBIT (TMR4CN0_TR4,     SFR_TMR4CN0, 2); ///< Timer 4 Run Control                
SI_SBIT (TMR4CN0_T4SPLIT, SFR_TMR4CN0, 3); ///< Timer 4 Split Mode Enable          
SI_SBIT (TMR4CN0_TF4CEN,  SFR_TMR4CN0, 4); ///< Timer 4 Capture Enable             
SI_SBIT (TMR4CN0_TF4LEN,  SFR_TMR4CN0, 5); ///< Timer 4 Low Byte Interrupt Enable  
SI_SBIT (TMR4CN0_TF4L,    SFR_TMR4CN0, 6); ///< Timer 4 Low Byte Overflow Flag     
SI_SBIT (TMR4CN0_TF4H,    SFR_TMR4CN0, 7); ///< Timer 4 High Byte Overflow Flag    

// UART1FCN1 (UART1 FIFO Control 1)
#define SFR_UART1FCN1 0xD8
SI_SBIT (UART1FCN1_RIE,    SFR_UART1FCN1, 0); ///< Receive Interrupt Enable 
SI_SBIT (UART1FCN1_RXTO0,  SFR_UART1FCN1, 1); ///< Receive Timeout Bit 0    
SI_SBIT (UART1FCN1_RXTO1,  SFR_UART1FCN1, 2); ///< Receive Timeout Bit 1    
SI_SBIT (UART1FCN1_RFRQ,   SFR_UART1FCN1, 3); ///< Receive FIFO Request     
SI_SBIT (UART1FCN1_TIE,    SFR_UART1FCN1, 4); ///< Transmit Interrupt Enable
SI_SBIT (UART1FCN1_TXHOLD, SFR_UART1FCN1, 5); ///< Transmit Hold            
SI_SBIT (UART1FCN1_TXNF,   SFR_UART1FCN1, 6); ///< TX FIFO Not Full         
SI_SBIT (UART1FCN1_TFRQ,   SFR_UART1FCN1, 7); ///< Transmit FIFO Request    

//------------------------------------------------------------------------------
// Interrupt Definitions
//------------------------------------------------------------------------------
#define INT0_IRQn    0  ///< External Interrupt 0      
#define TIMER0_IRQn  1  ///< Timer 0 Overflow          
#define INT1_IRQn    2  ///< External Interrupt 1      
#define TIMER1_IRQn  3  ///< Timer 1 Overflow          
#define UART0_IRQn   4  ///< UART0                     
#define TIMER2_IRQn  5  ///< Timer 2 Overflow / Capture
#define SPI0_IRQn    6  ///< SPI0                      
#define SMBUS0_IRQn  7  ///< SMBus0                    
#define PMATCH_IRQn  8  ///< Port Match                
#define ADC0WC_IRQn  9  ///< ADC0 Window Compare       
#define ADC0EOC_IRQn 10 ///< ADC0 End of Conversion    
#define PCA0_IRQn    11 ///< PCA0                      
#define CMP0_IRQn    12 ///< Comparator 0              
#define CMP1_IRQn    13 ///< Comparator 1              
#define TIMER3_IRQn  14 ///< Timer 3 Overflow / Capture
#define USB0_IRQn    15 ///< USB0 Events               
#define VBUS_IRQn    16 ///< VBUS / USB Charge Detect  
#define UART1_IRQn   17 ///< UART1                     
#define I2C0_IRQn    18 ///< I2C0 Slave                
#define TIMER4_IRQn  19 ///< Timer 4 Overflow / Capture

//------------------------------------------------------------------------------
// SFR Page Definitions
//------------------------------------------------------------------------------
#define CRC0_PAGE      0x00 ///< CRC0 Page       
#define LEGACY_PAGE    0x00 ///< Legacy SFR Page 
#define PCA0_PAGE      0x00 ///< PCA0 Page       
#define PG2_PAGE       0x10 ///< Page2           
#define TIMER2_PAGE    0x10 ///< Timer 2 Page    
#define TIMER3_PAGE    0x10 ///< Timer 3 Page    
#define TIMER4_PAGE    0x10 ///< Timer 4 Page    
#define I2CSLAVE0_PAGE 0x20 ///< I2C Slave 0 Page
#define PG3_PAGE       0x20 ///< Page3           
#define SMB0_PAGE      0x20 ///< SMBus 0 Page    
#define SPI0_PAGE      0x20 ///< SPI0 Page       
#define UART0_PAGE     0x20 ///< UART0 Page      
#define UART1_PAGE     0x20 ///< UART1 Page      
#define USB0_PAGE      0x20 ///< USB0 Page       

//-----------------------------------------------------------------------------
// SDCC PDATA External Memory Paging Support
//-----------------------------------------------------------------------------
*/
#if defined SDCC

//SI_SFR(_XPAGE, 0xAA); // Point to the EMI0CN register

#endif

#endif // SI_EFM8UB1_DEFS_H
//-eof--------------------------------------------------------------------------

