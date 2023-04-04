;--------------------------------------------------------
; File Created by C51
; Version 1.0.0 #1170 (Feb 16 2022) (MSVC)
; This file was generated Mon Apr 03 14:30:30 2023
;--------------------------------------------------------
$name remote
$optc51 --model-small
	R_DSEG    segment data
	R_CSEG    segment code
	R_BSEG    segment bit
	R_XSEG    segment xdata
	R_PSEG    segment xdata
	R_ISEG    segment idata
	R_OSEG    segment data overlay
	BIT_BANK  segment data overlay
	R_HOME    segment code
	R_GSINIT  segment code
	R_IXSEG   segment xdata
	R_CONST   segment code
	R_XINIT   segment code
	R_DINIT   segment code

;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	public _InitPinADC_PARM_2
	public _main
	public _getsn
	public _LCDprint
	public _LCD_4BIT
	public _WriteCommand
	public _WriteData
	public _LCD_byte
	public _LCD_pulse
	public _Timer2_ISR
	public _Timer0_ISR
	public _TIMER2_Init
	public _TIMER0_Init
	public _Volts_at_Pin
	public _ADC_at_Pin
	public _InitPinADC
	public _waitms
	public _Timer3us
	public _InitADC
	public __c51_external_startup
	public _LCDprint_PARM_3
	public _getsn_PARM_2
	public _LCDprint_PARM_2
	public _pwm_count
	public _pulse_width
	public _overflow_count
;--------------------------------------------------------
; Special Function Registers
;--------------------------------------------------------
_ACC            DATA 0xe0
_ADC0ASAH       DATA 0xb6
_ADC0ASAL       DATA 0xb5
_ADC0ASCF       DATA 0xa1
_ADC0ASCT       DATA 0xc7
_ADC0CF0        DATA 0xbc
_ADC0CF1        DATA 0xb9
_ADC0CF2        DATA 0xdf
_ADC0CN0        DATA 0xe8
_ADC0CN1        DATA 0xb2
_ADC0CN2        DATA 0xb3
_ADC0GTH        DATA 0xc4
_ADC0GTL        DATA 0xc3
_ADC0H          DATA 0xbe
_ADC0L          DATA 0xbd
_ADC0LTH        DATA 0xc6
_ADC0LTL        DATA 0xc5
_ADC0MX         DATA 0xbb
_B              DATA 0xf0
_CKCON0         DATA 0x8e
_CKCON1         DATA 0xa6
_CLEN0          DATA 0xc6
_CLIE0          DATA 0xc7
_CLIF0          DATA 0xe8
_CLKSEL         DATA 0xa9
_CLOUT0         DATA 0xd1
_CLU0CF         DATA 0xb1
_CLU0FN         DATA 0xaf
_CLU0MX         DATA 0x84
_CLU1CF         DATA 0xb3
_CLU1FN         DATA 0xb2
_CLU1MX         DATA 0x85
_CLU2CF         DATA 0xb6
_CLU2FN         DATA 0xb5
_CLU2MX         DATA 0x91
_CLU3CF         DATA 0xbf
_CLU3FN         DATA 0xbe
_CLU3MX         DATA 0xae
_CMP0CN0        DATA 0x9b
_CMP0CN1        DATA 0x99
_CMP0MD         DATA 0x9d
_CMP0MX         DATA 0x9f
_CMP1CN0        DATA 0xbf
_CMP1CN1        DATA 0xac
_CMP1MD         DATA 0xab
_CMP1MX         DATA 0xaa
_CRC0CN0        DATA 0xce
_CRC0CN1        DATA 0x86
_CRC0CNT        DATA 0xd3
_CRC0DAT        DATA 0xcb
_CRC0FLIP       DATA 0xcf
_CRC0IN         DATA 0xca
_CRC0ST         DATA 0xd2
_DAC0CF0        DATA 0x91
_DAC0CF1        DATA 0x92
_DAC0H          DATA 0x85
_DAC0L          DATA 0x84
_DAC1CF0        DATA 0x93
_DAC1CF1        DATA 0x94
_DAC1H          DATA 0x8a
_DAC1L          DATA 0x89
_DAC2CF0        DATA 0x95
_DAC2CF1        DATA 0x96
_DAC2H          DATA 0x8c
_DAC2L          DATA 0x8b
_DAC3CF0        DATA 0x9a
_DAC3CF1        DATA 0x9c
_DAC3H          DATA 0x8e
_DAC3L          DATA 0x8d
_DACGCF0        DATA 0x88
_DACGCF1        DATA 0x98
_DACGCF2        DATA 0xa2
_DERIVID        DATA 0xad
_DEVICEID       DATA 0xb5
_DPH            DATA 0x83
_DPL            DATA 0x82
_EIE1           DATA 0xe6
_EIE2           DATA 0xf3
_EIP1           DATA 0xbb
_EIP1H          DATA 0xee
_EIP2           DATA 0xed
_EIP2H          DATA 0xf6
_EMI0CN         DATA 0xe7
_FLKEY          DATA 0xb7
_HFO0CAL        DATA 0xc7
_HFO1CAL        DATA 0xd6
_HFOCN          DATA 0xef
_I2C0ADM        DATA 0xff
_I2C0CN0        DATA 0xba
_I2C0DIN        DATA 0xbc
_I2C0DOUT       DATA 0xbb
_I2C0FCN0       DATA 0xad
_I2C0FCN1       DATA 0xab
_I2C0FCT        DATA 0xf5
_I2C0SLAD       DATA 0xbd
_I2C0STAT       DATA 0xb9
_IE             DATA 0xa8
_IP             DATA 0xb8
_IPH            DATA 0xf2
_IT01CF         DATA 0xe4
_LFO0CN         DATA 0xb1
_P0             DATA 0x80
_P0MASK         DATA 0xfe
_P0MAT          DATA 0xfd
_P0MDIN         DATA 0xf1
_P0MDOUT        DATA 0xa4
_P0SKIP         DATA 0xd4
_P1             DATA 0x90
_P1MASK         DATA 0xee
_P1MAT          DATA 0xed
_P1MDIN         DATA 0xf2
_P1MDOUT        DATA 0xa5
_P1SKIP         DATA 0xd5
_P2             DATA 0xa0
_P2MASK         DATA 0xfc
_P2MAT          DATA 0xfb
_P2MDIN         DATA 0xf3
_P2MDOUT        DATA 0xa6
_P2SKIP         DATA 0xcc
_P3             DATA 0xb0
_P3MDIN         DATA 0xf4
_P3MDOUT        DATA 0x9c
_PCA0CENT       DATA 0x9e
_PCA0CLR        DATA 0x9c
_PCA0CN0        DATA 0xd8
_PCA0CPH0       DATA 0xfc
_PCA0CPH1       DATA 0xea
_PCA0CPH2       DATA 0xec
_PCA0CPH3       DATA 0xf5
_PCA0CPH4       DATA 0x85
_PCA0CPH5       DATA 0xde
_PCA0CPL0       DATA 0xfb
_PCA0CPL1       DATA 0xe9
_PCA0CPL2       DATA 0xeb
_PCA0CPL3       DATA 0xf4
_PCA0CPL4       DATA 0x84
_PCA0CPL5       DATA 0xdd
_PCA0CPM0       DATA 0xda
_PCA0CPM1       DATA 0xdb
_PCA0CPM2       DATA 0xdc
_PCA0CPM3       DATA 0xae
_PCA0CPM4       DATA 0xaf
_PCA0CPM5       DATA 0xcc
_PCA0H          DATA 0xfa
_PCA0L          DATA 0xf9
_PCA0MD         DATA 0xd9
_PCA0POL        DATA 0x96
_PCA0PWM        DATA 0xf7
_PCON0          DATA 0x87
_PCON1          DATA 0xcd
_PFE0CN         DATA 0xc1
_PRTDRV         DATA 0xf6
_PSCTL          DATA 0x8f
_PSTAT0         DATA 0xaa
_PSW            DATA 0xd0
_REF0CN         DATA 0xd1
_REG0CN         DATA 0xc9
_REVID          DATA 0xb6
_RSTSRC         DATA 0xef
_SBCON1         DATA 0x94
_SBRLH1         DATA 0x96
_SBRLL1         DATA 0x95
_SBUF           DATA 0x99
_SBUF0          DATA 0x99
_SBUF1          DATA 0x92
_SCON           DATA 0x98
_SCON0          DATA 0x98
_SCON1          DATA 0xc8
_SFRPAGE        DATA 0xa7
_SFRPGCN        DATA 0xbc
_SFRSTACK       DATA 0xd7
_SMB0ADM        DATA 0xd6
_SMB0ADR        DATA 0xd7
_SMB0CF         DATA 0xc1
_SMB0CN0        DATA 0xc0
_SMB0DAT        DATA 0xc2
_SMB0FCN0       DATA 0xc3
_SMB0FCN1       DATA 0xc4
_SMB0FCT        DATA 0xef
_SMB0RXLN       DATA 0xc5
_SMB0TC         DATA 0xac
_SMOD1          DATA 0x93
_SP             DATA 0x81
_SPI0CFG        DATA 0xa1
_SPI0CKR        DATA 0xa2
_SPI0CN0        DATA 0xf8
_SPI0DAT        DATA 0xa3
_SPI0FCN0       DATA 0x9a
_SPI0FCN1       DATA 0x9b
_SPI0FCT        DATA 0xf7
_SPI0PCF        DATA 0xdf
_TCON           DATA 0x88
_TH0            DATA 0x8c
_TH1            DATA 0x8d
_TL0            DATA 0x8a
_TL1            DATA 0x8b
_TMOD           DATA 0x89
_TMR2CN0        DATA 0xc8
_TMR2CN1        DATA 0xfd
_TMR2H          DATA 0xcf
_TMR2L          DATA 0xce
_TMR2RLH        DATA 0xcb
_TMR2RLL        DATA 0xca
_TMR3CN0        DATA 0x91
_TMR3CN1        DATA 0xfe
_TMR3H          DATA 0x95
_TMR3L          DATA 0x94
_TMR3RLH        DATA 0x93
_TMR3RLL        DATA 0x92
_TMR4CN0        DATA 0x98
_TMR4CN1        DATA 0xff
_TMR4H          DATA 0xa5
_TMR4L          DATA 0xa4
_TMR4RLH        DATA 0xa3
_TMR4RLL        DATA 0xa2
_TMR5CN0        DATA 0xc0
_TMR5CN1        DATA 0xf1
_TMR5H          DATA 0xd5
_TMR5L          DATA 0xd4
_TMR5RLH        DATA 0xd3
_TMR5RLL        DATA 0xd2
_UART0PCF       DATA 0xd9
_UART1FCN0      DATA 0x9d
_UART1FCN1      DATA 0xd8
_UART1FCT       DATA 0xfa
_UART1LIN       DATA 0x9e
_UART1PCF       DATA 0xda
_VDM0CN         DATA 0xff
_WDTCN          DATA 0x97
_XBR0           DATA 0xe1
_XBR1           DATA 0xe2
_XBR2           DATA 0xe3
_XOSC0CN        DATA 0x86
_DPTR           DATA 0x8382
_TMR2RL         DATA 0xcbca
_TMR3RL         DATA 0x9392
_TMR4RL         DATA 0xa3a2
_TMR5RL         DATA 0xd3d2
_TMR0           DATA 0x8c8a
_TMR1           DATA 0x8d8b
_TMR2           DATA 0xcfce
_TMR3           DATA 0x9594
_TMR4           DATA 0xa5a4
_TMR5           DATA 0xd5d4
_SBRL1          DATA 0x9695
_PCA0           DATA 0xfaf9
_PCA0CP0        DATA 0xfcfb
_PCA0CP1        DATA 0xeae9
_PCA0CP2        DATA 0xeceb
_PCA0CP3        DATA 0xf5f4
_PCA0CP4        DATA 0x8584
_PCA0CP5        DATA 0xdedd
_ADC0ASA        DATA 0xb6b5
_ADC0GT         DATA 0xc4c3
_ADC0           DATA 0xbebd
_ADC0LT         DATA 0xc6c5
_DAC0           DATA 0x8584
_DAC1           DATA 0x8a89
_DAC2           DATA 0x8c8b
_DAC3           DATA 0x8e8d
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
_ACC_0          BIT 0xe0
_ACC_1          BIT 0xe1
_ACC_2          BIT 0xe2
_ACC_3          BIT 0xe3
_ACC_4          BIT 0xe4
_ACC_5          BIT 0xe5
_ACC_6          BIT 0xe6
_ACC_7          BIT 0xe7
_TEMPE          BIT 0xe8
_ADGN0          BIT 0xe9
_ADGN1          BIT 0xea
_ADWINT         BIT 0xeb
_ADBUSY         BIT 0xec
_ADINT          BIT 0xed
_IPOEN          BIT 0xee
_ADEN           BIT 0xef
_B_0            BIT 0xf0
_B_1            BIT 0xf1
_B_2            BIT 0xf2
_B_3            BIT 0xf3
_B_4            BIT 0xf4
_B_5            BIT 0xf5
_B_6            BIT 0xf6
_B_7            BIT 0xf7
_C0FIF          BIT 0xe8
_C0RIF          BIT 0xe9
_C1FIF          BIT 0xea
_C1RIF          BIT 0xeb
_C2FIF          BIT 0xec
_C2RIF          BIT 0xed
_C3FIF          BIT 0xee
_C3RIF          BIT 0xef
_D1SRC0         BIT 0x88
_D1SRC1         BIT 0x89
_D1AMEN         BIT 0x8a
_D01REFSL       BIT 0x8b
_D3SRC0         BIT 0x8c
_D3SRC1         BIT 0x8d
_D3AMEN         BIT 0x8e
_D23REFSL       BIT 0x8f
_D0UDIS         BIT 0x98
_D1UDIS         BIT 0x99
_D2UDIS         BIT 0x9a
_D3UDIS         BIT 0x9b
_EX0            BIT 0xa8
_ET0            BIT 0xa9
_EX1            BIT 0xaa
_ET1            BIT 0xab
_ES0            BIT 0xac
_ET2            BIT 0xad
_ESPI0          BIT 0xae
_EA             BIT 0xaf
_PX0            BIT 0xb8
_PT0            BIT 0xb9
_PX1            BIT 0xba
_PT1            BIT 0xbb
_PS0            BIT 0xbc
_PT2            BIT 0xbd
_PSPI0          BIT 0xbe
_P0_0           BIT 0x80
_P0_1           BIT 0x81
_P0_2           BIT 0x82
_P0_3           BIT 0x83
_P0_4           BIT 0x84
_P0_5           BIT 0x85
_P0_6           BIT 0x86
_P0_7           BIT 0x87
_P1_0           BIT 0x90
_P1_1           BIT 0x91
_P1_2           BIT 0x92
_P1_3           BIT 0x93
_P1_4           BIT 0x94
_P1_5           BIT 0x95
_P1_6           BIT 0x96
_P1_7           BIT 0x97
_P2_0           BIT 0xa0
_P2_1           BIT 0xa1
_P2_2           BIT 0xa2
_P2_3           BIT 0xa3
_P2_4           BIT 0xa4
_P2_5           BIT 0xa5
_P2_6           BIT 0xa6
_P3_0           BIT 0xb0
_P3_1           BIT 0xb1
_P3_2           BIT 0xb2
_P3_3           BIT 0xb3
_P3_4           BIT 0xb4
_P3_7           BIT 0xb7
_CCF0           BIT 0xd8
_CCF1           BIT 0xd9
_CCF2           BIT 0xda
_CCF3           BIT 0xdb
_CCF4           BIT 0xdc
_CCF5           BIT 0xdd
_CR             BIT 0xde
_CF             BIT 0xdf
_PARITY         BIT 0xd0
_F1             BIT 0xd1
_OV             BIT 0xd2
_RS0            BIT 0xd3
_RS1            BIT 0xd4
_F0             BIT 0xd5
_AC             BIT 0xd6
_CY             BIT 0xd7
_RI             BIT 0x98
_TI             BIT 0x99
_RB8            BIT 0x9a
_TB8            BIT 0x9b
_REN            BIT 0x9c
_CE             BIT 0x9d
_SMODE          BIT 0x9e
_RI1            BIT 0xc8
_TI1            BIT 0xc9
_RBX1           BIT 0xca
_TBX1           BIT 0xcb
_REN1           BIT 0xcc
_PERR1          BIT 0xcd
_OVR1           BIT 0xce
_SI             BIT 0xc0
_ACK            BIT 0xc1
_ARBLOST        BIT 0xc2
_ACKRQ          BIT 0xc3
_STO            BIT 0xc4
_STA            BIT 0xc5
_TXMODE         BIT 0xc6
_MASTER         BIT 0xc7
_SPIEN          BIT 0xf8
_TXNF           BIT 0xf9
_NSSMD0         BIT 0xfa
_NSSMD1         BIT 0xfb
_RXOVRN         BIT 0xfc
_MODF           BIT 0xfd
_WCOL           BIT 0xfe
_SPIF           BIT 0xff
_IT0            BIT 0x88
_IE0            BIT 0x89
_IT1            BIT 0x8a
_IE1            BIT 0x8b
_TR0            BIT 0x8c
_TF0            BIT 0x8d
_TR1            BIT 0x8e
_TF1            BIT 0x8f
_T2XCLK0        BIT 0xc8
_T2XCLK1        BIT 0xc9
_TR2            BIT 0xca
_T2SPLIT        BIT 0xcb
_TF2CEN         BIT 0xcc
_TF2LEN         BIT 0xcd
_TF2L           BIT 0xce
_TF2H           BIT 0xcf
_T4XCLK0        BIT 0x98
_T4XCLK1        BIT 0x99
_TR4            BIT 0x9a
_T4SPLIT        BIT 0x9b
_TF4CEN         BIT 0x9c
_TF4LEN         BIT 0x9d
_TF4L           BIT 0x9e
_TF4H           BIT 0x9f
_T5XCLK0        BIT 0xc0
_T5XCLK1        BIT 0xc1
_TR5            BIT 0xc2
_T5SPLIT        BIT 0xc3
_TF5CEN         BIT 0xc4
_TF5LEN         BIT 0xc5
_TF5L           BIT 0xc6
_TF5H           BIT 0xc7
_RIE            BIT 0xd8
_RXTO0          BIT 0xd9
_RXTO1          BIT 0xda
_RFRQ           BIT 0xdb
_TIE            BIT 0xdc
_TXHOLD         BIT 0xdd
_TXNF1          BIT 0xde
_TFRQ           BIT 0xdf
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	rbank0 segment data overlay
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	rseg R_DSEG
_overflow_count:
	ds 1
_pulse_width:
	ds 2
_pwm_count:
	ds 2
_LCDprint_PARM_2:
	ds 1
_getsn_PARM_2:
	ds 2
_getsn_buff_1_81:
	ds 3
_getsn_sloc0_1_0:
	ds 2
_main_NS_Volt_1_87:
	ds 4
_main_EW_Volt_1_87:
	ds 4
_main_buffer1_1_87:
	ds 17
_main_buffer2_1_87:
	ds 17
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	rseg	R_OSEG
	rseg	R_OSEG
_InitPinADC_PARM_2:
	ds 1
	rseg	R_OSEG
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	rseg R_ISEG
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	DSEG
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	rseg R_BSEG
_LCDprint_PARM_3:
	DBIT	1
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	rseg R_PSEG
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	rseg R_XSEG
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	XSEG
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	rseg R_IXSEG
	rseg R_HOME
	rseg R_GSINIT
	rseg R_CSEG
;--------------------------------------------------------
; Reset entry point and interrupt vectors
;--------------------------------------------------------
	CSEG at 0x0000
	ljmp	_crt0
	CSEG at 0x000b
	ljmp	_Timer0_ISR
	CSEG at 0x002b
	ljmp	_Timer2_ISR
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	rseg R_HOME
	rseg R_GSINIT
	rseg R_GSINIT
;--------------------------------------------------------
; data variables initialization
;--------------------------------------------------------
	rseg R_DINIT
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:48: int pwm_count = 0;
	clr	a
	mov	_pwm_count,a
	mov	(_pwm_count + 1),a
	; The linker places a 'ret' at the end of segment R_DINIT.
;--------------------------------------------------------
; code
;--------------------------------------------------------
	rseg R_CSEG
;------------------------------------------------------------
;Allocation info for local variables in function '_c51_external_startup'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:50: char _c51_external_startup(void)
;	-----------------------------------------
;	 function _c51_external_startup
;	-----------------------------------------
__c51_external_startup:
	using	0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:53: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:54: WDTCN = 0xDE; // First key
	mov	_WDTCN,#0xDE
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:55: WDTCN = 0xAD; // Second key
	mov	_WDTCN,#0xAD
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:57: VDM0CN |= 0x80;
	orl	_VDM0CN,#0x80
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:58: RSTSRC = 0x02;
	mov	_RSTSRC,#0x02
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:65: SFRPAGE = 0x10;
	mov	_SFRPAGE,#0x10
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:66: PFE0CN = 0x20; // SYSCLK < 75 MHz.
	mov	_PFE0CN,#0x20
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:67: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:92: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:93: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:94: while ((CLKSEL & 0x80) == 0)
L002001?:
	mov	a,_CLKSEL
	jnb	acc.7,L002001?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:96: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:97: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:98: while ((CLKSEL & 0x80) == 0)
L002004?:
	mov	a,_CLKSEL
	jnb	acc.7,L002004?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:104: P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	orl	_P0MDOUT,#0x10
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:105: XBR0 = 0x01;	 // Enable UART0 on P0.4(TX) and P0.5(RX)
	mov	_XBR0,#0x01
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:106: XBR1 = 0X00;
	mov	_XBR1,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:107: XBR2 = 0x40; // Enable crossbar and weak pull-ups
	mov	_XBR2,#0x40
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:113: SCON0 = 0x10;
	mov	_SCON0,#0x10
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:114: CKCON0 |= 0b_0000_0000; // Timer 1 uses the system clock divided by 12.
	mov	_CKCON0,_CKCON0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:115: TH1 = 0x100 - ((SYSCLK / BAUDRATE) / (2L * 12L));
	mov	_TH1,#0xE6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:116: TL1 = TH1;	   // Init Timer1
	mov	_TL1,_TH1
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:117: TMOD &= ~0xf0; // TMOD: timer 1 in 8-bit auto-reload
	anl	_TMOD,#0x0F
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:118: TMOD |= 0x20;
	orl	_TMOD,#0x20
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:119: TR1 = 1; // START Timer1
	setb	_TR1
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:120: TI = 1;	 // Indicate TX0 ready
	setb	_TI
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:122: return 0;
	mov	dpl,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'InitADC'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:125: void InitADC (void)
;	-----------------------------------------
;	 function InitADC
;	-----------------------------------------
_InitADC:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:127: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:128: ADEN = 0; // Disable ADC
	clr	_ADEN
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:131: (0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	mov	_ADC0CN1,#0x80
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:133: (0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	mov	_ADC0CF0,#0x20
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:135: (0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	mov	_ADC0CF1,#0x1E
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:142: (0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.
	mov	_ADC0CN0,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:145: (0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	mov	_ADC0CF2,#0x3F
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:147: (0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3
	mov	_ADC0CN2,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:148: ADEN = 1; // Enable ADC
	setb	_ADEN
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer3us'
;------------------------------------------------------------
;us                        Allocated to registers r2 
;i                         Allocated to registers r3 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:152: void Timer3us(unsigned char us)
;	-----------------------------------------
;	 function Timer3us
;	-----------------------------------------
_Timer3us:
	mov	r2,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:157: CKCON0 |= 0b_0100_0000;
	orl	_CKCON0,#0x40
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:159: TMR3RL = (-(SYSCLK) / 1000000L); // Set Timer3 to overflow in 1us.
	mov	_TMR3RL,#0xB8
	mov	(_TMR3RL >> 8),#0xFF
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:160: TMR3 = TMR3RL;					 // Initialize Timer3 for first overflow
	mov	_TMR3,_TMR3RL
	mov	(_TMR3 >> 8),(_TMR3RL >> 8)
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:162: TMR3CN0 = 0x04;			 // Sart Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x04
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:163: for (i = 0; i < us; i++) // Count <us> overflows
	mov	r3,#0x00
L004004?:
	clr	c
	mov	a,r3
	subb	a,r2
	jnc	L004007?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:165: while (!(TMR3CN0 & 0x80))
L004001?:
	mov	a,_TMR3CN0
	jnb	acc.7,L004001?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:167: TMR3CN0 &= ~(0x80); // Clear overflow indicator
	anl	_TMR3CN0,#0x7F
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:163: for (i = 0; i < us; i++) // Count <us> overflows
	inc	r3
	sjmp	L004004?
L004007?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:169: TMR3CN0 = 0; // Stop Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'waitms'
;------------------------------------------------------------
;ms                        Allocated to registers r2 r3 
;j                         Allocated to registers r2 r3 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:172: void waitms(unsigned int ms)
;	-----------------------------------------
;	 function waitms
;	-----------------------------------------
_waitms:
	mov	r2,dpl
	mov	r3,dph
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:175: for (j = ms; j != 0; j--)
L005001?:
	cjne	r2,#0x00,L005010?
	cjne	r3,#0x00,L005010?
	ret
L005010?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:177: Timer3us(249);
	mov	dpl,#0xF9
	push	ar2
	push	ar3
	lcall	_Timer3us
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:178: Timer3us(249);
	mov	dpl,#0xF9
	lcall	_Timer3us
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:179: Timer3us(249);
	mov	dpl,#0xF9
	lcall	_Timer3us
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:180: Timer3us(250);
	mov	dpl,#0xFA
	lcall	_Timer3us
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:175: for (j = ms; j != 0; j--)
	dec	r2
	cjne	r2,#0xff,L005011?
	dec	r3
L005011?:
	sjmp	L005001?
;------------------------------------------------------------
;Allocation info for local variables in function 'InitPinADC'
;------------------------------------------------------------
;pinno                     Allocated with name '_InitPinADC_PARM_2'
;portno                    Allocated to registers r2 
;mask                      Allocated to registers r3 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:185: void InitPinADC (unsigned char portno, unsigned char pinno)
;	-----------------------------------------
;	 function InitPinADC
;	-----------------------------------------
_InitPinADC:
	mov	r2,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:188: mask = 1 << pinno;
	mov	b,_InitPinADC_PARM_2
	inc	b
	mov	a,#0x01
	sjmp	L006013?
L006011?:
	add	a,acc
L006013?:
	djnz	b,L006011?
	mov	r3,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:189: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:190: switch (portno)
	cjne	r2,#0x00,L006014?
	sjmp	L006001?
L006014?:
	cjne	r2,#0x01,L006015?
	sjmp	L006002?
L006015?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:192: case 0:
	cjne	r2,#0x02,L006005?
	sjmp	L006003?
L006001?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:193: P0MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P0MDIN,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:194: P0SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P0SKIP,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:195: break;
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:196: case 1:
	sjmp	L006005?
L006002?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:197: P1MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P1MDIN,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:198: P1SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P1SKIP,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:199: break;
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:200: case 2:
	sjmp	L006005?
L006003?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:201: P2MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P2MDIN,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:202: P2SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P2SKIP,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:206: }
L006005?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:207: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'ADC_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:210: unsigned int ADC_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function ADC_at_Pin
;	-----------------------------------------
_ADC_at_Pin:
	mov	_ADC0MX,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:213: ADINT = 0;
	clr	_ADINT
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:214: ADBUSY = 1; // Convert voltage at the pin
	setb	_ADBUSY
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:215: while (!ADINT); // Wait for conversion to complete
L007001?:
	jnb	_ADINT,L007001?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:216: return (ADC0);
	mov	dpl,_ADC0
	mov	dph,(_ADC0 >> 8)
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Volts_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers r2 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:219: float Volts_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function Volts_at_Pin
;	-----------------------------------------
_Volts_at_Pin:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:221: return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
	lcall	_ADC_at_Pin
	lcall	___uint2fs
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	mov	dptr,#0x6C8B
	mov	b,#0x53
	mov	a,#0x40
	lcall	___fsmul
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	clr	a
	push	acc
	mov	a,#0xFC
	push	acc
	mov	a,#0x7F
	push	acc
	mov	a,#0x46
	push	acc
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	___fsdiv
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'TIMER0_Init'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:224: void TIMER0_Init(void)
;	-----------------------------------------
;	 function TIMER0_Init
;	-----------------------------------------
_TIMER0_Init:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:226: CKCON0 |= 0b_0000_0010; // Set Timer0 Clock (T0X2) to 48 clk periods
	orl	_CKCON0,#0x02
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:227: TMOD &= 0b_1111_0000;	// Set the bits of Timer/Counter 0 to zero
	anl	_TMOD,#0xF0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:228: TMOD |= 0b_0000_0010;	// Timer/Counter 0 used as a 8-bit auto-reload timer
	orl	_TMOD,#0x02
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:230: TL0 = RELOAD_TIMER0; // initial value
	mov	_TL0,#0xD2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:231: TH0 = RELOAD_TIMER0; // reload value
	mov	_TH0,#0xD2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:233: ET0 = 1; // enable timer 0 interrupt
	setb	_ET0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:234: EA = 1;	 // enable global interrupts
	setb	_EA
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:236: TR0 = 0; // Stop Timer/Counter 0
	clr	_TR0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'TIMER2_Init'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:239: void TIMER2_Init(void)
;	-----------------------------------------
;	 function TIMER2_Init
;	-----------------------------------------
_TIMER2_Init:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:242: TMR2CN0 = 0x00;			// Stop Timer2; Clear TF2;
	mov	_TMR2CN0,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:243: CKCON0 |= 0b_0001_0000; // Timer 2 uses the system clock
	orl	_CKCON0,#0x10
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:246: TMR2RL = 29536;
	mov	_TMR2RL,#0x60
	mov	(_TMR2RL >> 8),#0x73
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:247: TMR2 = 0xffff; // Set to reload immediately
	mov	_TMR2,#0xFF
	mov	(_TMR2 >> 8),#0xFF
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:248: EA = 1;		   // enable global interrupts
	setb	_EA
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:249: ET2 = 1;	   // Enable Timer2 interrupts
	setb	_ET2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:250: TR2 = 0;	   // Start Timer2 (TMR2CN is bit addressable)
	clr	_TR2
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer0_ISR'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:253: void Timer0_ISR(void) interrupt INTERRUPT_TIMER0
;	-----------------------------------------
;	 function Timer0_ISR
;	-----------------------------------------
_Timer0_ISR:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:256: SFRPAGE = 0x0;
	mov	_SFRPAGE,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:260: TIMER_OUT_0 = !TIMER_OUT_0;
	cpl	_P2_0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:261: TIMER_OUT_0_INVERTED = !TIMER_OUT_0_INVERTED;
	cpl	_P2_1
	reti
;	eliminated unneeded push/pop psw
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;	eliminated unneeded push/pop acc
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer2_ISR'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:264: void Timer2_ISR(void) interrupt INTERRUPT_TIMER2
;	-----------------------------------------
;	 function Timer2_ISR
;	-----------------------------------------
_Timer2_ISR:
	push	acc
	push	psw
	mov	psw,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:266: SFRPAGE = 0x0;
	mov	_SFRPAGE,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:267: TF2H = 0; // Clear Timer2 interrupt flag
	clr	_TF2H
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:269: pwm_count++;
	inc	_pwm_count
	clr	a
	cjne	a,_pwm_count,L012009?
	inc	(_pwm_count + 1)
L012009?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:270: if (pwm_count == 200)
	mov	a,#0xC8
	cjne	a,_pwm_count,L012010?
	clr	a
	cjne	a,(_pwm_count + 1),L012010?
	sjmp	L012011?
L012010?:
	sjmp	L012002?
L012011?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:272: pwm_count = 0;
	clr	a
	mov	_pwm_count,a
	mov	(_pwm_count + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:273: TIMER_OUT_2 = 1;
	setb	_P1_6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:274: TIMER_OUT_0 = 0;
	clr	_P2_0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:275: TIMER_OUT_0_INVERTED = 1;
	setb	_P2_1
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:276: TR0 = 1;
	setb	_TR0
L012002?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:280: if (pwm_count == pulse_width)
	mov	a,_pulse_width
	cjne	a,_pwm_count,L012005?
	mov	a,(_pulse_width + 1)
	cjne	a,(_pwm_count + 1),L012005?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:282: TIMER_OUT_2 = 0;
	clr	_P1_6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:283: TR0 = 0;
	clr	_TR0
L012005?:
	pop	psw
	pop	acc
	reti
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_pulse'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:288: void LCD_pulse(void)
;	-----------------------------------------
;	 function LCD_pulse
;	-----------------------------------------
_LCD_pulse:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:290: LCD_E = 1;
	setb	_P2_6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:291: Timer3us(40);
	mov	dpl,#0x28
	lcall	_Timer3us
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:292: LCD_E = 0;
	clr	_P2_6
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_byte'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:295: void LCD_byte(unsigned char x)
;	-----------------------------------------
;	 function LCD_byte
;	-----------------------------------------
_LCD_byte:
	mov	r2,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:298: ACC = x; // Send high nible
	mov	_ACC,r2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:299: LCD_D7 = ACC_7;
	mov	c,_ACC_7
	mov	_P2_2,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:300: LCD_D6 = ACC_6;
	mov	c,_ACC_6
	mov	_P2_3,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:301: LCD_D5 = ACC_5;
	mov	c,_ACC_5
	mov	_P2_4,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:302: LCD_D4 = ACC_4;
	mov	c,_ACC_4
	mov	_P2_5,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:303: LCD_pulse();
	push	ar2
	lcall	_LCD_pulse
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:304: Timer3us(40);
	mov	dpl,#0x28
	lcall	_Timer3us
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:305: ACC = x; // Send low nible
	mov	_ACC,r2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:306: LCD_D7 = ACC_3;
	mov	c,_ACC_3
	mov	_P2_2,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:307: LCD_D6 = ACC_2;
	mov	c,_ACC_2
	mov	_P2_3,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:308: LCD_D5 = ACC_1;
	mov	c,_ACC_1
	mov	_P2_4,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:309: LCD_D4 = ACC_0;
	mov	c,_ACC_0
	mov	_P2_5,c
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:310: LCD_pulse();
	ljmp	_LCD_pulse
;------------------------------------------------------------
;Allocation info for local variables in function 'WriteData'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:313: void WriteData(unsigned char x)
;	-----------------------------------------
;	 function WriteData
;	-----------------------------------------
_WriteData:
	mov	r2,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:315: LCD_RS = 1;
	setb	_P3_0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:316: LCD_byte(x);
	mov	dpl,r2
	lcall	_LCD_byte
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:317: waitms(2);
	mov	dptr,#0x0002
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'WriteCommand'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:320: void WriteCommand(unsigned char x)
;	-----------------------------------------
;	 function WriteCommand
;	-----------------------------------------
_WriteCommand:
	mov	r2,dpl
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:322: LCD_RS = 0;
	clr	_P3_0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:323: LCD_byte(x);
	mov	dpl,r2
	lcall	_LCD_byte
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:324: waitms(5);
	mov	dptr,#0x0005
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_4BIT'
;------------------------------------------------------------
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:327: void LCD_4BIT(void)
;	-----------------------------------------
;	 function LCD_4BIT
;	-----------------------------------------
_LCD_4BIT:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:329: LCD_E = 0; // Resting state of LCD's enable is zero
	clr	_P2_6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:331: waitms(20);
	mov	dptr,#0x0014
	lcall	_waitms
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:333: WriteCommand(0x33);
	mov	dpl,#0x33
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:334: WriteCommand(0x33);
	mov	dpl,#0x33
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:335: WriteCommand(0x32); // Change to 4-bit mode
	mov	dpl,#0x32
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:338: WriteCommand(0x28);
	mov	dpl,#0x28
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:339: WriteCommand(0x0c);
	mov	dpl,#0x0C
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:340: WriteCommand(0x01); // Clear screen command (takes some time)
	mov	dpl,#0x01
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:341: waitms(20);			// Wait for clear screen command to finsih.
	mov	dptr,#0x0014
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'LCDprint'
;------------------------------------------------------------
;line                      Allocated with name '_LCDprint_PARM_2'
;string                    Allocated to registers r2 r3 r4 
;j                         Allocated to registers r5 r6 
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:344: void LCDprint(char *string, unsigned char line, bit clear)
;	-----------------------------------------
;	 function LCDprint
;	-----------------------------------------
_LCDprint:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:348: WriteCommand(line == 2 ? 0xc0 : 0x80);
	mov	a,#0x02
	cjne	a,_LCDprint_PARM_2,L018013?
	mov	r5,#0xC0
	sjmp	L018014?
L018013?:
	mov	r5,#0x80
L018014?:
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_WriteCommand
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:349: waitms(5);
	mov	dptr,#0x0005
	lcall	_waitms
	pop	ar4
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:350: for (j = 0; string[j] != 0; j++)
	mov	r5,#0x00
	mov	r6,#0x00
L018003?:
	mov	a,r5
	add	a,r2
	mov	r7,a
	mov	a,r6
	addc	a,r3
	mov	r0,a
	mov	ar1,r4
	mov	dpl,r7
	mov	dph,r0
	mov	b,r1
	lcall	__gptrget
	mov	r7,a
	jz	L018006?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:351: WriteData(string[j]); // Write the message
	mov	dpl,r7
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	lcall	_WriteData
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:350: for (j = 0; string[j] != 0; j++)
	inc	r5
	cjne	r5,#0x00,L018003?
	inc	r6
	sjmp	L018003?
L018006?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:352: if (clear)
	jnb	_LCDprint_PARM_3,L018011?
	mov	ar2,r5
	mov	ar3,r6
L018007?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:353: for (; j < CHARS_PER_LINE; j++)
	clr	c
	mov	a,r2
	subb	a,#0x10
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L018011?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:354: WriteData(' '); // Clear the rest of the line
	mov	dpl,#0x20
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:353: for (; j < CHARS_PER_LINE; j++)
	inc	r2
	cjne	r2,#0x00,L018007?
	inc	r3
	sjmp	L018007?
L018011?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getsn'
;------------------------------------------------------------
;len                       Allocated with name '_getsn_PARM_2'
;buff                      Allocated with name '_getsn_buff_1_81'
;j                         Allocated with name '_getsn_sloc0_1_0'
;c                         Allocated to registers r3 
;sloc0                     Allocated with name '_getsn_sloc0_1_0'
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:357: int getsn(char *buff, int len)
;	-----------------------------------------
;	 function getsn
;	-----------------------------------------
_getsn:
	mov	_getsn_buff_1_81,dpl
	mov	(_getsn_buff_1_81 + 1),dph
	mov	(_getsn_buff_1_81 + 2),b
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:362: for (j = 0; j < (len - 1); j++)
	clr	a
	mov	_getsn_sloc0_1_0,a
	mov	(_getsn_sloc0_1_0 + 1),a
	mov	a,_getsn_PARM_2
	add	a,#0xff
	mov	r7,a
	mov	a,(_getsn_PARM_2 + 1)
	addc	a,#0xff
	mov	r0,a
	mov	r1,#0x00
	mov	r2,#0x00
L019005?:
	clr	c
	mov	a,r1
	subb	a,r7
	mov	a,r2
	xrl	a,#0x80
	mov	b,r0
	xrl	b,#0x80
	subb	a,b
	jnc	L019008?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:364: c = getchar();
	push	ar2
	push	ar7
	push	ar0
	push	ar1
	lcall	_getchar
	mov	r3,dpl
	pop	ar1
	pop	ar0
	pop	ar7
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:365: if ((c == '\n') || (c == '\r'))
	cjne	r3,#0x0A,L019015?
	sjmp	L019001?
L019015?:
	cjne	r3,#0x0D,L019002?
L019001?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:367: buff[j] = 0;
	mov	a,_getsn_sloc0_1_0
	add	a,_getsn_buff_1_81
	mov	r4,a
	mov	a,(_getsn_sloc0_1_0 + 1)
	addc	a,(_getsn_buff_1_81 + 1)
	mov	r5,a
	mov	r6,(_getsn_buff_1_81 + 2)
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	clr	a
	lcall	__gptrput
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:368: return j;
	mov	dpl,_getsn_sloc0_1_0
	mov	dph,(_getsn_sloc0_1_0 + 1)
	ret
L019002?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:372: buff[j] = c;
	mov	a,r1
	add	a,_getsn_buff_1_81
	mov	r4,a
	mov	a,r2
	addc	a,(_getsn_buff_1_81 + 1)
	mov	r5,a
	mov	r6,(_getsn_buff_1_81 + 2)
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r3
	lcall	__gptrput
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:362: for (j = 0; j < (len - 1); j++)
	inc	r1
	cjne	r1,#0x00,L019018?
	inc	r2
L019018?:
	mov	_getsn_sloc0_1_0,r1
	mov	(_getsn_sloc0_1_0 + 1),r2
	sjmp	L019005?
L019008?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:375: buff[j] = 0;
	mov	a,_getsn_sloc0_1_0
	add	a,_getsn_buff_1_81
	mov	r2,a
	mov	a,(_getsn_sloc0_1_0 + 1)
	addc	a,(_getsn_buff_1_81 + 1)
	mov	r3,a
	mov	r4,(_getsn_buff_1_81 + 2)
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	clr	a
	lcall	__gptrput
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:376: return len;
	mov	dpl,_getsn_PARM_2
	mov	dph,(_getsn_PARM_2 + 1)
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;x                         Allocated to registers r4 r5 
;y                         Allocated to registers r6 r7 
;NS_Volt                   Allocated with name '_main_NS_Volt_1_87'
;EW_Volt                   Allocated with name '_main_EW_Volt_1_87'
;beacon                    Allocated to registers r2 r3 
;buffer1                   Allocated with name '_main_buffer1_1_87'
;buffer2                   Allocated with name '_main_buffer2_1_87'
;------------------------------------------------------------
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:379: void main(void)
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:385: int beacon = 1;
	mov	r2,#0x01
	mov	r3,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:389: TIMER_OUT_0 = 0;
	clr	_P2_0
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:390: TIMER_OUT_0_INVERTED = 1;
	setb	_P2_1
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:392: waitms(500); // Give PuTTY a chance to start.
	mov	dptr,#0x01F4
	push	ar2
	push	ar3
	lcall	_waitms
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:393: TIMER0_Init();
	lcall	_TIMER0_Init
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:394: TIMER2_Init();
	lcall	_TIMER2_Init
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:396: printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	mov	a,#__str_0
	push	acc
	mov	a,#(__str_0 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	dec	sp
	dec	sp
	dec	sp
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:401: __FILE__, __DATE__, __TIME__);
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:400: "Compiled: %s, %s\n\n",
	mov	a,#__str_4
	push	acc
	mov	a,#(__str_4 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#__str_3
	push	acc
	mov	a,#(__str_3 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#__str_2
	push	acc
	mov	a,#(__str_2 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#__str_1
	push	acc
	mov	a,#(__str_1 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xf4
	mov	sp,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:404: LCD_4BIT();
	lcall	_LCD_4BIT
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:405: TR2 = 1; // Start Timer2
	setb	_TR2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:407: InitPinADC(1, 1); // Configure P1.1 as North/South analog input
	mov	_InitPinADC_PARM_2,#0x01
	mov	dpl,#0x01
	lcall	_InitPinADC
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:408: InitPinADC(1, 2); // Configure P1.2 as East/West analog input
	mov	_InitPinADC_PARM_2,#0x02
	mov	dpl,#0x01
	lcall	_InitPinADC
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:409: InitADC();
	lcall	_InitADC
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:411: while (1)
L020032?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:415: NS_Volt = Volts_at_Pin(QFP32_MUX_P1_1); // North/South Voltage
	mov	dpl,#0x07
	push	ar2
	push	ar3
	lcall	_Volts_at_Pin
	mov	_main_NS_Volt_1_87,dpl
	mov	(_main_NS_Volt_1_87 + 1),dph
	mov	(_main_NS_Volt_1_87 + 2),b
	mov	(_main_NS_Volt_1_87 + 3),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:416: EW_Volt = Volts_at_Pin(QFP32_MUX_P1_2); // East/West Voltage
	mov	dpl,#0x08
	lcall	_Volts_at_Pin
	mov	_main_EW_Volt_1_87,dpl
	mov	(_main_EW_Volt_1_87 + 1),dph
	mov	(_main_EW_Volt_1_87 + 2),b
	mov	(_main_EW_Volt_1_87 + 3),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:419: if(NS_Volt < 1.9){
	mov	a,#0x33
	push	acc
	push	acc
	mov	a,#0xF3
	push	acc
	mov	a,#0x3F
	push	acc
	mov	dpl,_main_NS_Volt_1_87
	mov	dph,(_main_NS_Volt_1_87 + 1)
	mov	b,(_main_NS_Volt_1_87 + 2)
	mov	a,(_main_NS_Volt_1_87 + 3)
	lcall	___fslt
	mov	r6,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar3
	pop	ar2
	mov	a,r6
	jz	L020005?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:420: y = 2;
	mov	r6,#0x02
	mov	r7,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:421: printf("y = %d\n", y);
	push	ar2
	push	ar3
	push	ar6
	push	ar7
	mov	a,#0x02
	push	acc
	clr	a
	push	acc
	mov	a,#__str_5
	push	acc
	mov	a,#(__str_5 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar3
	pop	ar2
	ljmp	L020006?
L020005?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:423: else if(NS_Volt > 2.7){
	push	ar2
	push	ar3
	mov	a,#0xCD
	push	acc
	mov	a,#0xCC
	push	acc
	mov	a,#0x2C
	push	acc
	mov	a,#0x40
	push	acc
	mov	dpl,_main_NS_Volt_1_87
	mov	dph,(_main_NS_Volt_1_87 + 1)
	mov	b,(_main_NS_Volt_1_87 + 2)
	mov	a,(_main_NS_Volt_1_87 + 3)
	lcall	___fsgt
	mov	r4,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar3
	pop	ar2
	mov	a,r4
	jz	L020002?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:424: y = 0;
	mov	r6,#0x00
	mov	r7,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:425: printf("y = %d\n", y);
	push	ar2
	push	ar3
	push	ar6
	push	ar7
	clr	a
	push	acc
	push	acc
	mov	a,#__str_5
	push	acc
	mov	a,#(__str_5 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar3
	pop	ar2
	sjmp	L020006?
L020002?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:428: y = 1;
	mov	r6,#0x01
	mov	r7,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:429: printf("y = %d\n", y);
	push	ar2
	push	ar3
	push	ar6
	push	ar7
	mov	a,#0x01
	push	acc
	clr	a
	push	acc
	mov	a,#__str_5
	push	acc
	mov	a,#(__str_5 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar3
	pop	ar2
L020006?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:433: if(EW_Volt < 1.9){
	push	ar2
	push	ar3
	push	ar6
	push	ar7
	mov	a,#0x33
	push	acc
	push	acc
	mov	a,#0xF3
	push	acc
	mov	a,#0x3F
	push	acc
	mov	dpl,_main_EW_Volt_1_87
	mov	dph,(_main_EW_Volt_1_87 + 1)
	mov	b,(_main_EW_Volt_1_87 + 2)
	mov	a,(_main_EW_Volt_1_87 + 3)
	lcall	___fslt
	mov	r4,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar3
	pop	ar2
	mov	a,r4
	jz	L020011?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:434: x = 0;
	mov	r4,#0x00
	mov	r5,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:435: printf("x = %d\n", x);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	clr	a
	push	acc
	push	acc
	mov	a,#__str_6
	push	acc
	mov	a,#(__str_6 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
	ljmp	L020012?
L020011?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:437: else if(EW_Volt > 2.7){
	push	ar2
	push	ar3
	push	ar6
	push	ar7
	mov	a,#0xCD
	push	acc
	mov	a,#0xCC
	push	acc
	mov	a,#0x2C
	push	acc
	mov	a,#0x40
	push	acc
	mov	dpl,_main_EW_Volt_1_87
	mov	dph,(_main_EW_Volt_1_87 + 1)
	mov	b,(_main_EW_Volt_1_87 + 2)
	mov	a,(_main_EW_Volt_1_87 + 3)
	lcall	___fsgt
	mov	r0,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar3
	pop	ar2
	mov	a,r0
	jz	L020008?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:438: x = 2;
	mov	r4,#0x02
	mov	r5,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:439: printf("x = %d\n", x);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	mov	a,#0x02
	push	acc
	clr	a
	push	acc
	mov	a,#__str_6
	push	acc
	mov	a,#(__str_6 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
	sjmp	L020012?
L020008?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:442: x = 1;
	mov	r4,#0x01
	mov	r5,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:443: printf("x = %d\n", x);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	mov	a,#0x01
	push	acc
	clr	a
	push	acc
	mov	a,#__str_6
	push	acc
	mov	a,#(__str_6 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
L020012?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:447: if(Bpin){
	jnb	_P1_3,L020017?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:448: while(Bpin);
L020013?:
	jb	_P1_3,L020013?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:449: beacon = !beacon;
	mov	a,r2
	orl	a,r3
	cjne	a,#0x01,L020058?
L020058?:
	clr	a
	rlc	a
	mov	r2,a
	mov	r3,#0x00
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:450: printf("Beacon Toggled: %d", beacon);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	push	ar2
	push	ar3
	mov	a,#__str_7
	push	acc
	mov	a,#(__str_7 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfb
	mov	sp,a
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
L020017?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:454: if(beacon){
	mov	a,r2
	orl	a,r3
	jz	L020029?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:455: pulse_width = 200; // 100%
	mov	_pulse_width,#0xC8
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:456: LEDpin = 0; // LED on
	clr	_P1_4
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:457: sprintf(buffer1, "Beacon Mode     ");
	push	ar2
	push	ar3
	mov	a,#__str_8
	push	acc
	mov	a,#(__str_8 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer1_1_87
	push	acc
	mov	a,#(_main_buffer1_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:458: sprintf(buffer2, "Control Disabled");
	mov	a,#__str_9
	push	acc
	mov	a,#(__str_9 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
	ljmp	L020030?
L020029?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:461: LEDpin = 1; // LED off
	setb	_P1_4
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:462: sprintf(buffer1, "Controller Mode ");
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	mov	a,#__str_10
	push	acc
	mov	a,#(__str_10 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer1_1_87
	push	acc
	mov	a,#(_main_buffer1_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar7
	pop	ar6
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:464: switch (y*10+x){ // y*10+x
	mov	__mulint_PARM_2,r6
	mov	(__mulint_PARM_2 + 1),r7
	mov	dptr,#0x000A
	lcall	__mulint
	mov	a,dpl
	mov	b,dph
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
	add	a,r4
	mov	r4,a
	mov	a,r5
	addc	a,b
	mov	r5,a
	cjne	r4,#0x00,L020060?
	cjne	r5,#0x00,L020060?
	ljmp	L020023?
L020060?:
	cjne	r4,#0x01,L020061?
	cjne	r5,#0x00,L020061?
	ljmp	L020022?
L020061?:
	cjne	r4,#0x02,L020062?
	cjne	r5,#0x00,L020062?
	ljmp	L020021?
L020062?:
	cjne	r4,#0x0A,L020063?
	cjne	r5,#0x00,L020063?
	ljmp	L020024?
L020063?:
	cjne	r4,#0x0C,L020064?
	cjne	r5,#0x00,L020064?
	ljmp	L020020?
L020064?:
	cjne	r4,#0x14,L020065?
	cjne	r5,#0x00,L020065?
	ljmp	L020025?
L020065?:
	cjne	r4,#0x15,L020066?
	cjne	r5,#0x00,L020066?
	sjmp	L020018?
L020066?:
	cjne	r4,#0x16,L020067?
	cjne	r5,#0x00,L020067?
	sjmp	L020019?
L020067?:
	ljmp	L020026?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:466: case N:
L020018?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:467: pulse_width = 180; // 90%
	mov	_pulse_width,#0xB4
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:468: sprintf(buffer2, "Direction: N    ");
	push	ar2
	push	ar3
	mov	a,#__str_11
	push	acc
	mov	a,#(__str_11 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:469: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:471: case NE:
L020019?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:472: pulse_width = 160; // 80%
	mov	_pulse_width,#0xA0
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:473: sprintf(buffer2, "Direction: NE   ");
	push	ar2
	push	ar3
	mov	a,#__str_12
	push	acc
	mov	a,#(__str_12 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:474: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:476: case E:
L020020?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:477: pulse_width = 140; // 70%
	mov	_pulse_width,#0x8C
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:478: sprintf(buffer2, "Direction: E    ");
	push	ar2
	push	ar3
	mov	a,#__str_13
	push	acc
	mov	a,#(__str_13 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:479: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:481: case SE:
L020021?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:482: pulse_width = 120; // 60%
	mov	_pulse_width,#0x78
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:483: sprintf(buffer2, "Direction: SE   ");
	push	ar2
	push	ar3
	mov	a,#__str_14
	push	acc
	mov	a,#(__str_14 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:484: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:486: case S:
L020022?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:487: pulse_width = 100; // 50%
	mov	_pulse_width,#0x64
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:488: sprintf(buffer2, "Direction: S    ");
	push	ar2
	push	ar3
	mov	a,#__str_15
	push	acc
	mov	a,#(__str_15 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:489: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:491: case SW:
L020023?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:492: pulse_width = 80; // 40%
	mov	_pulse_width,#0x50
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:493: sprintf(buffer2, "Direction: SW   ");
	push	ar2
	push	ar3
	mov	a,#__str_16
	push	acc
	mov	a,#(__str_16 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:494: break;
	ljmp	L020030?
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:496: case W:
L020024?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:497: pulse_width = 60; // 30%
	mov	_pulse_width,#0x3C
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:498: sprintf(buffer2, "Direction: W    ");
	push	ar2
	push	ar3
	mov	a,#__str_17
	push	acc
	mov	a,#(__str_17 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:499: break;
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:501: case NW:
	sjmp	L020030?
L020025?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:502: pulse_width = 40; // 20%
	mov	_pulse_width,#0x28
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:503: sprintf(buffer2, "Direction: NW   ");
	push	ar2
	push	ar3
	mov	a,#__str_18
	push	acc
	mov	a,#(__str_18 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:504: break;
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:506: default: // Waiting
	sjmp	L020030?
L020026?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:507: pulse_width = 20; // 10%
	mov	_pulse_width,#0x14
	clr	a
	mov	(_pulse_width + 1),a
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:508: sprintf(buffer2, "Direction: None ");
	push	ar2
	push	ar3
	mov	a,#__str_19
	push	acc
	mov	a,#(__str_19 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_main_buffer2_1_87
	push	acc
	mov	a,#(_main_buffer2_1_87 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	pop	ar3
	pop	ar2
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:509: }
L020030?:
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:511: LCDprint(buffer1, 1, 1);
	mov	_LCDprint_PARM_2,#0x01
	setb	_LCDprint_PARM_3
	mov	dptr,#_main_buffer1_1_87
	mov	b,#0x40
	push	ar2
	push	ar3
	lcall	_LCDprint
;	C:\Users\natha\Downloads\UBC\Year_2\Term_2\ELEC291\Project 2\Project-2-Tracking-Robot\doc\remote.c:512: LCDprint(buffer2, 2, 1);
	mov	_LCDprint_PARM_2,#0x02
	setb	_LCDprint_PARM_3
	mov	dptr,#_main_buffer2_1_87
	mov	b,#0x40
	lcall	_LCDprint
	pop	ar3
	pop	ar2
	ljmp	L020032?
	rseg R_CSEG

	rseg R_XINIT

	rseg R_CONST
__str_0:
	db 0x1B
	db '[2J'
	db 0x00
__str_1:
	db 'EFM8 remote'
	db 0x0A
	db 'File: %s'
	db 0x0A
	db 'Compiled: %s, %s'
	db 0x0A
	db 0x0A
	db 0x00
__str_2:
	db 'C:'
	db 0x5C
	db 'Users'
	db 0x5C
	db 'natha'
	db 0x5C
	db 'Downloads'
	db 0x5C
	db 'UBC'
	db 0x5C
	db 'Year_2'
	db 0x5C
	db 'Term_2'
	db 0x5C
	db 'ELEC291'
	db 0x5C
	db 'Project 2'
	db 0x5C
	db 'Project-2-Tracking-Robot'
	db 0x5C
	db 'doc'
	db 0x5C
	db 'remote.c'
	db 0x00
__str_3:
	db 'Apr  3 2023'
	db 0x00
__str_4:
	db '14:30:30'
	db 0x00
__str_5:
	db 'y = %d'
	db 0x0A
	db 0x00
__str_6:
	db 'x = %d'
	db 0x0A
	db 0x00
__str_7:
	db 'Beacon Toggled: %d'
	db 0x00
__str_8:
	db 'Beacon Mode     '
	db 0x00
__str_9:
	db 'Control Disabled'
	db 0x00
__str_10:
	db 'Controller Mode '
	db 0x00
__str_11:
	db 'Direction: N    '
	db 0x00
__str_12:
	db 'Direction: NE   '
	db 0x00
__str_13:
	db 'Direction: E    '
	db 0x00
__str_14:
	db 'Direction: SE   '
	db 0x00
__str_15:
	db 'Direction: S    '
	db 0x00
__str_16:
	db 'Direction: SW   '
	db 0x00
__str_17:
	db 'Direction: W    '
	db 0x00
__str_18:
	db 'Direction: NW   '
	db 0x00
__str_19:
	db 'Direction: None '
	db 0x00

	CSEG

end
