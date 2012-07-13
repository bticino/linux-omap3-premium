#ifndef _INCLUDE_BOARDOMAP3BAIA_H_
#define _INCLUDE_BOARDOMAP3BAIA_H_

/*
   eMMC connection
   PIN 'MMC2_DAT7' is connected to NET 'MMC2_D[7]'
   PIN 'MMC2_DAT6' is connected to NET 'MMC2_D[6]'
   PIN 'MMC2_D5' is connected to NET 'MMC2_D[5]'
   PIN 'MMC2_DAT4' is connected to NET 'MMC2_D[4]'
   PIN 'MMC2_DAT3/MCSPI3_CS0/GPIO_135' is connected to NET 'MMC2_D[3]'
   PIN 'MMC2_DAT2/MCSPI3_CS1/GPIO_134' is connected to NET 'MMC2_D[2]'
   PIN 'MMC2_DAT1/GPIO_133' is connected to NET 'MMC2_D[1]'
   PIN 'MMC2_DAT0/MCSPI3_SOMI/GPIO_132' is connected to NET 'MMC2_D[0]'
   PIN 'MMC2_CMD/MCSPI3_SIMO/GPIO_131' is connected to NET 'MMC2_CMD'
   PIN 'MMC2_CLK/MCSPI3_CLK/GPIO_130' is connected to NET 'MMC2_CLK_R'
*/

/* PIN 'MMC1_DAT7/SIM_RST/GPIO_129' is connected to NET 'SD_WR_PR' */
#define OMAP3_BAIA_SDCARD_WR_PR			(129)

/* Internal use
   PIN 'MMC1_DAT6/SIM_PWRCTRL/GPIO_128' is connected to NET 'CAP_VDDU'
   PIN 'MMC1_DAT5/SIM_CLK/GPIO_127' is connected to NET 'CAP_VDD_BB'
*/

/* PIN 'MMC1_DAT4/SIM_IO/GPIO_126' is connected to NET 'PME_LAN0'
   Not to be used because GPIO_126 is alread used by TS_RESET
*/

/* SDcard connection
   PIN 'MMC1_DAT3/MS_DAT3/GPIO_125' is connected to NET 'SD_D[3]'
   PIN 'MMC1_DAT2/MS_DAT2/GPIO_124' is connected to NET 'SD_D[2]'
   PIN 'MMC1_DAT1/MS_DAT1/GPIO_123' is connected to NET 'SD_D[1]'
   PIN 'MMC1_DAT0/MS_DAT0/GPIO_122' is connected to NET 'SD_D[0]'
   PIN 'MMC1_CMD/MS_BS/GPIO_121' is connected to NET 'SD_CMD'
   PIN 'MMC1_CLK/MS_CLK/GPIO_120' is connected to NET 'SD_CLK_R'
*/

/* GPMC bus
   PIN 'GPMC_D0' is connected to NET 'GPMC_D[0]'
   PIN 'GPMC_D1' is connected to NET 'GPMC_D[1]'
   PIN 'GPMC_D2' is connected to NET 'GPMC_D[2]'
   PIN 'GPMC_D3' is connected to NET 'GPMC_D[3]'
   PIN 'GPMC_D4' is connected to NET 'GPMC_D[4]'
   PIN 'GPMC_D5' is connected to NET 'GPMC_D[5]'
   PIN 'GPMC_D6' is connected to NET 'GPMC_D[6]'
   PIN 'GPMC_D7' is connected to NET 'GPMC_D[7]'
   PIN 'GPMC_D8/GPIO_44' is connected to NET 'GPMC_D[8]'
   PIN 'GPMC_D9/GPIO_45' is connected to NET 'GPMC_D[9]'
   PIN 'GPMC_D10/GPIO_46' is connected to NET 'GPMC_D[10]'
   PIN 'GPMC_D11/GPIO_47' is connected to NET 'GPMC_D[11]'
   PIN 'GPMC_D12/GPIO_48' is connected to NET 'GPMC_D[12]'
   PIN 'GPMC_D13/GPIO_49' is connected to NET 'GPMC_D[13]'
   PIN 'GPMC_D14/GPIO_50' is connected to NET 'GPMC_D[14]'
   PIN 'GPMC_D15/GPIO_51' is connected to NET 'GPMC_D[15]'
   PIN 'GPMC_NOE' is connected to NET 'GPMC_nOE'
   PIN 'GPMC_NWE' is connected to NET 'GPMC_nWE'
   PIN 'GPMC_NCS3/SYS_NDMAREQ0/GPIO_54' is connected to NET 'GPMC_nCS3'
   PIN 'GPMC_A1/GPIO_34' is connected to NET 'GPMC_A[1]'
*/

/* PIN 'GPMC_WAIT3/SYS_NDMAREQ1/GPIO_65' is connected to NET '~HSUSB2_RESET' */
#define OMAP3_BAIA_EHCIPHYRESET			(65)

/* Not Connected
   PIN 'GPMC_WAIT0' is connected to NET 'GPMC_WAIT0'
   PIN 'GPMC_NWP/GPIO_62' is connected to NET 'GPMC_nWP'
   PIN 'GPMC_NBE0_CLE/GPIO_60' is connected to NET 'GPMC_CLE'
   PIN 'GPMC_NADV_ALE' is connected to NET 'GPMC_ALE'
   PIN 'GPMC_A2/GPIO_35' is connected to NET 'GPMC_A[2]'
   PIN 'GPMC_A3/GPIO_36' is connected to NET 'GPMC_A[3]'
   PIN 'GPMC_A4/GPIO_37' is connected to NET 'GPMC_A[4]'
   PIN 'GPMC_A5/GPIO_38' is connected to NET 'GPMC_A[5]'
   PIN 'GPMC_A6/GPIO_39' is connected to NET 'GPMC_A[6]'
   PIN 'GPMC_A7/GPIO_40' is connected to NET 'GPMC_A[7]'
   PIN 'GPMC_A8/GPIO_41' is connected to NET 'GPMC_A[8]'
   PIN 'GPMC_A9/SYS_NDMAREQ2/GPIO_42' is connected to NET 'GPMC_A[9]'
   PIN 'GPMC_A10/SYS_NDMAREQ3/GPIO_43' is connected to NET 'GPMC_A[10]'
   PIN 'GPMC_NCS0' is connected to NET 'GPMC_nCS0'
*/

/* PIN 'GPMC_NBE1/GPIO_61' is connected to NET 'SHTD_VIDEO_1V8' */
#define OMAP3_BAIA_SHTD_VIDEO_1V8		(61)

/* PIN 'GPMC_CLK/GPIO_59' is connected to NET 'ABIL_SOURCE_IP1V8' */
#define OMAP3_BAIA_ABIL_SOURCE_IP1V8		(59) /* send audio to aec */

/* I2S bus for TLV320AIC3104
   PIN 'GPMC_NCS7' is connected to NET 'IIS_FS'
   PIN 'GPMC_NCS6' is connected to NET 'IIS_DATAO'
   PIN 'GPMC_NCS5' is connected to NET 'IIS_DATAI'
   PIN 'GPMC_NCS4' is connected to NET 'IIS_CLK_UB'
*/

/* CVBS out
   PIN 'TV_VREF' is connected to NET 'XSIG040460'
   PIN 'TV_VFB1' is connected to NET 'XSIG040462'
   PIN 'TV_OUT1' is connected to NET 'CVBS_OUT'
*/

/* DISPLAY PARALLEL OUTPUT
   PIN 'DSS_ACBIAS/GPIO_69' is connected to NET 'DSS_ACBIAS'
   PIN 'DSS_VSYNC/GPIO_68' is connected to NET 'DSS_VSYNC'
   PIN 'DSS_HSYNC/GPIO_67/HW_DBG13' is connected to NET 'DSS_HSYNC'
   PIN 'DSS_PCLK/GPIO_66/HW_DBG12' is connected to NET 'DSS_PCLK_R'
   PIN 'DSS_D23/SDI_CLKN/DSS_D5/93' is connected to NET 'DSS_D[5]'
   PIN 'DD22/SDICLKP/MCSP3CS1/DD4/92' is connected to NET 'DSS_D[4]'
   PIN 'DD21/SDISTP/MCSPI3CS0/DD3/91' is connected to NET 'DSS_D[3]'
   PIN 'DSD20/SDIDN/MCSPI3SO/DSD2/90' is connected to NET 'DSS_D[2]'
   PIN 'DSD19/SDIHS/MCSPI3SI/DSD1/89' is connected to NET 'DSS_D[1]'
   PIN 'DSD18/SDIVS/MCSPI3CK/DSD0/88' is connected to NET 'DSS_D[0]'
   PIN 'DSS_D17/GPIO_87' is connected to NET 'DSS_D[17]'
   PIN 'DSS_D16/GPIO_86' is connected to NET 'DSS_D[16]'
   PIN 'DSS_D15/SDI_DAT3P/GPIO_85' is connected to NET 'DSS_D[15]'
   PIN 'DSS_D14/SDI_DAT3N/GPIO_84' is connected to NET 'DSS_D[14]'
   PIN 'DSS_D13/SDI_DAT2P/GPIO_83' is connected to NET 'DSS_D[13]'
   PIN 'DSS_D12/SDI_DAT2N/GPIO_82' is connected to NET 'DSS_D[12]'
   PIN 'DSS_D11/SDI_DAT1P/GPIO_81' is connected to NET 'DSS_D[11]'
   PIN 'DSS_D10/SDI_DAT1N/GPIO_80' is connected to NET 'DSS_D[10]'
   PIN 'DSS_D9/GPIO_79/HW_DBG17' is connected to NET 'DSS_D[9]'
   PIN 'DSS_D8/GPIO_78/HW_DBG16' is connected to NET 'DSS_D[8]'
   PIN 'DSS_D7/U1_RX/DSDT7/77/HWDB15' is connected to NET 'DSS_D[7]'
   PIN 'DSS_D6/U1_TX/DSDT6/76/HWDB14' is connected to NET 'DSS_D[6]'
*/

/* PIN 'DSS_D5/DY2/U3_TXIR/DSSDT5/75' is connected to NET 'RES_ZL2_1V8' */
#define OMAP3_BAIA_RES_ZL2_1V8			(75)

/* PIN 'DSS_D4/DX2/U3_RXIR/DSSDT4/74' is connected to NET 'RES_ZL1_1V8' */
#define OMAP3_BAIA_RES_ZL1_1V8			(74)

/* PIN 'DSS_D3/DY1/DSS656_DATA3/73' is connected to NET '~ABIL_DEM_VIDEO1V8' */
#define OMAP3_BAIA_ABIL_DEM_VIDEO1V8		(73)

/* PIN 'DSS_D2/DX1/DSSC656_DATA2/72' is connected to NET '~E2_WP' */
#define OMAP3_BAIA_EEPROM_WP			(72)

/* PIN 'DSS_D1/DY0/U1_RTS/DSS_DT1/71' is connected to NET '~RES_O_1V8' */
#define OMAP3_BAIA_RES_O_1V8			(71)

/* PIN 'DSS_D0/DX0/U1_CTS/DSS_DT0/70' is connected to NET 'WLAN_IRQ_B' */
#define OMAP3_BAIA_WLAN_IRQ_GPIO		(70)

/* PIN 'CAM_D11/GPIO_110/HW_DBG9' is connected to NET '~EN_AMPLI1V8' */
#define OMAP3_BAIA_EN_AMPLI			(110)

/* PIN 'CAM_D10/SSI2_WK/109/HW_DBG8' is connected to NET '~ABIL_FON_SCS1V8' */
#define OMAP3_BAIA_ABIL_FON_SCS			(109)

/* resistive:input, capacitive
   PIN 'CAM_D9/GPIO_108' is connected to NET 'nPENIRQ_1V8'
*/
#define OMAP3_BAIA_TS_NPENIRQ			(108)

/* PIN 'CAM_D8/GPIO_107' is connected to NET 'SOUND_1V8' */
#define OMAP3_BAIA_MOD_SOUND			(107)

/* VIDEO IN part1
   PIN 'CAM_D7/GPIO_106' is connected to NET 'VDIN_C7'
   PIN 'CAM_D6/GPIO_105' is connected to NET 'VDIN_C6'
   PIN 'CAM_D5/SSI2_RDYRX/104/HWDBG7' is connected to NET 'VDIN_C5'
   PIN 'CAM_D4/SSI2_FL_RX/103/HWDBG6' is connected to NET 'VDIN_C4'
   PIN 'CAM_D3/SSI2_DATRX/102/HWDBG5' is connected to NET 'VDIN_C3'
   PIN 'CAM_D2/SSI2_RDYTX/101/HWDBG4' is connected to NET 'VDIN_C2'
   PIN 'CAM_D1/CSI2_DY2/GPIO_100' is connected to NET 'VDIN_C1'
   PIN 'CAM_D0/CSI2_DX2/GPIO_99' is connected to NET 'VDIN_C0'
*/

/* PIN 'CAM_STROBE/GPIO_126/HW_DBG11' is connected to NET 'RESET_TOUCH_1V8' */
#define OMAP3_BAIA_TS_RESET			(126) /* resistive:output, capacitive:input */

/* PIN 'CAM_WEN/CAM_SHUT/167/HWDBG10' is connected to NET 'CLIP_1V8' */
#define OMAP3_BAIA_MOD_CLIP			(167)

/* VIDEO IN part2  TODO verify ITU
   PIN 'CAM_FLD/CAM_GL_RES/98/HWDBG3' is connected to NET 'VDIN_FIELD'
   PIN 'CAM_HS/SSI2_DATTX/94/HWDBG0' is connected to NET 'VDIN_HS'
   PIN 'CAM_VS/SSI2_FLAGTX/95/HWDBG1' is connected to NET 'VDIN_VS'
   PIN 'CAM_PCLK/GPIO_97/HW_DBG2' is connected to NET 'VDIN_CLK'
*/

/* PIN 'CAM_XCLKB/GPIO_111' is connected to NET 'VLCD_EN_1V8' */
#define OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8	(111)

/* PIN 'CAM_XCLKA/GPIO_96' is connected to NET 'EN_BL_1V8' */
#define OMAP3_BAIA_EN_BL			(96)

/* USB master
   PIN 'ED15/H2D1/29/M2TE/H2D1/J3/ST' is connected to NET 'ULPI2_D[1]'
   PIN 'ED14/H2/28/M2/H2/J2/SD/SD/ST' is connected to NET 'ULPI2_D[0]'
   PIN 'ED13/H2NXT/27/M2RM/H2NT/SDTD' is connected to NET 'ULPI2_NXT'
   PIN 'ED12/H2DIR/26/H2DR/SDTI_TXD0' is connected to NET 'ULPI2_DIR'
   PIN 'ED11/H2STP/25/M2RP/H2STP/SDK' is connected to NET 'ULPI2_STP'
   PIN 'ED10/U1RX/H2CK/24/H2CK/U1RX' is connected to NET 'ULPI2_UCLK'
*/

/* PIN 'ED9/SSEI/MC3D/H1N/23/MM1R/H1' is connected to NET '~PDEC_PWRDN' */
#define OMAP3_BAIA_NPDEC_PWRDN			(23)

/* PIN 'ED8/SDRMS/H1_DIR/22/HS1TLDIR' is connected to NET '~LAN_INTR0' */
#define OMAP3_BAIA_KS8851_MLL_GPIO_IRQ		(22)

/* Not connected
  PIN 'ED7/MCS3C/MC3D/H1D/21/M1/H1D' is connected to NET 'Batt_low_1V8'
*/

/* sdio WLAN
   PIN 'ED6/MCBS5DX/MC3D2/H1D/20/H1D' is connected to NET 'SDIO_D[2]'
   PIN 'ED5/MCBS5F/MC3D1/H1D5/19/H1D' is connected to NET 'SDIO_D[1]'
   PIN 'ED4/MCBS5DR/MC3D0/H1D/18/H1D' is connected to NET 'SDIO_D[0]'
   PIN 'ED3/MCSPI3CLK/H1D7/17/H1D7' is connected to NET 'SDIO_D[3]'
*/

/* PIN 'ED2/MCSCS0/MC3D/H1D/16/M1/H1' is connected to NET 'nCS4a_1V8' */
#define OMAP3_BAIA_NCS4A_ZL2_CS			(16)

/* Powering pull-up for factory reset
   PIN 'ED1/MCS3/MC3/H1D/15/M1E/H1D1' is connected to NET 'PBTN_DRV'
*/
#define OMAP3_BAIA_FACTORY_RESET_DRV		(15)

/* PIN 'ED0/MCS3/M3D4/H1D/14/M1R/H1D' is connected to NET 'nCS5a_1V8' */
#define OMAP3_BAIA_NCS5A_ZL1_CS			(14)

/* sdio WLAN
   PIN 'ETKCL/MMC3CMD/H1CK/13/H1TLCK' is connected to NET 'SDIO_CMD'
   PIN 'ECK/MCB5/M3/H1S/12/M1RP/H1TP' is connected to NET 'SDIO_CLK_R'
*/

/* Display 24 bit  DSS 18..23
   PIN 'SYS_BOOT6/GPIO_8' is connected to NET 'SYS_B6'
   PIN 'SYS_BOOT5/MMC2_DIR_DAT3/7' is connected to NET 'SYS_B5'
   PIN 'SYS_BOOT4/MMC2_DIR_DAT2/6' is connected to NET 'SYS_B4'
   PIN 'SYS_BOOT3/GPIO_5' is connected to NET 'SYS_B3'
   PIN 'SYS_BOOT1/GPIO_3' is connected to NET 'SYS_B1'
   PIN 'SYS_BOOT0/GPIO_2' is connected to NET 'SYS_B0'
*/

/* Internal use
   PIN 'SYS_BOOT2/GPIO_4' is connected to NET 'SYS_B2'
   PIN 'SYS_32K' is connected to NET 'SYS_32K'
   PIN 'SYS_XTALIN' is connected to NET 'SYS_XTALIN'
   PIN 'SYS_CLKREQ/GPIO_1' is connected to NET 'OM_CLKREQ'
*/

/* clkout2 for TLV
   PIN 'SYS_CLKOUT2/GPIO_186' is connected to NET 'SYS_CLKOUT'
*/

/* PIN 'SYS_CLKOUT1/GPIO_10' is connected to NET 'EN_BT1V8' */
#define OMAP3_BAIA_BLUETOOTH_RESET_1V8		(10)

/* Internal use
   PIN 'SYS_OFF_MODE/GPIO_9' is connected to NET 'OM_OFF_MODE'
   PIN 'SYS_NIRQ/GPIO_0' is connected to NET 'TPS_NIRQ'
   PIN 'SYS_NRESWARM/GPIO_30' is connected to NET 'OM_NRESWARM'
   PIN 'SYS_NRESPWRON' is connected to NET 'TPS_NRESPWRON'
*/

/* JTAG connection
   PIN 'JTG_EMU1/SDTITD0/SDTITXD1/31' is connected to NET 'TI_EMU1'
   PIN 'JTG_EMU0/SDTICLK/SDTITXD0/11' is connected to NET 'TI_EMU0'
   PIN 'JTAG_RTCK' is connected to NET 'RTCK'
   PIN 'JTAG_TCK' is connected to NET 'TCK'
   PIN 'JTAG_TDI' is connected to NET 'TDI'
   PIN 'JTAG_TMS' is connected to NET 'TMS'
   PIN 'JTAG_NTRST' is connected to NET 'nTRST'
   PIN 'JTAG_TDO' is connected to NET 'TO_TDI_PM'
*/

/* PIN 'HDSIO/SACK/I2C2SE/I2C3SE/170' is connected to NET 'PG_5V' */
#define OMAP3_BAIA_PG_5V		(170) /* power comes from usb or from reg. */

/* USB host
   PIN 'HSUSB0_NXT/GPIO_124' is connected to NET 'ULPI0_NXT'
   PIN 'HSUSB0_DIR/GPIO_122' is connected to NET 'ULPI0_DIR'
   PIN 'HSUSB0_STP/GPIO_121' is connected to NET 'ULPI0_STP'
   PIN 'HSUSB0_CLK/GPIO_120' is connected to NET 'ULPI0_UCLK'
   PIN 'HSUSB0_D7/GPIO_191' is connected to NET 'ULPI0_D[7]'
   PIN 'HSUSB0_D6/GPIO_190' is connected to NET 'ULPI0_D[6]'
   PIN 'HSUSB0_D5/GPIO_189' is connected to NET 'ULPI0_D[5]'
   PIN 'HSUSB0_D4/GPIO_188' is connected to NET 'ULPI0_D[4]'
   PIN 'HSUSB0_D3/UART3_CTS_RCTX/169' is connected to NET 'ULPI0_D[3]'
   PIN 'HSUSB0_D2/UART3_RTS_SD/131' is connected to NET 'ULPI0_D[2]'
   PIN 'HSUSB0_D1/UART3_RX_IRRX/130' is connected to NET 'ULPI0_D[1]'
   PIN 'HSUSB0_D0/UART3_TX_IRTX/125' is connected to NET 'ULPI0_D[0]'
*/

/* SCS PIC UART
   PIN 'MCBS3FSX/UT2RX/143/H3TLD7' is connected to NET 'RXD2_1V8'
   PIN 'MCBS3CKX/UART2_TX/142/H3TLD6' is connected to NET 'TXD2_1V8'
*/

/* PCM 2*Zarlink and TPS
   PIN  MCBS3DR/UART2_RTS/141/H3TLD5' is connected to NET 'McBSP3_DR'
   PIN 'MCBS3DX/UART2_CTS/140/H3TLD4' is connected to NET 'McBSP3_DX'
*/

/* I2S for TPS
   PIN 'MCBSP2_DX/GPIO_119' is connected to NET 'IIS3_DATAO'
   PIN 'MCBSP2_DR/GPIO_118' is connected to NET 'IIS3_DATAI'
   PIN 'MCBSP2_CLKX/GPIO_117' is connected to NET 'IIS3_CLK_B2'
   PIN 'MCBSP2_FSX/GPIO_116' is connected to NET 'IIS3_FS'
*/

/* PCM 2*Zarlink and TPS
   PIN 'MCBSP1_CLKX/MCBSP3_CLKX/162' is connected to NET 'McBSP3_CLKX'
   PIN 'MCBS1FSX/MCS4CS0/MCB3FSX/161' is connected to NET 'McBSP3_FSX'
*/

/* PIN 'MCBSP_CLKS/CAMSHUT/160/U1CTS' is connected to NET '~ABIL_FON_IP1V8' */
#define OMAP3_BAIA_ABIL_FON_IP			(160) /* mod enable */

/* spi4 Zarlink
PIN 'MCBSP1DR/MCSP4SO/MCBS3DR/159' is connected to NET 'SPI4_SOMI'
PIN 'MCBS1DX/MCSP4SI/MCBSP3DX/158' is connected to NET 'SPI4_SIMO'
*/

/* PIN 'MCBSP1_FSR/CAM_GLO_RST/157' is connected to NET '~LAN_RES_1V8' */
#define OMAP3_BAIA_KS8851_MLL_RESET		(157)


/* spi4 Zarlink
   PIN 'MCB1CLKR/MCSP4_CLK/SIMCD/156' is connected to NET 'SPI4_CLK'
*/

/* Usb master
   PIN 'MC2C1/T8PE/H2T3/H2D3/182/M2N' is connected to NET 'ULPI2_D[3]'
   PIN 'MCS2CS0/T11PE/H2TD6/H2D6/181' is connected to NET 'ULPI2_D[6]'
   PIN 'MCSP2SO/T10PE/H2TD5/H2D5/180' is connected to NET 'ULPI2_D[5]'
   PIN 'MCSP2SI/T9PE/H2TLD4/H2D4/179' is connected to NET 'ULPI2_D[4]'
   PIN 'MCSPI2CLK/H2TLD7/H2D7/178' is connected to NET 'ULPI2_D[7]'
   PIN 'MCS1CS3/H2TLD2/H2D2/177/M2TD' is connected to NET 'ULPI2_D[2]'
*/

/* spi1 resistive touch controller
   PIN 'MCSPI1_CS0/GPIO_174' is connected to NET 'SPI1_CS0'
   PIN 'MCSPI1_SOMI/GPIO_173' is connected to NET 'SPI1_SOMI'
   PIN 'MCSPI1_SIMO/GPIO_172' is connected to NET 'SPI1_SIMO'
   PIN 'MCSPI1_CLK/GPIO_171' is connected to NET 'SPI1_CLK'
*/

/* smartreflex
   PIN 'I2C4_SDA/SYS_NVMODE2' is connected to NET 'SR_SDA'
   PIN 'I2C4_SCL/SYS_NVMODE1' is connected to NET 'SR_SCL'
*/

/* PIN 'I2C3_SDA/GPIO_185' is connected to NET 'VLED_EN_1V8' */
#define OMAP3_BAIA_VLED_EN_1V8			(185)

/* PIN 'I2C3_SCL/GPIO_184' is connected to NET 'nCS6a_1V8' */
#define OMAP3_BAIA_WLAN_PMENA_GPIO		(184)

/* i2c2
   PIN 'I2C2_SDA/GPIO_183' is connected to NET 'GP_1P8_SDA'
   PIN 'I2C2_SCL/GPIO_168' is connected to NET 'GP_1P8_SCL'
*/

/* i2c1
   PIN 'I2C1_SDA' is connected to NET 'CNTL_SDA'
   PIN 'I2C1_SCL' is connected to NET 'CNTL_SCL'
*/

/* UART Console
   PIN 'UART3_TX_IRTX/GPIO_166' is connected to NET 'TXD3_1V8'
   PIN 'UART3_RX_IRRX/GPIO_165' is connected to NET 'RXD3_1V8'
*/

/* Reading Cfg factory reset
   PIN 'UART3_RTS_SD/GPIO_164' is connected to NET 'PBTN_RES'
*/
#define OMAP3_BAIA_FACTORY_RESET_READ		(164)

/*   PIN 'UART3_CTS_RCTX/GPIO_163' is connected to NET 'nPWR_FAIL_1V8' */
#define OMAP3_BAIA_NPWR_FAIL			(163)

/*
   PIN 'UART1RX/MCBS1CKR/MCSP4LK/151' is connected to NET 'RXD1_1V8'
   PIN 'UART1CTS/SI1RYTX/150/HU3TCLK' is connected to NET 'CTS1_1V8'
   PIN 'UART1_RTS/SSI1_FLAG_TX/IO149' is connected to NET 'RTS1_1V8'
   PIN 'UART1_TX/SSI1_DAT_TX/GPIO_148' is connected to NET 'TXD1_1V8'
*/

/* trough TPS65951 */
#define OMAP3_BAIA_TPS_OFFSET			(192) /* not used */
#define OMAP3_BAIA_TPS_SD_CARD_DET		(OMAP3_BAIA_TPS_OFFSET)
#define OMAP3_BAIA_TPS_PULS4			(OMAP3_BAIA_TPS_OFFSET + 1)
#define OMAP3_BAIA_TPS_PULS3			(OMAP3_BAIA_TPS_OFFSET + 2)
#define OMAP3_BAIA_TPS_PWM01V8			(OMAP3_BAIA_TPS_OFFSET + 6)
#define OMAP3_BAIA_TPS_PULS1			(OMAP3_BAIA_TPS_OFFSET + 7)
#define OMAP3_BAIA_TPS_PDEC_RES			(OMAP3_BAIA_TPS_OFFSET + 15)
#define OMAP3_BAIA_TPS_PULS2			(OMAP3_BAIA_TPS_OFFSET + 13)

#endif
