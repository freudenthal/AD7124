#ifndef _AD7124_H
#define _AD7124_H

#include "Arduino.h"
#include "SPI.h"

    /* Communication Register bits */
#define AD7124_COMM_REG_WEN    7
#define AD7124_COMM_REG_RW     6
#define AD7124_COMM_REG_RS_MASK 0x3F

    /* Status Register bits */
#define AD7124_STATUS_REG_RDY     7
#define AD7124_STATUS_REG_ERROR   6
#define AD7124_STATUS_REG_POR     4
#define AD7124_STATUS_REG_CH_MASK 0xF
#define AD7124_STATUS_REG_CH_SHIFT 0

    /* ADC_Control Register bits */
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   12
#define AD7124_ADC_CTRL_REG_CONT_READ      11
#define AD7124_ADC_CTRL_REG_DATA_STATUS    10
#define AD7124_ADC_CTRL_REG_CS_EN          9
#define AD7124_ADC_CTRL_REG_REF_EN         8
#define AD7124_ADC_CTRL_REG_POWER_MODE_MASK  (0x3 << 6)
#define AD7124_ADC_CTRL_REG_POWER_MODE_SHIFT 6
#define AD7124_ADC_CTRL_REG_MODE_MASK        (0xF << 2)
#define AD7124_ADC_CTRL_REG_MODE_SHIFT        2
#define AD7124_ADC_CTRL_REG_CLK_SEL_MASK     (0x3 << 0)
#define AD7124_ADC_CTRL_REG_CLK_SEL_SHIFT     0

    /* IO_Control_1 Register bits */
#define AD7124_IO_CTRL1_REG_GPIO_DAT2     23
#define AD7124_IO_CTRL1_REG_GPIO_DAT1     22
#define AD7124_IO_CTRL1_REG_GPIO_CTRL2    19
#define AD7124_IO_CTRL1_REG_GPIO_CTRL1    18
#define AD7124_IO_CTRL1_REG_PDSW          15
#define AD7124_IO_CTRL1_REG_IOUT1_SHIFT  11
#define AD7124_IO_CTRL1_REG_IOUT0_SHIFT  8
#define AD7124_IO_CTRL1_REG_IOUT_CH1_SHIFT  4
#define AD7124_IO_CTRL1_REG_IOUT_CH0_SHIFT  0
#define AD7124_IO_CTRL1_REG_IOUT1_MASK (0x7 << 11)
#define AD7124_IO_CTRL1_REG_IOUT0_MASK (0x7 << 8)
#define AD7124_IO_CTRL1_REG_IOUT_CH1_MASK (0xF << 4)
#define AD7124_IO_CTRL1_REG_IOUT_CH0_MASK (0xF << 0)

    /* IO_Control_2 Register bits */
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS7   15
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS6   14
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS5   11
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS4   10
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS3   5
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS2   4
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS1   1
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS0   0

    /* ID Register bits */
//#define AD7124_ID_REG_DEVICE_ID_MASK     (0xF << 4)
//#define AD7124_ID_REG_DEVICE_ID_SHIFT     4
//#define AD7124_ID_REG_SILICON_REV_MASK   (0xF << 0)
//#define AD7124_ID_REG_SILICON_REV_SHIFT   0

#define AD7124_ID_REG_DEVICE_ID_MASK     (0xF << 0)
#define AD7124_ID_REG_DEVICE_ID_SHIFT     0
#define AD7124_ID_REG_SILICON_REV_MASK   (0xF << 4)
#define AD7124_ID_REG_SILICON_REV_SHIFT   4

    /* Error Register bits */
#define AD7124_ERR_REG_LDO_CAP_ERR        19
#define AD7124_ERR_REG_ADC_CAL_ERR        18
#define AD7124_ERR_REG_ADC_CONV_ERR       17
#define AD7124_ERR_REG_ADC_SAT_ERR        16
#define AD7124_ERR_REG_AINP_OV_ERR        15
#define AD7124_ERR_REG_AINP_UV_ERR        14
#define AD7124_ERR_REG_AINM_OV_ERR        13
#define AD7124_ERR_REG_AINM_UV_ERR        12
#define AD7124_ERR_REG_REF_DET_ERR        11
#define AD7124_ERR_REG_DLDO_PSM_ERR       9
#define AD7124_ERR_REG_ALDO_PSM_ERR       7
#define AD7124_ERR_REG_SPI_IGNORE_ERR     6
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR   5
#define AD7124_ERR_REG_SPI_READ_ERR       4
#define AD7124_ERR_REG_SPI_WRITE_ERR      3
#define AD7124_ERR_REG_SPI_CRC_ERR        2
#define AD7124_ERR_REG_MM_CRC_ERR         1
#define AD7124_ERR_REG_ROM_CRC_ERR        0

    /* Error_En Register bits */
#define AD7124_ERREN_REG_LDO_CAP_CHK_MASK      (0x3 << 19)
#define AD7124_ERREN_REG_LDO_CAP_CHK_SHIFT     19
#define AD7124_ERREN_REG_MCLK_CNT_EN           22
#define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   21
#define AD7124_ERREN_REG_ADC_CAL_ERR_EN        18
#define AD7124_ERREN_REG_ADC_CONV_ERR_EN       17
#define AD7124_ERREN_REG_ADC_SAT_ERR_EN        16
#define AD7124_ERREN_REG_AINP_OV_ERR_EN        15
#define AD7124_ERREN_REG_AINP_UV_ERR_EN        14
#define AD7124_ERREN_REG_AINM_OV_ERR_EN        13
#define AD7124_ERREN_REG_AINM_UV_ERR_EN        12
#define AD7124_ERREN_REG_REF_DET_ERR_EN        11
#define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN 10
#define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      9
#define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN 8
#define AD7124_ERREN_REG_ALDO_PSM_ERR_EN       7
#define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     6
#define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   5
#define AD7124_ERREN_REG_SPI_READ_ERR_EN       4
#define AD7124_ERREN_REG_SPI_WRITE_ERR_EN      3
#define AD7124_ERREN_REG_SPI_CRC_ERR_EN        2
#define AD7124_ERREN_REG_MM_CRC_ERR_EN         1
#define AD7124_ERREN_REG_ROM_CRC_ERR           0

    /* Channel Registers 0-15 bits */
#define AD7124_CH_MAP_REG_CH_ENABLE    15
#define AD7124_CH_MAP_REG_SETUP_MASK     (0x7 << 12)
#define AD7124_CH_MAP_REG_SETUP_SHIFT     12
#define AD7124_CH_MAP_REG_AINP_MASK      (0x1F << 5)
#define AD7124_CH_MAP_REG_AINP_SHIFT      5
#define AD7124_CH_MAP_REG_AINM_MASK      (0x1F << 0)
#define AD7124_CH_MAP_REG_AINM_SHIFT      0

    /* Configuration Registers 0-7 bits */
#define AD7124_CFG_REG_BIPOLAR     11
#define AD7124_CFG_REG_BURNOUT_MASK  (0x3 << 9)
#define AD7124_CFG_REG_BURNOUT_SHIFT 9
#define AD7124_CFG_REG_REF_BUFP    8
#define AD7124_CFG_REG_REF_BUFM    7
#define AD7124_CFG_REG_AIN_BUFP    6
#define AD7124_CFG_REG_AINN_BUFM   5
#define AD7124_CFG_REG_REF_SEL_MASK  (0x3 << 3)
#define AD7124_CFG_REG_REF_SEL_SHIFT  3
#define AD7124_CFG_REG_PGA_MASK      (0x7 << 0)
#define AD7124_CFG_REG_PGA_SHIFT      0

    /* Filter Register 0-7 bits */
#define AD7124_FILT_REG_FILTER_MASK        ( 0x7 << 21)
#define AD7124_FILT_REG_FILTER_SHIFT        21
#define AD7124_FILT_REG_REJ60             20
#define AD7124_FILT_REG_POST_FILTER_MASK    ( 0x7 << 17)
#define AD7124_FILT_REG_POST_FILTER_SHIFT    17
#define AD7124_FILT_REG_SINGLE_CYCLE      16
#define AD7124_FILT_REG_FS_MASK             ( 0x7FF << 0)
#define AD7124_FILT_REG_FS_SHIFT             0

#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */

#define AD7124NumberOfRegisters 57
#define AD7124SPIBufferSize 8
#define AD7124ConfigurationSize 8
#define AD7124ChannelCount 16

class AD7124
{
	public:
		enum class Registers : uint8_t
		{
			Status = 0x00,
			Control = 0x01,
			Data = 0x02,
			IOCon1 = 0x03,
			IOCon2 = 0x04,
			ID = 0x05,
			Error = 0x06,
			Error_En = 0x07,
			Mclk_Count = 0x08,
			Channel_0 = 0x09,
			Channel_1 = 0x0A,
			Channel_2 = 0x0B,
			Channel_3 = 0x0C,
			Channel_4 = 0x0D,
			Channel_5 = 0x0E,
			Channel_6 = 0x0F,
			Channel_7 = 0x10,
			Channel_8 = 0x11,
			Channel_9 = 0x12,
			Channel_10 = 0x13,
			Channel_11 = 0x14,
			Channel_12 = 0x15,
			Channel_13 = 0x16,
			Channel_14 = 0x17,
			Channel_15 = 0x18,
			Config_0 = 0x19,
			Config_1 = 0x1A,
			Config_2 = 0x1B,
			Config_3 = 0x1C,
			Config_4 = 0x1D,
			Config_5 = 0x1E,
			Config_6 = 0x1F,
			Config_7 = 0x20,
			Filter_0 = 0x21,
			Filter_1 = 0x22,
			Filter_2 = 0x23,
			Filter_3 = 0x24,
			Filter_4 = 0x25,
			Filter_5 = 0x26,
			Filter_6 = 0x27,
			Filter_7 = 0x28,
			Offset_0 = 0x29,
			Offset_1 = 0x2A,
			Offset_2 = 0x2B,
			Offset_3 = 0x2C,
			Offset_4 = 0x2D,
			Offset_5 = 0x2E,
			Offset_6 = 0x2F,
			Offset_7 = 0x30,
			Gain_0 = 0x31,
			Gain_1 = 0x32,
			Gain_2 = 0x33,
			Gain_3 = 0x34,
			Gain_4 = 0x35,
			Gain_5 = 0x36,
			Gain_6 = 0x37,
			Gain_7 = 0x38,
			REG_NO = 0x39,
		};
		enum class RegAccess : uint8_t
		{
			ReadWrite = 1,
			Read = 2,
			Write = 3,
		};
		struct RegisterStructure
		{
			Registers addr;
			uint8_t size;
			RegAccess rw;
		};
		struct StatusRegister
		{
			bool Ready;
			bool Error;
			bool PowerOnReset;
			uint8_t CurrentChannel;
		};
		enum class PowerModeSettings : uint8_t
		{
			LowPower = 0,
			MidPower = 1,
			FullPower = 2,
			Count = 3
		};
		enum class OperatingModeSettings : uint8_t
		{
			Continuous = 0,
			Single = 1,
			Standby = 2,
			PowerDown = 3,
			Idle = 4,
			ChannelOffsetCalibration = 5,
			ChannelGainCalibration = 6,
			SystemOffsetCalibration = 7,
			SystemGainCalibration = 8,
			Count = 9
		};
		enum class ClockModeSettings : uint8_t
		{
			InternalNoOuput = 0,
			InternalOutput = 1,
			ExternalAsExpected = 2,
			ExternalDivide4 = 3,
			Count = 4
		};
		struct ControlRegister
		{
			bool DOUTFunction;
			bool ContinousRead;
			bool DataStatus;
			bool CSEnable;
			bool ReferenceEnable;
			PowerModeSettings PowerMode;
			OperatingModeSettings OperatingMode;
			ClockModeSettings ClockMode;
		};
		enum class ExcitationCurrentSettings : uint8_t
		{
			Off = 0,
			uA50 = 1,
			uA100 = 2,
			uA250 = 3,
			uA500 = 4,
			uA750 = 5,
			uA1000 = 6,
			Count = 7
		};
		enum class ExcitationCurrentOutputSettings : uint8_t
		{
			AIN0 = 0,
			AIN1 = 1,
			AIN2 = 4,
			AIN3 = 5,
			AIN4 = 10,
			AIN5 = 11,
			AIN6 = 13,
			AIN7 = 14,
			Count = 15
		};
		struct IOSettingsRegister
		{
			bool GPIOData0;
			bool GPIOData1;
			bool GPIOControl0;
			bool GPIOControl1;
			bool PowerDownSwitch;
			ExcitationCurrentSettings ExcitationCurrent0;
			ExcitationCurrentSettings ExcitationCurrent1;
			ExcitationCurrentOutputSettings ExcitationOuput0;
			ExcitationCurrentOutputSettings ExcitationOuput1;
		};
		struct IOBiasRegister
		{
			bool AIN7;
			bool AIN6;
			bool AIN5;
			bool AIN4;
			bool AIN3;
			bool AIN2;
			bool AIN1;
			bool AIN0;
		};
		struct ErrorRegister
		{
			bool ROM;
			bool Memory;
			bool SPICRC;
			bool SPIWrite;
			bool SPIRead;
			bool SPIClk;
			bool SPIIgnore;
			bool AnalogueLDO;
			bool DigitalLDO;
			bool RefDetect;
			bool UnderAINM;
			bool OverAINM;
			bool UnderAINP;
			bool OverAINP;
			bool Saturation;
			bool Convert;
			bool Calibration;
			bool LDOCapacitor;
		};
		enum class LDOCheckSettings : uint8_t
		{
			NotEnabled = 0,
			Analogue = 1,
			Digital = 2,
			Count = 3,
		};
		struct ErrorEnableRegister
		{
			LDOCheckSettings LDOCheck;
			bool ClockCountEnabled;
			bool LDOCapacitorDisconnect;
			bool Calibration;
			bool Convert;
			bool Saturation;
			bool OverAINP;
			bool UnderAINP;
			bool OverAINM;
			bool UnderAINM;
			bool RefDetect;
			bool DigitalLDOTest;
			bool DigitalLDO;
			bool AnalogueLDOTest;
			bool AnalogueLDO;
			bool SPIIgnore;
			bool SPIClk;
			bool SPIRead;
			bool SPIWrite;
			bool SPICRC;
			bool Memory;
			bool ROM;
		};
		enum class AnalogueInputSettings : uint8_t
		{
			AIN0 = 0,
			AIN1 = 1,
			AIN2 = 2,
			AIN3 = 3,
			AIN4 = 4,
			AIN5 = 5,
			AIN6 = 6,
			AIN7 = 7,
			Temperature = 16,
			Vss = 17,
			InternalReference = 18,
			DigitalGND = 19,
			VddVssP = 20,
			VddVssM = 21,
			IOVddVssP = 22,
			IOVddVssM = 23,
			AnalogueLDOVssP = 24,
			AnalogueLDOVssM = 25,
			DigitalLDOVssP = 26,
			DigitalLDOVssM = 27,
			PeakToPeakP = 28,
			PeakToPeakM = 29,
			Count = 30
		};
		struct ChannelRegister
		{
			bool Enable;
			uint8_t Configuration;
			AnalogueInputSettings Positive;
			AnalogueInputSettings Negative;
		};
		enum class BurnoutSettings : uint8_t
		{
			Off = 0,
			uA0p5 = 1,
			uA2 = 2,
			uA4 = 3,
			Count = 4
		};
		enum class ReferenceSettings : uint8_t
		{
			ReferenceInput1 = 0,
			ReferenceInput2 = 1,
			InternalReference = 2,
			AnalogueVoltage = 3,
			Count = 4
		};
		enum class GainSettings : uint8_t
		{
			Gain1x = 0,
			Gain2x = 1,
			Gain4x = 2,
			Gain8x = 3,
			Gain16x = 4,
			Gain32x = 5,
			Gain64x = 6,
			Gain128x = 7,
			Count = 8
		};
		struct ConfigurationRegister
		{
			bool Bipolar;
			BurnoutSettings Burnout;
			bool BufferREFP;
			bool BufferREFN;
			bool BufferAINP;
			bool BufferAINM;
			ReferenceSettings Reference;
			GainSettings Gain;
		};
		enum class FilterTypeSettings : uint8_t
		{
			Sinc4 = 0,
			Sinc3 = 2,
			FastSinc4 = 4,
			FastSince3 = 5,
			PostFiltering = 7,
			Count = 8
		};
		enum class PostFilerTypeSettings : uint8_t
		{
			SPS27 = 2,
			SPS25 = 3,
			SPS20 = 5,
			SPS17 = 6,
			Count = 7
		};
		struct FilterRegister
		{
			FilterTypeSettings FilterType;
			bool Reject5060Hz;
			PostFilerTypeSettings PostFilterType;
			bool SingleCycle;
			uint16_t DataRate;
		};
		enum class IDTypeSettings : uint8_t
		{
			None = 0x00,
			Normal = 0x04,
			TypeB = 0x06,
			Count = 0x07
		};
		struct IDRegister
		{
			IDTypeSettings IDType;
			uint8_t Revision;
		};
		struct DataRegister
		{
			uint32_t Data;
			bool HasStatus;
			StatusRegister Status;
		};
		AD7124(uint8_t _SelectPin);
		bool Begin();
		void Reset();
		bool IsSampling();
		bool SampleReady();
		StatusRegister GetStatusRegister();
		StatusRegister ConvertToStatus(uint8_t Data);
		IDRegister GetIDRegister();
		DataRegister GetDataRegister();
		ErrorRegister GetErrorRegister();
		ControlRegister GetControlRegister();
		void SetControlRegister(ControlRegister NewSettings);
		IOSettingsRegister GetIOSettingsRegister();
		void SetIOSettingsRegister(IOSettingsRegister NewSettings);
		IOBiasRegister GetIOBiasRegister();
		void SetIOBiasRegister(IOBiasRegister NewSettings);
		ErrorEnableRegister GetErrorEnableRegister();
		void SetErrorEnableRegister(ErrorEnableRegister NewSettings);
		ChannelRegister GetChannelRegister(uint8_t Channel);
		void SetChannelRegister(uint8_t Channel, ChannelRegister NewSettings);
		ConfigurationRegister GetConfigurationRegister(uint8_t Configuration);
		void SetConfigurationRegister(uint8_t Configuration, ConfigurationRegister NewSettings);
		FilterRegister GetFilterRegister(uint8_t Filter);
		void SetFilterRegister(uint8_t Configuration, FilterRegister NewSettings);
		uint32_t GetOffsetRegister(uint8_t Offset);
		void SetOffsetRegister(uint8_t Offset, uint32_t NewSettings);
		uint32_t GetGainRegister(uint8_t Gain);
		bool SetDataRate(uint8_t Channel, uint16_t DataRate);
		void SetGainRegister(uint8_t Gain, uint32_t NewSettings);
		SPISettings* GetSPISettings();
	private:
		void UpdateInternalControlRegisters(ControlRegister Register);
		Registers ConvertChannelToRegister(uint8_t Channel);
		Registers ConvertConfigurationToRegister(uint8_t Configuration);
		Registers ConvertFilterToRegister(uint8_t Filter);
		Registers ConvertOffsetToRegister(uint8_t Offset);
		Registers ConvertGainToRegister(uint8_t Gain);
		void SetCommunicationsBuffer(Registers Register, bool ReadWrite);
		void SendWriteCommand(Registers Register, uint8_t* DataToSend);
		void SendReadCommand(Registers Register, uint8_t* DataToCollect);
		uint8_t ComputeCRC8(uint8_t* pBuf, uint8_t bufSize);
		void WipeSPIBuffer();
		void SPIRead(uint8_t bytes_number);
		void SPIWrite(uint8_t bytes_number);
		void Print_uint8(uint8_t value);
		void Print_uint16(uint16_t value);
		SPISettings ConnectionSettings;
		static const float InternalClockSpeed;
		static const RegisterStructure RegisterDictionary[];
		static const ControlRegister InitControlSettings;
		static const IOSettingsRegister InitIOSettings;
		static const IOBiasRegister InitIOBiasSettings;
		static const ErrorEnableRegister InitErrorEnableSettings;
		static const ChannelRegister InitChannelSettings0;
		static const ChannelRegister InitChannelSettings1;
		static const ChannelRegister InitChannelSettings2;
		static const ChannelRegister InitChannelSettingsDefault;
		static const ConfigurationRegister InitConfigurationSettings0;
		static const ConfigurationRegister InitConfigurationSettings1;
		static const ConfigurationRegister InitConfigurationSettings2;
		static const ConfigurationRegister InitConfigurationSettingsDefault;
		static const FilterRegister	InitFilterSettings0;
		static const FilterRegister	InitFilterSettings1;
		static const FilterRegister	InitFilterSettings2;
		static const FilterRegister	InitFilterSettingsDefault;
		uint8_t SelectPin;
		uint8_t SPIBuffer[AD7124SPIBufferSize];
		//uint32_t SampleCompleteTime;
		//uint16_t DataRates[AD7124ConfigurationSize];
		//GainSettings Gains[AD7124ConfigurationSize];
		PowerModeSettings PowerMode;
		bool UseCRC;
		bool UseStatus;
		bool UseContinuous;
		bool SamplingActive;
		bool SampleIsReady;
		bool CRCSuccess;
};
#endif
