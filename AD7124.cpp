#include "AD7124.h"


const float AD7124::InternalClockSpeed = 614400.0;
const AD7124::RegisterStructure AD7124::RegisterDictionary[AD7124NumberOfRegisters] =
{
	{Registers::Status, 1, RegAccess::Read},
	{Registers::Control, 2, RegAccess::ReadWrite},
	{Registers::Data, 3, RegAccess::Read},
	{Registers::IOCon1, 3, RegAccess::ReadWrite},
	{Registers::IOCon2, 2, RegAccess::ReadWrite},
	{Registers::ID, 1, RegAccess::Read},
	{Registers::Error, 3, RegAccess::Read},
	{Registers::Error_En, 3, RegAccess::ReadWrite},
	{Registers::Mclk_Count, 1, RegAccess::Read},
	{Registers::Channel_0, 2, RegAccess::ReadWrite},
	{Registers::Channel_1, 2, RegAccess::ReadWrite},
	{Registers::Channel_2, 2, RegAccess::ReadWrite},
	{Registers::Channel_3, 2, RegAccess::ReadWrite},
	{Registers::Channel_4, 2, RegAccess::ReadWrite},
	{Registers::Channel_5, 2, RegAccess::ReadWrite},
	{Registers::Channel_6, 2, RegAccess::ReadWrite},
	{Registers::Channel_7, 2, RegAccess::ReadWrite},
	{Registers::Channel_8, 2, RegAccess::ReadWrite},
	{Registers::Channel_9, 2, RegAccess::ReadWrite},
	{Registers::Channel_10, 2, RegAccess::ReadWrite},
	{Registers::Channel_11, 2, RegAccess::ReadWrite},
	{Registers::Channel_12, 2, RegAccess::ReadWrite},
	{Registers::Channel_13, 2, RegAccess::ReadWrite},
	{Registers::Channel_14, 2, RegAccess::ReadWrite},
	{Registers::Channel_15, 2, RegAccess::ReadWrite},
	{Registers::Config_0, 2, RegAccess::ReadWrite},
	{Registers::Config_1, 2, RegAccess::ReadWrite},
	{Registers::Config_2, 2, RegAccess::ReadWrite},
	{Registers::Config_3, 2, RegAccess::ReadWrite},
	{Registers::Config_4, 2, RegAccess::ReadWrite},
	{Registers::Config_5, 2, RegAccess::ReadWrite},
	{Registers::Config_6, 2, RegAccess::ReadWrite},
	{Registers::Config_7, 2, RegAccess::ReadWrite},
	{Registers::Filter_0, 3, RegAccess::ReadWrite},
	{Registers::Filter_1, 3, RegAccess::ReadWrite},
	{Registers::Filter_2, 3, RegAccess::ReadWrite},
	{Registers::Filter_3, 3, RegAccess::ReadWrite},
	{Registers::Filter_4, 3, RegAccess::ReadWrite},
	{Registers::Filter_5, 3, RegAccess::ReadWrite},
	{Registers::Filter_6, 3, RegAccess::ReadWrite},
	{Registers::Filter_7, 3, RegAccess::ReadWrite},
	{Registers::Offset_0, 3, RegAccess::ReadWrite},
	{Registers::Offset_1, 3, RegAccess::ReadWrite},
	{Registers::Offset_2, 3, RegAccess::ReadWrite},
	{Registers::Offset_3, 3, RegAccess::ReadWrite},
	{Registers::Offset_4, 3, RegAccess::ReadWrite},
	{Registers::Offset_5, 3, RegAccess::ReadWrite},
	{Registers::Offset_6, 3, RegAccess::ReadWrite},
	{Registers::Offset_7, 3, RegAccess::ReadWrite},
	{Registers::Gain_0, 3, RegAccess::ReadWrite},
	{Registers::Gain_1, 3, RegAccess::ReadWrite},
	{Registers::Gain_2, 3, RegAccess::ReadWrite},
	{Registers::Gain_3, 3, RegAccess::ReadWrite},
	{Registers::Gain_4, 3, RegAccess::ReadWrite},
	{Registers::Gain_5, 3, RegAccess::ReadWrite},
	{Registers::Gain_6, 3, RegAccess::ReadWrite},
	{Registers::Gain_7, 3, RegAccess::ReadWrite},
};

const AD7124::ControlRegister AD7124::InitControlSettings
{
	.DOUTFunction = false,
	.ContinousRead = false,
	.DataStatus = true,
	.CSEnable = true,
	.ReferenceEnable = true,
	.PowerMode = PowerModeSettings::FullPower,
	.OperatingMode = OperatingModeSettings::Continuous,
	.ClockMode = ClockModeSettings::InternalNoOuput
};

const AD7124::IOSettingsRegister AD7124::InitIOSettings
{
	.GPIOData0 = false,
	.GPIOData1 = false,
	.GPIOControl0 = false,
	.GPIOControl1 = false,
	.PowerDownSwitch = false,
	.ExcitationCurrent0 = ExcitationCurrentSettings::Off,
	.ExcitationCurrent1 = ExcitationCurrentSettings::Off,
	.ExcitationOuput0 = ExcitationCurrentOutputSettings::AIN0,
	.ExcitationOuput1 = ExcitationCurrentOutputSettings::AIN0
};

const AD7124::IOBiasRegister AD7124::InitIOBiasSettings
{
	.AIN7 = false,
	.AIN6 = false,
	.AIN5 = false,
	.AIN4 = false,
	.AIN3 = false,
	.AIN2 = false,
	.AIN1 = false,
	.AIN0 = false
};

const AD7124::ErrorEnableRegister AD7124::InitErrorEnableSettings
{
	.LDOCheck = LDOCheckSettings::NotEnabled,
	.ClockCountEnabled = false,
	.LDOCapacitorDisconnect = false,
	.Calibration = false,
	.Convert = false,
	.Saturation = false,
	.OverAINP = false,
	.UnderAINP = false,
	.OverAINM = false,
	.UnderAINM = false,
	.RefDetect = false,
	.DigitalLDOTest = false,
	.DigitalLDO = false,
	.AnalogueLDOTest = false,
	.AnalogueLDO = false,
	.SPIIgnore = true,
	.SPIClk = false,
	.SPIRead = false,
	.SPIWrite = false,
	.SPICRC = false,
	.Memory = false,
	.ROM = false
};

const AD7124::ChannelRegister AD7124::InitChannelSettings0
{
	.Enable = true,
	.Configuration = 0,
	.Positive = AnalogueInputSettings::AIN7,
	.Negative = AnalogueInputSettings::AIN6
};

const AD7124::ChannelRegister AD7124::InitChannelSettings1
{
	.Enable = true,
	.Configuration = 1,
	.Positive = AnalogueInputSettings::AIN5,
	.Negative = AnalogueInputSettings::AIN4
};

const AD7124::ChannelRegister AD7124::InitChannelSettings2
{
	.Enable = false,
	.Configuration = 2,
	.Positive = AnalogueInputSettings::Temperature,
	.Negative = AnalogueInputSettings::Vss
};

const AD7124::ChannelRegister AD7124::InitChannelSettingsDefault
{
	.Enable = false,
	.Configuration = 0,
	.Positive = AnalogueInputSettings::Vss,
	.Negative = AnalogueInputSettings::Vss
};

const AD7124::ConfigurationRegister AD7124::InitConfigurationSettings0
{
	.Bipolar = false,
	.Burnout = BurnoutSettings::Off,
	.BufferREFP = true,
	.BufferREFN = true,
	.BufferAINP = false,
	.BufferAINM = false,
	.Reference = ReferenceSettings::InternalReference,
	.Gain = GainSettings::Gain1x
};

const AD7124::ConfigurationRegister AD7124::InitConfigurationSettings1
{
	.Bipolar = false,
	.Burnout = BurnoutSettings::Off,
	.BufferREFP = true,
	.BufferREFN = true,
	.BufferAINP = false,
	.BufferAINM = false,
	.Reference = ReferenceSettings::InternalReference,
	.Gain = GainSettings::Gain1x
};

const AD7124::ConfigurationRegister AD7124::InitConfigurationSettings2
{
	.Bipolar = false,
	.Burnout = BurnoutSettings::Off,
	.BufferREFP = true,
	.BufferREFN = true,
	.BufferAINP = true,
	.BufferAINM = true,
	.Reference = ReferenceSettings::InternalReference,
	.Gain = GainSettings::Gain1x
};

const AD7124::ConfigurationRegister AD7124::InitConfigurationSettingsDefault
{
	.Bipolar = false,
	.Burnout = BurnoutSettings::Off,
	.BufferREFP = true,
	.BufferREFN = true,
	.BufferAINP = true,
	.BufferAINM = true,
	.Reference = ReferenceSettings::InternalReference,
	.Gain = GainSettings::Gain1x
};

const AD7124::FilterRegister AD7124::InitFilterSettings0
{
	.FilterType = FilterTypeSettings::Sinc4,
	.Reject5060Hz = true,
	.PostFilterType = PostFilerTypeSettings::SPS27,
	.SingleCycle = false,
	.DataRate = 240
};

const AD7124::FilterRegister AD7124::InitFilterSettings1
{
	.FilterType = FilterTypeSettings::Sinc4,
	.Reject5060Hz = true,
	.PostFilterType = PostFilerTypeSettings::SPS27,
	.SingleCycle = false,
	.DataRate = 240
};

const AD7124::FilterRegister AD7124::InitFilterSettings2
{
	.FilterType = FilterTypeSettings::Sinc4,
	.Reject5060Hz = true,
	.PostFilterType = PostFilerTypeSettings::SPS27,
	.SingleCycle = false,
	.DataRate = 240
};

const AD7124::FilterRegister AD7124::InitFilterSettingsDefault
{
	.FilterType = FilterTypeSettings::Sinc4,
	.Reject5060Hz = true,
	.PostFilterType = PostFilerTypeSettings::SPS27,
	.SingleCycle = false,
	.DataRate = 240
};

void AD7124::PrintStatusRegister(const StatusRegister Register)
{
	Serial.print("[AD7124STATUS](");
	Serial.print(Register.Ready);
	Serial.print(",");
	Serial.print(Register.Error);
	Serial.print(",");
	Serial.print(Register.PowerOnReset);
	Serial.print(",");
	Serial.print(Register.CurrentChannel);
	Serial.print(")\n");
}

void AD7124::PrintControlRegister(const ControlRegister Register)
{
	Serial.print("[AD7124CONTROL](");
	Serial.print(Register.DOUTFunction);
	Serial.print(",");
	Serial.print(Register.ContinousRead);
	Serial.print(",");
	Serial.print(Register.DataStatus);
	Serial.print(",");
	Serial.print(Register.CSEnable);
	Serial.print(",");
	Serial.print(Register.ReferenceEnable);
	Serial.print(",");
	Serial.print((uint8_t)(Register.PowerMode));
	Serial.print(",");
	Serial.print((uint8_t)(Register.OperatingMode));
	Serial.print(",");
	Serial.print((uint8_t)(Register.ClockMode));
	Serial.print(")\n");
}

void AD7124::PrintIOSettingsRegister(const IOSettingsRegister Register)
{
	Serial.print("[AD7124IOSETTINGS](");
	Serial.print(Register.GPIOData0);
	Serial.print(",");
	Serial.print(Register.GPIOData1);
	Serial.print(",");
	Serial.print(Register.GPIOControl0);
	Serial.print(",");
	Serial.print(Register.GPIOControl1);
	Serial.print(",");
	Serial.print(Register.PowerDownSwitch);
	Serial.print(",");
	Serial.print((uint8_t)(Register.ExcitationCurrent0));
	Serial.print(",");
	Serial.print((uint8_t)(Register.ExcitationCurrent1));
	Serial.print(",");
	Serial.print((uint8_t)(Register.ExcitationOuput0));
	Serial.print(",");
	Serial.print((uint8_t)(Register.ExcitationOuput1));
	Serial.print(")\n");
}

void AD7124::PrintIOBiasRegister(const IOBiasRegister Register)
{
	Serial.print("[AD7124IOSETTINGS](");
	Serial.print(Register.AIN7);
	Serial.print(",");
	Serial.print(Register.AIN6);
	Serial.print(",");
	Serial.print(Register.AIN5);
	Serial.print(",");
	Serial.print(Register.AIN4);
	Serial.print(",");
	Serial.print(Register.AIN3);
	Serial.print(",");
	Serial.print(Register.AIN2);
	Serial.print(",");
	Serial.print(Register.AIN1);
	Serial.print(",");
	Serial.print(Register.AIN0);
	Serial.print(")\n");
}

void AD7124::PrintErrorRegister(const ErrorRegister Register)
{
	Serial.print("[AD7124ERROR](");
	Serial.print(Register.ROM);
	Serial.print(",");
	Serial.print(Register.Memory);
	Serial.print(",");
	Serial.print(Register.SPICRC);
	Serial.print(",");
	Serial.print(Register.SPIWrite);
	Serial.print(",");
	Serial.print(Register.SPIRead);
	Serial.print(",");
	Serial.print(Register.SPIClk);
	Serial.print(",");
	Serial.print(Register.SPIIgnore);
	Serial.print(",");
	Serial.print(Register.AnalogueLDO);
	Serial.print(",");
	Serial.print(Register.DigitalLDO);
	Serial.print(",");
	Serial.print(Register.RefDetect);
	Serial.print(",");
	Serial.print(Register.UnderAINM);
	Serial.print(",");
	Serial.print(Register.OverAINM);
	Serial.print(",");
	Serial.print(Register.UnderAINP);
	Serial.print(",");
	Serial.print(Register.OverAINP);
	Serial.print(",");
	Serial.print(Register.Saturation);
	Serial.print(",");
	Serial.print(Register.Convert);
	Serial.print(",");
	Serial.print(Register.Calibration);
	Serial.print(",");
	Serial.print(Register.LDOCapacitor);
	Serial.print(")\n");
}

void AD7124::PrintErrorEnableRegister(const ErrorEnableRegister Register)
{
	Serial.print("[AD7124ERROREN](");
	Serial.print((uint8_t)(Register.LDOCheck));
	Serial.print(",");
	Serial.print(Register.ClockCountEnabled);
	Serial.print(",");
	Serial.print(Register.LDOCapacitorDisconnect);
	Serial.print(",");
	Serial.print(Register.Calibration);
	Serial.print(",");
	Serial.print(Register.Convert);
	Serial.print(",");
	Serial.print(Register.Saturation);
	Serial.print(",");
	Serial.print(Register.OverAINP);
	Serial.print(",");
	Serial.print(Register.UnderAINP);
	Serial.print(",");
	Serial.print(Register.OverAINM);
	Serial.print(",");
	Serial.print(Register.UnderAINM);
	Serial.print(",");
	Serial.print(Register.RefDetect);
	Serial.print(",");
	Serial.print(Register.DigitalLDOTest);
	Serial.print(",");
	Serial.print(Register.DigitalLDO);
	Serial.print(",");
	Serial.print(Register.AnalogueLDOTest);
	Serial.print(",");
	Serial.print(Register.AnalogueLDO);
	Serial.print(",");
	Serial.print(Register.SPIIgnore);
	Serial.print(",");
	Serial.print(Register.SPIClk);
	Serial.print(",");
	Serial.print(Register.SPIRead);
	Serial.print(",");
	Serial.print(Register.SPIWrite);
	Serial.print(",");
	Serial.print(Register.SPICRC);
	Serial.print(",");
	Serial.print(Register.Memory);
	Serial.print(",");
	Serial.print(Register.ROM);
	Serial.print(")\n");
}

void AD7124::PrintChannelRegister(const ChannelRegister Register)
{
	Serial.print("[AD7124CHANNEL](");
	Serial.print(Register.Enable);
	Serial.print(",");
	Serial.print(Register.Configuration);
	Serial.print(",");
	Serial.print((uint8_t)(Register.Positive));
	Serial.print(",");
	Serial.print((uint8_t)(Register.Negative));
	Serial.print(")\n");
}

void AD7124::PrintConfigurationRegister(const ConfigurationRegister Register)
{
	Serial.print("[AD7124CONFIG](");
	Serial.print(Register.Bipolar);
	Serial.print(",");
	Serial.print((uint8_t)(Register.Burnout));
	Serial.print(",");
	Serial.print(Register.BufferREFP);
	Serial.print(",");
	Serial.print(Register.BufferREFN);
	Serial.print(",");
	Serial.print(Register.BufferAINP);
	Serial.print(",");
	Serial.print(Register.BufferAINM);
	Serial.print(",");
	Serial.print((uint8_t)(Register.Reference));
	Serial.print(",");
	Serial.print((uint8_t)(Register.Gain));
	Serial.print(")\n");
}

void AD7124::PrintFilterRegister(const FilterRegister Register)
{
	Serial.print("[AD7124FILTER](");
	Serial.print((uint8_t)(Register.FilterType));
	Serial.print(",");
	Serial.print(Register.Reject5060Hz);
	Serial.print(",");
	Serial.print((uint8_t)(Register.PostFilterType));
	Serial.print(",");
	Serial.print(Register.SingleCycle);
	Serial.print(",");
	Serial.print((uint8_t)(Register.DataRate));
	Serial.print(")\n");
}

void AD7124::PrintIDRegister(const IDRegister Register)
{
	Serial.print("[AD7124ID](");
	Serial.print((uint8_t)(Register.IDType));
	Serial.print(",");
	Serial.print(Register.Revision);
	Serial.print(")\n");
}

void AD7124::PrintDataRegister(const DataRegister Register)
{
	Serial.print("[AD7124DATA](");
	Serial.print(Register.Data);
	Serial.print(")\n");
	if (Register.HasStatus)
	{
		PrintStatusRegister(Register.Status);
	}
}

double AD7124::ConvertDataRegisterToDouble(const DataRegister Register, const float Vref)
{
	if (Register.HasStatus)
	{
		ChannelRegister ChannelRegisterActive = GetChannelRegister(Register.Status.CurrentChannel);
		ConfigurationRegister ConfigurationRegisterActive = GetConfigurationRegister(ChannelRegisterActive.Configuration);
		float GainScalar = ConvertGainToScalar(ConfigurationRegisterActive.Gain);
		return ConvertDataRegisterToDouble(Register, ConfigurationRegisterActive.Bipolar, GainScalar, Vref);
	}
	else
	{
		Serial.print("<AD7124ERROR>(Unable to find channel. Please use complete convert function.)\n");
		return 0.0;
	}
}

float AD7124::ConvertGainToScalar(GainSettings GainInput)
{
	switch(GainInput)
	{
		case GainSettings::Gain1x:
			return 1.0;
		case GainSettings::Gain2x:
			return 2.0;
		case GainSettings::Gain4x:
			return 4.0;
		case GainSettings::Gain8x:
			return 8.0;
		case GainSettings::Gain16x:
			return 16.0;
		case GainSettings::Gain32x:
			return 32.0;
		case GainSettings::Gain64x:
			return 64.0;
		case GainSettings::Gain128x:
			return 128.0;
		default:
			Serial.print("<AD7124ERROR>(Unable to convert gain.)\n");
			return 0.0;
	}
}

double AD7124::ConvertDataRegisterToDouble(const DataRegister Register, const bool Bipolar, const float Gain, const float Vref)
{
	if (Bipolar)
	{
		return ( (double)(Regsiter.Data) / ( (double)(8388608.0) ) - 1.0) * (double)(Vref)/(double)(Gain);
	}
	else
	{
		return (double)(Register.Data)*( ( (double)(Vref) ) /( (double)(16777216.0) * (double)(Gain)) );
	}
}

AD7124::AD7124(uint8_t _SelectPin)
{
	SelectPin = _SelectPin;
	pinMode(SelectPin, OUTPUT);
	digitalWrite(SelectPin, HIGH);
	ConnectionSettings = SPISettings(4000000, MSBFIRST, SPI_MODE3);
	UseCRC = false;
	UseStatus = false;
}

bool AD7124::Begin()
{
	PowerMode = PowerModeSettings::LowPower;
	UseCRC = false;
	UseStatus = false;
	UseContinuous = false;
	SampleIsReady = false;
	SamplingActive = false;
	CRCSuccess = false;
	Reset();
	delay(1);
	IDRegister IDReturn = GetIDRegister();
	if (IDReturn.IDType == IDTypeSettings::None)
	{
		return false;
	}
	SetChannelRegister(0,InitChannelSettings0);
	//GetChannelRegister(0);
	SetChannelRegister(1,InitChannelSettings1);
	//GetChannelRegister(1);
	SetChannelRegister(2,InitChannelSettings2);
	for (int Index = 3; Index < 16; ++Index)
	{
		SetChannelRegister(Index,InitChannelSettingsDefault);
	}
	SetFilterRegister(0,InitFilterSettings0);
	//GetFilterRegister(0);
	SetFilterRegister(1,InitFilterSettings1);
	//GetFilterRegister(1);
	SetFilterRegister(2,InitFilterSettings2);
	for (int Index = 3; Index < 8; ++Index)
	{
		SetFilterRegister(Index,InitFilterSettingsDefault);
	}
	SetConfigurationRegister(0,InitConfigurationSettings0);
	//GetConfigurationRegister(0);
	SetConfigurationRegister(1,InitConfigurationSettings1);
	//GetConfigurationRegister(1);
	SetConfigurationRegister(2,InitConfigurationSettings2);
	for (int Index = 3; Index < 8; ++Index)
	{
		SetConfigurationRegister(Index,InitConfigurationSettingsDefault);
	}
	SetIOSettingsRegister(InitIOSettings);
	//GetIOSettingsRegister();
	SetIOBiasRegister(InitIOBiasSettings);
	//GetIOBiasRegister();
	SetErrorEnableRegister(InitErrorEnableSettings);
	//GetErrorEnableRegister();
	SetControlRegister(InitControlSettings);
	//GetControlRegister();
	return true;
}

bool AD7124::SetDataRate(uint8_t Channel, uint16_t DataRate)
{
	if (Channel > (AD7124ChannelCount-1))
	{
		Channel = (AD7124ChannelCount-1);
	}
	ChannelRegister ChannelSettings = GetChannelRegister(Channel);
	FilterRegister RegisterSettings = GetFilterRegister(ChannelSettings.Configuration);
	RegisterSettings.DataRate = DataRate;
	SetFilterRegister(ChannelSettings.Configuration,RegisterSettings);
	return true;
}

void AD7124::Reset()
{
	for (uint8_t Index = 0; Index < AD7124SPIBufferSize; ++Index)
	{
		SPIBuffer[Index] = 0xFF;
	}
	SPIWrite((uint8_t)(AD7124SPIBufferSize));
}

bool AD7124::SampleReady()
{
	StatusRegister Status = GetStatusRegister();
	return !(Status.Ready);
}

bool AD7124::IsSampling()
{
	if (UseContinuous || SamplingActive)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t AD7124::ComputeCRC8(uint8_t* pBuf, uint8_t bufSize)
{
	uint8_t i   = 0;
	uint8_t crc = 0;
	while(bufSize)
	{
		for(i = 0x80; i != 0; i >>= 1)
		{
			if(((crc & 0x80) != 0) != ((*pBuf & i) != 0))
			{
				/* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			}
			else
			{
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}

AD7124::StatusRegister AD7124::GetStatusRegister()
{
	uint8_t ReturnData = 0;
	SendReadCommand(Registers::Status,&ReturnData);
	return ConvertToStatus(ReturnData);
}

AD7124::StatusRegister AD7124::ConvertToStatus(uint8_t Data)
{
	StatusRegister ReturnRegister;
	ReturnRegister.Ready = bitRead(Data, AD7124_STATUS_REG_RDY);
	ReturnRegister.Error = bitRead(Data, AD7124_STATUS_REG_ERROR);
	ReturnRegister.PowerOnReset = bitRead(Data, AD7124_STATUS_REG_POR);
	ReturnRegister.CurrentChannel = (Data & AD7124_STATUS_REG_CH_MASK) >> AD7124_STATUS_REG_CH_SHIFT;
	SampleIsReady = !(ReturnRegister.Ready);
	return ReturnRegister;
}

AD7124::IDRegister AD7124::GetIDRegister()
{
	IDRegister ReturnRegister;
	uint8_t ReturnData = 0;
	SendReadCommand(Registers::ID,&ReturnData);
	ReturnRegister.IDType = static_cast<IDTypeSettings>( (ReturnData & AD7124_ID_REG_DEVICE_ID_MASK) >> AD7124_ID_REG_DEVICE_ID_SHIFT );
	ReturnRegister.Revision = static_cast<uint8_t>( (ReturnData & AD7124_ID_REG_SILICON_REV_MASK) >> AD7124_ID_REG_SILICON_REV_SHIFT );
	return ReturnRegister;
}

AD7124::DataRegister AD7124::GetDataRegister()
{
	DataRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[4];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::Data,ReturnData.Array);
	//Serial.print("<DADC>(");
	//for (int8_t Index = 3; Index >= 0; Index--)
	//{
	//	Print_uint8(ReturnData.Array[Index]);
	//	Serial.print(",");
	//}
	//Serial.print(")\n");
	if (UseStatus)
	{
		ReturnRegister.HasStatus = true;
		ReturnRegister.Status = ConvertToStatus(ReturnData.Array[0]);
	}
	else
	{
		ReturnRegister.HasStatus = false;
	}
	ReturnRegister.Data = ReturnData.Single >> 8;
	SamplingActive = false;
	return ReturnRegister;
}

AD7124::ControlRegister AD7124::GetControlRegister()
{
	ControlRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::Control,ReturnData.Array);
	ReturnRegister.DOUTFunction = bitRead(ReturnData.Single, AD7124_ADC_CTRL_REG_DOUT_RDY_DEL);
	ReturnRegister.ContinousRead = bitRead(ReturnData.Single, AD7124_ADC_CTRL_REG_CONT_READ);
	ReturnRegister.DataStatus = bitRead(ReturnData.Single, AD7124_ADC_CTRL_REG_DATA_STATUS);
	ReturnRegister.CSEnable = bitRead(ReturnData.Single, AD7124_ADC_CTRL_REG_CS_EN);
	ReturnRegister.ReferenceEnable = bitRead(ReturnData.Single, AD7124_ADC_CTRL_REG_REF_EN);
	ReturnRegister.PowerMode = static_cast<PowerModeSettings>( (ReturnData.Single & AD7124_ADC_CTRL_REG_POWER_MODE_MASK) >> AD7124_ADC_CTRL_REG_POWER_MODE_SHIFT );
	ReturnRegister.OperatingMode = static_cast<OperatingModeSettings>( (ReturnData.Single & AD7124_ADC_CTRL_REG_MODE_MASK) >> AD7124_ADC_CTRL_REG_MODE_SHIFT );
	ReturnRegister.ClockMode = static_cast<ClockModeSettings>( (ReturnData.Single & AD7124_ADC_CTRL_REG_CLK_SEL_MASK) >> AD7124_ADC_CTRL_REG_CLK_SEL_SHIFT );
	UpdateInternalControlRegisters(ReturnRegister);
	return ReturnRegister;
}

void AD7124::SetControlRegister(ControlRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_ADC_CTRL_REG_DOUT_RDY_DEL, NewSettings.DOUTFunction);
	bitWrite(SendingData.Single, AD7124_ADC_CTRL_REG_CONT_READ, NewSettings.ContinousRead);
	bitWrite(SendingData.Single, AD7124_ADC_CTRL_REG_DATA_STATUS, NewSettings.DataStatus);
	bitWrite(SendingData.Single, AD7124_ADC_CTRL_REG_CS_EN, NewSettings.CSEnable);
	bitWrite(SendingData.Single, AD7124_ADC_CTRL_REG_REF_EN, NewSettings.ReferenceEnable);
	SendingData.Single |= static_cast<uint16_t>(NewSettings.ClockMode) << AD7124_ADC_CTRL_REG_CLK_SEL_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.OperatingMode) << AD7124_ADC_CTRL_REG_MODE_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.PowerMode) << AD7124_ADC_CTRL_REG_POWER_MODE_SHIFT;
	SendWriteCommand(Registers::Control, SendingData.Array);
	UpdateInternalControlRegisters(NewSettings);
}

void AD7124::UpdateInternalControlRegisters(ControlRegister Register)
{
	UseStatus = Register.DataStatus;
	UseContinuous = (Register.OperatingMode == OperatingModeSettings::Continuous);
	PowerMode = Register.PowerMode;
	if (Register.OperatingMode == OperatingModeSettings::Single)
	{
		SamplingActive = true;
	}
	else
	{
		SamplingActive = false;
	}
}

AD7124::IOSettingsRegister AD7124::GetIOSettingsRegister()
{
	IOSettingsRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::IOCon1, ReturnData.Array);
	ReturnRegister.GPIOData0 = bitRead(ReturnData.Single, AD7124_IO_CTRL1_REG_GPIO_DAT1);
	ReturnRegister.GPIOData1 = bitRead(ReturnData.Single, AD7124_IO_CTRL1_REG_GPIO_DAT2);
	ReturnRegister.GPIOControl0 = bitRead(ReturnData.Single, AD7124_IO_CTRL1_REG_GPIO_CTRL1);
	ReturnRegister.GPIOControl1 = bitRead(ReturnData.Single, AD7124_IO_CTRL1_REG_GPIO_CTRL2);
	ReturnRegister.PowerDownSwitch = bitRead(ReturnData.Single, AD7124_IO_CTRL1_REG_PDSW);
	ReturnRegister.ExcitationCurrent0 = static_cast<ExcitationCurrentSettings>( (ReturnData.Single & AD7124_IO_CTRL1_REG_IOUT0_MASK) >> AD7124_IO_CTRL1_REG_IOUT0_SHIFT );
	ReturnRegister.ExcitationCurrent1 = static_cast<ExcitationCurrentSettings>( (ReturnData.Single & AD7124_IO_CTRL1_REG_IOUT1_MASK) >> AD7124_IO_CTRL1_REG_IOUT1_SHIFT );
	ReturnRegister.ExcitationOuput0 = static_cast<ExcitationCurrentOutputSettings>( (ReturnData.Single & AD7124_IO_CTRL1_REG_IOUT_CH0_MASK) >> AD7124_IO_CTRL1_REG_IOUT_CH0_SHIFT );
	ReturnRegister.ExcitationOuput1 = static_cast<ExcitationCurrentOutputSettings>( (ReturnData.Single & AD7124_IO_CTRL1_REG_IOUT_CH1_MASK) >> AD7124_IO_CTRL1_REG_IOUT_CH1_SHIFT );
	return ReturnRegister;
}

void AD7124::SetIOSettingsRegister(IOSettingsRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_IO_CTRL1_REG_GPIO_DAT1, NewSettings.GPIOData0);
	bitWrite(SendingData.Single, AD7124_IO_CTRL1_REG_GPIO_DAT2, NewSettings.GPIOData1);
	bitWrite(SendingData.Single, AD7124_IO_CTRL1_REG_GPIO_CTRL1, NewSettings.GPIOControl0);
	bitWrite(SendingData.Single, AD7124_IO_CTRL1_REG_GPIO_CTRL2, NewSettings.GPIOControl1);
	bitWrite(SendingData.Single, AD7124_IO_CTRL1_REG_PDSW, NewSettings.PowerDownSwitch);
	SendingData.Single |= static_cast<uint32_t>(NewSettings.ExcitationCurrent0) << AD7124_IO_CTRL1_REG_IOUT0_SHIFT;
	SendingData.Single |= static_cast<uint32_t>(NewSettings.ExcitationCurrent1) << AD7124_IO_CTRL1_REG_IOUT1_SHIFT;
	SendingData.Single |= static_cast<uint32_t>(NewSettings.ExcitationOuput0) << AD7124_IO_CTRL1_REG_IOUT_CH0_SHIFT;
	SendingData.Single |= static_cast<uint32_t>(NewSettings.ExcitationOuput1) << AD7124_IO_CTRL1_REG_IOUT_CH1_SHIFT;
	SendWriteCommand(Registers::IOCon1, SendingData.Array);
}

AD7124::IOBiasRegister AD7124::GetIOBiasRegister()
{
	IOBiasRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::IOCon2, ReturnData.Array);
	ReturnRegister.AIN7 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS7);
	ReturnRegister.AIN6 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS6);
	ReturnRegister.AIN5 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS5);
	ReturnRegister.AIN4 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS4);
	ReturnRegister.AIN3 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS3);
	ReturnRegister.AIN2 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS2);
	ReturnRegister.AIN1 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS1);
	ReturnRegister.AIN0 = bitRead(ReturnData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS0);
	return ReturnRegister;
}

void AD7124::SetIOBiasRegister(IOBiasRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS7, NewSettings.AIN7);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS6, NewSettings.AIN6);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS5, NewSettings.AIN5);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS4, NewSettings.AIN4);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS3, NewSettings.AIN3);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS2, NewSettings.AIN2);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS1, NewSettings.AIN1);
	bitWrite(SendingData.Single, AD7124_IO_CTRL2_REG_GPIO_VBIAS0, NewSettings.AIN0);
	SendWriteCommand(Registers::IOCon2, SendingData.Array);
}

AD7124::ErrorRegister AD7124::GetErrorRegister()
{
	ErrorRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::Error, ReturnData.Array);
	ReturnRegister.LDOCapacitor = bitRead(ReturnData.Single, AD7124_ERR_REG_LDO_CAP_ERR);
	ReturnRegister.Calibration = bitRead(ReturnData.Single, AD7124_ERR_REG_ADC_CAL_ERR);
	ReturnRegister.Convert = bitRead(ReturnData.Single, AD7124_ERR_REG_ADC_CONV_ERR);
	ReturnRegister.Saturation = bitRead(ReturnData.Single, AD7124_ERR_REG_ADC_SAT_ERR);
	ReturnRegister.OverAINP = bitRead(ReturnData.Single, AD7124_ERR_REG_AINP_OV_ERR);
	ReturnRegister.UnderAINP = bitRead(ReturnData.Single, AD7124_ERR_REG_AINP_UV_ERR);
	ReturnRegister.OverAINM = bitRead(ReturnData.Single, AD7124_ERR_REG_AINM_OV_ERR);
	ReturnRegister.UnderAINM = bitRead(ReturnData.Single, AD7124_ERR_REG_AINM_UV_ERR);
	ReturnRegister.RefDetect = bitRead(ReturnData.Single, AD7124_ERR_REG_REF_DET_ERR);
	ReturnRegister.DigitalLDO = bitRead(ReturnData.Single, AD7124_ERR_REG_DLDO_PSM_ERR);
	ReturnRegister.AnalogueLDO = bitRead(ReturnData.Single, AD7124_ERR_REG_ALDO_PSM_ERR);
	ReturnRegister.SPIIgnore = bitRead(ReturnData.Single, AD7124_ERR_REG_SPI_IGNORE_ERR);
	ReturnRegister.SPIClk = bitRead(ReturnData.Single, AD7124_ERR_REG_SPI_SLCK_CNT_ERR);
	ReturnRegister.SPIRead = bitRead(ReturnData.Single, AD7124_ERR_REG_SPI_READ_ERR);
	ReturnRegister.SPIWrite = bitRead(ReturnData.Single, AD7124_ERR_REG_SPI_WRITE_ERR);
	ReturnRegister.SPICRC = bitRead(ReturnData.Single, AD7124_ERR_REG_SPI_CRC_ERR);
	ReturnRegister.Memory = bitRead(ReturnData.Single, AD7124_ERR_REG_MM_CRC_ERR);
	ReturnRegister.ROM = bitRead(ReturnData.Single, AD7124_ERR_REG_ROM_CRC_ERR);
	return ReturnRegister;
}

AD7124::ErrorEnableRegister AD7124::GetErrorEnableRegister()
{
	ErrorEnableRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	SendReadCommand(Registers::Error_En, ReturnData.Array);
	ReturnRegister.ClockCountEnabled = bitRead(ReturnData.Single, AD7124_ERREN_REG_MCLK_CNT_EN);
	ReturnRegister.LDOCapacitorDisconnect = bitRead(ReturnData.Single, AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN);
	ReturnRegister.Calibration = bitRead(ReturnData.Single, AD7124_ERREN_REG_ADC_CAL_ERR_EN);
	ReturnRegister.Convert = bitRead(ReturnData.Single, AD7124_ERREN_REG_ADC_CONV_ERR_EN);
	ReturnRegister.Saturation = bitRead(ReturnData.Single, AD7124_ERREN_REG_ADC_SAT_ERR_EN);
	ReturnRegister.OverAINP = bitRead(ReturnData.Single, AD7124_ERREN_REG_AINP_OV_ERR_EN);
	ReturnRegister.UnderAINP = bitRead(ReturnData.Single, AD7124_ERREN_REG_AINP_UV_ERR_EN);
	ReturnRegister.OverAINM = bitRead(ReturnData.Single, AD7124_ERREN_REG_AINM_OV_ERR_EN);
	ReturnRegister.UnderAINM = bitRead(ReturnData.Single, AD7124_ERREN_REG_AINM_UV_ERR_EN);
	ReturnRegister.RefDetect = bitRead(ReturnData.Single, AD7124_ERREN_REG_REF_DET_ERR_EN);
	ReturnRegister.DigitalLDOTest = bitRead(ReturnData.Single, AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN);
	ReturnRegister.DigitalLDO = bitRead(ReturnData.Single, AD7124_ERREN_REG_DLDO_PSM_ERR_ERR);
	ReturnRegister.AnalogueLDOTest = bitRead(ReturnData.Single, AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN);
	ReturnRegister.AnalogueLDO = bitRead(ReturnData.Single, AD7124_ERREN_REG_ALDO_PSM_ERR_EN);
	ReturnRegister.SPIIgnore = bitRead(ReturnData.Single, AD7124_ERREN_REG_SPI_IGNORE_ERR_EN);
	ReturnRegister.SPIClk = bitRead(ReturnData.Single, AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN);
	ReturnRegister.SPIRead = bitRead(ReturnData.Single, AD7124_ERREN_REG_SPI_READ_ERR_EN);
	ReturnRegister.SPIWrite = bitRead(ReturnData.Single, AD7124_ERREN_REG_SPI_WRITE_ERR_EN);
	ReturnRegister.SPICRC = bitRead(ReturnData.Single, AD7124_ERREN_REG_SPI_CRC_ERR_EN);
	ReturnRegister.Memory = bitRead(ReturnData.Single, AD7124_ERREN_REG_MM_CRC_ERR_EN);
	ReturnRegister.ROM = bitRead(ReturnData.Single, AD7124_ERREN_REG_ROM_CRC_ERR);
	ReturnRegister.LDOCheck = static_cast<LDOCheckSettings>( (ReturnData.Single & AD7124_ERREN_REG_LDO_CAP_CHK_MASK) >> AD7124_ERREN_REG_LDO_CAP_CHK_SHIFT );
	UseCRC = ReturnRegister.SPICRC;
	return ReturnRegister;
}

void AD7124::SetErrorEnableRegister(ErrorEnableRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_ERREN_REG_MCLK_CNT_EN, NewSettings.ClockCountEnabled);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN, NewSettings.LDOCapacitorDisconnect);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ADC_CAL_ERR_EN, NewSettings.Calibration);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ADC_CONV_ERR_EN, NewSettings.Convert);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ADC_SAT_ERR_EN, NewSettings.Saturation);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_AINP_OV_ERR_EN, NewSettings.OverAINP);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_AINP_UV_ERR_EN, NewSettings.UnderAINP);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_AINM_OV_ERR_EN, NewSettings.OverAINM);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_AINM_UV_ERR_EN, NewSettings.UnderAINM);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_REF_DET_ERR_EN, NewSettings.RefDetect);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN, NewSettings.DigitalLDOTest);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_DLDO_PSM_ERR_ERR, NewSettings.DigitalLDO);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN, NewSettings.AnalogueLDOTest);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ALDO_PSM_ERR_EN, NewSettings.AnalogueLDO);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_SPI_IGNORE_ERR_EN, NewSettings.SPIIgnore);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN, NewSettings.SPIClk);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_SPI_READ_ERR_EN, NewSettings.SPIRead);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_SPI_WRITE_ERR_EN, NewSettings.SPIWrite);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_SPI_CRC_ERR_EN, NewSettings.SPICRC);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_MM_CRC_ERR_EN, NewSettings.Memory);
	bitWrite(SendingData.Single, AD7124_ERREN_REG_ROM_CRC_ERR, NewSettings.ROM);
	SendingData.Single |= static_cast<uint32_t>(NewSettings.LDOCheck) << AD7124_ERREN_REG_LDO_CAP_CHK_SHIFT;
	SendWriteCommand(Registers::Error_En, SendingData.Array);
	UseCRC = NewSettings.SPICRC;
}

AD7124::ChannelRegister AD7124::GetChannelRegister(uint8_t Channel)
{
	ChannelRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	Registers RegisterToUse = ConvertChannelToRegister(Channel);
	SendReadCommand(RegisterToUse, ReturnData.Array);
	ReturnRegister.Enable = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.Configuration = static_cast<uint8_t>( (ReturnData.Single & AD7124_CH_MAP_REG_SETUP_MASK) >> AD7124_CH_MAP_REG_SETUP_SHIFT );
	ReturnRegister.Positive = static_cast<AnalogueInputSettings>( (ReturnData.Single & AD7124_CH_MAP_REG_AINP_MASK) >> AD7124_CH_MAP_REG_AINP_SHIFT );
	ReturnRegister.Negative = static_cast<AnalogueInputSettings>( (ReturnData.Single & AD7124_CH_MAP_REG_AINM_MASK) >> AD7124_CH_MAP_REG_AINM_SHIFT );
	return ReturnRegister;
}

void AD7124::SetChannelRegister(uint8_t Channel, ChannelRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_CH_MAP_REG_CH_ENABLE, NewSettings.Enable);
	//Serial.print("<CH>(");
	//Print_uint16(SendingData.Single);
	//Serial.print(",");
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Configuration) << AD7124_CH_MAP_REG_SETUP_SHIFT;
	//Print_uint16(SendingData.Single);
	//Serial.print(",");
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Positive) << AD7124_CH_MAP_REG_AINP_SHIFT;
	//Print_uint16(SendingData.Single);
	//Serial.print(",");
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Negative) << AD7124_CH_MAP_REG_AINM_SHIFT;
	//Print_uint16(SendingData.Single);
	//Serial.print(")\n");
	Registers RegisterToUse = ConvertChannelToRegister(Channel);
	SendWriteCommand(RegisterToUse, SendingData.Array);
}

AD7124::ConfigurationRegister AD7124::GetConfigurationRegister(uint8_t Configuration)
{
	ConfigurationRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	Registers RegisterToUse = ConvertConfigurationToRegister(Configuration);
	SendReadCommand(RegisterToUse, ReturnData.Array);
	ReturnRegister.Bipolar = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.BufferREFP = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.BufferREFN = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.BufferAINP = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.BufferAINM = bitRead(ReturnData.Single, AD7124_CH_MAP_REG_CH_ENABLE);
	ReturnRegister.Burnout = static_cast<BurnoutSettings>( (ReturnData.Single & AD7124_CFG_REG_BURNOUT_MASK) >> AD7124_CFG_REG_BURNOUT_SHIFT );
	ReturnRegister.Reference = static_cast<ReferenceSettings>( (ReturnData.Single & AD7124_CFG_REG_REF_SEL_MASK) >> AD7124_CFG_REG_REF_SEL_SHIFT );
	ReturnRegister.Gain = static_cast<GainSettings>( (ReturnData.Single & AD7124_CFG_REG_PGA_MASK) >> AD7124_CFG_REG_PGA_SHIFT );
	return ReturnRegister;
}

void AD7124::SetConfigurationRegister(uint8_t Configuration, ConfigurationRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[2];
		uint16_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_CFG_REG_BIPOLAR, NewSettings.Bipolar);
	bitWrite(SendingData.Single, AD7124_CFG_REG_REF_BUFP, NewSettings.BufferREFP);
	bitWrite(SendingData.Single, AD7124_CFG_REG_REF_BUFM, NewSettings.BufferREFN);
	bitWrite(SendingData.Single, AD7124_CFG_REG_AIN_BUFP, NewSettings.BufferAINP);
	bitWrite(SendingData.Single, AD7124_CFG_REG_AINN_BUFM, NewSettings.BufferAINM);
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Burnout) << AD7124_CFG_REG_BURNOUT_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Reference) << AD7124_CFG_REG_REF_SEL_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.Gain) << AD7124_CFG_REG_PGA_SHIFT;
	Registers RegisterToUse = ConvertConfigurationToRegister(Configuration);
	SendWriteCommand(RegisterToUse, SendingData.Array);
}

void AD7124::Print_uint8(uint8_t value)
{
	for(int8_t i = 7; i >= 0; i--)
	{
	  Serial.write( (value & (1 << i) ) ? '1' : '0');
	}
}

void AD7124::Print_uint16(uint16_t value)
{
	for(int8_t i = 15; i >= 0; i--)
	{
	  Serial.write( (value & (1 << i) ) ? '1' : '0');
	}
}

AD7124::FilterRegister AD7124::GetFilterRegister(uint8_t Filter)
{
	FilterRegister ReturnRegister;
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	Registers RegisterToUse = ConvertFilterToRegister(Filter);
	SendReadCommand(RegisterToUse, ReturnData.Array);
	ReturnRegister.Reject5060Hz = bitRead(ReturnData.Single, AD7124_FILT_REG_REJ60);
	ReturnRegister.SingleCycle = bitRead(ReturnData.Single, AD7124_FILT_REG_SINGLE_CYCLE);
	ReturnRegister.FilterType = static_cast<FilterTypeSettings>( (ReturnData.Single & AD7124_FILT_REG_FILTER_MASK) >> AD7124_FILT_REG_FILTER_SHIFT );
	ReturnRegister.PostFilterType = static_cast<PostFilerTypeSettings>( (ReturnData.Single & AD7124_FILT_REG_POST_FILTER_MASK) >> AD7124_FILT_REG_POST_FILTER_SHIFT );
	ReturnRegister.DataRate = static_cast<uint16_t>( (ReturnData.Single & AD7124_FILT_REG_FS_MASK) >> AD7124_FILT_REG_FS_SHIFT );
	//UpdateInternalFilter(Filter, ReturnRegister);
	return ReturnRegister;
}

void AD7124::SetFilterRegister(uint8_t Filter, FilterRegister NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = 0;
	bitWrite(SendingData.Single, AD7124_FILT_REG_REJ60, NewSettings.Reject5060Hz);
	bitWrite(SendingData.Single, AD7124_FILT_REG_SINGLE_CYCLE, NewSettings.SingleCycle);
	SendingData.Single |= static_cast<uint16_t>(NewSettings.DataRate) << AD7124_FILT_REG_FS_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.FilterType) << AD7124_FILT_REG_FILTER_SHIFT;
	SendingData.Single |= static_cast<uint16_t>(NewSettings.PostFilterType) << AD7124_FILT_REG_POST_FILTER_SHIFT;
	Registers RegisterToUse = ConvertFilterToRegister(Filter);
	//UpdateInternalFilter(Filter, ReturnRegister);
	SendWriteCommand(RegisterToUse, SendingData.Array);
}

/*
void AD7124::UpdateInternalFilter(uint8_t Filter, FilterRegister Register)
{
	DataRates[Configuration] = Register.DataRate;
}
*/

uint32_t AD7124::GetOffsetRegister(uint8_t Offset)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	Registers RegisterToUse = ConvertOffsetToRegister(Offset);
	SendReadCommand(RegisterToUse, ReturnData.Array);
	return ReturnData.Single;
}

void AD7124::SetOffsetRegister(uint8_t Offset, uint32_t NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = NewSettings;
	Registers RegisterToUse = ConvertOffsetToRegister(Offset);
	SendWriteCommand(RegisterToUse, SendingData.Array);
}

uint32_t AD7124::GetGainRegister(uint8_t Gain)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle ReturnData;
	ReturnData.Single = 0;
	Registers RegisterToUse = ConvertGainToRegister(Gain);
	SendReadCommand(RegisterToUse, ReturnData.Array);
	return ReturnData.Single;
}

void AD7124::SetGainRegister(uint8_t Gain, uint32_t NewSettings)
{
	union ArrayToSingle
	{
		uint8_t Array[3];
		uint32_t Single;
	};
	ArrayToSingle SendingData;
	SendingData.Single = NewSettings;
	Registers RegisterToUse = ConvertGainToRegister(Gain);
	SendWriteCommand(RegisterToUse, SendingData.Array);
}

AD7124::Registers AD7124::ConvertChannelToRegister(uint8_t Channel)
{
	uint8_t RegistryIndex = Channel + static_cast<uint8_t>(Registers::Channel_0);
	if ( ( RegistryIndex < static_cast<uint8_t>(Registers::Channel_0) ) || ( RegistryIndex > static_cast<uint8_t>(Registers::Channel_15) ) )
	{
		return Registers::REG_NO;
	}
	return static_cast<Registers>( RegistryIndex );
}

AD7124::Registers AD7124::ConvertConfigurationToRegister(uint8_t Configuration)
{
	uint8_t RegistryIndex = Configuration + static_cast<uint8_t>(Registers::Config_0);
	if ( ( RegistryIndex < static_cast<uint8_t>(Registers::Config_0) ) || ( RegistryIndex > static_cast<uint8_t>(Registers::Config_7) ) )
	{
		return Registers::REG_NO;
	}
	return static_cast<Registers>( RegistryIndex );
}

AD7124::Registers AD7124::ConvertFilterToRegister(uint8_t Filter)
{
	uint8_t RegistryIndex = Filter + static_cast<uint8_t>(Registers::Filter_0);
	if ( ( RegistryIndex < static_cast<uint8_t>(Registers::Filter_0) ) || ( RegistryIndex > static_cast<uint8_t>(Registers::Filter_7) ) )
	{
		return Registers::REG_NO;
	}
	return static_cast<Registers>( RegistryIndex );
}

AD7124::Registers AD7124::ConvertOffsetToRegister(uint8_t Offset)
{
	uint8_t RegistryIndex = Offset + static_cast<uint8_t>(Registers::Offset_0);
	if ( ( RegistryIndex < static_cast<uint8_t>(Registers::Offset_0) ) || ( RegistryIndex > static_cast<uint8_t>(Registers::Offset_7) ) )
	{
		return Registers::REG_NO;
	}
	return static_cast<Registers>( RegistryIndex );
}

AD7124::Registers AD7124::ConvertGainToRegister(uint8_t Gain)
{
	uint8_t RegistryIndex = Gain + static_cast<uint8_t>(Registers::Gain_0);
	if ( ( RegistryIndex < static_cast<uint8_t>(Registers::Gain_0) ) || ( RegistryIndex > static_cast<uint8_t>(Registers::Gain_7) ) )
	{
		return Registers::REG_NO;
	}
	return static_cast<Registers>( RegistryIndex );
}

/*
uint32_t AD7124::EstimateSampletime(uint16_t DataRate)
{
	float ClockDivider = 1.0;
	if (PowerMode == PowerModeSettings::MidPower)
	{
		ClockDivider = 4.0;
	}
	else if (PowerMode == PowerModeSettings::LowPower)
	{
		ClockDivider = 8.0;
	}
	SampleTimeMicroSeconds = (32.0 * DataRate) / (InternalClockSpeed * ClockDivider);
}
*/

void AD7124::SetCommunicationsBuffer(Registers Register, bool ReadWrite)
{
	if (Register == Registers::REG_NO)
	{
		return;
	}
	SPIBuffer[0] = 0;
	bitWrite(SPIBuffer[0], AD7124_COMM_REG_RW, ReadWrite);
	SPIBuffer[0] |= (static_cast<uint8_t>(Register) & AD7124_COMM_REG_RS_MASK);
}

void AD7124::SendWriteCommand(Registers Register, uint8_t* DataToSend)
{
	if (Register == Registers::REG_NO)
	{
		Serial.print("<AD7124>(Error in register write. No register given.)\n");
		return;
	}
	uint8_t DataBytes = RegisterDictionary[static_cast<uint8_t>(Register)].size;
	uint8_t BytesToWrite = 1 + DataBytes;
	WipeSPIBuffer();
	SetCommunicationsBuffer(Register,false);
	//Serial.print("<L>(");
	for (uint8_t Index = 0; Index < DataBytes; ++Index)
	{
		SPIBuffer[Index + 1] = DataToSend[DataBytes - Index - 1];
		//Print_uint8(DataToSend[Index]);
		//Serial.print(",");
	}
	//Serial.print(")\n");
	if (UseCRC)
	{
		SPIBuffer[BytesToWrite] = ComputeCRC8(SPIBuffer,BytesToWrite);
		BytesToWrite++;
	}
	SPIWrite(BytesToWrite);
}

void AD7124::SendReadCommand(Registers Register, uint8_t* DataToCollect)
{
	if (Register == Registers::REG_NO)
	{
		return;
	}
	uint8_t DataBytes = RegisterDictionary[static_cast<uint8_t>(Register)].size;
	if ( (Register == Registers::Data) && UseStatus)
	{
		DataBytes++;
	}
	uint8_t BytesToRead = 1 + DataBytes;
	if (UseCRC)
	{
		BytesToRead++;
	}
	WipeSPIBuffer();
	SetCommunicationsBuffer(Register,true);
	SPIRead(BytesToRead);
	if (UseCRC)
	{
		CRCSuccess = (SPIBuffer[BytesToRead-1] == ComputeCRC8(SPIBuffer,BytesToRead-1));
	}
	for (uint8_t Index = 0; Index < DataBytes; ++Index)
	{
		DataToCollect[DataBytes - Index - 1] = SPIBuffer[Index + 1];
	}
}

void AD7124::WipeSPIBuffer()
{
	for (size_t Index = 0; Index < AD7124SPIBufferSize; Index++)
	{
		SPIBuffer[Index] = 0;
	}
}

void AD7124::SPIRead(uint8_t bytesNumber)
{
	uint8_t count = 0;
	//Serial.print("<ADCR>(");
	//Serial.print(bytesNumber);
	//Serial.print(":");
	//Print_uint8(SPIBuffer[0]);
	//Serial.print(",");
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	for(count = 0;count < bytesNumber;count++)
	{
		SPIBuffer[count] =  SPI.transfer(SPIBuffer[count]);
	}
	SPI.endTransaction();
	digitalWrite(SelectPin, HIGH);
	//for (uint8_t Index = 1; Index < bytesNumber; Index++)
	//{
		//Print_uint8(SPIBuffer[Index]);
		//Serial.print(",");
	//}
	//Serial.print(")\n");
}

void AD7124::SPIWrite(uint8_t bytesNumber)
{
	uint8_t count = 0;
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	//Serial.print("<ADCW>(");
	//Serial.print(bytesNumber);
	//Serial.print(":");
	//for (uint8_t Index = 0; Index < bytesNumber; Index++)
	//{
		//Print_uint8(SPIBuffer[Index]);
		//Serial.print(",");
	//}
	//Serial.print(")\n");
	for(count = 0;count < bytesNumber;count++)
	{
		SPI.transfer(SPIBuffer[count]);  // write instruction
	}
	SPI.endTransaction();
	digitalWrite(SelectPin, HIGH);
}

SPISettings* AD7124::GetSPISettings()
{
	return &ConnectionSettings;
}
