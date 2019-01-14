# m90e26
Primarily to provide support for the Atmel/Microchip M90E26 chip as used on the WhatNick DIN Rail Energy Monitor

  https://github.com/whatnick/din_meter_atm90e26
  
  https://github.com/whatnick/din_power_atm90e26
  
  Used in the firmware for the above device to integrate with the IRMACOS cloud service

#MODE command syntax
MODE		{function}		[p0			[p1			[p2			[p3]]]]
   			1=L-gain		Ch=0/1		1/4/8/16/24
   			2=N-gain		Ch=0/1		1/2/4
   			3=SoftReset		Ch=0/1
   			4=ReCalibrate	Ch=0/1
   			5=Contrast		Min<=255	Max<=255
   			6=Display		Ch0=0/1/2	Ch1=0/1/2	
   			7=Blanking		Period<=255