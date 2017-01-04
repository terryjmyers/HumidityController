# HumidityController
Whole House Humidity Controller Replacement

I was fed up with the crappy mechanical humidity controller that came with my house.  So I made one from an accurate DHT22 Humidity/Temperature sensor.  Its controlled by an arduino uC, utilized a 4x20 LCD screen.  Displays humidity, temperature, and heat index.  Utilized a rotary encoder as the human interface.

Pressing the Rotary encoder pushbutton or rotating it turns the screens on and makes no other changes for about 500ms.  Rotating the rotary encoder while the screen is on changes the humidity setpoint.  Pushbuton the pushbutton turns the control ON/OFF.  Long pressing the pushbutton opens a menu that you can change the deadband(stored in EEPROM), temperature units (stored in EEPROM), and access a help menu which displays some text.

When the humidity is above the setpoint, the relay turns off.  When the humidity is below the setpoint minus the configured deadband the SSR is turned on.  The SSR closes a contact on the 15V RMS AC signal from the whole house humidifier, completing the connection and turns on the humdifier on the air handler.

The unit is powered from a 5V power supply which is powered from a 5V "USB" charger that I mounted inside the air handler.  I ran wires up to the humidity controllers.  If I had to do it again, I would have purchased a high voltage DC buck converter, and ran the 2 wires(red and black usualy) 24V AC from the thermostat  just to power the unit through a simple diode bridge-> capacitory> high voltage buck converter.  The high voltage version is required (at least in mine) as the 24V AC was really 29V AC which would have been 39V Dc when rectified, too high for the and the cheapo 35V buck converters from aliexpress.  But you can buy the cheapo HV version of the chip.

Features:

1. Configurable deadband control stored to EEPROM
2. LCD screen turns on with any input, but does not honor any input for 500ms, to allow the screen to turn on before making any changes
3. LCd screen turns off after 10 seconds.
