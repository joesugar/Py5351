/*
TrinketI2C is a port of the I2C Stick software with modifications
allowing it to be built using the Arduino IDE and run on the 
AdaFruit Trinket.

Copyright (c) 2015 Joseph Consugar
All rights reserved.

TrinketI2C is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

TrinketI2C is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with TrinketI2C. If not, see
<http://www.gnu.org/licenses/>.
*/

#ifndef TrinketI2C_H_
#define TrinketI2C_H_

class TrinketI2C
{
	private:
	public:
		TrinketI2C();
		void begin();
};

extern TrinketI2C TI2C; // instance that user can use

#endif
