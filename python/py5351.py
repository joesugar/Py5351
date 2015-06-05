import i2cpy

class Py5351(object):
    ''' Class to control an Si5351 using the I2C Stick interface.
    '''
    def __init__(self, xtal_freq = 25000000.0, xtal_corr = 0.0):
        ''' Initialization routine for the py5351 class.
        '''
        # Define any required constants.
        # Clock sources.
        self.CLK_SRC_PLLA = 0
        self.CLK_SRC_PLLB = 1
        
        # Clock drive.
        self.CLK_DRIVE_2MA = 0
        self.CLK_DRIVE_4MA = 1
        self.CLK_DRIVE_6MA = 2
        self.CLK_DRIVE_8MA = 3
        
        # Xtal load capacitance
        self.XTAL_LOAD_6PF  = 0x52
        self.XTAL_LOAD_8PF  = 0x92     
        self.XTAL_LOAD_10PF = 0xd2
        
        # Initialization code.
        self.i2c = i2cpy.core.Py2CStick(deviceAddress = 0x60)
        self.xtal_freq = float(xtal_freq)
        self.xtal_corr = float(xtal_corr)
        
        self.clk0_set_clock_source(self.CLK_SRC_PLLA)
        self.clk1_set_clock_source(self.CLK_SRC_PLLA)
        self.clk2_set_clock_source(self.CLK_SRC_PLLB)

        self.clk0_reset()
        self.clk1_reset()
        self.clk2_reset()
        
        self.plla_reset()
        self.pllb_reset()

    # Properties
    @property
    def xtal_corr(self):
        ''' Property encapsulating the crystal correction factor.
        '''
        return self.__xtal_corr
      
    @xtal_corr.setter
    def xtal_corr(self, xtal_corr):
        ''' Setter encapsulating the crystal correction factor.
        '''
        self.__xtal_corr = float(xtal_corr)
     
    @property
    def xtal_freq(self):
        ''' Property encapsulating the crystal frequency.
        '''
        return self.__xtal_freq
        
    @xtal_freq.setter
    def xtal_freq(self, xtal_freq):
        ''' Setter encapsulating the crystal frequency.
        '''
        self.__xtal_freq = float(xtal_freq)
    
    @xtal_load.setter
    def xtal_load(self, xtal_load):
        ''' Set the crystal load capacitance.
        '''
        assert (xtal_load == self.XTAL_LOAD_6PF) or \
            (xtal_load == self.XTAL_LOAD_8PF) or \
            (xtal_load == self.XTAL_LOAD_10PF), \
            "Invalid load capacitance."
        self.write_to_register(183, xtal_load)
            
    # Register access routines                    
    def write_to_register(self, reg_number, value):
        ''' Write a value to an Si5351 register.
        '''
        self.i2c.WriteRegister(reg_number, value)
        read = self.i2c.ReadRegister(reg_number, 1)
        assert read == value, "Error writing to register " + str(reg_number) + "." 
    
    def read_from_register(self, reg_number):
        ''' Read a value from an Si5351 register.
        '''
        return self.i2c.ReadRegister(reg_number, 1)
                                 
    # PLL routines                    
    def pll_calc_pvals(self, pll_freq_, xtal_freq_):
        ''' Calculate pll p values
            The xtal_freq needs to be the real crystal frequency,
            including any correction factor.
        '''
        pll_freq = float(pll_freq_)
        xtal_freq = float(xtal_freq_)
        
        a = int(pll_freq / xtal_freq)
        b_over_c = (pll_freq / xtal_freq) - a
        c = (1 << 20) - 1
        b = int(b_over_c * c)
        
        p1 = 128 * a + int(128 * float(b) / float(c)) - 512
        p2 = 128 * b - c * int(128 * float(b) / float(c))
        p3 = c 
        return (p1, p2, p3)

    # PLL reset
    def plla_reset(self):
        ''' Reset plla.
        '''
        self.i2c.WriteRegister(177, 0x20)

    def pllb_reset(self):
        ''' Reset pllb
        '''
        self.i2c.WriteRegister(177, 0x80)
    
    # PLL write to registers
    def plla_write_registers(self, p_values):
        ''' Write the plla registers.
        '''
        msna_p1, msna_p2, msna_p3 = p_values
        self.write_to_register(26, (msna_p3 >> 8) & 0x00FF)
        self.write_to_register(27, (msna_p3 >> 0) & 0x00FF)
        self.write_to_register(28, (msna_p1 >> 16) & 0x0003)
        self.write_to_register(29, (msna_p1 >> 8) & 0x00FF)
        self.write_to_register(30, (msna_p1 >> 0) & 0x00FF)
        self.write_to_register(31, ((msna_p3 >> 12) & 0x00F0) | ((msna_p2 >> 16) & 0x000F))
        self.write_to_register(32, (msna_p2 >> 8) & 0x00FF)
        self.write_to_register(33, (msna_p2 >> 0) & 0x00FF)    

    def pllb_write_registers(self, p_values):
        ''' Write the plla registers.
        '''
        msna_p1, msna_p2, msna_p3 = p_values
        self.write_to_register(34, (msna_p3 >> 8) & 0x00FF)
        self.write_to_register(35, (msna_p3 >> 0) & 0x00FF)
        self.write_to_register(36, (msna_p1 >> 16) & 0x0003)
        self.write_to_register(37, (msna_p1 >> 8) & 0x00FF)
        self.write_to_register(38, (msna_p1 >> 0) & 0x00FF)
        self.write_to_register(39, ((msna_p3 >> 12) & 0x00F0) | ((msna_p2 >> 16) & 0x000F))
        self.write_to_register(40, (msna_p2 >> 8) & 0x00FF)
        self.write_to_register(41, (msna_p2 >> 0) & 0x00FF)    

    # Clock routines
    def clk_calc_pvals(self, final_freq_):
        ''' Calculate clock p values.
        '''
        # Calculate the maximum pll frequency that can be used 
        # consistent with its frequency range.
        final_freq = float(final_freq_)
        if final_freq < 150000000.0:
            pll_div  = float(int(900000000.0 / final_freq))
        else:
            pll_div = 4.0
            
        if pll_div > 127.0:
            pll_div = 127.0
        
        pll_freq = final_freq * pll_div
        
        # Calculate multisynth values for the final divider.
        a  = pll_div
        b = 0
        c = 1
        
        p1 = 128 * int(pll_div) - 512
        p2 = 0
        p3 = 1
        return pll_freq, (p1, p2, p3)

    def corrected_xtal_freq(xtal_freq_):
        ''' Calculate a xtal frequency that takes the
            correction factor into account.
        '''
        xtal_freq = float(xtal_freq_)
        return xtal_freq * (1.0 + xtal_corr / 10000000.0)

    # clock power up        
    def clk0_powerup(self):
        value = self.read_from_register(16)
        self.write_to_register(16, value & 0x7f)
    
    def clk1_powerup(self):
        value = self.read_from_register(17)
        self.write_to_register(17, value & 0x7f)
        
    def clk2_powerup(self):
        value = self.read_from_register(18)
        self.write_to_register(18, value & 0x7f)

    # clock power down
    def clk0_powerdown(self):
        value = self.read_from_register(16)
        self.write_to_register(16, value | 0x80)
    
    def clk1_powerdown(self):
        value = self.read_from_register(17)
        self.write_to_register(17, value | 0x80)
        
    def clk2_powerdown(self):
        value = self.read_from_register(18)
        self.write_to_register(18, value | 0x80)
    
    # set clock sources
    def clk0_set_clock_source(self, source):
        assert (source == self.CLK_SRC_PLLA) or (source == self.CLK_SRC_PLLB), \
            "Invalid clock source."
        value = self.read_from_register(16)
        if source == self.CLK_SRC_PLLA:
            self.write_to_register(16, value & 0xdf)
        else:
            self.write_to_register(16, value | 0x20)

    def clk1_set_clock_source(self, source):
        assert (source == self.CLK_SRC_PLLA) or (source == self.CLK_SRC_PLLB), \
            "Invalid clock source."
        value = self.read_from_register(17)
        if source == self.CLK_SRC_PLLA:
            self.write_to_register(17, value & 0xdf)
        else:
            self.write_to_register(17, value | 0x20)

    def clk2_set_clock_source(self, source):
        assert (source == self.CLK_SRC_PLLA) or (source == self.CLK_SRC_PLLB), \
            "Invalid clock source."
        value = self.read_from_register(18)
        if source == self.CLK_SRC_PLLA:
            self.write_to_register(18, value & 0xdf)
        else:
            self.write_to_register(18, value | 0x20)                    
      
    # clock drive strength
    def clk0_set_clock_drive(self, clk_drive):     
        value = self.read_from_register(16)
        self.write_to_register((value & 0xfc) | (clk_drive & 0x03))
                             
    def clk1_set_clock_drive(self, clk_drive):     
        value = self.read_from_register(17)
        self.write_to_register((value & 0xfc) | (clk_drive & 0x03))
                             
    def clk2_set_clock_drive(self, clk_drive):     
        value = self.read_from_register(18)
        self.write_to_register((value & 0xfc) | (clk_drive & 0x03))
            
    # clock reset                             
    def clk0_reset(self):
        ''' Reset clock 0.
        '''
        # Power down the clock, then power it back up.
        value = self.read_from_register(16)
        self.write_to_register(16, value | 0x80)
        self.write_to_register(16, (value & 0x7f) | 0x0c)
            
    def clk1_reset(self):
        ''' Reset clock 0.
        '''
        value = self.read_from_register(17)
        self.write_to_register(17, value | 0x80)
        self.write_to_register(17, (value & 0x7f) | 0x0c)
        
    def clk2_reset(self):
        ''' Reset clock 0.
        '''
        # Power down the clock, then power it back up.
        value = self.read_from_register(18)
        self.write_to_register(18, value | 0x80)
        self.write_to_register(18, (value & 0x7f) | 0x0c)

    # clock phase offset
    def clk0_set_phase_offset(self, offset):
        ''' Set the clock phase offset.
        '''
        assert (offset >= 0) and (offset <= 127), \
            "Clock phase offset out of range."
        self.write_to_register(165, offset)

    def clk1_set_phase_offset(self, offset):
        ''' Set the clock phase offset.
        '''
        assert (offset >= 0) and (offset <= 127), \
            "Clock phase offset out of range."
        self.write_to_register(166, offset)
        
    def clk2_set_phase_offset(self, offset):
        ''' Set the clock phase offset.
        '''
        assert (offset >= 0) and (offset <= 127), \
            "Clock phase offset out of range."
        self.write_to_register(167, offset)
                         
    # clock write registers
    def clk0_write_registers(self, p_values):
        ''' Write the clock 0 registers.
        '''
        p1, p2, p3 = p_values
        self.write_to_register(42, (p3 >> 8) & 0x00FF)
        self.write_to_register(43, (p3 >> 0) & 0x00FF)
        self.write_to_register(44, (p1 >> 16) & 0x0003)
        self.write_to_register(45, (p1 >> 8) & 0x00FF)
        self.write_to_register(46, (p1 >> 0) & 0x00FF)
        self.write_to_register(47, ((p3 >> 12) & 0x00F0) | ((p2 >> 16) & 0x000F))
        self.write_to_register(48, (p2 >> 8) & 0x00FF)
        self.write_to_register(49, (p2 >> 0) & 0x00FF)

    def clk1_write_registers(self, p_values):
        ''' Write the clock 0 registers.
        '''
        p1, p2, p3 = p_values
        self.write_to_register(50, (p3 >> 8) & 0x00FF)
        self.write_to_register(51, (p3 >> 0) & 0x00FF)
        self.write_to_register(52, (p1 >> 16) & 0x0003)
        self.write_to_register(53, (p1 >> 8) & 0x00FF)
        self.write_to_register(54, (p1 >> 0) & 0x00FF)
        self.write_to_register(55, ((p3 >> 12) & 0x00F0) | ((p2 >> 16) & 0x000F))
        self.write_to_register(56, (p2 >> 8) & 0x00FF)
        self.write_to_register(57, (p2 >> 0) & 0x00FF)        
        
    def clk2_write_registers(self, p_values):
        ''' Write the clock 0 registers.
        '''
        p1, p2, p3 = p_values
        self.write_to_register(58, (p3 >> 8) & 0x00FF)
        self.write_to_register(59, (p3 >> 0) & 0x00FF)
        self.write_to_register(60, (p1 >> 16) & 0x0003)
        self.write_to_register(61, (p1 >> 8) & 0x00FF)
        self.write_to_register(62, (p1 >> 0) & 0x00FF)
        self.write_to_register(63, ((p3 >> 12) & 0x00F0) | ((p2 >> 16) & 0x000F))
        self.write_to_register(64, (p2 >> 8) & 0x00FF)
        self.write_to_register(65, (p2 >> 0) & 0x00FF)
                        
    def clk0_set_freq(self, freq = 10000000.0):
        ''' Set the si5351 frequency.
        '''
        # Initialize needed values.
        final_freq = float(freq)
        
        # Calculate multisynth values for the final divider.
        pll_freq, clk_pvals = self.clk_calc_pvals(final_freq)
        
        # Calculate multisynth values for the pll.
        # This calculation takes the specified crystal correction
        # factor into account.
        xtal_freq = corrected_xtal_freq(self.xtal_freq)
        pll_pvals = self.pll_calc_pvals(pll_freq, xtal_freq)  
        
        # Initialize the clock
        self.clk0_reset()
        self.clk1_reset()
        
        # Write the clock 0 registers.
        self.clk0_write_registers(clk_pvals)
        self.clk1_write_registers(clk_pvals)
        self.clk1_set_phase_offset(pll_freq / final_freq)
        
        # Write the plla registers.
        self.plla_write_registers(pll_pvals)

        # Reset the pll
        self.plla_reset()
    
    def clk2_set_freq(self, freq = 10000000.0):
        ''' Set the si5351 frequency.
        '''
        # Initialize needed values.
        final_freq = float(freq)
        
        # Calculate multisynth values for the final divider.
        pll_freq, clk_pvals = self.clk_calc_pvals(final_freq)
        
        # Calculate multisynth values for the pll.
        xtal_freq = corrected_xtal_freq(self.xtal_freq)
        pll_pvals = self.pll_calc_pvals(pll_freq, xtal_freq)  
        
        # Initialize the clock
        self.clk2_reset()
        
        # Write the clock 0 registers.
        self.clk2_write_registers(clk_pvals)
        
        # Write the plla registers.
        self.pllb_write_registers(pll_pvals)
        
        # Reset the pll
        self.pllb_reset()
                    
if __name__ == "__main__":
    py5351 = Py5351()
    py5351.clk0_set_freq(10000000.0)
    py5351.clk2_set_freq( 5000000.0)
    
    
