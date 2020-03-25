
int slic_load_settings(struct si3226x_slic *slic)
{
	int rc = 0;
	struct si3226x_line *lines = slic->lines;
	
	rc = slic_unlock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 47, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 80, 0x2F);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 764, 0x0020C480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 768, 0x051EB82A);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 767, 0x03D70A20);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 915, 0x0FFF0000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 916, 0x01999A00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 919, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 920, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 970, 0x00800000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1004, 0x00F18900);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1005, 0x00809D80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1006, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1007, 0x01C00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1540, 0x00400000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1541, 0x00400000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1542, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1545, 0x00500000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1546, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1547, 0x00500000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1553, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1554, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1558, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1560, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1585, 0x00300000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1586, 0x00300000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1587, 0x00100000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1588, 0x00FFC000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1589, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1590, 0x0FDA4000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 759, 0x07FEB800);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 756, 0x005B05B2);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 73, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 967, 0x03A2E8BA);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1018, 0x03000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1017, 0x05000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1013, 0x01000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1012, 0x03700000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1011, 0x04B80200);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1010, 0x00823000);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 68, 0x60);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 98, 0x80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 533, 0x71EB851);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 626, 0x723F235);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 627, 0x57A9804);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 918, 0x0036000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1616, 0x1100000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 973, 0xFFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 975, 0xE49BA5);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 516, 0x10038D);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 513, 0x4EDDB9);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 514, 0x0806D6);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1641, 0x200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1643, 0x000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1565, 0xC00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 750, 0x206280);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 971, 0x1F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 972, 0x51EB80);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 47, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 80, 0x2F);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 764, 0x0020C480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 768, 0x051EB82A);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 767, 0x03D70A20);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 915, 0x0FFF0000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 916, 0x01999A00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 919, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 920, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 970, 0x00800000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1004, 0x00F18900);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1005, 0x00809D80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1006, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1007, 0x01C00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1540, 0x00400000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1541, 0x00400000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1542, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1545, 0x00500000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1546, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1547, 0x00500000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1553, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1554, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1558, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1560, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1585, 0x00300000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1586, 0x00300000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1587, 0x00100000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1588, 0x00FFC000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1589, 0x00F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1590, 0x0FDA4000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 759, 0x07FEB800);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 756, 0x005B05B2);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 73, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 967, 0x03A2E8BA);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1018, 0x03000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1017, 0x05000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1013, 0x01000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1012, 0x03700000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1011, 0x04B80200);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1010, 0x00823000);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 68, 0x60);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 98, 0x80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 533, 0x71EB851);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 626, 0x723F235);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 627, 0x57A9804);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 918, 0x0036000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1616, 0x1100000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 973, 0xFFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 975, 0xE49BA5);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 516, 0x10038D);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 513, 0x4EDDB9);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 514, 0x0806D6);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1641, 0x200000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1643, 0x000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1565, 0xC00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 750, 0x206280);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 971, 0x1F00000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 972, 0x51EB80);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 27, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 28, 0x01);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 27, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 28, 0x01);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1548, 0x00800000);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 30, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 47, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1538, 0x700000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1555, 0x100000);
	if (rc)
		return rc;
	
    msleep(100);
    
	rc = slic_write_ram(lines + 0, 1538, 0x600000);
	if (rc)
		return rc;
	
    msleep(500);
    
	rc = slic_write_ram(lines + 0, 1551, 0x000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1538, 0x400000);
	if (rc)
		return rc;
	
    msleep(500);
    
	rc = slic_lock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1548, 0x00800000);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 30, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 47, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1538, 0x700000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1555, 0x100000);
	if (rc)
		return rc;
	
    msleep(100);
    
	rc = slic_write_ram(lines + 1, 1538, 0x600000);
	if (rc)
		return rc;
	
    msleep(500);
    
	rc = slic_write_ram(lines + 1, 1551, 0x000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1538, 0x400000);
	if (rc)
		return rc;
	
    msleep(500);
    
	rc = slic_lock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 27, 0xC0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 28, 0x18);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 27, 0xC0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 28, 0x18);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 47, 0x10);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 80, 0x3F);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 46, 0x04);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 47, 0x10);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 80, 0x3F);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 46, 0x04);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 540, 0x07F97D80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 541, 0x0006CC00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 542, 0x1FFC1480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 543, 0x1FFC7B80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 546, 0x07F36B80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 547, 0x000A8E00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 548, 0x1FF90F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 549, 0x1FFAE500);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 563, 0x001AF400);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 564, 0x1FC86A80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 565, 0x01E9AE00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 566, 0x00652F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 567, 0x01F4AF00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 568, 0x1F57E000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 569, 0x00485E00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 570, 0x1FF3A680);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 571, 0x1FF83700);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 572, 0x00011D00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 573, 0x01706980);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 574, 0x066A8480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 653, 0x00920F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 654, 0x1EE31980);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 655, 0x008ADF00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 656, 0x0F92E500);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 657, 0x186CE880);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 45, 0x53);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 544, 0x085C6880);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 906, 0x013E3100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 545, 0x013E3100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 658, 0x07AF6F80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 659, 0x18509100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 660, 0x075EDF00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 27, 0x40);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 28, 0x00);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 540, 0x07F97D80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 541, 0x0006CC00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 542, 0x1FFC1480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 543, 0x1FFC7B80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 546, 0x07F36B80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 547, 0x000A8E00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 548, 0x1FF90F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 549, 0x1FFAE500);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 563, 0x001AF400);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 564, 0x1FC86A80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 565, 0x01E9AE00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 566, 0x00652F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 567, 0x01F4AF00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 568, 0x1F57E000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 569, 0x00485E00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 570, 0x1FF3A680);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 571, 0x1FF83700);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 572, 0x00011D00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 573, 0x01706980);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 574, 0x066A8480);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 653, 0x00920F00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 654, 0x1EE31980);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 655, 0x008ADF00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 656, 0x0F92E500);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 657, 0x186CE880);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 45, 0x53);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 544, 0x085C6880);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 906, 0x013E3100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 545, 0x013E3100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 658, 0x07AF6F80);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 659, 0x18509100);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 660, 0x075EDF00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 26, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 27, 0x40);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 28, 0x00);
	if (rc)
		return rc;
	
	rc = slic_calibrate(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 755, 0x00050000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 844, 0x07EFE000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 845, 0x001B9F2E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 846, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 843, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 637, 0x15E5200E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 860, 0x00D16348);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 848, 0x0068E9B4);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 847, 0x0FFFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 850, 0x00006000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 849, 0x00006000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 753, 0x00C49BA0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 896, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 768, 0x051EB82A);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 39, 0x80);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 40, 0x3E);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 41, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 42, 0x7D);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 920, 0x01893740);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 38, 0x98);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 66, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 749, 0x02AC55FE);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 482, 0x02AC55FE);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 483, 0x003126E8);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 972, 0x00FFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 974, 0x0083126A);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 975, 0x009374B8);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 1560, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 755, 0x00050000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 844, 0x07EFE000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 845, 0x001B9F2E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 846, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 843, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 637, 0x15E5200E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 860, 0x00D16348);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 848, 0x0068E9B4);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 847, 0x0FFFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 850, 0x00006000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 849, 0x00006000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 753, 0x00C49BA0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 896, 0x00000000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 768, 0x051EB82A);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 39, 0x80);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 40, 0x3E);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 41, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 42, 0x7D);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 920, 0x01893740);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 38, 0x98);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 66, 0x00);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 749, 0x02AC55FE);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 482, 0x02AC55FE);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 483, 0x003126E8);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 972, 0x00FFFFFF);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 974, 0x0083126A);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 975, 0x009374B8);
	if (rc)
		return rc;
	
	rc = slic_unlock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 1560, 0x00200000);
	if (rc)
		return rc;
	
	rc = slic_lock_channel(lines + 1);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 634, 0x1C8A024C);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 635, 0x1F909679);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 636, 0x0040A0E0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 638, 0x1D5B21A9);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 639, 0x1DD87A3E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 640, 0x05A38633);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 641, 0x050D2839);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 642, 0x03FE7F0F);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 643, 0x00B4F3C3);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 644, 0x005D0FA6);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 645, 0x002D8D96);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 853, 0x005B0AFB);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 852, 0x006D4060);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 701, 0x00008000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 858, 0x0048D595);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 859, 0x003FBAE2);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 702, 0x00008000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 854, 0x000F0000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 855, 0x00080000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 856, 0x00140000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 857, 0x00140000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 748, 0x01BA5E35);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 752, 0x0051EB85);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 0, 751, 0x00418937);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 634, 0x1C8A024C);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 635, 0x1F909679);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 636, 0x0040A0E0);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 638, 0x1D5B21A9);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 639, 0x1DD87A3E);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 640, 0x05A38633);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 641, 0x050D2839);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 642, 0x03FE7F0F);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 643, 0x00B4F3C3);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 644, 0x005D0FA6);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 645, 0x002D8D96);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 853, 0x005B0AFB);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 852, 0x006D4060);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 701, 0x00008000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 858, 0x0048D595);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 859, 0x003FBAE2);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 702, 0x00008000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 854, 0x000F0000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 855, 0x00080000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 856, 0x00140000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 857, 0x00140000);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 748, 0x01BA5E35);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 752, 0x0051EB85);
	if (rc)
		return rc;
	
	rc = slic_write_ram(lines + 1, 751, 0x00418937);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 0, 30, 0x1);
	if (rc)
		return rc;
	
	rc = slic_write_reg(lines + 1, 30, 0x1);
	if (rc)
		return rc;
	
	return rc;
}

