#!python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 19:02:01 2017
@author: Chris
"""

import time
import pigpio
import spidev

def main():
    
    pi = pigpio.pi()
    
	is_done = pi.read(22)
	print('DONE = ', is_done)
    pi.write(24, 1)  #Asserts RESETN, taking SDB out of reset condition
    time.sleep(0.5)
    
    spi = spidev.SpiDev()
    spi.open(0, 1)
    spi.max_speed_hz = 7800000
    resp = spi.xfer2([0x10, 0x00])   #sends the read command
    print('Bytes: ', resp)
    resp = spi.xfer2([0x00, 0x00, 0x00, 0x00]) #reads the reply
    print('Bytes: ', resp)
    spi.close()
    
    xo2channel = 1  #On the standard SPI bus, hardware is wired to CE1. On the aux bus it's CE2.
    xo2baud = 8000000   #request 8 MHz
	xo2flags = 0
    xo2 = pi.spi_open(xo2channel, xo2baud)
    #xo2flags = 256  #The integer equivalent of 0...0100000000 in binary, this sets the A bit high, indicating the aux bus
    #xo2 = pi.spi_open(xo2channel,xo2baud,xo2flags)
    print(xo2)
    
    pi.spi_write(xo2, b'\x10\x00')
    (b, d) = pi.spi_read(xo2, 4)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x11\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x12\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x13\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x14\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x15\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x16\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x17\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x18\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x19\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x1a\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x1b\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x1c\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x1d\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x1d\x00')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('The {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x20\x80')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('After writing, the {} received bytes are: '.format(b), d)
    
    pi.spi_write(xo2, b'\x21\x80')
    (b, d) = pi.spi_read(xo2, 2)
	d = list(d)
    print('After writing, the {} received bytes are: '.format(b), d)
    
    pi.spi_close(xo2)
    
    time.sleep(0.5)
    pi.write(24, 0)  #Deasserts RESETN, puting SDB into reset condition
    
    pi.stop()
    
main()
