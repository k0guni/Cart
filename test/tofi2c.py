import smbus
import time

i2c_ch = 0
i2c_address = 0x52

bus = smbus. SMBus ( i2c_ch )

while True :
   
    try :

        bus. write_byte ( i2c_address , 0 )
        time . sleep ( 0.01 )
       
        b1 = bus. read_byte_data (i2c_address , 0 )
        b2 = bus. read_byte_data (i2c_address , 1 )
               
        word = ( b1 << 8 ) + b2
        print ( word )

        time . sleep ( 0.1 )
   
    except OSError :
        print ( 'error' )
