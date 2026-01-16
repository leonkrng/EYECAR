import gpiod
from smbus2 import SMBus

chip = gpiod.Chip("/dev/gpiochip0")
pin17 = chip.get_line(17)  
pin4  = chip.get_line(4)   


def select_camera(new_cam):

    match new_cam:
        case 0:
            # Lower camera
            pin17.request(consumer="init", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            pin4.request(consumer="init", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

            # /dev/i2c-1
            with SMBus(1) as bus:                 
                bus.write_byte_data(0x70, 0x00, 0x01)

        case 1:
            # Upper camera
            pin17.request(consumer="init", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            pin4.request(consumer="init", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

            # /dev/i2c-1
            with SMBus(1) as bus:                 
                bus.write_byte_data(0x70, 0x00, 0x02)




