import time
import smbus

# BNO055 sensor constants
BNO055_ADDRESS = 0x29
BNO055_REGISTER_SYS_TRIGGER = 0x3F
BNO055_RESET_SEQUENCE = [0x20, 0x0B]

def reset_bno055_sensor(bus):
    try:
        # Send reset sequence to the sensor
        bus.write_i2c_block_data(BNO055_ADDRESS, BNO055_REGISTER_SYS_TRIGGER, BNO055_RESET_SEQUENCE)
        time.sleep(2)  # Wait for the sensor to reset
        print("BNO055 sensor reset successfully.")
    except Exception as e:
        print(f"Error occurred while resetting sensor: {e}")

def initialize_bno055(bus):
    try:
        # Set BNO055 to CONFIG mode
        bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x00)
        time.sleep(0.02)
        
        # Set BNO055 to NDOF mode
        bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x0C)
        time.sleep(10.0)
        print("BNO055 sensor initialized successfully.")
    except Exception as e:
        print(f"Error occurred while initializing sensor: {e}")

def main():
    # Create I2C bus
    bus = smbus.SMBus(1)  # Use I2C bus 1 on the Jetson Nano
    
    try:
        # Perform reset
        reset_bno055_sensor(bus)
        
        # Initialize the sensor
        initialize_bno055(bus)
        
    except KeyboardInterrupt:
        # Handle keyboard interrupt (Ctrl+C)
        print("Program terminated by user.")

if __name__ == "__main__":
    main()
