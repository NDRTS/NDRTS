import os
# Run commands to setup GPIO permissions
# os.system('sudo usermod -aG gpio jetson')
os.system('sudo chown root:gpio /dev/gpiochip0')
os.system('sudo chown root:gpio /dev/gpiochip1')
os.system('sudo chmod 660 /dev/gpiochip0')
os.system('sudo chmod 660 /dev/gpiochip1')
os.system('sudo chmod 666 /dev/ttyACM0')
os.system('sudo chmod 666 /dev/ttyACM1')
os.system('sudo chmod 666 /dev/ttyACM2')
print("Succes")