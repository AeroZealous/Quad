pwd

#This section ensures that arduino finds the libraries that you are using
if [ ! -d "~/Arduino/libraries" ]; then
	#statements
	echo "Copying Library files from ./lib to ~/Arduino/libraries"
	cp -r ./lib ~/Arduino/libraries
	echo "Done copying"
else 
	echo "~/Arduino/libraries not found"
	exit
fi

#this section prevents the ioctl:Permission denied error in Arduino IDE.
#A temporary solution is chmod 666 /dev/ttyACM0 but it doesnt hold between reboots
echo "Adding $USER to dialout group. You will be asked to enter password"
sudo usermod -a -G dialout $USER