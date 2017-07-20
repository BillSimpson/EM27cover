import serial
import time
import msvcrt   # note this only works on windows

# open com port for the Arduino device with 1 second timeout
com = serial.Serial()
com.port = 'COM6'
com.baudrate = 9600
com.timeout = 1
com.dtr = False
com.open()

print 'Opened port to Arduino for EM27Cover control'
print 'Type > to get a prompt to send commands'

# set flag to not send next character until prompted
send_next_char = False

# continuous loop
while True:
	# try to read a line from the com port (Arduino)
	# note that this returns after timeout (1 sec)
	inputline = com.readline()
	# if the line is not empty, write it to output and console
	if len(inputline) > 0:
		nowtimeUTCstr=time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())
                #opens logfile
		fname = 'EM27CoverLog-'+time.strftime('%Y-%m-%d')+'.txt'
		f = open(fname,'a')
                #writes time/date to file
		writeline = nowtimeUTCstr+' UTC -- '+inputline.strip()
		f.write(writeline+'\n')
		f.close()
	        #prints info to command line for assurance
		print writeline
	# check for an input character and send it to Arduino if it exists
	if msvcrt.kbhit():
		inchar = msvcrt.getche()
		# if we are ready to send command, send it
		if send_next_char:
			com.write(inchar)
		# check if this is the prompt request character ">"
		if (inchar == '>'):
			send_next_char = True
			print ' Next character will be sent to Arduino'
		else:
			send_next_char = False


	
