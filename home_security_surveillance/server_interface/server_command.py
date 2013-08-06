#!/usr/bin/python
import socket, serial, thread, time, os, sys

s = socket.socket()         # Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

myIP='127.0.0.1'            # Adresse des eigenen Rechners
port = 12345                # Reserve a port for your service.
s.bind((myIP, port))        # Bind to the port
s.listen(5)                 # Now wait for client connection.
responseString = 0
responseReadyToBeRead = 0
awaitingResponse = 0
repeatAction = 0
## Get packets from client here and differentiate between commands, exit flags etc.
def tcp_receive():
    data = c.recv( 1024 )
    global response, awaitingResponse, responseReadyToBeRead, repeatAction
    if(data):
        splitted_msg = data.split(' ')
        # Do the message parsing and react accordingly
        if (splitted_msg[0] == "command"):
            print "[Interface] Received a command: ", data
            
            if (splitted_msg[1] == "update"):
                uart_command = []
                uart_command.append("0 X 5")
                print "[Interface]  Writing the command on UART: ", ''.join(uart_command), "\n"
                port.write(''.join(uart_command)) 
                response = ""
                elapsed = 0
                start = time.clock()
                while not responseReadyToBeRead:
                    awaitingResponse = 1
                    elapsed = (time.clock() - start)
                    if (repeatAction == 1):
                        print "Action had to be repeated!"
                        port.write(''.join(uart_command)) 
                        repeatAction = 0
                    if elapsed >= 4:
                        tcp_send("PROBLEM")
                        responseReadyToBeRead = 1
                    pass

                responseReadyToBeRead = 0
                awaitingResponse = 0
                
            if (splitted_msg[1] == "info"):
                uart_command = []
                uart_command.append("0 ")
                uart_command.append(splitted_msg[2])
                uart_command.append(" 1")
                print "Writing the command on UART: ", ''.join(uart_command) 
                print "\n"
                port.write(''.join(uart_command))
                response = ""
                elapsed = 0
                start = time.clock()
                while not responseReadyToBeRead:
                    awaitingResponse = 1
                    elapsed = (time.clock() - start)
                    if (repeatAction == 1):
                        print "Action had to be repeated!"
                        port.write(''.join(uart_command)) 
                        repeatAction = 0
                    if elapsed >= 4:
                        tcp_send("PROBLEM")
                        responseReadyToBeRead = 1
                    pass
                
                responseReadyToBeRead = 0
                awaitingResponse = 0
                #print ("[Interface] Response sent to GUI: ", response)

            if (splitted_msg[1] == "disable"):
                uart_command = []
                uart_command.append("0 ")
                uart_command.append(splitted_msg[2])
                uart_command.append(" 2")
                print "Writing the command on UART: ", ''.join(uart_command) 
                print "\n"
                port.write(''.join(uart_command))
                response = "DISABLED"
                tcp_send(response)
                print ("[Interface] Response sent to GUI: ", response)     

            if (splitted_msg[1] == "enable"):
                uart_command = []
                uart_command.append("0 ")
                uart_command.append(splitted_msg[2])
                uart_command.append(" 3")
                print "Writing the command on UART: ", ''.join(uart_command) 
                print "\n"
                port.write(''.join(uart_command))
                response = "ENABLED"
                tcp_send(response)
                print ("[Interface] Response sent to GUI: ", response)    

        elif (splitted_msg[0] == "quit"):
            s.close()
            os._exit(0)

def tcp_send(message):
    c.send( message )

def initialise_uart():
    global port
    port = serial.Serial(sys.argv[1], baudrate=115200, timeout=0.001)

def write_uart(message):
    port.write(message)
    
def read_uart():
    rcv = port.read(1)
    return rcv

def UART_r():
    global response, responseReadyToBeRead, awaitingResponse, repeatAction
    # UART RECEIVE PART 
    while(True):
        uart_response = []
        uart_part = read_uart()
        while(uart_part and uart_part != 'Q'):
            uart_response.append(uart_part)
            uart_part = read_uart()
            
        if (uart_response):
            uart_response = ''.join(uart_response)	
            print ("[Commander] UART: ",  uart_response)
        
        # Command response information:
        if "ROUTING INFO" in uart_response:
                response = ""
                print "[Interface] ROUTING INFO in der Nachricht vorhanden!"
                arrayOfNumbersInString = [int(s) for s in uart_response.split() if s.isdigit()]
                response = "RESPONSE "
                response = response + str(arrayOfNumbersInString[len(arrayOfNumbersInString)-1]) + " "
                if(awaitingResponse):
                    responseReadyToBeRead = 1
                    print ("[Interface] Got this route response: ", response)
                    tcp_send(response)
                    print ("[Interface] Response sent to GUI: ", response)
                    awaitingResponse = 0
                else:
                    print ("[Interface] Got this eventful route response: ", response)
                    response = "ROUTVENT " + response
                    tcp_send(response)
                    print ("[Interface] ROUTVENT sent to GUI: ", response)
                    awaitingResponse = 0              
        if "RESPONSE INFO" in uart_response:
                response = ""
                print "[Interface] RESPONSE INFO in der Nachricht vorhanden!"
                arrayOfNumbersInString = [int(s) for s in uart_response.split() if s.isdigit()]
                response = "RESPONSE "
                for i in arrayOfNumbersInString:
                    response = response + str(i) + " "
                if(awaitingResponse):
                    responseReadyToBeRead = 1
                    print ("[Interface] Got this info response: ", response)
                    tcp_send(response)
                    print ("[Interface] Response sent to GUI: ", response)
                    awaitingResponse = 0
                else:
                    print ("[Interface] Got this eventful info response: ", response)
                    response = "EVENT " + response
                    tcp_send(response)
                    print ("[Interface] Event sent to GUI: ", response)
        if "WARNING INFO" in uart_response:
                response = ""
                print "[Interface] Got a warning from a node."
                arrayOfNumbersInString = [int(s) for s in uart_response.split() if s.isdigit()]
                print arrayOfNumbersInString
                response = str(arrayOfNumbersInString[1])
                response = "WARNVENT " + response
                tcp_send(response)
                print ("[Interface] WARNVENT sent to GUI: ", response)
        elif "PROBLEM" in uart_response:
                response = "PROBLEM"
                if(awaitingResponse):
                    print ("[Interface] Got a command parsing error. Execute action again!")
                    repeatAction = 1
        # Event information
        elif "EVENT INFO" in uart_response:
                pass

def TCP_rw_UART_w():
    while(1):   
        tcp_receive()
    
def create_Threads():
    # Create two threads as follows
    try:
        thread.start_new_thread( UART_r, () )
        thread.start_new_thread( TCP_rw_UART_w, () )
    except:
        print "Error: unable to start thread"

##### BEGIN HERE #####
initialise_uart()
print "[Interface] Waiting for a device to connect..."
c, addr = s.accept()     # Establish connection with client.
print '[Interface] Got connection from', addr

create_Threads()

while True:
    pass