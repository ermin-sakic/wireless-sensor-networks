// Main oscillator frequency - needed for delay.h
#define F_CPU 7372800

// Include directives
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>
#include <util/delay.h>

// Defines
#define INITIALISATION_BYTE 255
#define COMMAND_BYTE 254
#define RESPONSE_BYTE 253
#define EVENT_BYTE 252
#define MAC_BYTE 1
#define DEBUG 1
#define FETCH_ALL 0
#define GET_INFO 1
#define TURN_OFF_NODE 2

// Function prototypes
//Echtzeitige (in rx_task) Weiterleitung der Daten (Schnittstelle mit PY?) zum Server. 
//Wenn es viel dauern wird, koennen wir es spaeter als ein anderer Task implementieren
void sendDataToServer(void);

// Global variables intializations
uint8_t init_ctr = 0;
uint8_t counterNodesInVincinity = 0;
uint8_t tx_buf_full = 0;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t fwd_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t test_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t IDsIdentified[20];              // Wir versuchen strikt allozierte arrays einzusetzen - wo möglich
uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
int8_t routeTableEntry = 0;
int8_t routeTableUpdate = 0;
uint8_t txPower = 3;				// Erlaubte Werte: 3, 7, 11, 15, 19, 23, 27, 31
uint8_t commandToBeSent = 0;
uint8_t commandIdentifier = 0;
uint8_t targetNodeIdentifier = 0;
uint8_t routeMessagesOver = 0;
uint8_t eventsEnabled = 1;
uint8_t getRoutingInfo = 0;
uint8_t tasksDisabled = 1;
uint8_t cnt = 0;
int val = -1;						// Can be -1
uint8_t twoDimensionalRoutingTable[8][7]; // Predefined maximum of 6 Nodes with 5 Neighbors - Commander can be neighbor too!

// Task-specific declarations
nrk_task_type INIT_TASK;
nrk_task_type TX_COMMAND_TASK;
nrk_task_type RX_TASK;
nrk_task_type UART_RX_TASK;

NRK_STK init_task_stack[NRK_APP_STACKSIZE];
NRK_STK tx_command_task_stack[NRK_APP_STACKSIZE];
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
NRK_STK uart_rx_task_stack[NRK_APP_STACKSIZE];


void nrk_create_taskset();

int main() {
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);
	nrk_init();

	nrk_kprintf(PSTR("Commander Node with MAC "));
	printf("%i\n",MAC_BYTE);
	nrk_kprintf(PSTR(" booting...\r\n"));
	printf("    Q    ");

	// bmac - the radio interface configuration
	bmac_task_config();
	// init bmac on channel 25
	bmac_init(25);

	bmac_addr_decode_set_my_mac(MAC_BYTE);
	//bmac_addr_decode_enable();
	bmac_auto_ack_enable();    //Eventuell können wir das auch einsetzen.

	// Enable AES 128 bit encryption
	// When encryption is active, messages from plaintext
	// source will still be received.
	bmac_encryption_set_key(aes_key, 16);
	bmac_encryption_enable();
	// bmac_encryption_disable();

	// By default the RX check rate is 100ms
	// below shows how to change that
	nrk_time_t check_period;
	int8_t val;

	check_period.secs = 0;
	check_period.nano_secs = 25 * NANOS_PER_MS;
	val = bmac_set_rx_check_rate(check_period);
	if (val == NRK_ERROR)
		nrk_kprintf(PSTR("ERROR setting bmac rate\r\n" ));
	printf("    Q    ");

	// The default Clear Channel Assement RSSI threshold is -35
	bmac_set_cca_thresh(-35);

	// This sets the next RX buffer.
	// This can be called at anytime before releaseing the packet
	// if you wish to do a zero-copy buffer switch
	bmac_rx_pkt_set_buffer(fwd_buf, RF_MAX_PAYLOAD_SIZE);

	nrk_create_taskset();
	nrk_start();

	return 0;
}

void init_task()
{	
	while (!bmac_started())
		//printf("BMAC not started yet!\n");
		nrk_wait_until_next_period();		

	int bufferSize;
	printf("init_task begins\n");
	printf("    Q    ");

	bmac_set_rf_power(txPower);

	for (;;) {
		if(routeTableUpdate == 0)
		{
			bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
			//printf("Sending: %s with BufSize = %d and Ctr = %d\n",tx_buf, bufferSize, init_ctr);

			nrk_led_set(YELLOW_LED);;
			bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
			init_ctr++;
			printf("Initialisation packet sent. ctr: %d\n",init_ctr);
			printf("    Q    ");

			//      Task gets control again after TX complete
			//	bmac_disable();
			nrk_led_clr(YELLOW_LED);

			if(routeTableEntry == 1)
				routeTableUpdate = 1;
			//nrk_wait_until_next_period();
			//tx_buf_full = 1;
		}

		nrk_wait_until_next_period();
	}
}

// Hier anhand der Größe des ankommenden Paktes, Unterscheidung treffen ob es sich um ein Initialisierungs, Command
// oder Forwarding Packet handelt...
void rx_task ()
{
	uint8_t i, len;
	int8_t rssi;
	uint8_t *local_rx_buf;

	// This should be called by all tasks using bmac that
	// do not call bmac_init()...
	while (!bmac_started ())
		nrk_wait_until_next_period ();
	//printf("rx_task begins\n");

	for (;;) 
	{
		// Wait until an RX packet is received
		bmac_wait_until_rx_pkt();
		printf("Got some kind of a package.");
		printf("    Q    ");
		// Get the RX packet
		nrk_led_set(GREEN_LED);
		local_rx_buf = bmac_rx_pkt_get(&len, &rssi);
		//if (bmac_rx_pkt_is_encrypted() == 1)
		//	nrk_kprintf(PSTR( "Packet Encrypted\r\n" ));
		//printf("Got RX packet len=%d RSSI=%d [", len, rssi);
		//for (i = 0; i < len; i++)
		//	printf("%c", local_rx_buf[i]);
		//printf("]\r\n");

		unsigned char alreadyInTheArray = 0;
		// Sicherstellen dass der Eintrag nicht schon vorhanden ist
		for( int i = 0; i < counterNodesInVincinity; i++ )
		{
			if ( local_rx_buf[4] - '0' == IDsIdentified[i] )
				alreadyInTheArray = 1;
		}

		// Hier die Unterscheidung zwischen verschiedenen Packetarten (routing, forwarding etc) machen und entsprechend weitermachen
		// Initialisation package identifier
		if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '5' && !alreadyInTheArray )
		{
			//printf("Got initialization information\r\n");

			//printf("Got routing information for node %c\r\n", local_rx_buf[4]);
			routeMessagesOver = local_rx_buf[4] - '0';

			if (counterNodesInVincinity == 0 )
			{
				IDsIdentified[counterNodesInVincinity] = local_rx_buf[4] - '0';
				//IDsIdentifiedStr[counterNodesInVincinity] = local_rx_buf[4];
				counterNodesInVincinity++;

				//bmac_ local_rx_buf[4] - '0'addr_decode_dest_mac( local_rx_buf[4] - '0' );
				//bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
				//printf("Sending to %d: %s with BufSize = %d\n",local_rx_buf[4] - '0',tx_buf, bufferSize);
				//routeTableUpdate = 0;
				nrk_led_set(RED_LED);

				//or_bmac_set_dest_here
			}
			else
			{
				nrk_led_clr(RED_LED);
				IDsIdentified[counterNodesInVincinity] = local_rx_buf[4] - '0';
				//IDsIdentifiedStr[counterNodesInVincinity] = local_rx_buf[4];
				counterNodesInVincinity++;
				//routeTableUpdate = 0;

				//bmac_addr_decode_dest_mac( local_rx_buf[4] - '0' );
				//bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
				//printf("Sending: %s with BufSize = %d and Ctr = %d\n",tx_buf, bufferSize, init_ctr);
				//routeTableUpdate = 0;
				/*
							nrk_led_set(YELLOW_LED);
							bmac_set_rf_power(txPower);
							while(val != NRK_OK)
								val= bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
							val = -1;
							printf("Initialisation packet sent targeted at node: %d\n",IDsIdentified[counterNodesInVincinity-1]);
				 */
				//or_bmac_set_dest_here*/
			}
			routeTableUpdate = 0;
			routeTableEntry = 1;
			//bmac_addr_decode_enable();   //DAS AUF JEDEN FALL ÄNDERN!
			// Ab diesem Zeitpunkt nur an diesen Node gerichteten Pakete empfangen
			if(tasksDisabled)
			{
				nrk_activate_task(&TX_COMMAND_TASK);  			// Ab hier nur über routeMessagesOver node Kommandos
				nrk_activate_task (&UART_RX_TASK);				// ... verschicken
				tasksDisabled = 0;
			}

			printf("DIRECT NEIGHBORS TO COMMANDER: ");
			for( int i = 0; i < counterNodesInVincinity; i++ )
			{
				printf("%d ", IDsIdentified[i]);
			}
			printf("\n");
			printf("    Q    ");
		}
		// Response package identifier
		else if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '3' )
		{
			/*printf("Got a response information from node %d: [", local_rx_buf[4] - '0');
			for (i = 0; i < len; i++)
				printf("%c", local_rx_buf[i]);
			printf("]\r\n");
			 */
			int i = 0;
			printf("RESPONSE INFO ");
			for (i = 0; i < len; i++)
				printf("%c", local_rx_buf[i]);
			printf("    Q    ");
		}
		else if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '1' )
		{
			printf("ROUTING INFO ");
			for (i = 0; i < len; i++)
				printf("%c", local_rx_buf[i]);
			printf("    Q    ");

			// NEW: Save the routing data into a 2-dimensional table;
			// Sample routing info msg: "251 5 1 12346"
			printf("Node %c's Neighbors: ", local_rx_buf[4]);
			for (i=8; i<len; i++)
			{
				printf("%c ", local_rx_buf[i]);
				twoDimensionalRoutingTable[local_rx_buf[4]-'0'][i-8] = local_rx_buf[i] - '0';
			}
			printf("    Q    ");
		}
		else if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '2' )
		{
			printf("WARNING INFO ");
			for (i=0; i<len; i++)
			{
				printf("%c", local_rx_buf[i]);
			}
			printf("    Q    ");
			//Stub: Handle Node warnings and react in GUI appropriately.
		}
		else
		{
			printf("Redundant packages ignored.\n");
			for (i = 0; i < len; i++)
				printf("%c", local_rx_buf[i]);
			printf("    Q    ");
		}

		nrk_led_clr(GREEN_LED);
		local_rx_buf[0] = '\0';
		// Release the RX buffer so future packets can arrive
		bmac_rx_pkt_release ();
	}
}


// Allows for receiving commands from the Raspberry Pi over UART interface
void uart_rx_task()
{
	char c;

	nrk_sig_t uart_rx_signal;
	// Get the signal for UART RX
	uart_rx_signal = nrk_uart_rx_signal_get();
	// Register task to wait on signal
	nrk_signal_register(uart_rx_signal);
	char receivedString[32];
	uint8_t lenString = 0;
	uint8_t messageType = 0;

	while (1) {
		lenString = 0;
		while (nrk_uart_data_ready(NRK_DEFAULT_UART)) {
			//printf("Got a char!\n");
			c = getchar();
			// c is a Byte received. You can concatenate it to a string
			receivedString[lenString] = c;
			lenString++;
		}
		printf("Received UART String %s", receivedString);
		printf("    Q    ");

		// Parsing time, commando looks like this: "0 X 5" or "0 4 1"...
		messageType = receivedString[0] - '0';

		if(receivedString[2] == 'X')
		{
			getRoutingInfo = 1;
		}
		else
			targetNodeIdentifier = receivedString[2] - '0';

		if(messageType == 0 && receivedString[1] == ' ' && receivedString[3] == ' ') // A command Request
		{
			commandIdentifier = receivedString[4] - '0';

			if(!getRoutingInfo){
				uint8_t foundConnection = 0;
				for(int i = 0; i < counterNodesInVincinity; i++)
				{
					if(IDsIdentified[i] == targetNodeIdentifier)
					{
						printf("The node %d is directly reachable.", targetNodeIdentifier);
						printf("    Q    ");
						routeMessagesOver = targetNodeIdentifier;
						foundConnection = 1;
						commandToBeSent = 1;
					}
				}

				if(!foundConnection)
				{
					//if(!foundConnection)
					/*{
					for(int counter=0; counter<6; counter++)
					{
						for(int counter2 = 0; counter2 < counterNodesInVincinity; counter2++)
						{
							printf("Elements in the line %d: ", targetNodeIdentifier );
											if(twoDimensionalRoutingTable[targetNodeIdentifier][counter] == IDsIdentified[counter2])
											{
												printf(" %d", twoDimensionalRoutingTable[targetNodeIdentifier][counter]);
												printf("The node %d is reachable over direct connection %d", targetNodeIdentifier, IDsIdentified[counter2]);
												routeMessagesOver = IDsIdentified[counter2];
												foundConnection = 1;
												commandToBeSent = 1;
											}
						}
					}
					 */


					for(int counter=0; counter<7; counter++)
					{
						for(int counter2 = 0; counter2 < counterNodesInVincinity; counter2++)
						{
							printf("Elements in the line %d: ", IDsIdentified[counter2]);
							printf("    Q    ");
							if(twoDimensionalRoutingTable[IDsIdentified[counter2]][counter] == targetNodeIdentifier)
							{
								printf(" %d", twoDimensionalRoutingTable[IDsIdentified[counter2]][counter]);
								printf("The node %d is reachable over direct connection %d", targetNodeIdentifier, IDsIdentified[counter2]);
								printf("    Q    ");
								routeMessagesOver = IDsIdentified[counter2];
								foundConnection = 1;
								commandToBeSent = 1;
							}
						}
					}

				}

				if(foundConnection == 0)
					printf("For some reason, the %d not found.", targetNodeIdentifier);
				else
					printf("Sending the command %d to %d\n", commandIdentifier, targetNodeIdentifier);
				printf("    Q    ");

			}
			else if(getRoutingInfo)
			{
				commandToBeSent = 1;
				printf("Sending the route command to all neighbors");
				printf("    Q    ");
			}
		}
		else
		{
			printf("PROBLEM");
			printf("    Q    ");
		}

		nrk_event_wait(SIG(uart_rx_signal));
	}


	/*
	for(;;){
		// parsed from Python UART information
		uint8_t messageType = 0;

		// Placeholder for now but using the received UART information, the initialization will follow similarly
		switch(messageType)
		{
		case 0: // If message a command...
			targetNodeIdentifier = 5;
			commandIdentifier = 1;
			commandToBeSent = 1;
			break;
		}
		nrk_wait_until_next_period();
	}
	 */

}

void tx_command_task()
{
	while (!bmac_started())
		//printf("BMAC not started yet!\n");
		nrk_wait_until_next_period();

	uint8_t bufferSize;

	//bmac_addr_decode_dest_mac( routeMessagesOver );
	for (;;) {
		if( commandToBeSent == 1 )
		{
			printf("Decoding enable");
			printf("    Q    ");
			bmac_addr_decode_enable();
			if(getRoutingInfo == 0)
			{
				bmac_addr_decode_dest_mac( routeMessagesOver );
				printf("Set dest address to %d", routeMessagesOver);
				printf("    Q    ");
				// Hier bestimmen was verschickt werden sollte.
				bufferSize = sprintf(tx_buf, "%d %d %d %d", COMMAND_BYTE, MAC_BYTE, targetNodeIdentifier, commandIdentifier);
				printf("Sending: %s with BufSize = %d\n",tx_buf, bufferSize);
				printf("    Q    ");
				//printf("Sending out the command %d to the node %d\n", commandIdentifier, routeMessagesOver);
				nrk_led_set(RED_LED);
				bmac_set_rf_power(txPower);
				/*while(val != NRK_OK)
					val= bmac_tx_pkt(tx_buf, 9);
					*/
				val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				if(val==NRK_OK) cnt++;
				else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
				val = -1;

				printf("Command sent.\n");
				printf("    Q    ");

				//  Task gets control again after TX complete
				//	bmac_disable();
				nrk_led_clr(RED_LED);
				commandToBeSent = 0;
			}
			else if(getRoutingInfo==1)
			{
				for(int i = 0; i < counterNodesInVincinity; i++)
				{
					// Hier bestimmen was verschickt werden sollte.
					bmac_addr_decode_dest_mac( IDsIdentified[i] );
					targetNodeIdentifier = IDsIdentified[i];
					bufferSize = sprintf(tx_buf, "%d %d %d %d", COMMAND_BYTE, MAC_BYTE, targetNodeIdentifier, commandIdentifier);
					printf("Sending: %s with BufSize = %d\n",tx_buf, bufferSize);
					printf("    Q    ");
					nrk_led_set(RED_LED);
					bmac_set_rf_power(txPower);
					/*while(val != NRK_OK)
						val= bmac_tx_pkt(tx_buf, 9);
						*/
					val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
					if(val==NRK_OK) cnt++;
					else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
					val = -1;

					nrk_led_clr(RED_LED);
				}
				commandToBeSent = 0;
				getRoutingInfo=0;
			}
		}
		nrk_wait_until_next_period();
	}
}

void nrk_create_taskset() {
	RX_TASK.task = rx_task;
	nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
	RX_TASK.prio = 3;
	RX_TASK.FirstActivation = TRUE;
	RX_TASK.Type = BASIC_TASK;
	RX_TASK.SchType = PREEMPTIVE;
	RX_TASK.period.secs = 0;
	RX_TASK.period.nano_secs = 30 * NANOS_PER_MS;
	RX_TASK.cpu_reserve.secs = 0;
	RX_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	RX_TASK.offset.secs = 0;
	RX_TASK.offset.nano_secs = 0;
	nrk_activate_task (&RX_TASK);

	UART_RX_TASK.task = uart_rx_task;
	nrk_task_set_stk( &UART_RX_TASK, uart_rx_task_stack, NRK_APP_STACKSIZE);
	UART_RX_TASK.prio = 3;
	UART_RX_TASK.FirstActivation = TRUE;
	UART_RX_TASK.Type = BASIC_TASK;
	UART_RX_TASK.SchType = PREEMPTIVE;
	UART_RX_TASK.period.secs = 0;
	UART_RX_TASK.period.nano_secs = 30* NANOS_PER_MS;
	UART_RX_TASK.cpu_reserve.secs = 0;
	UART_RX_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	UART_RX_TASK.offset.secs = 0;
	UART_RX_TASK.offset.nano_secs = 0;

	TX_COMMAND_TASK.task = tx_command_task;
	nrk_task_set_stk(&TX_COMMAND_TASK, tx_command_task_stack, NRK_APP_STACKSIZE);
	TX_COMMAND_TASK.prio = 2;
	TX_COMMAND_TASK.FirstActivation = TRUE;
	TX_COMMAND_TASK.Type = BASIC_TASK;
	TX_COMMAND_TASK.SchType = PREEMPTIVE;
	TX_COMMAND_TASK.period.secs = 0;
	TX_COMMAND_TASK.period.nano_secs = 70* NANOS_PER_MS;
	TX_COMMAND_TASK.cpu_reserve.secs = 0;
	TX_COMMAND_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	TX_COMMAND_TASK.offset.secs = 0;
	TX_COMMAND_TASK.offset.nano_secs = 0;

	INIT_TASK.task = init_task;
	nrk_task_set_stk(&INIT_TASK, init_task_stack, NRK_APP_STACKSIZE);
	INIT_TASK.prio = 1;
	INIT_TASK.FirstActivation = TRUE;
	INIT_TASK.Type = BASIC_TASK;
	INIT_TASK.SchType = PREEMPTIVE;
	INIT_TASK.period.secs = 0;
	INIT_TASK.period.nano_secs = 80 * NANOS_PER_MS;
	INIT_TASK.cpu_reserve.secs = 0;
	INIT_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	INIT_TASK.offset.secs = 0;
	INIT_TASK.offset.nano_secs = 0;
	nrk_activate_task(&INIT_TASK);
}
