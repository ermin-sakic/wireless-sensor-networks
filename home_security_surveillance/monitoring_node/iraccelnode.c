// Main oscillator frequency - needed for delay.h
#define F_CPU 7372800

// Include directives
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>
#include <util/delay.h>
#include <stdbool.h>
#include <adc_driver.h>		//for battery voltage
#include <mda100_driver.h>	//for temp, brightness and IR		
//#include <mts310cb_driver.h>	//for accel

// Defines
#define INITIALISATION_BYTE 255
#define COMMAND_BYTE 254
#define RESPONSE_BYTE 253
#define EVENT_BYTE 252
#define ROUTING_BYTE 251
#define MAC_BYTE 6
#define DEBUG 1
#define TEMPERATURE_BRIGHTNESS_NODE 0
#define IR_ACCEL_NODE 1
// Function prototypes

// Global variables intializations
uint8_t init_ctr = 0;
uint8_t counterNodesInVincinity = 0;
uint8_t tx_buf_full = 0;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t forward_rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t forward_rx_buf2[RF_MAX_PAYLOAD_SIZE];
uint8_t forward_rx_len = 0;
uint8_t forward_rx_len2 = 0;
uint8_t test_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t IDsIdentified[20];              // Wir versuchen strikt allozierte arrays einzusetzen - wo möglich
char IDsIdentifiedStr[20];
uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
uint8_t routeTableEntry = 0;
uint8_t routeTableUpdate = 0;
uint8_t packetToForward = 0;
uint8_t txPower = 15;
uint8_t routeMessagesOver = 0;
uint8_t responseDataReady = 0;
uint8_t nodeSecure = 0;
uint8_t directConnectionToCommander, gotConnectionToCommander = 0;
uint8_t transmitMessageTo = 0;
uint8_t modeChange = 0;
uint8_t alreadyInTheArray = 0;
uint8_t bufferSize;
uint8_t voltage_int;
uint8_t accel = 0;
uint8_t temp = 0;
uint8_t bright = 0;
uint8_t ir = 0;
uint8_t commandIdentifier;
uint8_t targetNodeIdentifier;
uint8_t i = 0;
uint8_t counter = 0;					
uint8_t routingDataReady = 0;
uint8_t routingTableShared = 0;
uint8_t raiseWarning = 0;
uint8_t sendWarningEvent = 0;      //NEW
uint8_t cnt = 0;
int val = 0;				// Can be -1
double voltage = 0.0;

// Task-specific declarations
nrk_task_type INIT_TASK;
nrk_task_type TX_TASK;
nrk_task_type RX_TASK;
nrk_task_type WARNING_TASK;

NRK_STK init_task_stack[NRK_APP_STACKSIZE];
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
NRK_STK warning_task_stack[NRK_APP_STACKSIZE];

void nrk_create_taskset();

int main() {
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

	nrk_init();

	nrk_time_set(0, 0);

	nrk_kprintf(PSTR("IR/Accel Node with MAC "));
	printf("%i\n",MAC_BYTE);
	nrk_kprintf(PSTR(" booting...\r\n"));


	// bmac - the radio interface configuration
	bmac_task_config();
	// init bmac on channel 25
	bmac_init(25);

	//bmac_addr_decode_enable();
	bmac_addr_decode_set_my_mac(MAC_BYTE);
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

	// The default Clear Channel Assement RSSI threshold is -35
	bmac_set_cca_thresh(-35);

	// This sets the next RX buffer.
	// This can be called at anytime before releaseing the packet
	// if you wish to do a zero-copy buffer switch
	bmac_rx_pkt_set_buffer(forward_rx_buf, RF_MAX_PAYLOAD_SIZE);

	// Initialize the analog to digital converter
	adc_init();

	//DDRE ^= (1 << 1); //Braucht man das?

	if(TEMPERATURE_BRIGHTNESS_NODE)
	{
		// Initialize the mda100 sensorboard driver - for temp/bright
		mda100_init();
		// Switch on the power for the light sensor
		mda100_TemperatureSensor_Power(POWER_ON);
		// Switch on the power for the light sensor
		mda100_LightSensor_Power(POWER_ON);
	}
	else if(IR_ACCEL_NODE)
	{
		// Initialize the mda100 sensorboard driver - for IR
		mda100_init();
		// Initialize the mts310 sensorboard driver
		//mts310cb_init();
		// Switch on the power for the acceleration sensor
		//mts310cb_Accelerometer_Power(POWER_ON);
	}

	nrk_create_taskset();
	nrk_start();

	return 0;
}

void init_task(){
	while (!bmac_started())
		//printf("BMAC not started yet!\n");
		nrk_wait_until_next_period();
	bmac_set_rf_power(txPower);
	int bufferSize;
	printf("init_task begins\n");

	for (;;) {
		//printf("was here4;");
		if(routeTableUpdate == 0)
		{
			nrk_led_set(YELLOW_LED);
			if(directConnectionToCommander)
			{
				bmac_set_rf_power(3);
				bufferSize = sprintf(tx_buf, "%d %d %d", INITIALISATION_BYTE, MAC_BYTE, 3);  // Hier bestimmen was verschickt werden sollte.
				bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				bmac_set_rf_power(txPower);
			}
			else{
				bufferSize = sprintf(tx_buf, "%d %d %d", INITIALISATION_BYTE, MAC_BYTE, txPower);  // Hier bestimmen was verschickt werden sollte.
				//printf("Sending: %s with BufSize = %d and Ctr = %d\n",tx_buf, bufferSize, init_ctr);
				bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
			}
			init_ctr++;
			printf("Initialisation packet sent. ctr: %d\n",init_ctr);

			//  Task gets control again after TX complete
			//	bmac_disable();
			nrk_led_clr(YELLOW_LED);

			if(routeTableEntry == 1){
				if(directConnectionToCommander)
				{
					txPower = 3;
					bmac_set_rf_power(3);
				}
				bmac_addr_decode_dest_mac( IDsIdentified[counterNodesInVincinity-1] );
				bufferSize = sprintf(tx_buf, "%d %d %d", INITIALISATION_BYTE, MAC_BYTE, txPower);  // Hier bestimmen was verschickt werden sollte.

				/*while(val != NRK_OK)
					val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				val = -1;*/
				val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				if(val==NRK_OK) cnt++;
				else
				{
					nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
					val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				}

				printf("Initialisation packet sent targeted at node: %d\n",IDsIdentified[counterNodesInVincinity-1]);
				routeTableUpdate = 1;
				txPower = 15;
				bmac_set_rf_power(txPower);
			}
			//nrk_wait_until_next_period();
		}

		nrk_wait_until_next_period();
	}
}

// Hier anhand der Größe des ankommenden Paketes, Unterscheidung treffen ob es sich um ein Initialisierungs, Command
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
	printf("rx_task begins\n");

	for (;;) 
	{	//printf("was here3;");
		// Wait until an RX packet is received
		bmac_wait_until_rx_pkt();

		nrk_kprintf(PSTR("R Generic1\n"));
		// Get the RX packet
		nrk_led_set(GREEN_LED);
		local_rx_buf = bmac_rx_pkt_get(&len, &rssi);
		//if (bmac_rx_pkt_is_encrypted() == 1)
		//	nrk_kprintf(PSTR( "Packet Encrypted\r\n" ));
		printf("Got RX packet len=%d RSSI=%d [", len, rssi);
		for (i = 0; i < len; i++)
			printf("%c", local_rx_buf[i]);
		nrk_kprintf(PSTR("]\r\n"));

		alreadyInTheArray = 0;
		// Sicherstellen dass der Eintrag nicht schon vorhanden ist
		for( i = 0; i < counterNodesInVincinity; i++ )
		{
			if ( local_rx_buf[4] - '0' == IDsIdentified[i] )
			{
				alreadyInTheArray = 1;
			}
		}
		//printf("Got to here");

		// Hier die Unterscheidung zwischen verschiedenen Packetarten (routing, forwarding etc) machen und entsprechend weitermachen
		// a) Initialisation package identifier
		if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '5' && !alreadyInTheArray )
		{
			nrk_kprintf(PSTR("Got initialization information\r\n"));
			if(local_rx_buf[4] == '1' && routeMessagesOver == 0)
			{


				gotConnectionToCommander = 1;
				directConnectionToCommander = 1;
				routeMessagesOver = 1;
				transmitMessageTo = 1;

				printf("Got routing information for commander node %c\r\n", local_rx_buf[4]);
				bmac_set_rf_power(txPower);

				/*
				int EndNode = 4; // :)
				sprintf(tx_buf, "%d %d %d %d", COMMAND_BYTE, MAC_BYTE, EndNode, 4);
				nrk_kprintf(PSTR("Sending out the command 4 to all nodes which can hear me.\n"));

				while(val != NRK_OK)
					val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
				val = -1;
				 */

				bmac_addr_decode_enable();   //CHECK IF NEEDS a CHANGE!
				//bmac_set_dest_here

				nrk_kprintf(PSTR("Sending out the command 4 to all nodes which can hear me.\n"));
				for( i = 0; i < counterNodesInVincinity; i++ )
				{
					txPower = 31;
					bmac_set_rf_power(txPower);
					if(IDsIdentified[i] != 1)
					{
						bmac_addr_decode_dest_mac( IDsIdentified[i] );
						sprintf(tx_buf, "%d %d %d %d", COMMAND_BYTE, MAC_BYTE, IDsIdentified[i], 4);
						val = -1;
						/*while(val != NRK_OK)
							val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
						printf("Command 4 sent out to node %d.\n", IDsIdentified[i]);
						 */
						val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
						if(val==NRK_OK) cnt++;
						else
						{
							nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
							val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
							if(val==NRK_OK) cnt++;
							else{
								val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
							}
						}
						val = -1;
					}
					txPower=15;
					bmac_set_rf_power(txPower);
				}
			}
			else
				printf("Got routing information for node %c\r\n", local_rx_buf[4]);


			if (counterNodesInVincinity == 0 )
			{
				IDsIdentified[counterNodesInVincinity] = local_rx_buf[4] - '0';
				IDsIdentifiedStr[counterNodesInVincinity] = local_rx_buf[4];
				counterNodesInVincinity++;

				//bmac_ local_rx_buf[4] - '0'addr_decode_dest_mac( local_rx_buf[4] - '0' );
				//bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
				//printf("Sending to %d: %s with BufSize = %d\n",local_rx_buf[4] - '0',tx_buf, bufferSize);
				routeTableUpdate = 0;
				nrk_led_set(RED_LED);

				//or_bmac_set_dest_here
			}
			else
			{
				nrk_led_clr(RED_LED);
				IDsIdentified[counterNodesInVincinity] = local_rx_buf[4] - '0';
				IDsIdentifiedStr[counterNodesInVincinity] = local_rx_buf[4];
				counterNodesInVincinity++;
				//routeTableUpdate = 0;

				//bmac_addr_decode_dest_mac( local_rx_buf[4] - '0' );
				//bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
				//printf("Sending: %s with BufSize = %d and Ctr = %d\n",tx_buf, bufferSize, init_ctr);
				routeTableUpdate = 0;
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

			routeTableEntry = 1;

			nrk_kprintf(PSTR("Identified neighbors: "));
			for( i = 0; i < counterNodesInVincinity; i++ )
			{
				printf("%d ", IDsIdentified[i]);
			}
			nrk_kprintf(PSTR("\n"));
		}
		// b) Command package identifier
		else if( local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '4')
		{
			targetNodeIdentifier = local_rx_buf[6] - '0';
			commandIdentifier = local_rx_buf[8] - '0';
			//printf("Got to here 2");
			// In case the command was sent to this node...
			if (targetNodeIdentifier == MAC_BYTE)
			{
				//printf("Got to here 3, commandIdentfier %d", commandIdentifier);
				// Kommandos unterscheiden und entsprechend reagieren, TODO
				switch (commandIdentifier)
				{
				case 1:
					nrk_kprintf (PSTR("Got command 1: \n")); //(z.B. Battery Voltage, Event State etc.)
					if(directConnectionToCommander)
						transmitMessageTo = 1;
					responseDataReady = 1;
					break;

				case 2:
					nrk_kprintf (PSTR("Got command 2, shutting down the sensoring system and sending a response to commander: "));
					nodeSecure = 0;
					break;

				case 3:
					nrk_kprintf (PSTR("Got command 3, upping the sensoring system and sending a response to commander:"));
					nodeSecure = 1;
					break;

				case 4:
					if(!directConnectionToCommander && !gotConnectionToCommander)
					{
						printf ("Got command 4, routing all messages over the node %c which has direct connection to commander.", local_rx_buf[4]);
						transmitMessageTo = local_rx_buf[4] - '0';
						bmac_addr_decode_enable();				// Verlangsamt - disabled das die ankommenden Nachrichten?!
						//bmac_addr_decode_dest_mac( transmitMessageTo );
						gotConnectionToCommander = 1;
						if(IR_ACCEL_NODE)
							nrk_activate_task(&WARNING_TASK);
						//TELL OTHER NODES TO SEND MESSAGES OVER ME HERE (MORE Command 4 packets!).
					}
					else if(directConnectionToCommander)
					{
						bufferSize = sprintf(tx_buf, "%d %d", INITIALISATION_BYTE, MAC_BYTE);  // Hier bestimmen was verschickt werden sollte.
						//printf("Sending: %s with BufSize = %d and Ctr = %d\n",tx_buf, bufferSize, init_ctr);
						bmac_addr_decode_dest_mac(1);
						bmac_set_rf_power(txPower);
						/*while(val != NRK_OK)
							val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
						 */
						val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));
						if(val==NRK_OK) cnt++;
						else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
						val=-1;
						printf("Additional initialisation packet sent.");
					}
					break;
				case 5:

					nrk_kprintf (PSTR("Got command 5: "));  		// Fetch info from all nodes
					if(directConnectionToCommander)
					{
						routingDataReady = 1;
						transmitMessageTo = 1;							// Weil nur der direkte Nachbar vom Commander
						// das empfangen kann!

						if(IR_ACCEL_NODE)
							nrk_activate_task(&WARNING_TASK);

						nrk_kprintf(PSTR("Sending out the command 1 to all registered nodes. TODO\n"));
						uint8_t forwardFlag=0;
						for( counter = 0; counter < counterNodesInVincinity; counter++ )
						{
							//while(packetToForward)
							//	nrk_wait_until_next_period ();
							if(IDsIdentified[counter] != 1 && forwardFlag==0)
							{
								for (i = 0; i < len; i++)
								{
									if(i == 6)
										forward_rx_buf[i] = (char)(((int)'0')+IDsIdentified[counter]);
									else if (i == 8)
										forward_rx_buf[i] = '6';
									else
										forward_rx_buf[i] = local_rx_buf[i];  		// In separaten globalen Forwarding Array schreiben
								}
								forward_rx_buf[len] = '\0';
								printf("This package is being sent to node %c", forward_rx_buf[6]);

								forward_rx_len = len;							   // Weil der Kommando-Array immer aus 9 Zeichen besteht
								forwardFlag = 1;
							}

							if(IDsIdentified[counter] != 1 && forwardFlag==1)
							{
								for (i = 0; i < len; i++)
								{
									if(i == 6)
										forward_rx_buf2[i] = (char)(((int)'0')+IDsIdentified[counter]);
									else if (i == 8)
										forward_rx_buf2[i] = '6';
									else
										forward_rx_buf2[i] = local_rx_buf[i];  		// In separaten globalen Forwarding Array schreiben
								}
								forward_rx_buf2[len] = '\0';
								printf("This package is being sent to node %c", forward_rx_buf2[6]);

								forward_rx_len2 = len;						   // Weil der Kommando-Array immer aus 9 Zeichen besteht
								forwardFlag = 2;
							}
						}
						if(forwardFlag == 1)
							packetToForward = 1;					  	   // Forward Flag auf 1
						else if(forwardFlag == 2)
							packetToForward = 2;					  	   // Forward Flag auf 1
					}

					//for (i=len; i<len+5; i++)
					//	forward_rx_buf[i] = ' ';
					break;

					// Share neighbors to the commanding node
				case 6:

					//if(!routingTableShared)
					//{
					if(IR_ACCEL_NODE)
						nrk_activate_task(&WARNING_TASK);

					printf("Got commando 6. ");
					routingDataReady = 1;
					gotConnectionToCommander = 1;
					//}
					break;
				}
			}
			else // Im Falle, dass das Paket nicht an diesen Node verschickt wurde und weitergeleitet sein soll
			{
				if(directConnectionToCommander)				// NEW: Allgemein einschalten?
				{
					printf("The package needs to be forwarded to node %d!", targetNodeIdentifier);

					for (i = 0; i < len; i++)
						forward_rx_buf[i] = local_rx_buf[i];  // In separaten globalen Forwarding array schreiben
					forward_rx_len = len;						 // Weil der Kommando-Array immer aus 9 Zeichen besteht
					forward_rx_buf[len] = '\0';
					packetToForward = 1;					  // global Flag auf 1
				}
			}
		}
		// c) Response info/Route info/Event info package identifier
		else if( (local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '3')
				|| (local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '1')
				|| (local_rx_buf[0] == '2' && local_rx_buf[1] == '5' && local_rx_buf[2] == '2'))
		{
			if (directConnectionToCommander)
			{
				nrk_kprintf(PSTR("R This response package needs to be forwarded to node 1!"));

				for (i = 0; i < len; i++)
					forward_rx_buf[i] = local_rx_buf[i];  // in separaten globalen Forwarding array schreiben
				forward_rx_buf[6] = '1';
				forward_rx_len = len;
				forward_rx_buf[len] = '\0';
				packetToForward = 1;					  // global Flag auf 1
			}
			else
			{
				printf("R This response package needs to be forwarded to a node %d!", transmitMessageTo);
				for (i = 0; i < len; i++)
					forward_rx_buf[i] = local_rx_buf[i];  // in separaten globalen Forwarding array schreiben
				forward_rx_buf[6] = (char)(((int)'0')+transmitMessageTo);
				forward_rx_len = len;
				forward_rx_buf[len] = '\0';
				packetToForward = 1;					  // global Flag auf 1
			}
		}
		else
		{
			nrk_kprintf(PSTR("Redundant package received: ["));
			for (i = 0; i < len; i++)
				printf("%c", local_rx_buf[i]);
			nrk_kprintf(PSTR("]"));
		}

		nrk_led_clr(GREEN_LED);
		// Release the RX buffer so future packets can arrive
		bmac_rx_pkt_release ();
	}
}

void transmit_task() {

	char localString[2];

	while (!bmac_started())
		//printf("BMAC not started yet!\n");
		nrk_wait_until_next_period();

	for (;;)
	{	//printf("was here5;");
		if(routingDataReady == 1)
		{

			printf("S Should be sending route info soon to %d.", transmitMessageTo);
			bmac_addr_decode_dest_mac(transmitMessageTo);

			bufferSize = sprintf(tx_buf, "%d %d %d %d%s", ROUTING_BYTE, MAC_BYTE, 1, MAC_BYTE, IDsIdentifiedStr);
			printf("S Sending Routing data: %s to node %d",tx_buf,transmitMessageTo);

			nrk_led_set (RED_LED);
			bmac_set_rf_power(txPower);
			/*hile(val != NRK_OK)
				val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			 */
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;

			nrk_kprintf(PSTR("S Sent!"));
			// Task gets control again after TX complete
			nrk_led_clr (RED_LED);
			routingDataReady = 0;
			routingTableShared = 1;
		}
		if(responseDataReady == 1)
		{
			bmac_addr_decode_dest_mac(transmitMessageTo);
			//Battery Voltage as Data to test tx_task - B
			adc_init();
			voltage = adc_GetBatteryVoltage();
			voltage_int = voltage * 10;

			if(TEMPERATURE_BRIGHTNESS_NODE)
			{
				temp = 25;
				bright = 250;
				temp = mda100_TemperatureSensor_GetDegreeCelsius();
				bright = mda100_LightSensor_GetCounts();
			}
			else if(IR_ACCEL_NODE)
			{
				//ir = 25;
				temp = 0;
				bright = 0;
				ir = nrk_gpio_get(UART1_TXD); //8.Reihe C-te Spalte aufm Board
				//accel = mts310cb_Accelerometer_x_GetCounts();
				//accel = mts310cb_Accelerometer_y_GetCounts(); - Vorsicht! Neues Data für den Puffer. 2 mal accel (accel_x und accel_y) - B
			}

			adc_Powersave();
			bufferSize = sprintf(tx_buf, "%d %d %d %d %d %d %d %d %d", RESPONSE_BYTE, MAC_BYTE, 1, voltage_int, temp, bright, ir, accel, nodeSecure);
			printf("S Sending: %s to node %d",tx_buf,transmitMessageTo);
			//bufferSize = sprintf(tx_buf, "%d Battery Voltage of node %d: %.1lf", packetNumber, MAC_BYTE, voltage);

			nrk_led_set (RED_LED);
			bmac_set_rf_power(txPower);
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;

			nrk_kprintf(PSTR("S Sent!"));
			// Task gets control again after TX complete
			nrk_led_clr (RED_LED);
			responseDataReady = 0;
		}
		if(sendWarningEvent == 1)
		{
			bmac_addr_decode_dest_mac(transmitMessageTo);
			//Battery Voltage as Data to test tx_task - B
			adc_init();
			voltage = adc_GetBatteryVoltage();
			voltage_int = voltage * 10;
			uint8_t ir2 = 0;

			if(TEMPERATURE_BRIGHTNESS_NODE)
			{
				temp = 25;
				bright = 250;
				temp = mda100_TemperatureSensor_GetDegreeCelsius();
				bright = mda100_LightSensor_GetCounts();
			}
			else if(IR_ACCEL_NODE)
			{
				temp = 0;
				bright = 0;
				ir2 = 1;
			}

			adc_Powersave();
			bufferSize = sprintf(tx_buf, "%d %d %d %d %d %d %d %d %d", EVENT_BYTE, MAC_BYTE, 1, voltage_int, temp, bright, ir2, accel, nodeSecure);
			printf("S Sending Warning: %s to node %d",tx_buf,transmitMessageTo);
			//bufferSize = sprintf(tx_buf, "%d Battery Voltage of node %d: %.1lf", packetNumber, MAC_BYTE, voltage);

			nrk_led_set (RED_LED);
			bmac_set_rf_power(txPower);
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;
			ir = 0;
			nrk_kprintf(PSTR("W Warning Sent!"));
			// Task gets control again after TX complete
			nrk_led_clr (RED_LED);
			sendWarningEvent = 0;
		}
		if(packetToForward == 1)
		{
			nrk_led_set (YELLOW_LED);

			transmitMessageTo = forward_rx_buf[6] - '0';
			bmac_addr_decode_dest_mac(transmitMessageTo);

			printf("F Forwarding package to node %d: [", transmitMessageTo);

			for (i = 0; i < forward_rx_len; i++) {
				tx_buf[i] = forward_rx_buf[i];
				printf ("%c", forward_rx_buf[i]);
			}
			tx_buf[forward_rx_len] = '\0';
			nrk_led_clr (YELLOW_LED);

			printf ("]");

			//bmac_set_rf_power(txPower);
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;
			nrk_kprintf(PSTR("F Forwarded!"));
			packetToForward = 0;
		}
		if(packetToForward == 2)
		{
			nrk_led_set (YELLOW_LED);

			transmitMessageTo = forward_rx_buf[6] - '0';
			bmac_addr_decode_dest_mac(transmitMessageTo);

			printf("F Forwarding package to node %d: [", transmitMessageTo);

			for (i = 0; i < forward_rx_len; i++) {
				tx_buf[i] = forward_rx_buf[i];
				printf ("%c", forward_rx_buf[i]);
			}
			tx_buf[forward_rx_len] = '\0';
			nrk_led_clr (YELLOW_LED);

			printf ("]");

			//bmac_set_rf_power(txPower);
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;
			nrk_kprintf(PSTR("F Forwarded!"));

			//**********
			nrk_led_set (YELLOW_LED);

			transmitMessageTo = forward_rx_buf2[6] - '0';
			bmac_addr_decode_dest_mac(transmitMessageTo);

			printf("F Forwarding package to node %d: [", transmitMessageTo);

			for (i = 0; i < forward_rx_len2; i++) {
				tx_buf[i] = forward_rx_buf2[i];
				printf ("%c", forward_rx_buf2[i]);
			}
			tx_buf[forward_rx_len2] = '\0';
			nrk_led_clr (YELLOW_LED);

			printf ("]");

			//bmac_set_rf_power(txPower);
			val = bmac_tx_pkt(tx_buf, strlen((char *)tx_buf));  // Start sending...
			if(val==NRK_OK) cnt++;
			else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
			val = -1;
			nrk_kprintf(PSTR("F Forwarded!"));

			packetToForward = 0;
		}
		nrk_wait_until_next_period ();
	}
}

void warning_task()
{
	while (!bmac_started())
		//printf("BMAC not started yet!\n");
		nrk_wait_until_next_period();

	printf("Warning_task begins\n");
	uint8_t counter = 0;
	for (;;) {
		ir = 0;
		//Do the check here if something changed on the IR Sensor since the last time
		ir = nrk_gpio_get(UART1_TXD); //8.Reihe C-te Spalte aufm Board
		printf("GPIO tried, got %d.\n", ir);

		if(ir && nodeSecure && counter ==0)
		{
			nrk_led_set(RED_LED);
			printf("Sending warning packet to node 1.");

			sendWarningEvent = 1; // Handle in TX and commander appropriately
			//  Task gets control again after TX complete
			//	bmac_disable();
			nrk_led_clr(RED_LED);
		}
		if(ir && nodeSecure)
			counter++;
		if(counter==40)
			counter=0;
		nrk_wait_until_next_period();
	}
}

void nrk_create_taskset() {
	RX_TASK.task = rx_task;
	nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
	RX_TASK.prio = 2;
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

	TX_TASK.task = transmit_task;
	nrk_task_set_stk(&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
	TX_TASK.prio = 2;
	TX_TASK.FirstActivation = TRUE;
	TX_TASK.Type = BASIC_TASK;
	TX_TASK.SchType = PREEMPTIVE;
	TX_TASK.period.secs = 0;
	TX_TASK.period.nano_secs = 70 * NANOS_PER_MS;
	TX_TASK.cpu_reserve.secs = 0;
	TX_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	TX_TASK.offset.secs = 0;
	TX_TASK.offset.nano_secs = 0;
	nrk_activate_task(&TX_TASK);

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

	WARNING_TASK.task = warning_task;
	nrk_task_set_stk(&WARNING_TASK, warning_task_stack, NRK_APP_STACKSIZE);
	WARNING_TASK.prio = 1;
	WARNING_TASK.FirstActivation = TRUE;
	WARNING_TASK.Type = BASIC_TASK;
	WARNING_TASK.SchType = PREEMPTIVE;
	WARNING_TASK.period.secs = 0;
	WARNING_TASK.period.nano_secs = 200 * NANOS_PER_MS;
	WARNING_TASK.cpu_reserve.secs = 0;
	WARNING_TASK.cpu_reserve.nano_secs = 300 * NANOS_PER_MS;
	WARNING_TASK.offset.secs = 0;
	WARNING_TASK.offset.nano_secs = 0;
	//nrk_activate_task(&WARNING_TASK);
}
