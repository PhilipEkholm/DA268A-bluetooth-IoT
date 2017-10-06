/*
 * Description: This is a sketch for the BTBee Standalone module implementing
 * a Arduino Nano (Atmel Mega 168) running as a Bluetooth client.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:  1.0
 *  Author:   alex.rodzevski@mah.se
 *
 */

#include "SoftwareSerial.h"

#define RX_PIN           2
#define TX_PIN           3
#define CONN_STAT_PIN    10  /* Arduino Digital I/O 10 (Atmel PB2), O:disconnected 1:connected */
#define DIS_CONN_PIN     A2  /* Arduino Analogue Input 2 (Atmel PC2), disconnect on rising edge */

#define SERIAL_DEFAULT_BAUD_RATE    9600
#define BT_SERIAL_DEFAULT_BAUD_RATE 38400

#define DISCONNECT() do {           \
  digitalWrite(DIS_CONN_PIN, HIGH); \
  delay(50);                        \
  digitalWrite(DIS_CONN_PIN, LOW);  \
} while (0)

/* Comment out to de-activate debug prints */
#define DEBUG
#ifdef DEBUG
#define D_WRITE(x)    Serial.write(x)
#define D_PRINT(x)    Serial.print(x)
#define D_PRINTLN(x)  Serial.println(x)
#else
#define D_WRITE(x)
#define D_PRINT(x)
#define D_PRINTLN(x)
#endif

SoftwareSerial blueToothSerial(RX_PIN,TX_PIN);

char term_buf[64];
char bt_buf[64];

void setup()
{
	Serial.begin(SERIAL_DEFAULT_BAUD_RATE);

	pinMode(CONN_STAT_PIN, INPUT);
	pinMode(DIS_CONN_PIN, OUTPUT);
	digitalWrite(DIS_CONN_PIN, LOW);

	memset(term_buf, '\0', sizeof(term_buf));
	memset(bt_buf, '\0', sizeof(bt_buf));

	Serial.println("Starting BTBee Stand-alone comm");
	blueToothSerial.begin(BT_SERIAL_DEFAULT_BAUD_RATE);
	delay(500);
	bt_config();
}

void loop()
{
	static int i = 0, master_conn = 0;

	/* Handle incomming Bluetooth strings */
	while (blueToothSerial.available() && i < sizeof(bt_buf)) {
		bt_buf[i++] = blueToothSerial.read();
		delay(1);
	}

	/* Parse the string upon new-line */

	if (bt_buf[i-1] == '\n') {
	  	Serial.println(bt_buf);

		if(strstr(bt_buf, "LINK LOSS")) {
			DISCONNECT();
			master_conn = 0;
		}
		else if(strstr(bt_buf, "CONNECT:OK")) {
			delay(200);

			while (blueToothSerial.available()) {
				Serial.write(blueToothSerial.read());
				delay(1);
			}
			delay(2000);

			blueToothSerial.println("\nHello Master, I will loopback everything you send me!\n");
			master_conn = 1;
		}
		else if (master_conn) {
			blueToothSerial.println(bt_buf);
		}

		/* clear buffer & pointer */
		memset(bt_buf, '\0', sizeof(bt_buf));
		i = 0;
	}
}

/*
 * serialEvent, callback function for chars comming in on the
 * USB-UART serial line (PC Terminal or similar).
 */
void serialEvent()
{
	static int i = 0;

	/* Handle incomming Terminal strings  */
	while (Serial.available() && i < sizeof(term_buf)) {
		term_buf[i++] = Serial.read();
		delay(1);
	}

	/* Parse the string upon new-line */
	if (term_buf[i-1] == '\n') {
		D_PRINT("\nTERM:");
		D_PRINTLN(term_buf);

		if(strncmp(term_buf, "CONF", strlen("CONF")) == 0)
			bt_config();
		else if(strncmp(term_buf, "DISC", strlen("DISC")) == 0)
			DISCONNECT();
		else if (digitalRead(CONN_STAT_PIN))
			blueToothSerial.println(term_buf);

		/* clear buffer & pointer */
		memset(term_buf, '\0', sizeof(term_buf));
		i = 0;
	}
}

void bt_config()
{
	sendBlueToothCommand("STNA=BTBeeStandalone");
	sendBlueToothCommand("RTADDR");      /* Print local address */
	sendBlueToothCommand("STPIN=1234");  /* PIN = 1234 */
	sendBlueToothCommand("STWMOD=0");    /* 0 = slave */
	sendBlueToothCommand("STAUTO=0");    /* 0 = Auto-connect forbidden */
	sendBlueToothCommand("STOAUT=1");    /* 1 = Permit Paired device to connect me */
	delay(2000);                         /* Required delay for state to change */
	sendBlueToothCommand("INQ=1");       /* 1 = Enable been inquired (slave) */
}

void sendBlueToothCommand(char cmd[])
{
	char bt_comm[48], cmd_resp[80];
	unsigned long start_time;
	int i;

	if (strlen(cmd) > (sizeof(bt_comm) - 5)) {
		Serial.println("Error, too long BT cmd");
		return;
	}

	/* Send the BT command according to the module-syntax:
	 * |\r|\n|+|command_chars|\r|\n|
	 */
	i = 0;
	memset(bt_comm, '\0', sizeof(bt_comm));

	bt_comm[i++] = '\r';
	bt_comm[i++] = '\n';
	bt_comm[i++] = '+';
	strncpy(&bt_comm[i], cmd, strlen(cmd));

	i += strlen(cmd);
	bt_comm[i++] = '\r';
	bt_comm[i] = '\n';
	blueToothSerial.print(bt_comm);

	/* Wait for cmd response and look for "OK" */
	i = 0;
	memset(cmd_resp, '\0', sizeof(cmd_resp));
	start_time = millis();

	while (millis() < (start_time + 5000)) {
		if (blueToothSerial.available()) {
			cmd_resp[i++] = blueToothSerial.read();
			D_WRITE(cmd_resp[i-1]);
		}

		if (strstr(cmd_resp, "OK")) {
			while(blueToothSerial.available() && blueToothSerial.read() != -1);
				return;
	  	}
	}

	Serial.println("\nCMD Time-Out");
}



















