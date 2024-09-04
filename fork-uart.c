/**
 * @file    fork-uart.c
 *
 * @brief Serial Port Programming in C (Serial Port Read)
 * Non Cannonical mode
 * Sellecting the Serial port Number on Linux
 * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc
 * termios structure -  /usr/include/asm-generic/termbits.h
 * use "man termios" to get more info about  termios structure
 * @author  Kevin Cotton
 * @date    2024-08-02
 */
#define _GNU_SOURCE

#include <stdio.h>	 // For printf and fgets
#include <stdlib.h>	 // For exit
#include <string.h>	 // For strlen and strncmp
#include <unistd.h>	 // For read, write, close, and fork
#include <fcntl.h>	 // For open
#include <termios.h> // For struct termios
#include <sys/wait.h>  // Include for wait()

void vInitPortSerie();

int fd; // File Descriptor

// device port série à utiliser
// const char *portTTY = "/dev/ttyGS0";
// const char *portTTY = "/dev/ttyS0";
const char *portTTY = "/dev/ttyS1";
// const char *portTTY = "/dev/ttyS2";
// const char *portTTY = "/dev/ttyS3";
// const char *portTTY = "/dev/ttyS4";
// const char *portTTY = "/dev/ttyS5";
// const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

void main(void)
{
	pid_t pid;

	vInitPortSerie();

	pid = fork();

	if (pid == 0)
	{
		// Processus enfant
		printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console... \n");
		// Write data to serial port
		char user_input[256];

		// Main loop: get input from user and send it to the serial port
		while (1)
		{
			static unsigned char ucReadValue;
			ucReadValue = getchar();

			// terminate the process if 'q' is entered by the user
			if (ucReadValue == 'q')
			{
				return;
			}

			// Write user input to the serial port
			int written_bytes = write(fd, &ucReadValue, sizeof(ucReadValue));
		}
	}
	else
	{
		// Processus parent
		printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port serie...\n");
		// Read data from serial port
		tcflush(fd, TCIFLUSH); // Discards old data in the rx buffer
		char read_buffer = '0';  // Buffer to store the data received
		int bytes_read = 0;	   // Number of bytes read by the read() system call
		int i = 0;

		while (read_buffer != '!')
		{
			bytes_read = read(fd, &read_buffer, 1); // Read the data

			printf(" Bytes Recus : %d --> ", bytes_read); // Print the number of bytes read
			for (i = 0; i < bytes_read; i++)			  // printing only the received characters
				printf("%c", read_buffer);
			printf("\n");
		}
		close(fd);
		wait(NULL);
	}
}

void vInitPortSerie()
{
	// Opening the Serial Port
	fd = open(portTTY, O_RDWR | O_NOCTTY);
	// O_RDWR   - Read/Write access to serial port
	// O_NOCTTY - No terminal will control the process
	// Open in blocking mode,read will wait
	// Setting the Attributes of the serial port using termios structure
	struct termios SerialPortSettings;	// Create the structure
	tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed
	// 8N1 Mode
	SerialPortSettings.c_cflag &= ~PARENB;		  // Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;		  // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;		  // Clears the mask for setting the data size
	SerialPortSettings.c_cflag |= CS8;			  // Set the data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;		  // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non Cannonical mode, Disable echo, Disable signal

	SerialPortSettings.c_oflag &= ~OPOST; // No Output Processing

	// Setting Time outs
	SerialPortSettings.c_cc[VMIN] = 1;	// Read at least 1 character(s)
	SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinitely (in increments of 100ms, 0 for indefinetly)
}
