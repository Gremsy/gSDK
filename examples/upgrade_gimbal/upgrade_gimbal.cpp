#include "upgrade_gimbal.h"
#include <iostream>
#include <chrono>
#include <thread>


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, string &path)
{
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -p <path>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path
        if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--path") == 0) {
            if (argc > i + 1) {
                path = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }

    if (path == "")
    {
        GSDK_DebugError("A path is required\n");
        printf("%s\n", commandline_usage);
        throw EXIT_FAILURE;
    }
    
    // end: for each input argument
    // Done!
}

int main(int argc, char **argv)
{
    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------
    // Default input arguments   
    #ifdef __APPLE__
    char *uart_name = (char *)"/dev/tty.usbmodem1";
    #else
    char *uart_name = (char *)"/dev/ttyUSB0";
    #endif
    int baudrate = 115200;
    string path = "";
    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate, path);
    int time = 0;
    int s = 0 , f = 0;


    Serial_Port serial_port(uart_name, baudrate);
    Boot_loader boot_loader(&serial_port, path);

    //FLASH CODE
    boot_loader.init();
    boot_loader.run();

    return 1;
}
