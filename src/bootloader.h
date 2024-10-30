#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#include "serial_port.h"
#include "gimbal_protocol.h"
#include "gimbal_interface.h"
#include "parsers/utils.h"
#include <string>
#include <iostream>
#include <sys/time.h>
#include <cstring> 
#include <fstream>
#include <vector>


using namespace std;

#define STM32_MAX_RX_FRAME	256	/* cmd read memory */
#define STM32_MAX_TX_FRAME	(1 + 256 + 1)	/* cmd write memory */

#define STM32_MAX_PAGES		0x0000ffff
#define STM32_MASS_ERASE	0x00100000 /* > 2 x max_pages */

class Boot_loader
{
public:
    typedef enum {
        ERR_OK = 0,
        ERR_UNKNOWN,	/* Generic error */
        ERR_NACK,
        ERR_TIMEOUT,
        ERR_NO_CMD,	/* Command not available in bootloader */
    }err_t;

    Boot_loader(Serial_Port *serial_port , string path);
    Boot_loader() = delete;
    ~Boot_loader();

    bool init();
    bool run();

    void start_read_thread(void);

private:

    struct buffer
    {
        uint8_t  data[STM32_MAX_RX_FRAME];
        uint16_t head;
        uint16_t tail;
    };
    
    enum State{
        PARSER_HEX,
        CONNECT,
        GET_CMD,
        GET_ID,
        ERASE,
        WRITE,
        COMPLETE,
    };

    enum stm32_name_t{
        LOW_DENSITY = 0,
        MED_DENSITY = 1,
        HIG_DENSITY = 2,
        F2XX = 3,
        F4XX = 4,
        H7XX = 5,
        F4XX_CUSTOM = 6,
        COUNT,
    } ;

    struct stm32_cmd {
	uint8_t get;
	uint8_t gvr;
	uint8_t gid;
	uint8_t rm;
	uint8_t go;
	uint8_t wm;
	uint8_t er; /* this may be extended erase */
	uint8_t wp;
	uint8_t uw;
	uint8_t rp;
	uint8_t ur;
	uint8_t	crc;
    };

    struct devices_t
    {
        uint16_t ID;
        string name;
        uint32_t sramStart, sramEnd;
        uint32_t flashStart, flashEnd;
        uint32_t flashKbpp;
        uint32_t optStart, optEnd;
        uint32_t sysMemStart, sysMemEnd;
        uint32_t numBytes;
    };

    struct bin_t
    {
        uint32_t address;
        uint32_t flash_count;
        uint32_t flash_index;
        vector<uint8_t>  flash_data;
    };


    std::vector<bin_t>  binArray;
    uint16_t binArrayCount  = 0;
    uint32_t binArraySize = 0;
    uint32_t binArraySizeIndex = 0;

    stm32_cmd   cmd;
    buffer      rxBuffer        = {0};
    uint8_t	    bl_version      = 0;
    uint16_t    pid             = 0;
    bool        debug           = 0;
    bool        time_to_exit    = false;
    bool        init_flag       = false;
    uint8_t     state           = 0;
    devices_t   *device         = NULL;
    string		filename;
    devices_t   devices[7]      = 
    {
            //{0x412, "STM32F10xxx-Low-density", 0x20000000, 0x20000FFF, 0x08000000, 0x08007FFF, 1, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF},
            {
                .ID = 0x412,
                .name = "STM32F10xxx-Low-density",
                .sramStart = 0x20000000,
                .sramEnd = 0x20000FFF,
                .flashStart = 0x08000000,
                .flashEnd = 0x08007FFF,
                .flashKbpp = 1,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
            //{ 0x410, "STM32F10xxx-Medium-density", 0x20000000, 0x20004FFF, 0x08000000, 0x0801FFFF, 1, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF},
            {    
                .ID = 0x410,
                .name = "STM32F10xxx-Medium-density",
                .sramStart = 0x20000000,
                .sramEnd = 0x20004FFF,
                .flashStart = 0x08000000,
                .flashEnd = 0x0801FFFF,
                .flashKbpp = 1,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
            // { 0x414, "STM32F10xxx-High-density", 0x20000000, 0x2000BFFF, 0x08000000, 0x0807FFFF, 2, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF},
            {
                .ID = 0x414,
                .name = "STM32F10xxx-High-density",
                .sramStart = 0x20000000,
                .sramEnd = 0x2000BFFF,
                .flashStart = 0x08000000,
                .flashEnd = 0x0807FFFF,
                .flashKbpp = 2,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
            //{ 0x411, "STM32F2xxxx", 0x20000000, 0x2000BFFF, 0x08000000, 0x0807FFFF, 2, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF},
            {
                .ID = 0x411,
                .name = "STM32F2xxxx",
                .sramStart = 0x20000000,
                .sramEnd = 0x2000BFFF,
                .flashStart = 0x08000000,
                .flashEnd = 0x0807FFFF,
                .flashKbpp = 2,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
            //{ 0x413, "STM32F40xx/41xxx", 0x20000000, 0x2000BFFF, 0x08000000, 0x0807FFFF, 2, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF}
            {
                .ID = 0x413,
                .name = "STM32F40xx/41xxx",
                .sramStart = 0x20000000,
                .sramEnd = 0x2000BFFF,
                .flashStart = 0x08000000,
                .flashEnd = 0x0807FFFF,
                .flashKbpp = 2,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
            //{ 0x450, "STM32H7xx", 0x20000000, 0x2000BFFF, 0x08020000, 0x080BFFFF, 2, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF}
            {
                .ID           = 0x450,
                .name         = "STM32H7xx",
                .sramStart    = 0x20000000,
                .sramEnd      = 0x2000BFFF,
                .flashStart   = 0x08000000,
                .flashEnd     = 0x08200000,
                .flashKbpp    = 2,
                .optStart     = 0x1FFFF800,
                .optEnd       = 0x1FFFF80F,
                .sysMemStart  = 0x1FFFF000,
                .sysMemEnd    = 0x1FFFF7FF,
            },

            //{ 0x420, "STM32F4_CUSTOM", 0x20000000, 0x2000BFFF, 0x08020000, 0x080BFFFF, 2, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF7FF}
            {    
                .ID = 0x420,
                .name = "STM32F4_CUSTOM",
                .sramStart = 0x20000000,
                .sramEnd = 0x2000BFFF,
                .flashStart = 0x08020000,
                .flashEnd = 0x080BFFFF,
                .flashKbpp = 2,
                .optStart = 0x1FFFF800,
                .optEnd = 0x1FFFF80F,
                .sysMemStart = 0x1FFFF000,
                .sysMemEnd = 0x1FFFF7FF,
            },
    };

    Serial_Port *_serial_port;
    pthread_mutex_t mutex;
    pthread_t read_tid  = 0;

    void clear_buffer();
    void write_buffer(const uint8_t *buf, uint16_t len);
    uint16_t read_buffer(uint8_t *buf, uint16_t len);
    void  read_thread(void);
    int   erase_page_get();
    void  erase_extended_memory(int numPages);
    void  erase_memory(int numPages);
    void  send_command(uint8_t cmd);
    err_t get_ack_timeout(uint8_t cmd ,uint8_t timeout); // Time out in second
    err_t send_command_timeout(uint8_t cmd , uint8_t timeout);
    void  update_status_bar(int progress, int total, const std::string& status);
    err_t send_connect_seq();
    err_t send_get_cmd_seq();
    err_t send_get_id_seq();
    err_t parse_hex_sep();
    err_t write_mem(uint32_t address,const uint8_t data[], unsigned int len);
    err_t read_mem(uint32_t address,uint8_t data[], unsigned int len);

    err_t erase_sep();
    err_t write_seq();
};


#endif // SERIAL_PORT_H_