#include "bootloader.h"



#define STM32_ACK	0x79
#define STM32_NACK	0x1F
#define STM32_BUSY	0x76

#define STM32_CMD_INIT	0x7F
#define STM32_CMD_GET	0x00	/* get the version and command supported */

#define ELAR            0x04 // Extended Linear Address Record
#define DR              0x00 // Data Record
#define SLAR            0x05 // Start Linear Address Record
#define EFR             0x01 // End of File Record
#define NONE            0x10 // none

#define STM32_TIMEOUT	    1	/* seconds */


const int NUM_BYTES_IN_PAGE_LO          = 1024;     //1kb
const int NUM_BYTES_IN_PAGE_MD          = 1024;     //1Kb
const int NUM_BYTES_IN_PAGE_HD          = 2048;     //2kb
const int NUM_BYTES_IN_PAGE_F2          = 16384;    //16kb
const int NUM_BYTES_IN_PAGE_H7          = 128*1024; //128kb
const int NUM_BYTES_IN_PAGE_F4_CUSTOM   = 128*1024; //128Kb
/// SECTOR                                0           1           2           3           4           5           6           7           8           9           10          11
static const int F4_MAP[]             = { 16,        16,         16,         16,         64,         128,        128,        128,        128,        128,        128,        128 };
static const uint64_t F4_SECTOR_END_MAP [] = { 0X08003FFF, 0X08007FFF, 0X0800BFFF, 0X0800FFFF, 0X0801FFFF, 0X0803FFFF, 0X0805FFFF, 0X0807FFFF, 0X0809FFFF, 0X080BFFFF, 0X080DFFFF, 0X080FFFFF, };

void *start_bootloader_read_thread(void *args);
// ----------------------------------------------------------------------------------
//   Status bar
// ------------------- ---------------------------------------------------------------

void Boot_loader::update_status_bar(int progress, int total, const std::string& status) {
    if (debug) return;
    
    if (progress <= 0) progress = 0;
    
    int bar_width = 50;
    float percent = (float)progress / total;
    int pos = bar_width * percent;
    std::cout << "\r" << std::string(100, ' ') << "\r"; 
    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }

    if (percent >= 100)
    {
        percent = 100;
        std::cout<< "\33[92m" << "] " << int(percent * 100.0) << "% " << status << "\033[0m" <<"\r";
    }
    else{
        std::cout << "] " << int(percent * 100.0) << "% " << status << "\r";
    }
    
    std::cout.flush();
}
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
static uint64_t get_time_usec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000000 + _timestamp.tv_usec;
}

static uint64_t get_time_msec()
{
    static struct timeval _timestamp;
    gettimeofday(&_timestamp, NULL);
    return _timestamp.tv_sec * 1000 + _timestamp.tv_usec / 1000;
}
// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------

Boot_loader::~Boot_loader()
{
    time_to_exit = true;
    usleep(1000);
    _serial_port->reset();
    usleep(1000000);
    this -> _serial_port->close_serial();
    this -> _serial_port = NULL;
    pthread_mutex_destroy(&mutex);
    pthread_cancel(read_tid);
}

Boot_loader::Boot_loader(Serial_Port *serial_port,string path)
{
    this -> filename = path;
    this -> _serial_port = serial_port;
    pthread_mutex_init(&mutex, NULL);
}


bool Boot_loader::init()
{
    uint8_t len, val, buf[257];
    int i, new_cmds;
    _serial_port->reset();
    usleep(1000000);
    if (_serial_port->status != SERIAL_PORT_BOOT)
    {
        _serial_port->open_serial(Serial_Port::BOOT_MODE);
    }
    usleep(100000);
    if (_serial_port->status == SERIAL_PORT_BOOT)
    {
        init_flag = true;
    }else
    {
        GSDK_DebugError("Cannot go into BootLoader mode!\n")
        return false;
    }

    int result = pthread_create(&read_tid, NULL, &start_bootloader_read_thread, this);
    if (result) throw result;

    binArray = vector<bin_t>(20);
    
    return true;    
    
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------

bool Boot_loader::run()
{
    if(!init_flag){
        GSDK_DebugError("BootLoader is not initialized yet!\n")
        return false;
    }
    uint8_t total_e = 0,len, val, buf[257] ,total = 100, progess = 0;
    string status =  "Parse Hex File";
    while (1)
    {
        switch (state)
        {
        case PARSER_HEX:
            if (parse_hex_sep() == ERR_OK)
            {
                if (debug)
                {
                    for (size_t i = 0; i < binArrayCount; i++)
                    {
                        GSDK_DebugInfo("Star:0x%08x - End: 0x%08x",binArray[i].address,binArray[i].address+binArray[i].flash_count);
                    }
                }
                GSDK_DebugSuccess("[\u2713] %s",status.c_str());
                status = "Connecting";
                state = CONNECT;
            }
            else
            {
                clear_buffer();
                total_e++;
            }
                break;
        case CONNECT:
            if (send_connect_seq() == ERR_OK) // Connect with GIMBAL
            {
                GSDK_DebugSuccess("[\u2713] %s",status.c_str());
                status  = "Get CMD";
                state = GET_CMD;   
                usleep(4000000);
            }
            else
            {
                clear_buffer();
                usleep(10000);
                _serial_port->boot();
                usleep(10000);
                total_e++;
            }
            break;
        case GET_CMD:
            if (send_get_cmd_seq() == ERR_OK)
            {
                if (debug)
                {
                    for (size_t i = 0; i < sizeof(stm32_cmd)/sizeof(uint8_t); i++)
                    {
                        GSDK_DebugInfo("CMD [%ld]:0x%2x",i, *((&cmd.get)+i))
                    }
                }
                GSDK_DebugSuccess("[\u2713] %s",status.c_str());
                status  = "Get ID";
                state = GET_ID ;
            }
            else
            {
                clear_buffer();
                total_e++;
            }
            break;
        case GET_ID:
            if (send_get_id_seq() == ERR_OK)
            {
                if (debug)
                {
                    GSDK_DebugInfo("Chip name: %s",device->name.c_str());
                }
                GSDK_DebugSuccess("[\u2713] %s",status.c_str());
                status  = "Erase memory";
                state = ERASE;
            }
            else
            {
                clear_buffer();
                total_e++;
            }
            break;
        case ERASE:
            if (erase_sep() == ERR_OK)
            {
                GSDK_DebugSuccess("[\u2713] %s",status.c_str());
                status  = "Flash memory";
                state = WRITE;
                update_status_bar(0, 100, status);
                usleep(5000000);
            }
            else
            {
                clear_buffer();
                total_e++;
            }
            break;
        case WRITE:

            if (write_seq() == ERR_OK)
            {
                state = COMPLETE;
            }
            else
            {
                clear_buffer();
                total_e++;
            }
            break;
        case COMPLETE:
            usleep(10000);
            GSDK_DebugSuccess("\nCode flashed successfully!");
            return true;
            break;
        default:
            break;
        }
        
        if (total_e > 20)
        {
            GSDK_DebugError("\nCannot running, failed in step %s!",status.c_str());
            return false;
        }
        usleep(100000);
    }
    return true;
}
// ------------------------------------------------------------------------------
//   Helper function
// ------------------------------------------------------------------------------

Boot_loader::err_t Boot_loader::send_connect_seq()
{
    uint8_t ack ,cmd = STM32_CMD_INIT;
    _serial_port -> write_buf(&cmd,1);
    return  get_ack_timeout(cmd,STM32_TIMEOUT);
}

Boot_loader::err_t Boot_loader::send_get_cmd_seq()
{
    uint16_t len = 0;
    uint8_t buf[17];
    uint8_t val;
    err_t result =  send_command_timeout(STM32_CMD_GET,STM32_TIMEOUT);
    if (result != ERR_OK) return result;
    len = read_buffer(buf,1);
    if (len != 1) return ERR_UNKNOWN;
    len = buf[0];   // length of cmd
    len = read_buffer(buf + 1, 0);
    if (len != buf[0] + 2) return ERR_UNKNOWN;
    bl_version = buf[1];
    for (size_t i = 1; i <= buf[0] + 1; i++)
    {
        val = buf[i + 1];
        *((&cmd.get)+(i - 1)) = val;
    }

    return ERR_OK;
}

Boot_loader::err_t Boot_loader::send_get_id_seq()
{
    uint16_t len = 0;
    uint8_t buf[20];
    uint8_t val;
    err_t result =  send_command_timeout(cmd.gid,STM32_TIMEOUT);
    if (result != ERR_OK) return result;
    if(read_buffer(buf, 1) != 1 || buf[0] + 1 < 2) return ERR_UNKNOWN;
    if(read_buffer(buf + 1, buf[0] + 1) != buf[0] + 1) return ERR_UNKNOWN;
    pid = (buf[1] << 8) | buf[2];
    if (debug)
    {
        if (buf[0] + 1 > 2) {
            GSDK_DebugInfo( "This bootloader returns %d extra bytes in PID:", buf[0] + 1);
            for (int i = 3; i <= buf[0] + 1 ; i++)
                GSDK_DebugInfo( " %02x", buf[i]);
	    }
    }
    
    if (debug)
    {
        GSDK_DebugInfo("Device ID: 0x%03x", pid);
    }

    for (size_t i = 0; i < 7; i++)  // MAX DEVICES
    {
        if (devices[i].ID == pid)
        {
            
            device = &devices[i];
            return ERR_OK;
        }
        
    };
    GSDK_DebugError("Cannot find a consistent chip!");
    return ERR_UNKNOWN;
}

Boot_loader::err_t Boot_loader::parse_hex_sep()
{
    bool flag = false;
    string line;
    uint32_t writeBaseAddess;
    ifstream file(filename);
	if (!file.is_open()) 
    {
        if (debug)
        {
            GSDK_DebugError("Cannot open this file!");
        }

        return ERR_UNKNOWN;
    }
    while (getline(file, line))
    {
        if (line[0] != ':') {
            GSDK_DebugError("This file is not a hex file!");
            file.close();
            return ERR_UNKNOWN;
        }
        string recType      = "";
        string recLen       = "";
        string recOffset    = "";
        string recData      = "";
        uint32_t len        = 0;
        uint32_t offset     = 0;
        uint32_t type       = 0;

        recLen = line.substr(1,2);
        len = stringhex_to_byte(recLen);

        recOffset = line.substr(3,4);
        offset = stringhex_to_byte(recOffset);

        recType = line.substr(7,2);
        type = stringhex_to_byte(recType);

        recData = line.substr(9,(int)(2 * len));
        
        if (type == DR)
        {
            if (debug)
            {
                if (!flag)
                {
                    flag = true;
                    GSDK_DebugInfo("Write hexfile DR!");
                }
               
            }
            string byteData = "";
            uint32_t byteIndex = 0;
            int recDataLen = recData.length();
            int index = binArrayCount - 1;

            for (int i = 0; i < recDataLen; i += 2)
            {
                byteData = recData.substr(i, 2);
                binArray[index].flash_data[offset + byteIndex] = (uint8_t)stringhex_to_byte(byteData);
                byteIndex++;
            }
            /// den so hang cua mang count len
            binArray[index].flash_count = offset + byteIndex;

            /// size nap cho chip
            binArraySize += byteIndex;
        }
        else if (type == ELAR)
        {
            
            uint64_t SIZE = 128*1024;
            writeBaseAddess = stringhex_to_byte(recData + recOffset);
            binArray[binArrayCount].address     = writeBaseAddess;
            binArray[binArrayCount].flash_data  = vector<uint8_t>(SIZE);
            binArray[binArrayCount].flash_index = 0;
            binArray[binArrayCount].flash_count = 0;
            usleep(10000);
            /// default data
            for (uint64_t i = 0; i < SIZE; i++)
            {
                binArray[binArrayCount].flash_data[i] = 0xFF;
            }
            if(binArrayCount != 0)
            {
                binArraySize = binArray[binArrayCount].address - binArray[0].address;
            }

            binArrayCount++;

            if (debug)
            {
                flag = false;
                GSDK_DebugInfo("Write hexfile ELAR! base address: %s " , uint32_to_hexstring(writeBaseAddess).c_str());
            }
            
        }
        else if (type == SLAR)
        {
            if (debug)
            {
                flag = false;
                GSDK_DebugInfo("Write hexfile SLAR!");
            }
        }
        else if (type == EFR)
        {
            if (debug)
            {
                flag = false;
                GSDK_DebugInfo("Write hexfile EFR! size all: %u ",binArraySize);
            }
            file.close();
            return ERR_OK;
        }
        
    }
    if (debug)
    {
        GSDK_DebugError("Cannot parse this file!");
    }
    file.close();
    return ERR_UNKNOWN;
}
int Boot_loader::erase_page_get()
{
    if (device->ID == devices[(int)LOW_DENSITY].ID)
    {
        return (int)(binArraySize / NUM_BYTES_IN_PAGE_LO) + 1;
    }
    else if (device->ID == devices[(int)MED_DENSITY].ID)
    {
        return (int)(binArraySize / NUM_BYTES_IN_PAGE_MD) + 1;
    }
    else if (device->ID == devices[(int)HIG_DENSITY].ID)
    {
        return (int)(binArraySize / NUM_BYTES_IN_PAGE_HD) + 1;
    }
    else if (device->ID == devices[(int)F2XX].ID)
    {
        return (int)(binArraySize / NUM_BYTES_IN_PAGE_F2) + 1;
    }
    else if (device->ID == devices[(int)H7XX].ID)
    {
        return (int)(binArraySize / NUM_BYTES_IN_PAGE_H7) + 1;
    }
    else if (device->ID == devices[(int)F4XX_CUSTOM].ID)
    {
        int index = binArrayCount - 1; /// get end index of the array
        uint size = binArray[index].address + binArray[index].flash_count;
        /// Checking size in MCU

        for(int i = 5; i < 9; i++)
        {
            if(size < F4_SECTOR_END_MAP[i])
            {
                return i - 4;
            }
        }
    }
    else 
    {
        int index = binArrayCount - 1;  /// get end index of the array
        uint size = binArray[index].address + binArray[index].flash_count;
        /// Checking size in MCU
        for (int i = 0; i < 9; i++)
        {
            if (size < F4_SECTOR_END_MAP[i])
            {
                return i + 1;
            }
        }
    }
    return 0;
}
void Boot_loader::erase_extended_memory(int numPages)
{
    if (numPages == 0xFF) //Erase full flash
    {
        uint8_t buff[3];

        buff[0] = 0xFF;
        buff[1] = 0xFF;
        buff[2] = 0x00;

        _serial_port -> write_buf(buff,3);
        return;
    }
    
    uint8_t checksum = 0;
    uint8_t buff[2 * numPages + 3];


    buff[0] = 0x00;         
    checksum = (uint8_t)(checksum ^ buff[0]);

    buff[1] = (uint8_t)(numPages - 1);
    checksum = (uint8_t)(checksum ^ buff[1]);
    _serial_port -> write_buf(buff,2);
    for (int i = 0; i < numPages; i++)
    {
        buff[2 * i + 2] = 0x00;
        checksum = (uint8_t)(checksum ^ buff[2 * i + 2]);

        if (device->ID == devices[(int)F4XX_CUSTOM].ID)
        {
            buff[2 * i + 3] = (uint8_t)(5 + i);
            checksum = (uint8_t)(checksum ^ buff[2 * i + 3]);
        }
        else
        {
            buff[2 * i + 3] = (uint8_t)i;
            checksum = (uint8_t)(checksum ^ buff[2 * i + 3]);
        }
    }

    buff[2 * numPages + 2] = checksum;
    _serial_port -> write_buf(buff + 2,2 * numPages + 1);
}

void Boot_loader::erase_memory(int numPages)
{
    
    if (numPages == 0xFF) //Erase full flash
    {
        uint8_t checksum = 0;
        uint8_t buff[2];

        buff[0] = 0xFF;
        checksum = (uint8_t)(checksum ^ buff[0]);

        buff[1] = 0x00;
        checksum = (uint8_t)(checksum ^ buff[1]);

        _serial_port -> write_buf(buff,2);
        return;
    }
    
    uint8_t checksum = 0;
    uint8_t buff[2 * numPages + 2];

    buff[0] = numPages - 1;
    checksum = (checksum ^ buff[0]);

    for (int i = 0; i < numPages; i++)
    {
        buff[2 * i + 1] = 0x00;
        checksum = (uint8_t)(checksum ^ buff[2 * i + 1]);

        if (device->ID == devices[(int)F4XX_CUSTOM].ID)
        {
            buff[2 * i + 2] = (uint8_t)(5 + i);
            checksum = (uint8_t)(checksum ^ buff[2 * i + 2]);
        }
        else
        {
            buff[2 * i + 2] = (uint8_t)i;
            checksum = (uint8_t)(checksum ^ buff[2 * i + 2]);
        }
    }
    buff[2 * numPages + 1] = checksum;
    _serial_port -> write_buf(buff,2 * numPages + 2);
}

Boot_loader::err_t Boot_loader::erase_sep()
{
    err_t result =  send_command_timeout(cmd.er,STM32_TIMEOUT);
    if (result != ERR_OK) return result;
    int numPages = erase_page_get();
    if (cmd.er == 0x44)
    {
        erase_extended_memory(numPages);
    }
    else if (cmd.er == 0x43)
    {
        erase_memory(numPages);
    }

    if (debug)
    {
        GSDK_DebugInfo("Erase pages size: 0x%02x",numPages);
    }
    return get_ack_timeout(cmd.er,STM32_TIMEOUT);
}   
Boot_loader::err_t Boot_loader::write_mem(uint32_t address,const uint8_t data[], unsigned int len){
    uint8_t cs, buf[256 + 2];
	unsigned int i, aligned_len;
    err_t s_err;
	if (!len)
		return ERR_OK;

	if (len > 256) {
        if (debug)
        {
            GSDK_DebugError("Error: WRITE length limit at 256 bytes\n");
        }
		return ERR_UNKNOWN;
	}

	/* must be 32bit aligned */
	if (address & 0x3) {
        if (debug)
        {
            GSDK_DebugError("Error: WRITE address must be 4 byte aligned\n");
        }
		return ERR_UNKNOWN;
	}
    /*write command*/
    s_err = send_command_timeout(cmd.wm,STM32_TIMEOUT);
    if (s_err != ERR_OK ) return s_err;
    address = be_u32(address);
	buf[0] = (uint8_t)(address & 0xFF);
	buf[1] = (uint8_t)((address >> 8) & 0xFF);
	buf[2] = (uint8_t)((address >> 16) & 0xFF);
    buf[3] = (uint8_t)((address >> 24) & 0xFF);
	buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
    _serial_port -> write_buf(buf, 5);
    s_err = get_ack_timeout(cmd.wm,STM32_TIMEOUT);
    if (s_err != ERR_OK ) return s_err;
    /*data*/
    aligned_len = (len + 3) & ~3;
	cs = aligned_len - 1;
	buf[0] = aligned_len - 1;
	for (i = 0; i < len; i++) {
		cs ^= data[i];
		buf[i + 1] = data[i];
	}
    /* padding data */
	for (i = len; i < aligned_len; i++) {
		cs ^= 0xFF;
		buf[i + 1] = 0xFF;
	}
    buf[aligned_len + 1] = cs;
    _serial_port->write_buf(buf, aligned_len + 2);
    s_err = get_ack_timeout(cmd.wm,STM32_TIMEOUT);

    return s_err;
}
Boot_loader::err_t Boot_loader::read_mem(uint32_t address,uint8_t data[], unsigned int len){

	uint8_t buf[5];
	if (!len)
		return ERR_OK;

	if (len > 256) {
		fprintf(stderr, "Error: READ length limit at 256 bytes\n");
		return ERR_UNKNOWN;
	}

	if (!cmd.rm) {
        if (debug)
        {
            GSDK_DebugError( "Error: READ command not implemented in bootloader.\n");
        }
		return ERR_NO_CMD;
	}
    err_t result = send_command_timeout(cmd.rm,STM32_TIMEOUT);
	if (result != ERR_OK) return result;
    address = be_u32(address);
	buf[0] = (uint8_t)(address & 0xFF);
	buf[1] = (uint8_t)((address >> 8) & 0xFF);
	buf[2] = (uint8_t)((address >> 16) & 0xFF);
    buf[3] = (uint8_t)((address >> 24) & 0xFF);
	buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
	_serial_port->write_buf(buf, 5);

    result = get_ack_timeout(0xFF,STM32_TIMEOUT);
    if (result != ERR_OK) return result;
    
    result = send_command_timeout(len - 1, STM32_TIMEOUT);
	if (result != ERR_OK) return result;

    uint64_t now = get_time_msec();
	while (get_time_msec() < now + 1000* STM32_TIMEOUT){
	    uint16_t length = read_buffer(data,len);
        if (length == len) return ERR_OK;
    }

    if (debug)
    {
        GSDK_DebugError("Time out on read memory!");
    }
    
	return ERR_TIMEOUT;
}

Boot_loader::err_t Boot_loader::write_seq(){
    int progess = 0;
    string status  = "Flash memory";
    for (size_t i = 0; i < binArrayCount; i++)
    {
        unsigned int max_wlen, max_rlen;
        max_wlen = STM32_MAX_TX_FRAME - 2;	/* skip len and crc */
		max_wlen &= ~3;	/* 32 bit aligned */

		max_rlen = STM32_MAX_RX_FRAME;
		max_rlen = max_rlen < max_wlen ? max_rlen : max_wlen;
        while (binArray[i].flash_index < binArray[i].flash_count)
        {
            uint32_t left = binArray[i].flash_count - binArray[i].flash_index;
            uint32_t len  = max_wlen > left ? left : max_wlen;
            len           = len > binArray[i].flash_count -  binArray[i].flash_index ? binArray[i].flash_count -  binArray[i].flash_index : len;

            if (len == 0)
            {
                return ERR_UNKNOWN;
            }
            if (debug)
            {
                GSDK_DebugInfo("Write address: 0x%08x - index: %u - count: %u",binArray[i].address,binArray[i].flash_index,binArray[i].flash_count);
            }
			err_t s_err = write_mem( binArray[i].address, (&binArray[i].flash_data[0] + binArray[i].flash_index), len);
			if (s_err != ERR_OK) {
                if (debug)
                {
				    GSDK_DebugError("Failed to send cmd write memory ");
                }
                return s_err;
			}
            binArraySizeIndex += len;
            progess = (int)(binArraySizeIndex * 100 / binArraySize);
            update_status_bar(progess, 100, status);

            binArray[i].address += len;
            binArray[i].flash_index += len; 
            usleep(10000);
        }
    }
    return ERR_OK;
}

void Boot_loader::send_command(const uint8_t cmd)
{
    uint8_t buf[2];

	buf[0] = cmd;
	buf[1] = cmd ^ 0xFF;

    _serial_port->write_buf(buf,2);
}

Boot_loader::err_t Boot_loader::get_ack_timeout(uint8_t cmd,uint8_t timeout){
	uint8_t byte;
    uint8_t len;
    uint64_t now = get_time_msec();
	while (get_time_msec() < now + 1000* timeout){
		len = read_buffer(&byte,1);
        if (len >= 1)
        {
            if (byte == STM32_ACK)
                return ERR_OK;
            if (byte == STM32_NACK){
                if (debug)
                {
                    GSDK_DebugWarning("Got NACK from device on command 0x%02x\n", cmd);  
                }
                return ERR_NACK;
            }
            if (byte != STM32_BUSY) {
                if (debug)
                {
                    GSDK_DebugWarning("Unexpected reply from device on command 0x%02x\n", cmd);
                }
                
                return ERR_UNKNOWN;
            }
        }
        
        usleep(3000);
	} 
    if (debug)
    {
        GSDK_DebugWarning("Timeout on command 0x%02x\n", cmd);
    }
    return ERR_TIMEOUT;
}
Boot_loader::err_t Boot_loader::send_command_timeout(uint8_t cmd , uint8_t timeout){
    send_command(cmd);
    return get_ack_timeout(cmd,timeout);
}
// ------------------------------------------------------------------------------
//  W/R rxBuffer
// ------------------------------------------------------------------------------
void Boot_loader::clear_buffer(){
    pthread_mutex_lock(&mutex);
    memset(rxBuffer.data ,0 ,sizeof(rxBuffer.data));
    rxBuffer.head = 0;
    rxBuffer.tail = 0;
    pthread_mutex_unlock(&mutex);
}
void Boot_loader::write_buffer(const uint8_t *buf, uint16_t len){
   
    size_t remain;
    if(rxBuffer.head >= rxBuffer.tail){ // check available size in buffer
        remain = STM32_MAX_RX_FRAME - (rxBuffer.head - rxBuffer.tail);
    }else{
        remain = rxBuffer.tail - rxBuffer.head;
    }
    if (remain >= len)    // rxBuffer is not full
    {
        pthread_mutex_lock(&mutex);
        for (size_t i = 0; i < len; i++)
        {
            rxBuffer.data[rxBuffer.head] = buf[i];
            rxBuffer.head = (rxBuffer.head + 1) % STM32_MAX_RX_FRAME;
        }
        pthread_mutex_unlock(&mutex);
    }
    
}

uint16_t Boot_loader::read_buffer(uint8_t *buf, uint16_t len){
    
    uint16_t available;
    if(rxBuffer.head >= rxBuffer.tail){ // check available size in buffer
        available = rxBuffer.head - rxBuffer.tail;
    }else{
        available = STM32_MAX_RX_FRAME - rxBuffer.tail + rxBuffer.head;
    }
    
    if (available == 0) 
    {
        
        return 0;
    }

    if (len == 0) // Get full buffer
    {
        pthread_mutex_lock(&mutex);
        for (size_t i = 0; i < available; i++)
        {
            buf[i] = rxBuffer.data[rxBuffer.tail];
            rxBuffer.tail = (rxBuffer.tail + 1) % STM32_MAX_RX_FRAME;
        }
        pthread_mutex_unlock(&mutex);
        return available;
    }

    if (available >= len)
    {
        pthread_mutex_unlock(&mutex);
        for (size_t i = 0; i < len; i++)
        {
            buf[i] = rxBuffer.data[rxBuffer.tail];
            rxBuffer.tail = (rxBuffer.tail + 1) % STM32_MAX_RX_FRAME;
        }
        pthread_mutex_unlock(&mutex);
        return len;
    }
    
    return 0;
}
// ------------------------------------------------------------------------------
//  Read thread
// ------------------------------------------------------------------------------
void Boot_loader::read_thread(){
    while (!time_to_exit) {
        if (_serial_port->status == SERIAL_PORT_BOOT)
        {
            char buf[256] = { 0 };
            int result = _serial_port->read_message(buf, 256);
            if (result > 0)
            {
                write_buffer((uint8_t *)buf,result);
		}
        }
    }
}
void Boot_loader::start_read_thread(){
    this->read_thread();
}
// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void *start_bootloader_read_thread(void *args)
{
    // takes an bootloader object argument
    Boot_loader *boot_loader = (Boot_loader *)args;
    // run the object's read thread
    boot_loader->start_read_thread();
    // done!
    return NULL;
}