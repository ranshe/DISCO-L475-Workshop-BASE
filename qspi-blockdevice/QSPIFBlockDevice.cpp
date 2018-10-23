
/* mbed Microcontroller Library
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#include "./mbed-os/drivers/qspi.h"

#define DEVICE_QSPI 1

#include "QSPIFBlockDevice.h"

/* Default QSPIF Parameters */
/****************************/
#define QSPIF_DEFAULT_READ_SIZE  1
#define QSPIF_DEFAULT_PROG_SIZE  1
#define QSPIF_DEFAULT_SE_SIZE    4096
#define QSPI_MAX_STATUS_REGISTER_SIZE 2
#define QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC 50
#define QSPIF_DEFAULT_TIMEOUT_MSEC    1


/* SFDP Header Parsing */
/***********************/
#define QSPIF_SFDP_HEADER_SIZE 8
#define QSPIF_PARAM_HEADER_SIZE 8

/* Basic Parameters Table Parsing */
/**********************************/
#define SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES 64 /* 16 DWORDS */
//READ Instruction support according to BUS Configuration
#define QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE 2
#define QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE 16
#define QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE 27
#define QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE 9
#define QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE 11
#define QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE 23
#define QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE 15
#define QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE 13
#define QSPIF_BASIC_PARAM_TABLE_PAGE_SIZE_BYTE 40
// Quad Enable Params
#define QSPIF_BASIC_PARAM_TABLE_QER_BYTE 58
#define QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE 56
// Erase Types Params
#define QSPIF_BASIC_PARAM_ERASE_TYPE_1_BYTE 29
#define QSPIF_BASIC_PARAM_ERASE_TYPE_2_BYTE 31
#define QSPIF_BASIC_PARAM_ERASE_TYPE_3_BYTE 33
#define QSPIF_BASIC_PARAM_ERASE_TYPE_4_BYTE 35
#define QSPIF_BASIC_PARAM_ERASE_TYPE_1_SIZE_BYTE 28
#define QSPIF_BASIC_PARAM_ERASE_TYPE_2_SIZE_BYTE 30
#define QSPIF_BASIC_PARAM_ERASE_TYPE_3_SIZE_BYTE 32
#define QSPIF_BASIC_PARAM_ERASE_TYPE_4_SIZE_BYTE 34
#define QSPIF_BASIC_PARAM_4K_ERASE_TYPE_BYTE 1

// Debug Printouts
#define QSPIF_DEBUG_ERROR     1
#define QSPIF_DEBUG_WARNING 2
#define QSPIF_DEBUG_INFO     3
#define QSPIF_DEBUG_DEBUG     4
#define QSPIF_DEBUG_TRACE     5
// #define QSPIF_DEFAULT_DEBUG_LEVEL  QSPIF_DEBUG_INFO
// #define QSPIF_DEFAULT_DEBUG_LEVEL  QSPIF_DEBUG_TRACE
#define QSPIF_DEFAULT_DEBUG_LEVEL  QSPIF_DEBUG_WARNING


enum qspif_default_instructions {
    QSPIF_NOP  = 0x00, // No operation
    QSPIF_PP = 0x02, // Page Program data
    QSPIF_READ = 0x03, // Read data
    QSPIF_SE   = 0x20, // 4KB Sector Erase
    QSPIF_SFDP = 0x5a, // Read SFDP

    QSPIF_WRSR = 0x01, // Write Status/Configuration Register
    QSPIF_WRDI = 0x04, // Write Disable
    QSPIF_RDSR = 0x05, // Read Status Register
    QSPIF_WREN = 0x06, // Write Enable

    QSPIF_RSTEN = 0x66, // Reset Enable
    QSPIF_RST = 0x99, // Reset
    QSPIF_RDID = 0x9f, // Read Manufacturer and JDEC Device ID
};

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define QSPIF_LOG(level,...) {\
    	if (level <= QSPIF_DEFAULT_DEBUG_LEVEL) {\
    	  char str[256];\
      	  sprintf(str, "\n[%s][%s:%d], ", __FILENAME__, __FUNCTION__, __LINE__);\
            sprintf(str+strlen(str),__VA_ARGS__);\
      	  printf(str);\
    	}\
    }

// Mutex is used for some QSPI Driver commands that must be done sequentially with no other commands in between
// e.g. (1)Set Write Enable, (2)Program, (3)Wait Memory Ready
SingletonPtr<PlatformMutex> QSPIFBlockDevice::_mutex;


/********* Public API Functions *********/
/****************************************/

QSPIFBlockDevice::QSPIFBlockDevice(PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName csel,
                                   int clock_mode, int freq)
    : _qspi(io0, io1, io2, io3, sclk, csel, clock_mode), _deviceSizeBytes(0)
{
    //_cs = 1;
    is_initialized = false;
    _minCommonEraseSize = 0;
    _regions_count = 1;
    _region_erase_types[0] = 0;

    //Default Bus Setup 1_1_1 with 0 dummy and mode cycles
    _inst_width = QSPI_CFG_BUS_SINGLE;
    _address_width = QSPI_CFG_BUS_SINGLE;
    _address_size = QSPI_CFG_ADDR_SIZE_24;
    _alt_width = QSPI_CFG_BUS_SINGLE;
    _alt_size = QSPI_CFG_ALT_SIZE_8;
    _data_width = QSPI_CFG_BUS_SINGLE;
    _dummy_and_mode_cycles = 0;

    if (QSPI_STATUS_OK != _qspiSetFrequency(freq)) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Set Frequency Failed");
    }
}


int QSPIFBlockDevice::init()
{

    uint8_t vendor_device_ids[4];
    size_t data_length = 3;

    _mutex->lock();
    if (is_initialized == true) {
        _mutex->unlock();
        return 0;
    }

    // Soft Reset
    if ( -1 == _resetFlashMem()) {
        QSPIF_LOG(QSPIF_DEBUG_INFO, "ERROR: init - Unable to initialize flash memory, tests failed\n");
        return -1;
    } else {
        QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO: Initialize flash memory OK\n");
    }



    /* Read Manufacturer ID (1byte), and Device ID (2bytes)*/
    int status = _qspiSendReadCommand(QSPIF_RDID, (char *)vendor_device_ids, 0x0 /*address*/, data_length);
    if (status != QSPI_STATUS_OK) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Read Vendor ID Failed");
        return status;
    }

    switch (vendor_device_ids[0]) {
        case 0xbf:
            // SST devices come preset with block protection
            // enabled for some regions, issue write disable instruction to clear
            _setWriteEnable();
            //_cmdwrite(0x98, 0, 0, 0x0, NULL);
            _qspiSendGeneralCommand(QSPIF_WRDI, -1, NULL, 0, NULL, 0);

            break;
    }

    //Synchronize Device
    if ( false == _isMemReady()) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - _isMemReady Failed");
        return BD_ERROR_DEVICE_ERROR;
    }


    /**************************** Parse SFDP Header ***********************************/
    uint32_t basic_table_addr = NULL;
    size_t basic_table_size = 0;
    uint32_t sector_map_table_addr = NULL;
    size_t sector_map_table_size = 0;
    if ( 0 != _sfdpParseSFDPHeaders(basic_table_addr, basic_table_size, sector_map_table_addr, sector_map_table_size)) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Parse SFDP Headers Failed");
        return BD_ERROR_DEVICE_ERROR;
    }


    /**************************** Parse Basic Parameters Table ***********************************/
    if ( 0 != _sfdpParseBasicParamTable(basic_table_addr, basic_table_size) ) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Parse Basic Param Table Failed");
        return BD_ERROR_DEVICE_ERROR;
    }


    /**************************** Parse Sector Map Table ***********************************/
    _region_size_bytes[0] =
        _deviceSizeBytes; // If there's no region map, we have a single region sized the entire device size
    _region_high_boundary[0] = _deviceSizeBytes - 1;

    if ( (sector_map_table_addr != NULL) && (0 != sector_map_table_size) ) {
        QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO: init - Parsing Sector Map Table - addr: 0x%xh, Size: %d", sector_map_table_addr,
                  sector_map_table_size);
        if ( 0 != _sfdpParseSectorMapTable(sector_map_table_addr, sector_map_table_size) ) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Parse Sector Map Table Failed");
            return BD_ERROR_DEVICE_ERROR;
        }
    }


    // Configure  BUS Mode to 1_1_1 for all commands other than Read
    _qspiConfiureFormat( QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                         QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);
    /*// Soft Reset
    if( -1 == _resetFlashMem()) {
        QSPIF_LOG(QSPIF_DEBUG_INFO,"ERROR: init - Unable to initialize flash memory, tests failed\n");
        return -1;
    } else {
        QSPIF_LOG(QSPIF_DEBUG_INFO,"INFO: Initialize flash memory OK\n");
    }*/

    is_initialized = true;
    _mutex->unlock();

    return 0;
}

int QSPIFBlockDevice::deinit()
{
    _mutex->lock();
    if (is_initialized == false) {
        _mutex->unlock();
        return 0;
    }
    qspi_status_t status = _qspiSendGeneralCommand(QSPIF_WRDI, -1, NULL, 0, NULL, 0);
    int result = 0;

    if (status != QSPI_STATUS_OK)  {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Write Disable failed");
        result = -1;
    }
    is_initialized = false;
    _mutex->unlock();

    return result;
}


int QSPIFBlockDevice::read(void *buffer, bd_addr_t addr, bd_size_t size)
{


    int status = 0;

    QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO Inst: 0x%xh", _readInstruction);

    _mutex->lock();

    _qspiConfiureFormat(
        _inst_width, //Bus width for Instruction phase
        _address_width, //Bus width for Address phase
        _address_size,
        _alt_width, //Bus width for Alt phase
        _alt_size,
        _data_width, //Bus width for Data phase
        _dummy_and_mode_cycles);
    //_qspiConfiureFormat( QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 8);

    if (QSPI_STATUS_OK != _qspiSendReadCommand(_readInstruction, buffer, addr, size)) {
        status = -1;
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Read failed\n");
    }

    // All commands other than Read use default 1-1-1 Bus mode (Program/Erase are constrained by flash memory performance less than that of the bus)
    _qspiConfiureFormat( QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                         QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);

    _mutex->unlock();
    return status;

}




int QSPIFBlockDevice::program(const void *buffer, bd_addr_t addr, bd_size_t size)
{
    qspi_status_t result = QSPI_STATUS_OK;
    bool program_failed = false;
    int status = 0;
    uint32_t offset = 0;
    uint32_t chunk = 0;
    bd_size_t writtenBytes = 0;

    while (size > 0) {

        // Write on _pageSizeBytes boundaries (Default 256 bytes a page)
        offset = addr % _pageSizeBytes;
        chunk = (offset + size < _pageSizeBytes) ? size : (_pageSizeBytes - offset);
        writtenBytes = chunk;

        _mutex->lock();

        //Send WREN
        if (_setWriteEnable() != 0) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Write Enabe failed\n");
            program_failed = true;
            goto Exit_Point;
        }

        if ( false == _isMemReady()) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Device not ready, write failed\n");
            program_failed = true;
            goto Exit_Point;
        }

        result = _qspiSendProgramCommand(_progInstruction, buffer, addr, &writtenBytes);
        if ( (result != QSPI_STATUS_OK) || (chunk != writtenBytes) ) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Write failed");
            program_failed = true;
            goto Exit_Point;
        }

        buffer = static_cast<const uint8_t *>(buffer) + chunk;
        addr += chunk;
        size -= chunk;

        wait_ms(QSPIF_DEFAULT_TIMEOUT_MSEC);

        if ( false == _isMemReady()) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Device not ready after write, failed\n");
            program_failed = true;
            goto Exit_Point;
        }
        _mutex->unlock();


    }

Exit_Point:
    if (program_failed) {
        _mutex->unlock();
        status = -1;
    }

    return status;
}


int QSPIFBlockDevice::erase(bd_addr_t addr, bd_size_t inSize)
{

    int type = 0;
    uint32_t chunk = 4096;
    unsigned int curEraseInst = _eraseInstruction;
    int size = (int)inSize;
    bool erase_failed = false;
    int status = 0;
    // Find region of erased address
    int region = _utilsFindAddrRegion((int)addr);
    // Erase Types of selected region
    uint8_t bitfield = _region_erase_types[region];


    while (size > 0) {

        // iterate to find next Largest erase type (1) supported by region, 2) smaller than size)
        // find the matching instruction and erase size chunk for that type.
        type = _utilsIterateNextLargestEraseType(bitfield, (int)size, (int)addr, _region_high_boundary[region]);
        curEraseInst = _eraseTypeInstArr[type];
        chunk = _eraseTypeSizeArr[type];

        QSPIF_LOG(QSPIF_DEBUG_DEBUG, " Debug: addr: 0x%xh, size:%d, Inst: 0x%xh, chunk: %d , ",
                  (int)addr, (int)size, curEraseInst, chunk);

        _mutex->lock();

        if (_setWriteEnable() != 0) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Erase - Write Enable failed");
            erase_failed = true;
            goto Exit_Point;
        }

        if ( false == _isMemReady()) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Erase Device not ready - failed");
            erase_failed = true;
            goto Exit_Point;
        }

        if (QSPI_STATUS_OK != _qspiSendEraseCommand(curEraseInst, addr, size) ) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Erase command failed!");
            erase_failed = true;
            goto Exit_Point;
        }

        addr += chunk;
        size -= chunk;

        if ( (size > 0) && (addr > _region_high_boundary[region]) ) {
            // erase crossed to next region
            region++;
            bitfield = _region_erase_types[region];
        }
        wait_ms(QSPIF_DEFAULT_TIMEOUT_MSEC);
        if ( false == _isMemReady()) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI After Erase Device not ready - failed\n");
            erase_failed = true;
            goto Exit_Point;
        }
        _mutex->unlock();

    }


Exit_Point:
    if (erase_failed) {
        _mutex->unlock();
        status = -1;
    }

    return status;
}



bd_size_t QSPIFBlockDevice::get_read_size() const
{
    return QSPIF_DEFAULT_READ_SIZE;
}

bd_size_t QSPIFBlockDevice::get_program_size() const
{
    return QSPIF_DEFAULT_PROG_SIZE;
}

bd_size_t QSPIFBlockDevice::get_erase_size() const
{
    return _minCommonEraseSize;
}


// Find minimal erase size supported by address region
bd_size_t QSPIFBlockDevice::get_erase_size(bd_addr_t addr)
{
    // Find region of current address
    int region = _utilsFindAddrRegion((int)addr);

    int minRegionEraseSize = _minCommonEraseSize;
    int8_t type_mask = 0x01;
    int i_ind = 0;

    if (region != -1) {
        type_mask = 0x01;

        for (i_ind = 0; i_ind < 4; i_ind++) {
            // loop through erase types supported by region
            if (_region_erase_types[region] & type_mask) {

                minRegionEraseSize = _eraseTypeSizeArr[i_ind];
                break;
            }
            type_mask = type_mask << 1;
        }

        if (i_ind == 4) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: no erase type was found for region addr");
        }
    }

    return (bd_size_t)minRegionEraseSize;

}

bd_size_t QSPIFBlockDevice::size() const
{
    return _deviceSizeBytes;
}


/*********************************************************/
/********** SFDP Parsing and Detection Functions *********/
/*********************************************************/

int QSPIFBlockDevice::_sfdpParseSectorMapTable(uint32_t sector_map_table_addr, size_t sector_map_table_size)
{
    uint8_t sector_map_table[SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES]; /* Up To 16 DWORDS = 64 Bytes */
    uint32_t tmpRegionSize = 0;
    int i_ind = 0;
    int prevBoundary = 0;
    // Default set to all type bits 1-4 are common
    _minCommonEraseType = 0x0F;

    qspi_status_t status = _qspiSendReadCommand(QSPIF_SFDP, (char *)sector_map_table, sector_map_table_addr /*address*/,
                           sector_map_table_size);
    if (status != QSPI_STATUS_OK) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Read SFDP First Table Failed");
        return -1;
    }

    //QSPIF_LOG(QSPIF_DEBUG_DEBUG,\nDEBUG: "Read table: %x %x %x %x %x %x %x %x %x[56] %x[57] %x[58] %x[59] %x[52]\n",sector_map_table[0],
    //		sector_map_table[1],
    //		sector_map_table[2],
    //		sector_map_table[3],
    //		sector_map_table[4],
    //		sector_map_table[5],
    //		sector_map_table[6],
    //		sector_map_table[7], sector_map_table[56], sector_map_table[57], sector_map_table[58], sector_map_table[59], sector_map_table[52]);

    // Currently we support only Single Map Descriptor
    if (! ( (sector_map_table[0] & 0x3) == 0x03 ) && (sector_map_table[1]  == 0x0) ) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Sector Map - Supporting Only Single! Map Descriptor (not map commands)");
        return -1;
    }

    _regions_count = sector_map_table[2] + 1;
    if (_regions_count > QSPIF_MAX_REGIONS) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Supporting up to %d regions, current setup to %d regions - fail",
                  QSPIF_MAX_REGIONS, _regions_count);
        return -1;
    }

    // Loop through Regions and set for each one: size, supported erase types, high boundary offset
    // Calculate minimum Common Erase Type for all Regions
    for (i_ind = 0; i_ind < _regions_count; i_ind++) {
        tmpRegionSize = ((*((uint32_t *)&sector_map_table[(i_ind + 1) * 4])) >> 8) & 0x00FFFFFF; // bits 9-32
        _region_size_bytes[i_ind] = (tmpRegionSize + 1) * 256; // Region size is 0 based multiple of 256 bytes;
        _region_erase_types[i_ind] = sector_map_table[(i_ind + 1) * 4] & 0x0F; // bits 1-4
        _minCommonEraseType &= _region_erase_types[i_ind];
        _region_high_boundary[i_ind] = (_region_size_bytes[i_ind] - 1) + prevBoundary;
        prevBoundary = _region_high_boundary[i_ind] + 1;
    }

    // Calc minimum Common Erase Size from _minCommonEraseType
    uint8_t type_mask = 0x01;
    for (i_ind = 0; i_ind < 4; i_ind++) {
        if (_minCommonEraseType & type_mask) {
            _minCommonEraseType = i_ind;
            _minCommonEraseSize = _eraseTypeSizeArr[i_ind];
            break;
        }
        type_mask = type_mask << 1;
    }

    if (i_ind == 4) {
        // No common erase type was found between regions
        _minCommonEraseType = 0;
        _minCommonEraseSize = -1;
    }

    return 0;
}


int QSPIFBlockDevice::_sfdpParseBasicParamTable(uint32_t basic_table_addr, size_t basic_table_size)
{
    uint8_t param_table[SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES]; /* Up To 16 DWORDS = 64 Bytes */

    qspi_status_t status = _qspiSendReadCommand(QSPIF_SFDP, (char *)param_table, basic_table_addr /*address*/,
                           basic_table_size);
    if (status != QSPI_STATUS_OK) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Read SFDP First Table Failed, Status %d",status);
        return -1;
    }

    QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Read table: %x %x %x %x %x %x %x %x %x[56] %x[57] %x[58] %x[59] %x[52]\n",
              param_table[0],
              param_table[1],
              param_table[2],
              param_table[3],
              param_table[4],
              param_table[5],
              param_table[6],
              param_table[7], param_table[56], param_table[57], param_table[58], param_table[59], param_table[52]);


    // Check address size, currently only supports 3byte addresses
    if ((param_table[2] & 0x4) != 0 || (param_table[7] & 0x80) != 0) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - verify 3byte addressing Failed");
        return -1;
    }

    // Get device density (stored in bits - 1)
    uint32_t density_bits = (
                                (param_table[7] << 24) |
                                (param_table[6] << 16) |
                                (param_table[5] << 8 ) |
                                param_table[4] );
    _deviceSizeBytes = (density_bits + 1) / 8;

    // Set Default read/program/erase Instructions
    _readInstruction = QSPIF_READ;
    _progInstruction = QSPIF_PP;
    _eraseInstruction = QSPIF_SE;

    // Set Page Size (QSPI write must be done on Page limits)
    _pageSizeBytes = _sfdpDetectPageSize(param_table);

    // Detect and Set Erase Types
    bool shouldSetQuadEnable = false;
    bool isQPIMode = false;
    _sfdpDetectEraseTypesInstAndSize(param_table, _erase4KInst, _eraseTypeInstArr, _eraseTypeSizeArr);
    _eraseInstruction = _erase4KInst;


    // Detect and Set fastest Bus mode (default 1-1-1)
    _sfdpDetectBestBusReadMode(param_table, shouldSetQuadEnable, isQPIMode, _readInstruction);

    if (true == shouldSetQuadEnable) {
        // Set Quad Enable and QPI Bus modes if Supported
        QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO: init - Setting Quad Enable");
        _sfdpSetQuadEnabled(param_table);
        if (true == isQPIMode) {
            QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO: init - Setting QPI mode");
            _sfdpSetQPIEnabled(param_table);
        }
    }
    return 0;
}

int QSPIFBlockDevice::_sfdpParseSFDPHeaders(uint32_t& basic_table_addr, size_t& basic_table_size,
        uint32_t& sector_map_table_addr, size_t& sector_map_table_size)
{
    uint8_t sfdp_header[QSPIF_SFDP_HEADER_SIZE];
    uint8_t param_header[QSPIF_PARAM_HEADER_SIZE];
    size_t data_length = QSPIF_SFDP_HEADER_SIZE;
    bd_addr_t addr = 0x0;

    // Set 1-1-1 bus mode for SFDP header parsing
    _qspiConfiureFormat( QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE, QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                         QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 8);

    qspi_status_t status = _qspiSendReadCommand(QSPIF_SFDP, (char *)sfdp_header, addr /*address*/, data_length);
    if (status != QSPI_STATUS_OK) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Read SFDP Failed");
        return -1;
    }


    QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG11: Read SFDP Header: %x %x %x %x %x %x %x %x\n", sfdp_header[0],
              sfdp_header[1],
              sfdp_header[2],
              sfdp_header[3],
              sfdp_header[4],
              sfdp_header[5],
              sfdp_header[6],
              sfdp_header[7]);


    // Verify SFDP signature for sanity
    // Also check that major/minor version is acceptable
    if (!(memcmp(&sfdp_header[0], "SFDP", 4) == 0 && sfdp_header[5] == 1)) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - _verify SFDP signature and version Failed");
        return -1;
    } else {
        QSPIF_LOG(QSPIF_DEBUG_INFO, "INFO: init - verified SFDP Signature and version Successfully");
    }

    // Discover Number of Parameter Headers
    int number_of_param_headers = (int)(sfdp_header[6]) + 1;
    QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: number of Param Headers: %d", number_of_param_headers);


    addr += QSPIF_SFDP_HEADER_SIZE;
    data_length = QSPIF_PARAM_HEADER_SIZE;

    // Loop over Param Headers and parse them (currently supported Basic Param Table and Sector Region Map Table)
    for (int i_ind = 0; i_ind < number_of_param_headers; i_ind++) {

        status = _qspiSendReadCommand(QSPIF_SFDP, (char *)param_header, addr, data_length);
        if (status != QSPI_STATUS_OK) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: init - Read Param Table %d Failed", i_ind + 1);
            return -1;
        }
        // The SFDP spec indicates the standard table is always at offset 0
        // in the parameter headers, we check just to be safe
        if (param_header[2] != 1) {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Param Table %d - Major Version should be 1!", i_ind + 1);
            return -1;
        }

        if ((param_header[0] == 0) && (param_header[7] == 0xFF)) {
            // Found Basic Params Table: LSB=0x00, MSB=0xFF
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Found Basic Param Table at Table: %d", i_ind + 1);
            basic_table_addr = ( (param_header[6] << 16) | (param_header[5] << 8) | (param_header[4]) );
            // Supporting up to 64 Bytes Table (16 DWORDS)
            basic_table_size = ((param_header[3] * 4) < SFDP_DEFAULT_BASIC_PARAMS_TABLE_SIZE_BYTES) ? (param_header[11] * 4) : 64;

        } else if ((param_header[0] == 81) && (param_header[7] == 0xFF)) {
            // Found Sector Map Table: LSB=0x81, MSB=0xFF
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Found Sector Map Table at Table: %d", i_ind + 1);
            sector_map_table_addr = ( (param_header[6] << 16) | (param_header[5] << 8) | (param_header[4]) );

            sector_map_table_size = param_header[3] * 4;

        }
        addr += QSPIF_PARAM_HEADER_SIZE;

    }
    return 0;
}



int QSPIFBlockDevice::_sfdpSetQPIEnabled(uint8_t *basicParamTablePtr)
{
    uint8_t config_reg[1];

    // QPI 4-4-4 Enable Procedure is specified in 5 Bits
    uint8_t en_seq_444_value = ( ((basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE] & 0xF0) >> 4) | ((
                                     basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_444_MODE_EN_SEQ_BYTE + 1] & 0x01) << 4 ));

    switch (en_seq_444_value) {
        case 1:
        case 2:
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _setQPIEnabled - send command 38h");
            /*if (_setWriteEnable() != 0) {
            	printf("ERROR: Write Enabe failed\n");
            	return -1;
            }
            if( false == _isMemReady()) {
            	printf("ERROR: Device not ready, write failed\n");
            	return -1;
            }*/

            _qspiSendGeneralCommand(0x38, -1, NULL, 0, NULL, 0);
            /*wait_ms(QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC);
            if( false == _isMemReady()) {
            	printf("ERROR: Device not ready after write, failed\n");
            	return -1;
            }*/


            break;

        case 4:
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _setQPIEnabled - send command 35h");
            _qspiSendGeneralCommand(0x35, -1, NULL, 0, NULL, 0);
            break;

        case 8:
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _setQPIEnabled - set config bit 6 and send command 71h");
            _qspiSendGeneralCommand(0x65, 0x800003, NULL, 0, (char *)config_reg, 1);
            config_reg[1] |= 0x40; //Set Bit 6
            _qspiSendGeneralCommand(0x71, 0x800003, NULL, 0, (char *)config_reg, 1);
            break;

        case 16:
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _setQPIEnabled - reset config bits 0-7 and send command 61h");
            _qspiSendGeneralCommand(0x65, -1, NULL, 0, (char *)config_reg, 1);
            config_reg[1] &= 0x7F; //Reset Bit 7 of CR




            /*
            if (_setWriteEnable() != 0) {
            	printf("ERROR: Write Enabe failed\n");
            	return -1;
            }
            if( false == _isMemReady()) {
            	printf("ERROR: Device not ready, write failed\n");
            	return -1;
            }*/

            _qspiSendGeneralCommand(0x61, -1, NULL, 0, (char *)config_reg, 1);


            /* wait_ms(QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC);
            if( false == _isMemReady()) {
            	printf("ERROR: Device not ready after write, failed\n");
            	return -1;
            }



            _qspiSendGeneralCommand(0x65, -1, NULL, 0, (char *)config_reg, 1);

            if (_setWriteEnable() != 0) {
            	printf("ERROR: Write Enabe failed\n");
            	return -1;
            }
            if( false == _isMemReady()) {
            	printf("ERROR: Device not ready, write failed\n");
            	return -1;
            }
            config_reg[1] = 0x00; //Reset Bits 0-7
            _qspiSendGeneralCommand(0x61, -1, NULL, 0, (char *)config_reg, 1);
            wait_ms(QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC);*/
            /*	if( false == _isMemReady()) {
            	printf("ERROR: Device not ready after write, failed\n");
            	return -1;
            }*/

            break;

        default:
            QSPIF_LOG(QSPIF_DEBUG_WARNING, "WARNING: _setQPIEnabled - Unsuported En Seq 444 configuration");
            break;


    }

    return 0;
}



int QSPIFBlockDevice::_sfdpSetQuadEnabled(uint8_t *basicParamTablePtr)
{

    int sr_read_size = QSPI_MAX_STATUS_REGISTER_SIZE;
    int sr_write_size = QSPI_MAX_STATUS_REGISTER_SIZE;

    int status_reg_setup[QSPI_MAX_STATUS_REGISTER_SIZE];
    uint8_t status_reg[QSPI_MAX_STATUS_REGISTER_SIZE];
    unsigned int writeRegisterInst = QSPIF_WRSR;
    unsigned int readRegisterInst = QSPIF_RDSR;

    uint8_t qer_value = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_QER_BYTE] & 0x70) >> 4;



    switch (qer_value) {
        case 0:
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Device Does not Have a QE Bit, continue based on Read Inst");
            return 0;

        case 1:
        case 4:
            status_reg_setup[0] = 0;
            status_reg_setup[1] = 0x02;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Setting QE Bit, Bit 1 of Status Reg 2");
            break;

        case 2:
            status_reg_setup[0] = 0x40;
            status_reg_setup[1] = 0;
            sr_write_size = 1;
            sr_read_size = 1;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Setting QE Bit, Bit 6 of Status Reg 1");
            break;

        case 3:
            status_reg_setup[0] = 0x80; // Bit 7 of Status Reg 1
            status_reg_setup[1] = 0;
            sr_write_size = 1;
            sr_read_size = 1;
            writeRegisterInst = 0x3E;
            readRegisterInst = 0x3F;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Setting QE Bit, Bit 7 of Status Reg 1");
            break;
        case 5:
            status_reg_setup[0] = 0;
            status_reg_setup[1] = 0x02;
            readRegisterInst = 0x35;
            sr_read_size = 1;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Setting QE Bit, Bit 1 of Status Reg 2 -special read command");
            break;
        default:
            QSPIF_LOG(QSPIF_DEBUG_WARNING, "WARNING: _setQuadEnable - Unsuported QER configuration");
            break;


    }

    // Read Status Register
    if (QSPI_STATUS_OK == _qspiSendGeneralCommand(readRegisterInst, -1, NULL, 0, (char *)status_reg,
            sr_read_size) ) {  // store received values in status_value
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_reg[0]);
    } else {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Reading Status Register failed");
        return -1;
    }

    // Set Bits for Quad Enable
    status_reg[0] |= status_reg_setup[0];
    status_reg[1] |= status_reg_setup[1];


    // Write new Status Register Setup
    if (_setWriteEnable() != 0) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Write Enabe failed\n");
        return -1;
    }

    if ( false == _isMemReady()) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Device not ready, write failed");
        return -1;
    }

    if (QSPI_STATUS_OK == _qspiSendGeneralCommand(writeRegisterInst, -1, (char *)status_reg, sr_write_size, NULL,
            0) ) {  // Write QE to status_register
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _setQuadEnable - Writing Status Register Success: value = 0x%x",
                  (int)status_reg[0]);
    } else {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: _setQuadEnable - Writing Status Register failed");
        return -1;
    }

    wait_ms(QSPI_STATUS_REGISTER_WRITE_TIMEOUT_MSEC);

    if ( false == _isMemReady()) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Device not ready after write, failed");
        return -1;
    }


    // For Debug
    memset(status_reg, 0, QSPI_MAX_STATUS_REGISTER_SIZE);
    if (QSPI_STATUS_OK == _qspiSendGeneralCommand(readRegisterInst, -1, NULL, 0, (char *)status_reg,
            sr_read_size) ) {  // store received values in status_value
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_reg[0]);
    } else {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Reading Status Register failed");
        return -1;
    }


    return 0;//((status_reg[0] & QSPI_STATUS_BIT_QE) != 0 ? 0 : -1);
}


int QSPIFBlockDevice::_sfdpDetectPageSize(uint8_t *basicParamTablePtr)
{
    int page2PowerSize = ( (int)basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_PAGE_SIZE_BYTE]) >> 4;
    int pageSize = _utilsMathPower(2, page2PowerSize);
    QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: _detectPageSize - Page Size: %d", pageSize);
    return pageSize;
}



int QSPIFBlockDevice::_sfdpDetectEraseTypesInstAndSize(uint8_t *basicParamTablePtr, unsigned int& erase4KInst,
        unsigned int *eraseTypeInstArr, unsigned int *eraseTypeSizeArr)
{
    erase4KInst = 0xff;
    bool found4KEraseType = false;
    uint8_t bitfield = 0x01;

    // Erase 4K Inst is taken either from param table legacy 4K erase or superseded by erase Instruction for type of size 4K
    erase4KInst = basicParamTablePtr[QSPIF_BASIC_PARAM_4K_ERASE_TYPE_BYTE];

    // Loop Erase Types 1-4
    for (int i_ind = 0; i_ind < 4; i_ind++) {
        eraseTypeInstArr[i_ind] = 0xff; //0xFF default for unsupported type
        eraseTypeSizeArr[i_ind] = _utilsMathPower(2, basicParamTablePtr[QSPIF_BASIC_PARAM_ERASE_TYPE_1_SIZE_BYTE + 2 * i_ind]);
        if (eraseTypeSizeArr[i_ind] > 1) {
            // if size==1 type is not supported
            eraseTypeInstArr[i_ind] = basicParamTablePtr[QSPIF_BASIC_PARAM_ERASE_TYPE_1_BYTE + 2 * i_ind];

            if ((eraseTypeSizeArr[i_ind] < _minCommonEraseSize) || (_minCommonEraseSize == 0) ) {
                //Set default minimal common erase for singal region
                _minCommonEraseSize = eraseTypeSizeArr[i_ind];
            }

            if (eraseTypeSizeArr[i_ind] == 4096) {
                found4KEraseType = true;
                if (erase4KInst != eraseTypeInstArr[i_ind]) {
                    //Verify 4KErase Type is identical to Legacy 4K erase type specified in Byte 1 of Param Table
                    erase4KInst = eraseTypeInstArr[i_ind];
                    QSPIF_LOG(QSPIF_DEBUG_WARNING,
                              "WARNING: _detectEraseTypesInstAndSize - Default 4K erase Inst is different than erase type Inst for 4K");

                }
            }
            _region_erase_types[0] |= bitfield; // If there's no region map, set region "0" types as defualt;
        }

        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Erase Type %d - Inst: 0x%xh, Size: %d", (i_ind + 1), eraseTypeInstArr[i_ind],
                  eraseTypeSizeArr[i_ind]);
        bitfield = bitfield << 1;
    }

    if (false == found4KEraseType) {
        QSPIF_LOG(QSPIF_DEBUG_WARNING, "WARNING: Couldn't find Erase Type for 4KB size");
    }
    return 0;
}


int QSPIFBlockDevice::_sfdpDetectBestBusReadMode(uint8_t *basicParamTablePtr, bool& setQuadEnable, bool& isQPIMode,
        unsigned int& readInst)
{

    bool isDone = false;

    setQuadEnable = false;
    isQPIMode = false;
    int dummyCycles = 0;
    uint8_t examinedByte = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE];

    do { // compound statement is the loop body



        if (examinedByte & 0x10) {
            // QPI 4-4-4 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE];
            setQuadEnable = true;
            isQPIMode = true;
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_444_READ_INST_BYTE - 1] & 0x1F);
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 4-4-4, Instruction: 0x%xh", _readInstruction);
            //_inst_width = QSPI_CFG_BUS_QUAD;
            _address_width = QSPI_CFG_BUS_QUAD;
            _data_width = QSPI_CFG_BUS_QUAD;

            break;
        }


        examinedByte = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE];
        if (examinedByte & 0x40) {
            //  Fast Read 1-4-4 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE];
            setQuadEnable = true;
            // dummy cycles + mode cycles = Dummy Cycles
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_144_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_QUAD;
            _data_width = QSPI_CFG_BUS_QUAD;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 1-4-4, Instruction: 0x%xh", _readInstruction);
            break;
        }
        if (examinedByte & 0x80) {
            //  Fast Read 1-1-4 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE];
            setQuadEnable = true;
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_114_READ_INST_BYTE - 1] & 0x1F);
            _data_width = QSPI_CFG_BUS_QUAD;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 1-1-4, Instruction: 0x%xh", _readInstruction);
            break;
        }
        examinedByte = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_QPI_READ_SUPPOR_BYTE];
        if (examinedByte & 0x01) {
            //  Fast Read 2-2-2 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_222_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_DUAL;
            _data_width = QSPI_CFG_BUS_DUAL;
            QSPIF_LOG(QSPIF_DEBUG_INFO, "/nINFO: Read Bus Mode set to 2-2-2, Instruction: 0x%xh", _readInstruction);
            break;
        }

        examinedByte = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_FAST_READ_SUPPORT_BYTE];
        if (examinedByte & 0x20) {
            //  Fast Read 1-2-2 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_122_READ_INST_BYTE - 1] & 0x1F);
            _address_width = QSPI_CFG_BUS_DUAL;
            _data_width = QSPI_CFG_BUS_DUAL;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 1-2-2, Instruction: 0x%xh", _readInstruction);
            break;
        }
        if (examinedByte & 0x01) {
            // Fast Read 1-1-2 Supported
            readInst = basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE];
            _dummy_and_mode_cycles = (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE - 1] >> 5)
                                     + (basicParamTablePtr[QSPIF_BASIC_PARAM_TABLE_112_READ_INST_BYTE - 1] & 0x1F);
            _data_width = QSPI_CFG_BUS_DUAL;
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 1-1-2, Instruction: 0x%xh", _readInstruction);
            break;
        }
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "/nDEBUG: Read Bus Mode set to 1-1-1, Instruction: 0x%xh", _readInstruction);
        isDone = true;
    } while (isDone == false);

    return 0;
}


int QSPIFBlockDevice::_resetFlashMem()
{
    int status = 0;
    char status_value[2] = {0};
    QSPIF_LOG(QSPIF_DEBUG_ERROR, "INFO: _resetFlashMem:\n");
    //Read the Status Register from device
    if (QSPI_STATUS_OK == _qspiSendGeneralCommand(QSPIF_RDSR, -1, NULL, 0, status_value,
            1) ) {  // store received values in status_value
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Reading Status Register Success: value = 0x%x\n", (int)status_value[0]);
    } else {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Reading Status Register failed\n");
        status = -1;
    }

    if (0 == status) {
        //Send Reset Enable
        if (QSPI_STATUS_OK == _qspiSendGeneralCommand(QSPIF_RSTEN, -1, NULL, 0, NULL,
                0) ) {   // store received values in status_value
            QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Sending RSTEN Success\n");
        } else {
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Sending RSTEN failed\n");
            status = -1;
        }


        if (0 == status) {
            //Send Reset
            if (QSPI_STATUS_OK == _qspiSendGeneralCommand(QSPIF_RST, -1, NULL, 0, NULL,
                    0)) {   // store received values in status_value
                QSPIF_LOG(QSPIF_DEBUG_DEBUG, "DEBUG: Sending RST Success\n");
            } else {
                QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Sending RST failed\n");
                status = -1;
            }

            _isMemReady();
        }
    }

    return status;
}


bool QSPIFBlockDevice::_isMemReady()
{
    char status_value[2];
    int retries = 0;
    bool memReady = true;

    do {
        retries++;
        //Read the Status Register from device
        if (QSPI_STATUS_OK != _qspiSendGeneralCommand(QSPIF_RDSR, -1, NULL, 0, status_value,
                2)) {   // store received values in status_value
            QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Reading Status Register failed\n");
        }
    } while ( (status_value[0] & 0x1) != 0 && retries < 10000 );

    if ((status_value[0] & 0x1) != 0) {
        QSPIF_LOG(QSPIF_DEBUG_DEBUG, "ERROR: _isMemReady FALSE\n");
        memReady = false;
    }
    return memReady;
}


int QSPIFBlockDevice::_setWriteEnable()
{
    int status = 0;
    if (QSPI_STATUS_OK !=  _qspiSendGeneralCommand(QSPIF_WREN, -1, NULL, 0, NULL, 0)) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR:Sending WREN command FAILED\n");
        status = -1;
    }
    return status;
}



/*********************************************/
/************* Utility Functions *************/
/*********************************************/
int QSPIFBlockDevice::_utilsMathPower(int base, int exp)
{
    int result = 1;
    while (exp) {
        result *= base;
        exp--;
    }
    return result;
}


int QSPIFBlockDevice::_utilsFindAddrRegion(int offset)
{
    if ((offset > (int)_deviceSizeBytes) || (_regions_count == 0)) {
        return -1;
    }

    if (_regions_count == 1) {
        return 0;
    }

    for (int i_ind = _regions_count - 2; i_ind >= 0; i_ind--) {

        if (offset > _region_high_boundary[i_ind]) {
            return (i_ind + 1);
        }
    }
    return -1;

}


int QSPIFBlockDevice::_utilsIterateNextLargestEraseType(uint8_t& bitfield, int size, int offset, int boundry)
{
    uint8_t type_mask = 0x08;
    int i_ind  = 0;
    int largestEraseType = 0;
    for (i_ind = 3; i_ind >= 0; i_ind--) {
        if (bitfield & type_mask) {
            largestEraseType = i_ind;
            if ( (size > _eraseTypeSizeArr[largestEraseType]) &&
                    ((boundry - offset) > _eraseTypeSizeArr[largestEraseType]) ) {
                break;
            } else {
                bitfield &= ~type_mask;
            }
        }
        type_mask = type_mask >> 1;
    }

    if (i_ind == 4) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: no erase type was found for current region addr");
    }
    return largestEraseType;

}


/***************************************************/
/*********** QSPI Driver API Functions *************/
/***************************************************/

qspi_status_t QSPIFBlockDevice::_qspiSetFrequency(int freq)
{
    return _qspi.set_frequency(freq);
}


qspi_status_t QSPIFBlockDevice::_qspiSendReadCommand(unsigned int readInst, void *buffer, bd_addr_t addr,
        bd_size_t size)
{
    // Check the address and size fit onto the chip.

    /* OFR_DBG */
    //MBED_ASSERT(is_valid_read(addr, size));

    size_t bufLen = size;

    if (_qspi.read( readInst, -1, (unsigned int )addr, (char *)buffer, &bufLen) != QSPI_STATUS_OK ) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: Read failed");
        return QSPI_STATUS_ERROR;
    }

    return QSPI_STATUS_OK;

}


qspi_status_t QSPIFBlockDevice::_qspiSendProgramCommand(unsigned int progInst, const void *buffer, bd_addr_t addr,
        bd_size_t *size)
{
    // Check the address and size fit onto the chip.
    //MBED_ASSERT(is_valid_program(addr, (*size)));
    qspi_status_t result = QSPI_STATUS_OK;

    result = _qspi.write( progInst, -1, addr/*(((unsigned int)addr) & 0x00FFFF00)*/, (char *)buffer, (size_t *)size);
    if (result != QSPI_STATUS_OK) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Write failed");
    }

    return result;
}


qspi_status_t QSPIFBlockDevice::_qspiSendEraseCommand(unsigned int eraseInst, bd_addr_t addr, bd_size_t size)
{
    // Check the address and size fit onto the chip.
    //MBED_ASSERT(is_valid_erase(addr, size));

    qspi_status_t result = QSPI_STATUS_OK;

    result = _qspi.command_transfer(eraseInst, // command to send
                                    (((int)addr) & 0x00FFF000), // Align addr to 4096
                                    NULL,                 // do not transmit
                                    0,              // do not transmit
                                    NULL,                 // just receive two bytes of data
                                    0); // store received values in status_value
    if (QSPI_STATUS_OK != result) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR: QSPI Erase failed");
    }

    return result;

}


qspi_status_t QSPIFBlockDevice::_qspiSendGeneralCommand(unsigned int instruction, bd_addr_t addr, const char *tx_buffer,
        size_t tx_length, const char *rx_buffer, size_t rx_length)
{


    qspi_status_t status = _qspi.command_transfer(instruction, (int)addr, tx_buffer, tx_length, rx_buffer, rx_length);

    if (QSPI_STATUS_OK != status) {
        QSPIF_LOG(QSPIF_DEBUG_ERROR, "ERROR:Sending Generic command: %x", instruction);
    }

    return status;
}


qspi_status_t QSPIFBlockDevice::_qspiConfiureFormat(qspi_bus_width_t inst_width, qspi_bus_width_t address_width,
        qspi_address_size_t address_size, qspi_bus_width_t alt_width, qspi_alt_size_t alt_size, qspi_bus_width_t data_width,
        int dummy_cycles)
{
    _qspi.configure_format( inst_width, address_width, address_size, alt_width, alt_size, data_width, dummy_cycles);

    return QSPI_STATUS_OK;
}





