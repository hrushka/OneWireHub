#include <EEPROM.h>
#include "DS2434.h"

DS2434::DS2434(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7) : OneWireItem(ID1, ID2, ID3, ID4, ID5, ID6, ID7)
{
    // disable bus-features:
    // The DS2434 is NOT compatible with multidrop -> only one device on bus!
    skip_multidrop = true;

    clearMemory();
    clearScratchpad();
}

// As this device is not multidrop, it needs to handle ALL commands from the master
void DS2434::duty(OneWireHub * const hub)
{
    uint8_t start_byte, cmd, data;
    uint32_t time_now;
    if (hub->recv(&cmd))  return;

    switch (cmd)
    {

    case 0x11:      // Read Scratchpad
        if (hub->recv(&start_byte))  return;
        if (start_byte >= PAGE4_ADDR) return;
        for (uint8_t nByte = start_byte; nByte < PAGE4_ADDR; ++nByte)
        {
            // read through scratchpad until comm-error is raised
            if (hub->send(&scratchpad[nByte], 1)) return;
        }
        // TODO: is endless reading needed? probably not
        break;

    case 0x17:      // Write Scratchpad
        if (hub->recv(&start_byte))  return;
        if (start_byte >= PAGE4_ADDR) return; // when out of limits
        for (uint8_t nByte = start_byte; nByte < PAGE4_ADDR; ++nByte)
        {
            if (hub->recv(&data, 1)) return;
            scratchpad[nByte] = data;
        }
        break;

    case 0x22:      // copy scratchpad SP1 to NV1
        
        if (memory[0x62] & 0b100u) return; // check LOCK-Status
        writeMemory(&scratchpad[PAGE1_ADDR], PAGE_SIZE, PAGE1_ADDR);
        request_persist = 0x22; // Request memory to be persisted

        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0x25:      // copy scratchpad SP2 to NV2
        writeMemory(&scratchpad[PAGE2_ADDR], PAGE_SIZE, PAGE2_ADDR);
        request_persist = 0x25; // Request memory to be persisted

        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0x28:      // copy scratchpad SP3 to SRAM
        writeMemory(&scratchpad[PAGE3_ADDR], PAGE_SIZE, PAGE3_ADDR);
        break;

    case 0x71:      // Recall Memory, NV1 to SP1
        readMemory(&scratchpad[PAGE1_ADDR], PAGE_SIZE, PAGE1_ADDR);
        break;

    case 0x77:      // Recall Memory, NV2 to SP2
        readMemory(&scratchpad[PAGE2_ADDR], PAGE_SIZE, PAGE2_ADDR);
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0x7A:      // Recall Memory, SRAM to SP3
        readMemory(&scratchpad[PAGE3_ADDR], PAGE_SIZE, PAGE3_ADDR);
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0x43:      // lock NV1
        lockNV1();
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;
    case 0x44:      // unlock NV1
        unlockNV1();
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0xD2:      // trigger temperature-reading
        request_temp = true;
        timer_temp = millis() + DURATION_TEMP_ms;
        break;

    case 0xB2:      // read Page 4 and 5
        if (hub->recv(&start_byte))  return;
        if (start_byte < PAGE4_ADDR) return; // when out of limits
        if (start_byte >= PAGE6_ADDR) return;

        // update status byte, TODO: done here because laptop waits -> check duration
        time_now = millis();
        if (time_now >= timer_nvwr)     memory[0x62] &= ~0b10u; // erase busy-flag
        else                            memory[0x62] |= 0b10u; // set busy-flag
        if (time_now >= timer_temp)     memory[0x62] &= ~0b1u; // erase busy-flag
        else                            memory[0x62] |= 0b1u; // set busy-flag

        for (uint8_t nByte = start_byte; nByte < PAGE6_ADDR; ++nByte)
        {
            if (hub->send(&memory[nByte], 1)) return;
        }
        break;

    case 0xB5:      // increment Cycle
        if (++memory[0x83] == 0u)
        {
            // after overflow of LSB
            memory[0x82]++;
        }
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0xB8:      // reset Cycle
        memory[0x82] = 0u;
        memory[0x83] = 0u;
        // NOTE: OP occupies real NV for ~ 10 ms (NVB-Bit)
        timer_nvwr = millis() + DURATION_NVWR_ms;
        break;

    case 0x8E:      // secret command just to avoid triggering an error
        break;
    case 0x84:      // second secret command
        if (hub->recv(&data))  return;
        break;

    default:
        hub->raiseSlaveError(cmd);
    }
}

void DS2434::clearMemory(void)
{
    memset(memory, static_cast<uint8_t>(0xFF), MEM_SIZE);
}

void DS2434::clearScratchpad(void)
{
    memset(scratchpad, static_cast<uint8_t>(0xFF), SCRATCHPAD_SIZE);
}

bool DS2434::writeMemory(const uint8_t* const source, const uint16_t length, const uint16_t position)
{
    if (position >= MEM_SIZE) return false;
    const uint16_t _length = (position + length >= MEM_SIZE) ? (MEM_SIZE - position) : length;
    memcpy(&memory[position],source,_length);
    return true;
}

bool DS2434::readMemory(uint8_t* const destination, const uint16_t length, const uint16_t position) const
{
    if (position >= MEM_SIZE) return false;
    const uint16_t _length = (position + length >= MEM_SIZE) ? (MEM_SIZE - position) : length;
    memcpy(destination,&memory[position],_length);
    return (_length==length);
}

void DS2434::setTemperature(const int8_t temp_degC)
{
    int8_t value = temp_degC;

    if (value > 126) value = 126;
    if (value < -40) value = -40;
    memory[0x61] = value;

    if (value < 0) value = 0;
    uint8_t uvalue = static_cast<uint8_t>(value);

    memory[0x60] = uvalue << 1u;

    // reset request
    request_temp = false;
}

bool DS2434::getTemperatureRequest() const
{
    return (request_temp);
}

void DS2434::lockNV1()
{
    memory[0x62] |= 0b100u;
}

void DS2434::unlockNV1()
{
    memory[0x62] &= ~0b100u;
}

void DS2434::setBatteryCounter(uint16_t value)
{
    *((uint16_t *) &memory[0x82]) = value;
}

void DS2434::setID(uint16_t value)
{
    *((uint16_t *) &memory[0x80]) = value;
}

/*
 * This will persiste memory to the EEPROM. Due to the time
 * it takes to write, we'll need to poll in between to avoid dropping
 * requests.
 * 
 * Returns: true if memory needs to be persisted
 */
bool DS2434::checkPersistMemory(OneWireHub& hub){

    // Only write if there has been a request
    if(request_persist == 0x00){
        return false;
    }

    uint8_t address_start;
    uint8_t address_end;

    switch(request_persist){

        case 0x01: // Initialize - write everything the first time
            address_start = PAGE1_ADDR;
            address_end   = PAGE6_ADDR;
            break;
        case 0x22: // Copy SP1 to NV1
            address_start = PAGE1_ADDR;
            address_end   = PAGE2_ADDR;
            break;
        case 0x25: // Copy SP2 to NV2
            address_start = PAGE2_ADDR;
            address_end   = PAGE3_ADDR;
            break;
        case 0xB5: // Update charge cycle counts
        case 0xB8:
            address_start = 0x82;
            address_end   = 0x84;
            break;
        default:
            request_persist = 0x00;
            return true;
            break;
    }

    // Loop through each byte and write to EEPROM
    for (uint8_t nByte = address_start; nByte < address_end; ++nByte)
    {
        uint8_t byte_mem = memory[nByte];
        uint8_t byte_rom = EEPROM.read(nByte);

        // Only write if the byte has changed
        if(byte_mem != byte_rom){
            EEPROM.write(nByte, byte_mem);

            // Poll after each write due to write time (~3ms per byte)
            hub.poll();
        }
    }

    // If this is an initial write, write a status byte
    if(request_persist == 0x01){
        EEPROM.write(PAGE6_ADDR, PERSIST_STATUS);
    }

    request_persist = 0x00;
    return true;
}

/*
 * Restores the memory from EEPROM. This should only happen on startup.
 * All other times, use the "memory" variable.
 * 
 * Returns: true if memory was found and restored
 */
bool DS2434::checkRestoreMemory(){

    uint8_t status = EEPROM.read(PAGE6_ADDR);

    // If the status byte has not been set, return
    if(status != PERSIST_STATUS){
        return false;
    }

    // Restore all the memory from EEPROM
    for (uint8_t nByte = PAGE1_ADDR; nByte < PAGE6_ADDR; ++nByte)
    {
        memory[nByte] = EEPROM.read(nByte);
    }

    return true;
}

/*
 * Triggers the system to persist all memory to the EEPROM
 */
void DS2434::persistAllMemory(){
    request_persist == 0x01;
    return;
}
