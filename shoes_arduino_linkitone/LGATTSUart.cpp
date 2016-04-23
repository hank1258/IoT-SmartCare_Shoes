#include "vmbtgatt.h"
#include "LGATTSUart.h"

#ifdef APP_LOG
#undef APP_LOG
#define APP_LOG(...) Serial.printf(__VA_ARGS__); \
    Serial.println();
#endif


static LGATTServiceInfo g_uart_decl[] =
{
    {TYPE_SERVICE, "E20A39F4-73F5-4BC4-A12F-17D1AD07A961", TRUE, 0, 0, 0},
    {TYPE_CHARACTERISTIC, "08590F7E-DB05-467E-8757-72F6FAEB13D4", FALSE, VM_GATT_CHAR_PROP_READ, VM_GATT_PERM_READ, 0}, // tx for periphral; rx for central
    {TYPE_CHARACTERISTIC, "08590F7F-DB05-467E-8757-72F6FAEB13D4", FALSE, VM_GATT_CHAR_PROP_READ, VM_GATT_PERM_READ, 0}, // tx for periphral; rx for central
    {TYPE_END, 0, 0, 0, 0, 0}

};

uint16_t LGATTSUart::getHandle(int32_t type)
{
    if (0 == type)
    {
        return _handle_notify;
    }
    else if (1 == type)
    {
        return _handle_write;
    }

    return 0;
}

// prepare the data for profile
LGATTServiceInfo *LGATTSUart::onLoadService(int32_t index)
{
    
    return g_uart_decl;
}

// characteristic added
boolean LGATTSUart::onCharacteristicAdded(LGATTAttributeData &data)
{
    const VM_BT_UUID *uuid = &(data.uuid);
    APP_LOG("LGATTSUart::onCharacteristicAdded f[%d] uuid[12] = [0x%x] len[%d]", data.failed, uuid->uuid[12], uuid->len);
    if (!data.failed)
    {
        if (0x7E == uuid->uuid[12])
        {
            _handle_notify = data.handle;
        }
        else if (0x7F == uuid->uuid[12])
        {
            _handle_write = data.handle;
        }
        
    }
    return true;
}
// connected or disconnected
boolean LGATTSUart::onConnection(const LGATTAddress &addr, boolean connected)
{
    _connected = connected;
    APP_LOG("LGATTSUart::onConnection connected [%d], [%x:%x:%x:%x:%x:%x]", _connected, 
        addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);


    return true;
}
// read action comming from master
boolean LGATTSUart::onRead(LGATTReadRequest &data)
{
    APP_LOG("LGATTSUart::onRead _connected [%d]", _connected);
    
      Serial.println(data.attr_handle);

    
    if (_connected)
    {
        LGATTAttributeValue value = {0};
        const char *str1 = "shoes,Dw0waaBM";
        const char *str2 = "WfG3TGkNPoFkfViI";
        if(data.attr_handle==258){
        memcpy(value.value, str1, strlen(str1));
        value.len = strlen(str1);
        }
        else if(data.attr_handle==260){
        memcpy(value.value, str2, strlen(str2));
        value.len = strlen(str2);
        }
        APP_LOG("LGATTSUart::onRead onRead [%d][%s]", value.len, value.value);
        

        data.ackOK(value);
        
    }
    return true;
}
// write action comming from master
boolean LGATTSUart::onWrite(LGATTWriteRequest &data)
{
    APP_LOG("LGATTSUart::onWrite _connected [%d]", _connected);
    // todo read UART data.
    if (_connected)
    {
        // if need to rsp to central.
        if (data.need_rsp)
        {
            LGATTAttributeValue value;
            value.len = 0;
            data.ackOK();
        }

        /*
        if (data.offset + data.value.len <= ATT_MAX_VALUE_LEN)
        {
            _value.len = data.value.len;
            memcpy(&_value.value, data.value.value + data.offset, data.value.len);
        }
        */
        APP_LOG("central data on peripheral rx[%s][%d]", data.value.value, data.value.len);
    }
    return true;
}




