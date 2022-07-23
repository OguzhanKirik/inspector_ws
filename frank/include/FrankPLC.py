
class newPLC():
    SENDER_AMS = '1.2.3.4.1.1'
    HOSTNAME = '192.168.0.80' # or IP
    PLC_IP = '192.168.0.81'
    PLC_USERNAME = 'Administrator'
    PLC_PASSWORD = '1'
    ROUTE_NAME = 'ROSRoute'
    NET_ID = '5.62.232.252.1.1'
    Connected = False
    plc=  0
    TriggerFreq=100