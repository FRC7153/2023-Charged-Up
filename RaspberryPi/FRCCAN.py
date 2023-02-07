## IMPORTS
import os
import can
import threading

## LOGGER
logger = None

## SYSTEM BROADCAS MESSAGES
kBROADCASTS = ["DISABLE", "SYSTEM HALT", "SYSTEM RESET", "DEVICE ASSIGN", "DEVICE QUERY", "HEARTBEAT", "SYNC", "UPDATE", "FIRMWARE VERSION", "ENUMERATE", "SYSTEM RESUME"]

## START CAN BUS
def initCAN(log):
        global logger
        logger = log
        os.system("sudo ip link set can0 type can bitrate 1000000")
        os.system("sudo ifconfig can0 up")
        bus = can.ThreadSafeBus(channel="can0", bustype="socketcan")
        bus.set_filters(None)
        return bus

## FORMAT INT TO PADDED BINARY
def intToBinary(value, length):
        value = "{0:b}".format(value).zfill(length)

        if len(value) > length:
                if logger != None:
                        logger.warn(f"Int is too large for binary (value is {value}, must be below {length} bits)")
                return "0" * length

        return value

## FORMAT CAN ARBITRATOR
def formatArb(devType, manuCode, apiClass, apiIndex, deviceId):
        arb = intToBinary(devType, 5)
        arb += intToBinary(manuCode, 8)
        arb += intToBinary(apiClass, 6)
        arb += intToBinary(apiIndex, 4)
        arb += intToBinary(deviceId, 6)
        return int(arb, 2)

## FRC CAN DEVICE
class Device:
        def __init__(self, bus, canID, logger):
                self.__bus = bus
                self.__arbId = canID
                self.__logger = logger
                self.__listenThread = None
                self.__listening = False
                self.__handlers = {}
                self.__broadcasters = []

        ## WARNING
        def __warn(self, msg):
                self.__logger.warn(f"(DEVICE {self.__arbId}): {msg}")

        ## ADD FILTER
        def __safeAddFilter(self, canFilter):
                if self.__bus.filters == None:
                        self.__bus.set_filters([canFilter])
                else:
                        self.__bus.filters.append(canFilter)

        ## ADD EVENT
        def receive(self, apiClass):
                def decorator(func):
                        if apiClass in self.__handlers:
                                self.__warn(f"Handler with API class '{apiClass}' already exists")
                                return func
                        
                        self.__handlers[apiClass] = func

                        self.__safeAddFilter({
                                "can_id": formatArb(10, 8, apiClass, 0, self._arbId),
                                "can_mask": 0b11111111111111111110000111111,
                                "extended": True
                        })

                        return func
                return decorator

        ## ADD BROADCAST EVENT
        def receiveBroadcast(self):
                def decorator(func):
                        if len(self.__broadcasters) == 0:
                                self.__safeAddFilter({
                                        "can_id": formatArb(0, 0, 0, 0, 0),
                                        "can_mask": 0b11111111111111111110000000000,
                                        "extended": True
                                })
                                
                        self.__broadcasters.append(func)
                        return func
                return decorator

        ## PACKET HANDLER
        def __packetHandlerWrapper(self, msg):
                binary = "{0:b}".format(msg.arbitration_id).zfill(29)
                apiClass = int(binary[14:20], 2)
                apiIndex = int(binary[20:24], 2)

                if binary[:20] == "0"*19: #broadcast message
                        for b in self.__broadcasters:
                                try:
                                        b(apiIndex)
                                except Exception as e:
                                        self.__warn(f"Could not run broadcast handler: {e}")
                        return

                if not apiClass in self.__handlers:
                        return

                try:
                        self.__handlers[apiClass](apiIndex, msg.data)
                except Exception as e:
                        self.__warn(f"Could not run API handler (class: {apiClass}, index: {apiIndex}): {e}")

        ## LISTEN
        def __listenThreadLoop(self):
                print(f"Started listening to {self.__bus}")
                while self.__listening:
                        msg = self.__bus.recv(5.0)
                        if (msg != None and self.__handler != None): # Received data:
                                threading.Thread(target=self.__packetHandlerWrapper, args=(msg,)).start()
                print(f"Stopped listening to {self.__bus}")

        ## START LISTEN
        def startListening(self):
                self.__listening = True
                self.__listenThread = threading.Thread(target=self.__listenThreadLoop)
                self.__listenThread.start()

        ## STOP LISTEN
        def stopListening(self):
                self.__listening = False

        ## SEND DATA
        def send(apiClass, apiIndex, data):
                if apiClass in self.__handlers:
                        self.__warn(f"sent data to API class ({apiClass}) that is currently defined as a listener (this will trigger an event)")
                
                msg = can.Message(
                        arbitration_id = formatArb(10, 8, apiClass, apiIndex, self._arbId),
                        data = data,
                        is_extended_id = True
                )

                try:
                        bus.send(msg)
                except Exception as e:
                        self.__warn(f"WARNING: Could not send message (class: {apiClass}, index: {apiIndex}) ({e})")
