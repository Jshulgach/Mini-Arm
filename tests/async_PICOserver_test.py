""" This script tests the asyncio library with setting up a server to handle socket streaming 
    and exchanging of data. A non-blocking coroutine that handles incoming bytestreams will 
    receive numbers that are read and stored in a list that gets accessed by a main function.
    A separate beeping corouting is just running in the background.
"""

import asyncio
import wifi
import os

wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))

class AsyncTest:
    def __init__(self):
        self.numbers = [] # list that new numbers will get stored to

    async def serve_client(self, reader, writer):
        """ Function that runs once a client has connected to the server"""
        print("Client connected")
        while True:
            #request = await reader.readline()
            request = await reader.read(10)
            request = ''.join([chr(b) for b in request]).replace('\x00','')
            if request == b'': break            
            print("Adding {} to queue:".format(int(request)))
            self.numbers.append(int(request))
        writer.close()
        print('Server shutdown')

    async def beep(self, interval):
        """ print 'beep' once every interval (in seconds) """
        while True:
            print('beep...')
            await asyncio.sleep(1 / int(interval))  # Don't forget the await!

    async def main(self):
        """ Keeping the main tasks and coroutines in a single main function """
        print('Setting up webserver...')
        asyncio.create_task(asyncio.start_server(self.serve_client, "192.168.1.177", 1000))
        #asyncio.create_task(asyncio.start_server(self.serve_client, "localhost", 1000))
        asyncio.create_task( self.beep( 1 ) ) 

        while True:
            if len(self.numbers)>0:
                print("Reading {} from list".format(self.numbers.pop(0)))
            print("Entire list: {}".format(self.numbers))
            await asyncio.sleep(5)

obj = AsyncTest()
asyncio.run(obj.main()) # Need to pass the async function into the run method to start