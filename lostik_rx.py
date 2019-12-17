#!/usr/bin/env python3
import time
import sys
import serial
import argparse 

from serial.threaded import LineReader, ReaderThread

freqs = [902e6 + 1e6 * i for i in range(27)]

parser = argparse.ArgumentParser(description='LoRa Radio mode receiver.')
parser.add_argument('port', help="Serial port descriptor")
args = parser.parse_args()

class PrintLines(LineReader):

    def connection_made(self, transport):
        print("connection made")
        self.transport = transport
        self.send_cmd('radio set freq %d' % (freqs[0]))
        self.send_cmd('sys get ver')
        self.send_cmd('mac pause')
        self.send_cmd('radio set pwr 10')
        self.send_cmd('radio rx 0')
        self.send_cmd("sys set pindig GPIO10 0")
        self.frame_count = 0

    def handle_line(self, data):
        if data == "ok" or data == 'busy':
            return
        if data == "radio_err":
            self.send_cmd('radio rx 0')
            return
        
        self.send_cmd('radio get freq')
        self.send_cmd("sys set pindig GPIO10 1", delay=0)
        print(data)
        self.frame_count += 1
        time.sleep(.1)
        self.send_cmd('radio set freq %d' % (freqs[self.frame_count % len(freqs)]))
        self.send_cmd("sys set pindig GPIO10 0", delay=1)
        self.send_cmd('radio rx 0')

    def connection_lost(self, exc):
        if exc:
            print(exc)
        print("port closed")

    def send_cmd(self, cmd, delay=.5):
        self.transport.write(('%s\r\n' % cmd).encode('UTF-8'))
        time.sleep(delay)

ser = serial.Serial(args.port, baudrate=57600)
with ReaderThread(ser, PrintLines) as protocol:
    while(1):
        pass