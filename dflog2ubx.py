#!/usr/bin/env python

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)
'''

import fnmatch
import json
import os
import struct
import time


def checksum(data):
	ck_a = 0
	ck_b = 0

	for c in data[2:] :
		ck_a += ord(c)
		ck_a %= 256
		ck_b += ck_a
		ck_b %= 256
	return struct.pack("<BB",ck_a,ck_b) 

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("-o", "--output", default=None, help="output ubx file name")
parser.add_argument("-c", "--camlog_output", default=None, help="output camlog result")
parser.add_argument("log", metavar="LOG")

args = parser.parse_args()

from pymavlink import mavutil
output = open(args.output, "w")
camlog_output = open(args.camlog_output, "w")
mlog = mavutil.mavlink_connection(args.log, planner_format=False,
                                  notimestamps=False,
                                  robust_parsing=False,
                                  dialect="ardupilotmega",
                                  zero_time_base=False)
num_meas = 0
num_words = 0
data = ""
total_meas = 0
last_time_us = 0
while True:
	m = mlog.recv_match(blocking=False)
	if m is None:
		break
	if m.get_type() == "GRXH":
		print "Delta Time", (m.TimeUS - last_time_us)/1000
		last_time_us = m.TimeUS
		data = struct.pack("<BBBBHdHbBBBBB",
			0xB5,0x62,0x02,0x15,16+32*m.numMeas,
			m.rcvTime,
			m.week,
			m.leapS,
			m.numMeas,
			m.recStat,0,0,0)
		total_meas = m.numMeas
	elif m.get_type() == "GRXS":
		data = data + struct.pack("<ddfBBBBHBBBBBB",
			m.prMes,
			m.cpMes,
			m.doMes,
			m.gnss,
			m.sv,
			0,
			m.freq,
			m.lock,
			m.cno,
			m.prD,
			m.cpD,
			m.doD,
			m.trk,0)
		num_meas += 1
		if num_meas == total_meas:
			data = data + checksum(data)
			output.write(data)
			num_meas = 0
	elif m.get_type() == "GSFH":
		data = struct.pack("<BBBBHBBBBBBBB",
			0xB5,0x62,0x02,0x13,8+4*m.numWords,
			m.gnssId,
			m.svId,
			0,
			m.freqId,
			m.numWords,
			0,
			m.version,
			0)
		total_words = m.numWords
	elif m.get_type() == "GSFS":
		data = data + struct.pack("<I", m.data)
		num_words += 1
		if num_words == total_words:
			data = data + checksum(data)
			output.write(data)
			num_words = 0
	elif m.get_type() == "GTIM":
		trig = trig + 1
		camlog_data = "%d,%d,%d\n" % m.towMsR % m.towMsF % m.trig
		camlog_output.write(camlog_data)
