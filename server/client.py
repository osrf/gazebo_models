#!/usr/bin/env python

import socket
import request_pb2

TCP_IP = '127.0.0.1'
TCP_PORT = 5014
BUFFER_SIZE = 1024

msg = request_pb2.Request()
msg.id = int(0)
msg.request = "models"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(msg.SerializeToString())
data = s.recv(BUFFER_SIZE)
s.close()

response = request_pb2.Response()
response.ParseFromString(data)

model_list = request_pb2.ModelList()
model_list.ParseFromString(response.data)


print "response:", response
print "models:", model_list.model
