#!/usr/bin/env python

import socket
import struct
from sys import *
from xml.dom import minidom

import request_pb2

TCP_IP = '127.0.0.1'
TCP_PORT = 50199
BUFFER_SIZE = 1024

models = {}

#################################################
# Read the main manifest file
def read_manifest():
  xmldoc = minidom.parse('manifest.xml')
  model_list = xmldoc.getElementsByTagName("model")
  
  for model in model_list:
    model_name = model.getElementsByTagName("name")[0].childNodes[0].data
    model_path = model.getElementsByTagName("path")[0].childNodes[0].data
    models[model_name] = model_path

read_manifest()

print 'here'
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
print 'here2'

conn, addr = s.accept()
print 'Connection address:', addr
while 1:
  request = request_pb2.Request()
  data = conn.recv(BUFFER_SIZE)
  if not data:
    break

  request.ParseFromString(data)

  print "Request: ", request.request

  response = request_pb2.Response()
  response.id = request.id
  response.request = request.request

  if request.request == "models":
    response.response = "success"

    model_list_msg = request_pb2.ModelList()
    response.type = model_list_msg.DESCRIPTOR.full_name
    for key in models:
      model_list_msg.model.append(key)
    response.data = model_list_msg.SerializeToString()

  print "Response Size:", len(response.SerializeToString())
  hex_size = hex(len(response.SerializeToString()))
  conn.send(hex_size.rjust(8))
  conn.send(response.SerializeToString())  # echo
conn.close()
