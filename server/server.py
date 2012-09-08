#!/usr/bin/env python

import socket
import struct
from sys import *
from xml.dom import minidom

import request_pb2

TCP_IP = '127.0.0.1'
TCP_PORT = 5018
BUFFER_SIZE = 1024

models = []

#################################################
# Read the main manifest file
def read_manifest():
  xmldoc = minidom.parse('manifest.xml')
  model_list = xmldoc.getElementsByTagName("model")
  
  for model in model_list:
    model_name = model.childNodes[0].data
    models.append(model_name)
  


read_manifest()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print 'Connection address:', addr
while 1:
  print "here"
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
    for m in models:
      model_list_msg.model.append(m)
    response.data = model_list_msg.SerializeToString()

  print "Response Size:", len(response.SerializeToString())
  hex_size = hex(len(response.SerializeToString()))
  conn.send(hex_size.rjust(8))
  conn.send(response.SerializeToString())  # echo
conn.close()
