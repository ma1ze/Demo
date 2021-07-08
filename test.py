from threading import Thread
import struct
import time
import hashlib
import base64
import socket
import time
import types
import multiprocessing
import os
mode = "initialize"
pic_size = 0
pic_receive = 0
pic = ""
pic_repeat = []

class returnCrossDomain(Thread):
	def __init__(self, connection):
		Thread.__init__(self)
		self.con = connection
		self.isHandleShake = False

	def run(self):
		global mode
		global pic_size
		global pic_receive
		global pic
		global pic_repeat
		while True:
			if not self.isHandleShake:
				# 开始握手阶段
				header = self.analyzeReq()
				secKey = header['Sec-WebSocket-Key'];

				acceptKey = self.generateAcceptKey(secKey)

				response = "HTTP/1.1 101 Switching Protocols\r\n"
				response += "Upgrade: websocket\r\n"
				response += "Connection: Upgrade\r\n"
				response += "Sec-WebSocket-Accept: %s\r\n\r\n" % (acceptKey.decode('utf-8'))
				self.con.send(response.encode())
				self.isHandleShake = True
				if(mode=="initialize"):
					mode = "get_order"
				print('response:\r\n' + response)
				# 握手阶段结束

				#读取命令阶段
			elif mode == "get_order":
				opcode = self.getOpcode()
				if opcode == 8:
					self.con.close()
				self.getDataLength()
				clientData = self.readClientData()
				print('客户端数据：' + str(clientData))
				# 处理数据
				ans = self.answer(clientData)
				self.sendDataToClient(ans)

				# if (ans != "Unresolvable Command!" and ans != "hello world"):
				# 	pic_size = int(clientData[3:])
				# 	pic_receive = 0
				# 	pic = ""
				# 	pic_repeat=[]
				# 	print("需要接收的数据大小：", pic_size)
				# mode = "get_pic"

				#读取图片阶段
			elif mode == "get_pic":
				opcode = self.getOpcode()
				if opcode == 8:
					self.con.close()
				self.getDataLength()
				clientData = self.readClientData()
				print('客户端数据：' + str(clientData))
				pic_receive += len(clientData)
				pic += clientData
				if pic_receive < pic_size:
					self.sendDataToClient("Receive:"+str(pic_receive)+"/"+str(pic_size))
					print("图片接收情况:",pic_receive,"/",pic_size)
					#print("当前图片数据:",pic)
				else:
					print("完整图片数据:",pic)
					self.sendDataToClient("Receive:100%")
					result = self.process(pic)
					self.sendDataToClient(result)
					pic_size = 0
					pic_receive = 0
					pic = ""
					pic_repeat=[]
					mode = "get_order"
				# 处理数据

				# self.sendDataToClient(clientData)

	def legal(self, string):  # python总会胡乱接收一些数据。。只好过滤掉
		if len(string) == 0:
			return 0
		elif len(string) <= 100:
			if self.loc(string) != len(string):
				return 0
			else:
				if mode != "get_pic":
					return 1
				elif len(string) + pic_receive == pic_size:
					return 1
				else:
					return 0
		else:
			if self.loc(string) > 100:
				if mode != "get_pic":
					return 1
				elif string[0:100] not in pic_repeat:
					pic_repeat.append(string[0:100])
					return 1
				else:
					return -1  # 收到重复数据，需要重定位
			else:
				return 0

	def loc(self, string):
		i = 0
		while(i<len(string) and self.rightbase64(string[i])):
			i = i+1
		return i

	def rightbase64(self, ch):
		if (ch >= "a") and (ch <= "z"):
			return 1
		elif (ch >= "A") and (ch <= "Z"):
			return 1
		elif (ch >= "0") and (ch <= "9"):
			return 1
		elif ch == '+' or ch == '/' or ch == '|' or ch == '=' or ch == ' ' or ch == "'" or ch == '!' or ch == ':':
			return 1
		else:
			return 0

	def analyzeReq(self):
		reqData = self.con.recv(1024).decode()
		reqList = reqData.split('\r\n')
		headers = {}
		for reqItem in reqList:
			if ': ' in reqItem:
				unit = reqItem.split(': ')
				headers[unit[0]] = unit[1]
		return headers

	def generateAcceptKey(self, secKey):
		sha1 = hashlib.sha1()
		sha1.update((secKey + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode())
		sha1_result = sha1.digest()
		acceptKey = base64.b64encode(sha1_result)
		return acceptKey

	def getOpcode(self):
		first8Bit = self.con.recv(1)
		first8Bit = struct.unpack('B', first8Bit)[0]
		opcode = first8Bit & 0b00001111
		return opcode

	def getDataLength(self):
		second8Bit = self.con.recv(1)
		second8Bit = struct.unpack('B', second8Bit)[0]
		masking = second8Bit >> 7
		dataLength = second8Bit & 0b01111111
		#print("dataLength:",dataLength)
		if dataLength <= 125:
			payDataLength = dataLength
		elif dataLength == 126:
			payDataLength = struct.unpack('H', self.con.recv(2))[0]
		elif dataLength == 127:
			payDataLength = struct.unpack('Q', self.con.recv(8))[0]
		self.masking = masking
		self.payDataLength = payDataLength
		#print("payDataLength:", payDataLength)



	def readClientData(self):

		if self.masking == 1:
			maskingKey = self.con.recv(4)
		data = self.con.recv(self.payDataLength)

		if self.masking == 1:
			i = 0
			trueData = ''
			for d in data:
				trueData += chr(d ^ maskingKey[i % 4])
				i += 1
			return trueData
		else:
			return data

	def sendDataToClient(self, text):
		sendData = ''
		sendData = struct.pack('!B', 0x81)

		length = len(text)
		if length <= 125:
			sendData += struct.pack('!B', length)
		elif length <= 65536:
			sendData += struct.pack('!B', 126)
			sendData += struct.pack('!H', length)
		elif length == 127:
			sendData += struct.pack('!B', 127)
			sendData += struct.pack('!Q', length)

		sendData += struct.pack('!%ds' % (length), text.encode())
		dataSize = self.con.send(sendData)

	def answer(self,data):
		if(data[0:3]=="111"):
			return "111"
		elif(data[0:3]=="222"):
			return "answer for 222"
		elif (data[0:3] == "333"):
			return "answer for 333"
		else:
			return "Unresolvable Command!"

	def padding(self,data):
		missing_padding = 4 - len(data) % 4
		if missing_padding:
			data += '='*missing_padding
		return data

	def process(self,pic):

		#此处是图片处理阶段

		return pic

def main():
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.bind(('127.0.0.1', 9999))
	sock.listen(5)
	while True:
		try:
			connection, address = sock.accept()

			returnCrossDomain(connection).start()


		except:
			time.sleep(1)

if __name__ == "__main__":
	main()
