#-*-coding:utf-8-*-
import socket
import struct

from tesse.msgs import Interface
from tesse.msgs import DataResponse

# 修改端口号防止与ipykernel冲突
class Env(object):
    def __init__(
        self,
        simulation_ip='localhost',
        own_ip='localhost',
        position_port=19000,
        metadata_port=19001,
        image_port=19002,
        step_port=19005,
        # des_port = 9006,
        done_port = 19009
    ):
        self.simulation_ip = simulation_ip
        self.own_ip = own_ip
        self.position_port = position_port
        self.metadata_port = metadata_port
        self.image_port = image_port
        self.step_port = step_port

    def get_port(self, msg):
        if msg.get_interface() == Interface.POSITION:
            port = self.position_port
        elif msg.get_interface() == Interface.METADATA:
            port = self.metadata_port
        elif msg.get_interface() == Interface.IMAGE:
            port = self.image_port
        elif msg.get_interface() == Interface.STEP:
            port = self.step_port
        else:
            raise ValueError("Invalid message interface: {}".format(msg.get_interface()))

        return port


    def send(self, msg):
        if self.get_port(msg) == self.step_port:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # tcp socket
            s.connect((self.simulation_ip, self.get_port(msg)))
            s.send(msg.encode())
            s.recv(3)
            s.close()
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp socket
            s.sendto(msg.encode(), (self.simulation_ip, self.get_port(msg)))
            s.close()


    def request(self, msg, timeout=1):
        # setup receive socket
        recv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv.settimeout(timeout)
        recv.bind((self.own_ip, self.get_port(msg)))
        recv.listen(1)

        # send request
        self.send(msg)

        # wait for a connection
        try:
            conn, addr = recv.accept()
            conn.setblocking(1)
        except socket.timeout:
            recv.close()
            return

        # get message tag
        tag = conn.recv(4).decode('utf-8')
        if tag not in ['mult', 'meta', 'cami', 'scni', 'obji']:
            conn.close()
            recv.close()
            raise ValueError('Unknown tag received {}'.format(tag))

        # get maximum message payload length
        if tag == 'mult':
            header = conn.recv(8)
            payload_length_imgs = struct.unpack("I", header[:4])[0]
            payload_length_meta = struct.unpack("I", header[4:])[0]
            max_payload_length = payload_length_imgs + payload_length_meta
        else:
            header = conn.recv(4)
            max_payload_length = struct.unpack("I", header)[0]

        # allocate payload buffer
        payload = bytearray(max_payload_length)

        # get payload
        total_bytes_read = 0
        payload_view = memoryview(payload)
        while total_bytes_read < max_payload_length:
            bytes_read = conn.recv_into(payload_view, max_payload_length - total_bytes_read)
            if bytes_read == 0:
                break
            payload_view = payload_view[bytes_read:]
            total_bytes_read += bytes_read
        payload_view = memoryview(payload)[:total_bytes_read]

        # close socket
        conn.close()
        recv.close()

        # parse payload buffer
        if tag == 'mult':
            imgs_payload = payload_view[:-payload_length_meta]
            meta_payload = payload_view[-payload_length_meta:]
            # ignore the default metadata payload that is currently returned when
            # DataRequest(metadata=False)
            if len(meta_payload) == 4 and struct.unpack('I', meta_payload)[0] == 0:
                meta_payload = None
            return DataResponse(images=imgs_payload, metadata=meta_payload)
        else:
            return DataResponse(metadata=payload_view)
