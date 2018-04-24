# import numpy as np
# import matplotlib.pyplot as plt
# import socket

# # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# server_address = '127.0.0.1'
# server_port = 4400

# server = (server_address, server_port)
# # sock.bind(server)
# sock.connect(server)
# print("Listening on " + server_address + ":" + str(server_port))

# plt.axis([-50, 50, -50, 50])

# while True:
#     # payload, client_address = sock.recvfrom(1024)
#     size = sock.recv(2)
#     payload = sock.recv(int(str(size)))
    
#     print("Received - " + str(payload));
#     [x, y, color] = payload.split(',')
#     if (int(color) == 0):
#         plt.scatter(x,y, color = 'yellow')
#     if (int(color) == 1):
#         plt.scatter(x,y, color ='blue')
#     plt.pause(0.0001);
    
# plt.draw()
