# Author    :   Hossam Elwahsh
# Date      :   18/9/2023
# File      :   server.py
# Brief     :   Contains chat room server implementation

import socket
import threading
from commands import *
from server_config import *


# Object for client connections
class ClientObj:
    connection = None
    address = None
    username = None

    def __init__(self, connection, address, username):
        self.connection = connection
        self.address = address
        self.username = username


# Clients list array
list_of_clients = []

# The first argument AF_INET is the address domain of the
# socket. This is used when we have an Internet Domain with
# any two hosts The second argument is the type of socket.
# SOCK_STREAM means that data or characters are read in
# a continuous flow.
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# checks whether sufficient arguments have been provided
# if len(sys.argv) != 3:
#     print("Correct usage: script, IP address, port number")
#     exit()

# binds the server to an entered IP address and at the
# specified port number.
# The client must be aware of these parameters

server.bind((IP_address, Port))

# listens for # active connections. This number can be
# increased as per convenience.
server.listen(Supported_clients_count)


def client_thread():
    # Get client object
    clientData = list_of_clients[len(list_of_clients) - 1]

    clientData.connection.send("\nWelcome to this chatroom!\nUse /help for list of commands\n\n".encode())
    while True:
        try:
            message = clientData.connection.recv(2048)
            if message:

                # Decode message
                message = message.decode()

                # Remove new line breaks
                message = message.replace('\n', '')

                # Match message
                match message:
                    case CommandMapping.HELP.value:
                        clientData.connection.send(
                            "\n############\nCOMMANDS LIST:\n\n".encode())
                        # Print all available commands
                        for cmd in list_of_commands:
                            clientData.connection.send(f"- {cmd.command_help_line}\n".encode())

                        clientData.connection.send("############\n".encode())

                    case CommandMapping.BUZZ.value:
                        broadcast(f'[{clientData.username}] BUZZ\n', clientData.connection)

                    case CommandMapping.LED_RED.value:
                        broadcast(f'[{clientData.username}] RED_ON\n', clientData.connection)

                    case CommandMapping.LED_BLUE.value:
                        broadcast(f'[{clientData.username}] BLUE_ON\n', clientData.connection)

                    case CommandMapping.LED_GREEN.value:
                        broadcast(f'[{clientData.username}] GREEN_ON\n', clientData.connection)

                    case CommandMapping.LED_YELLOW.value:
                        broadcast(f'[{clientData.username}] YELLOW_ON\n', clientData.connection)

                    case CommandMapping.LED_MAGENTA.value:
                        broadcast(f'[{clientData.username}] MAGENTA_ON\n', clientData.connection)

                    case _:
                        if message.startswith(CommandMapping.SET_USERNAME.value):

                            # Parse username argument
                            un = message[10:]
                            if un != '':
                                # update username
                                clientData.username = un.replace(' ', '_')  # handle white spaces
                                clientData.connection.send(
                                    f"\n\n[SERV] Username set to {clientData.username}\n\n".encode())
                                print(f"\n{clientData.address} Username set to ({clientData.username})\n\n")

                            else:
                                # print error
                                print(f"\n{clientData.username} Trying to set bad username {un}\n\n")
                                clientData.connection.send("[SERV] Bad username inserted\n".encode())

                        elif message.startswith(CommandMapping.PRIVATE_MSG.value):

                            # Parse username argument
                            un = message.split()[1]  # Get username
                            pm = message[len(CommandMapping.PRIVATE_MSG.value) + len(un) + 2:]
                            if un != '':
                                # send private message
                                # find client
                                to_client = find_client_with_username(un)
                                if to_client is not None:
                                    # Send message to client
                                    to_client.connection.send(f"PM from [{clientData.username}]: {pm}\n".encode())

                                    # User confirmation
                                    clientData.connection.send(
                                        f"\n\n[SERV] Msg sent to [{un}]\n\n".encode())

                                    print(f"\nPM {clientData.username} -> {to_client.username}: {pm}\n\n")

                                else:
                                    clientData.connection.send(
                                        f"\n\n[SERV] username ({un}) doesn't exist.\n\n".encode())

                                    print(f"\n[ERROR] PM {clientData.username} -> {un}, username doesn't exist\n\n")


                            else:
                                # print error
                                clientData.connection.send("[SERV] Bad format\n".encode())

                        else:
                            # prints the message and address of the
                            # user who just sent the message on the server
                            # terminal
                            print(f"[{clientData.username}] ", message)

                            # Calls broadcast function to send message to all
                            message_to_send = "[" + clientData.username + "] " + message + '\n'
                            broadcast(message_to_send, clientData.connection)

            else:
                # Message may have no content if the connection
                # is broken, in this case we remove the connection
                print(f'[{clientData.username}] connection lost')
                remove(clientData)

                # close thread
                return 0

        except:
            continue


# Using the below function, we broadcast the message to all
# clients who's object is not the same as the one sending
# the message

def broadcast(message, connection):
    for client in list_of_clients:
        if client != connection:
            try:
                client.connection.send(message.encode())
            except:
                client.connection.close()

                # if the link is broken, we remove the client
                remove(client)


# The following function simply removes the object
# from the list that was created at the beginning of
# the program

def remove(client):
    if client in list_of_clients:
        list_of_clients.remove(client)


# finds a client by username
def find_client_with_username(client_username):
    for client in list_of_clients:
        if client.username == client_username:
            return client
        else:
            continue

    return None


def accept_new_clients():
    print(f"\nServer started on {IP_address}:{Port}\n\n")

    while True:
        # Accepts a connection request and stores two parameters,
        # conn which is a socket object for that user, and addr
        # which contains the IP address of the client that just
        # connected
        conn, (ip_addr, client_port) = server.accept()

        # Create new client object
        new_client = ClientObj(conn, ip_addr, ip_addr)

        # Maintains a list of clients for ease of broadcasting
        # a message to all available people in the chatroom
        list_of_clients.append(new_client)

        # prints the address of the user that just connected
        print(ip_addr + " connected")

        # creates and individual thread for every user
        # that connects
        threading.Thread(target=client_thread).start()

    conn.close()
    server.close()


def server_start():
    # START THREADS
    # Start Accepting Clients
    threading.Thread(target=accept_new_clients()).start()
