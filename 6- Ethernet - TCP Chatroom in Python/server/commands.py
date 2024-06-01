# Author    :   Hossam Elwahsh
# Date      :   18/9/2023
# File      :   commands.py
# Brief     :   Contains class, enums of commands used in a chat room server


from enum import Enum

# Chat Room Commands Class
class Command:
    command_str = None
    command_help_line = None

    def __init__(self, command_str, command_help_line):
        self.command_str = command_str
        self.command_help_line = command_help_line


class CommandMapping(Enum):
    HELP = '/help'
    BUZZ = '/buzz'
    SET_USERNAME = '/username'
    PRIVATE_MSG = '/pm'
    LED_RED = '/red'
    LED_BLUE = '/blue'
    LED_GREEN = '/green'
    LED_YELLOW = '/yellow'
    LED_MAGENTA = '/magenta'
    LED_CYAN = '/cyan'


# Commands list
list_of_commands = [
    # Server Commands
    Command(CommandMapping.HELP, "/help for list of commands"),

    # Chat Commands
    Command(CommandMapping.BUZZ, "/buzz : sends BUZZ"),
    Command(CommandMapping.SET_USERNAME, "/username [username] : sets your username"),
    Command(CommandMapping.PRIVATE_MSG, "/pm [username] [msg] : send a private msg"),

    # Led Control
    Command(CommandMapping.LED_RED, "/red : turns on red LED"),
    Command(CommandMapping.LED_BLUE, "/blue : turns on blue LED"),
    Command(CommandMapping.LED_GREEN, "/green : turns on green LED"),
    Command(CommandMapping.LED_YELLOW, "/yellow : turns on yellow LED"),
    Command(CommandMapping.LED_MAGENTA, "/magenta : turns on magenta LED"),
]
