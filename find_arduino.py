from boards import boards_dict
from serial.tools import list_ports
import re

def check_ports():
    not_arduinos = []
    arduinos = []
    for connection in list_ports.comports():
        port,hwid,desc = connection
        vid_pid = re.search(r"(?<=VID\:PID\=)[0-9|A-Z|a-z]{4}\:[0-9|A-Z|a-z]{4}", desc)
        vid_pid = None if vid_pid is None else vid_pid.group()
        if vid_pid is None:
            not_arduinos.append(connection)
        else:
            try:
                board = boards_dict[vid_pid]
                arduinos.append((board,connection))
            except KeyError:
                not_arduinos.append(connection)
    return arduinos, not_arduinos

if __name__ == "__main__":
    arduinos, not_arduinos = check_ports()
    for ard in arduinos:
        print(ard)
    for nard in not_arduinos:
        print(nard)
