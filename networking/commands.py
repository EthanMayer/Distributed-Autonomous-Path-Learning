from enum import Enum

class NetworkCommand(Enum):
    uploadPath = 1 # [Client to server] Upload a path for the robot to follow. Format:
    # [uploadPath enum value number as ASCII] [path length] [path bytes]
    setPath = 2 # [Server to client] Tells a robot to follow a path. Format:
    # [setPath enum value number as ASCII] [path length] [path bytes]

    # https://stackoverflow.com/questions/43634618/how-do-i-test-if-int-value-exists-in-python-enum-without-using-try-catch
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_ 