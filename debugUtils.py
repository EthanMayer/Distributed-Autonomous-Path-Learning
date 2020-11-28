import logging
import traceback

def enterREPL():
    while True:
        try:
            code = input("> ")
            exec(code)
        except KeyboardInterrupt:
            return
        except:
            logging.error(traceback.format_exc())