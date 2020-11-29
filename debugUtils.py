import logging
import traceback

# Captures local variables used in the callee!
def enterREPL(locals=locals()):
    while True:
        try:
            code = input("> ")
            exec(code, locals=locals)
        except KeyboardInterrupt:
            return
        except:
            logging.error(traceback.format_exc())