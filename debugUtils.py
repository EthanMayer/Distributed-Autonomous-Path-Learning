import logging
import traceback

# Must pass `locals` parameter as `locals()`, which
# captures local variables used in the callee!
def enterREPL(locals):
    while True:
        try:
            code = input("> ")
            exec(code, None, locals)
        except KeyboardInterrupt:
            return
        except:
            logging.error(traceback.format_exc())