import logging
import traceback

# Must pass `locals` parameter as `locals()`, which
# captures local variables used in the callee!
def enterREPL(locals):
    if locals is None:
        raise Exception("Need locals() to be passed to enterREPL()")

    while True:
        try:
            code = input("> ")
            exec(code, globals(), locals)
        except KeyboardInterrupt:
            return
        except:
            logging.error(traceback.format_exc())