import logging
import traceback

# Must pass `globals` parameter as `globals()`
# and `locals` parameter as `locals()`, which
# captures global and local variables used in the 
# callee's file and scope, respectively.
def enterREPL(globals, locals):
    if globals is None:
        raise Exception("Need globals() to be passed to enterREPL()")
    if locals is None:
        raise Exception("Need locals() to be passed to enterREPL()")

    while True:
        try:
            code = input("> ")
            exec(code, globals, locals)
        except KeyboardInterrupt:
            return
        except:
            logging.error(traceback.format_exc())