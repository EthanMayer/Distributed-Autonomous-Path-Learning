def enterREPL():
    while True:
        try:
            code = input("> ")
            eval(code)
        except KeyboardInterrupt:
            return
        finally:
            logging.error(traceback.format_exc())