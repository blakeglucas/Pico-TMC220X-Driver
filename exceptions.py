import sys

class TMC_UART_MissingRXTXException(Exception):

    def __init__(self) -> None:
        super().__init__('TMC UART: if serialport != 0, RX and TX pins need to be provided')

class TMC_UART_WriteError(Exception):

    def __init__(self) -> None:
        super().__init__(f'TMC UART: Err in write {sys.stderr}')

class TMC_UART_WriteCheckFailedException(Exception):

    def __init__(self, reg, val, ifcnt1, ifcnt2) -> None:
        super().__init__(f'Write call succeeded but was not successful: reg={reg},val={val},ifcnt={ifcnt1}{ifcnt2}')

class TMC_UART_ReadError(Exception):

    def __init__(self) -> None:
        super().__init__(f'TMC UART: Err in read {sys.stderr}')
        
class TMC_UART_RepeatedInvalidResponseException(Exception):

    def __init__(self) -> None:
        super().__init__('TMC UART: no valid response received after configured retry attempts')