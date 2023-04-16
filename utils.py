def set_bit(value, bit):
        return value | bit

def clear_bit(value, bit):
    return value & ~bit

Enum = object

try:
    import enum
    Enum = enum.Enum
except:
    pass