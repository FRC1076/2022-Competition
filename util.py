def clamp(value : float, lower : float = -1.0, upper : float = 1.0):
    if (value > upper):
        return upper

    if (value < lower):
        return lower
    
    return value