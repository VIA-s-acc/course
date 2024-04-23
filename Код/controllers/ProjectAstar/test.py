import math

def shortest_rotation(current_angle, target_angle):
    # Нормализация углов к диапазону [0, 2*pi)
    current_angle = current_angle % (2 * math.pi)
    target_angle = target_angle % (2 * math.pi)
    
    diff = (target_angle - current_angle + 2 * math.pi) % (2 * math.pi)
    
    if diff <= math.pi:
        return "H"
    else:
        return "A"

