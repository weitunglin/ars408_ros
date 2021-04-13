import math
def FCW_TTC(vel, acc, dist):
    acc = acc if acc != 0 else 0.01
    key = vel**2 + 2 * acc * dist
    if key >= 0:
        ttc = (-vel + math.sqrt(key)) / acc
    if key < 0:
        return None
    if ttc < 0:
        return None
    return ttc

print(FCW_TTC(1.25,0.01, 22.4))
# print(math.sqrt(0))