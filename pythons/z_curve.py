def mask_z(x,y):
    bx = x;by = y
    mask1_x,mask1_y = 0b0000111100001111,0b0000111100001111
    mask2_x,mask2_y = 0b0011001100110011,0b0011001100110011
    mask3_x,mask3_y = 0b0101010101010101,0b0101010101010101


    bx = (bx << 4) | bx
    by = (by << 4) | by
    bx = bx & mask1_x
    by = by & mask1_y

    bx = (bx << 2) | bx
    by = (by << 2) | by
    bx = bx & mask2_x
    by = by & mask2_y

    bx = (bx << 1) | bx
    by = (by << 1) | by
    bx = bx & mask3_x
    by = by & mask3_y

    bx = bx << 1
    z = bx | by
    return z

def calc_z(x,y):
    x = np.uint16(x)
    y = np.uint16(y)
    bx = x >> 8
    by = y >> 8
    hz = mask_z(bx,by)

    bx = x & 0b11111111
    by = y & 0b11111111

    lz = mask_z(bx,by)
    z = hz << 8 | lz
    return z

from functools import wraps
import time
def stop_watch(func) :
    @wraps(func)
    def wrapper(*args, **kargs) :
        start = time.time()
        result = func(*args,**kargs)
        elapsed_time =  time.time() - start
        print(f"{func.__name__}は{elapsed_time}秒かかりました")
        return result
    return wrapper

import numpy as np
N = 2
# x = np.array([2,4],dtype=np.float64)
# obs = np.array([[np.random.randint(10),np.random.randint(10)] for i in range(10-1)],dtype=np.float64)

def get_near_obs(x,obs,N):
    z_x = calc_z(x[0],x[1])
    z_obs = np.apply_along_axis(lambda x:calc_z(x[0],x[1]),1,obs)
    z_obs_diff = np.apply_along_axis(lambda x:(x-z_x)**2,0,z_obs)
    indexs = np.argsort(z_obs_diff)
    return indexs[:N] 

# print(get_near_obs(x,obs,2))
# x,y = 23245,35159
# print(calc_z(100,100),calc_z(101,101),calc_z(102,101))





