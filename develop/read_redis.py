


from redis_base import RedisBase
import time

redis_obj = RedisBase("localhost", 1, 1)

while True:
    
    t = redis_obj.read("TIMESTAMP")
    print(t)
    time.sleep(0.001)


