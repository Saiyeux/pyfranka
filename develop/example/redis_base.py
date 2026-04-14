import redis
import msgpack

class RedisBase():
    def __init__(self, host, database, socket_connect_timeout):
        self.__r = redis.StrictRedis(host=host, port=6379, db=database, socket_connect_timeout=socket_connect_timeout)

    def write(self, key, value):
        try:
            self.__r.set(key.encode('utf-8'), msgpack.dumps(value))
        except Exception as e:
            print(Exception, e)
            return None

    def read(self, key):
        try:
            return msgpack.unpackb(self.__r.get(key.encode('utf-8')))
        except Exception as e:
            print(Exception, e)
            print("data is not existed")
            return None