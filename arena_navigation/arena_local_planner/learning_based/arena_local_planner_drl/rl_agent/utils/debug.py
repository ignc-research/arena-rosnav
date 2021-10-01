import time
from functools import wraps
import rospy
from sensor_msgs.msg import LaserScan
import contextlib
import sys
import tqdm
import numpy as np
def timeit(f):
    @wraps(f)
    def timed(*args, **kw):

        ts = time.time()
        result = f(*args, **kw)
        te = time.time()

        print ('func:%r args:[%r, %r] took: %2.4f sec' % \
          (f.__name__, args, kw, te-ts))
        return result

    return timed

import zmq
class NPPSERVER:
  def __init__(self,port) -> None:
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f'tcp://*:{port}')
    self.socket = socket
  def send_nparray(self,array:np.ndarray):
    md = dict(
        dtype = str(array.dtype),
        shape = array.shape,
    )
    self.socket.send_json(md,0|zmq.SNDMORE)
    return self.socket.send(array, 0, copy=True, track=False)


class DummyFile(object):
  file = None
  def __init__(self, file):
    self.file = file

  def write(self, x):
    # Avoid print() second call (useless \n)
    if len(x.rstrip()) > 0:
        tqdm.tqdm.write(x, file=self.file)
  def flush(self):
    pass

@contextlib.contextmanager
def nostdout():
    save_stdout = sys.stdout
    sys.stdout = DummyFile(sys.stdout)
    yield
    sys.stdout = save_stdout