from time import time
from functools import wraps

def timer(func):
    @wraps(func)
    def time_wrapper(*args, **kwargs):
        start_time = time()
        result = func(*args, **kwargs)
        end_time = time()
        total_time = end_time - start_time

        from . import VERBOSITY
        if VERBOSITY:
            print(f"Function '{func.__name__}' took {total_time:.4f} seconds")
        return result
    return time_wrapper