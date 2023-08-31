import time

class TimerError(Exception):
     """A custom exception used to report errors in use of Timer class"""

class Timer:
    def __init__(self, timer_repeat_times = 10):
        self._start_time = None
        self.elapsed_time = None
        self.average_time = None
        self.timer_repeat_times = timer_repeat_times

    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")

        self._start_time = time.perf_counter()

    def stop(self):
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")

        self.elapsed_time = time.perf_counter() - self._start_time
        self._start_time = None
        return self.elapsed_time

    def str_elapsed_time(self):
        return f"Elapsed time: {self.elapsed_time:0.8f} seconds"

    def str_average(self):
        return f"Average time with {self.timer_repeat_times} tries: {self.average_time:0.8f} seconds"

    def time_average(self, test_function):
        accumulated_time = 0.0
        for _ in range(self.timer_repeat_times - 1):
            self.start()
            test_function()
            accumulated_time = accumulated_time + self.stop()
        self.start()
        packed_return = test_function()
        self.average_time  = (accumulated_time + self.stop())/self.timer_repeat_times
        return packed_return
