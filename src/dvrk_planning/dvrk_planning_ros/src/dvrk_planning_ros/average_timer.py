import time

class TimerError(Exception):
     """A custom exception used to report errors in use of Timer class"""

class AverageTimer:
    def __init__(self):
        self._start_time = None
        
        # Statistics
        self.total_time = 0.0
        self.tries = 0

    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")

        self._start_time = time.perf_counter()

    def stop(self):
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")

        elapsed_time = time.perf_counter() - self._start_time
        self.total_time = self.total_time + elapsed_time
        self.tries = self.tries + 1
        self._start_time = None

    def str_statistics_report(self):
        str_return = f"Tries: {self.tries}\n"
        str_return = str_return + f"Average Time: {self.total_time/self.tries:0.8f} seconds\n"
        str_return = str_return + f"Total Time: {self.total_time:0.8f} seconds"
        return str_return
