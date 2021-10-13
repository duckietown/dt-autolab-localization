import time
from collections import defaultdict
from typing import Dict, Optional, List, Union, Callable

# @dataclasses.dataclass
# class T2ProfilerDataPoint:
#     tick: Optional[float] = None
#     tock: Optional[float] = None


# | x | 12 | 12 | 12 |

TABLE_SEPARATOR = "+-{0}-+------------+------------+------------+\n"

TABLE_HEADER_FMT = "| {0} |  # Calls   |  Time(ms)  |  Freq(Hz)  |\n"

TABLE_ROW_FMT = "| {key} | {count} | {duration} | {frequency} |\n"


class T2Profiler:

    __enabled__: bool = True

    class ProfilingContext:
        def __init__(self, key: str):
            self._key = key

        def __enter__(self):
            T2Profiler.tick(self._key)

        def __exit__(self, exc_type, exc_val, exc_tb):
            T2Profiler.tock(self._key)

    _buffer: Dict[str, float] = {}
    _data: Dict[str, List[float]] = {}
    _count: Dict[str, int] = defaultdict(lambda: 0)

    @staticmethod
    def tick(key: str):
        if not T2Profiler.__enabled__:
            return
        # ---
        now = time.time() * 1000
        T2Profiler._buffer[key] = now

    @staticmethod
    def tock(key: str):
        if not T2Profiler.__enabled__:
            return
        # ---
        now = time.time() * 1000
        # ---
        try:
            tick = T2Profiler._buffer[key]
            del T2Profiler._buffer[key]
        except KeyError:
            return
        # ---
        if key not in T2Profiler._data:
            T2Profiler._data[key] = []
        T2Profiler._data[key].append((now - tick))
        T2Profiler._count[key] += 1

    @staticmethod
    def enabled(status: bool):
        T2Profiler.__enabled__ = status

    @staticmethod
    def print():
        if not T2Profiler.__enabled__:
            return
        # ---
        # find longest key
        column_size = max(*[len(k) for k in T2Profiler._count.keys()])
        # compile table
        table = ""
        table += TABLE_SEPARATOR.format("-" * column_size)
        table += TABLE_HEADER_FMT.format("Key".ljust(column_size, ' '))
        table += TABLE_SEPARATOR.format("-" * column_size)
        for key in sorted(T2Profiler._count.keys()):
            durations = T2Profiler._data[key]
            count = T2Profiler._count[key]
            # compute stats
            avg_duration = sum(durations) / float(count)
            avg_frequency = 1.0 / (avg_duration / 1000.0)
            row = TABLE_ROW_FMT.format(
                key=key.ljust(column_size, ' '),
                count=str(count).rjust(10, ' '),
                duration=str(round(avg_duration, 2)).rjust(10, ' '),
                frequency=str(round(avg_frequency, 1)).rjust(10, ' '),
            )
            table += row
        table += TABLE_SEPARATOR.format("-" * column_size)
        print()
        print("Engine Profiling Information:")
        print(table)
        print()

    @staticmethod
    def profile(key: str) -> ProfilingContext:
        return T2Profiler.ProfilingContext(key)

    @staticmethod
    def profiled(key_or_function: Optional[Union[str, Callable]] = None):
        if isinstance(key_or_function, str):
            decorator_key = key_or_function

            def wrapper_factory(func):
                key = func.__name__ if decorator_key is None else decorator_key

                def wrapper(*args, **kwargs):
                    with T2Profiler.profile(key):
                        return func(*args, **kwargs)

                return wrapper

            return wrapper_factory

        elif callable(key_or_function):
            func = key_or_function
            key = func.__name__

            def wrapper(*args, **kwargs):
                with T2Profiler.profile(key):
                    return func(*args, **kwargs)

            return wrapper

        else:
            return None


