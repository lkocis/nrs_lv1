"""Thread-safe software bias estimation for six-axis FT samples."""

from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class ZeroResult:
    """A completed software-zero operation."""

    generation: int
    offset: tuple


class ZeroingManager:
    """Collect samples for software zeroing and apply the resulting bias."""

    def __init__(self):
        self._condition = threading.Condition()
        self._generation = 0
        self._active_generation = None
        self._target_count = 0
        self._sample_count = 0
        self._sums = [0.0] * 6
        self._offset = (0.0,) * 6
        self._completed = None
        self._closed = False

    @property
    def offset(self):
        """Return the current immutable six-axis bias."""
        with self._condition:
            return self._offset

    @property
    def collecting(self):
        """Report whether an operation is currently collecting samples."""
        with self._condition:
            return self._active_generation is not None

    def request(self, sample_count):
        """Start a new zero operation and return its generation number."""
        if sample_count <= 0:
            raise ValueError('sample_count must be greater than zero')
        with self._condition:
            self._generation += 1
            self._active_generation = self._generation
            self._target_count = sample_count
            self._sample_count = 0
            self._sums = [0.0] * 6
            self._condition.notify_all()
            return self._generation

    def reset_progress(self):
        """Restart an active collection after a TCP reconnect."""
        with self._condition:
            if self._active_generation is None:
                return
            self._sample_count = 0
            self._sums = [0.0] * 6

    def process(self, sample):
        """Consume a sample, returning corrected data or a zero result."""
        if len(sample) != 6:
            raise ValueError('an FT sample must contain exactly six values')

        with self._condition:
            if self._active_generation is None:
                corrected = tuple(
                    value - bias
                    for value, bias in zip(sample, self._offset)
                )
                return corrected, None

            for index, value in enumerate(sample):
                self._sums[index] += value
            self._sample_count += 1
            if self._sample_count < self._target_count:
                return None, None

            generation = self._active_generation
            self._offset = tuple(
                total / self._sample_count for total in self._sums
            )
            result = ZeroResult(generation, self._offset)
            self._completed = result
            self._active_generation = None
            self._condition.notify_all()
            return None, result

    def wait(self, generation, timeout_sec):
        """Wait for one requested generation to complete."""
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while not self._closed:
                if (
                    self._completed is not None
                    and self._completed.generation == generation
                ):
                    return self._completed
                if self._active_generation != generation:
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
        return None

    def cancel(self, generation):
        """Cancel *generation* if it is still active."""
        with self._condition:
            if self._active_generation == generation:
                self._active_generation = None
                self._condition.notify_all()

    def close(self):
        """Wake waiting service callbacks during node shutdown."""
        with self._condition:
            self._closed = True
            self._active_generation = None
            self._condition.notify_all()
