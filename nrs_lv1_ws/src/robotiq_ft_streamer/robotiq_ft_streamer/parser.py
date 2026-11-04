"""Incremental parser for the Robotiq FT controller TCP stream."""

import math


class SampleParser:
    """Extract finite six-axis samples from arbitrary TCP byte chunks."""

    def __init__(self, max_buffer_bytes=65536):
        if max_buffer_bytes < 64:
            raise ValueError('max_buffer_bytes must be at least 64')
        self._max_buffer_bytes = max_buffer_bytes
        self._buffer = ''

    def reset(self):
        """Discard any incomplete frame from the previous connection."""
        self._buffer = ''

    def feed(self, chunk):
        """Return every complete valid sample found in *chunk*."""
        if isinstance(chunk, bytes):
            text = chunk.decode('ascii', errors='ignore')
        else:
            text = str(chunk)
        self._buffer += text

        samples = []
        while True:
            start = self._buffer.find('(')
            if start < 0:
                self._trim_without_start_marker()
                break

            if start:
                self._buffer = self._buffer[start:]

            end = self._buffer.find(')', 1)
            if end < 0:
                self._trim_incomplete_frame()
                break

            payload = self._buffer[1:end]
            self._buffer = self._buffer[end + 1:]
            values = self._parse_payload(payload)
            if values is not None:
                samples.append(values)

        return samples

    @staticmethod
    def _parse_payload(payload):
        fields = payload.split(',')
        if len(fields) != 6:
            return None
        try:
            values = tuple(float(field.strip()) for field in fields)
        except ValueError:
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    def _trim_without_start_marker(self):
        if len(self._buffer) > self._max_buffer_bytes:
            self._buffer = ''

    def _trim_incomplete_frame(self):
        if len(self._buffer) <= self._max_buffer_bytes:
            return
        latest_start = self._buffer.rfind('(')
        self._buffer = self._buffer[latest_start:]
        if len(self._buffer) > self._max_buffer_bytes:
            self._buffer = ''
