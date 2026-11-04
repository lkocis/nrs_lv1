"""Reconnect-capable TCP client for the Robotiq FT controller stream."""

import socket
import time

from robotiq_ft_streamer.parser import SampleParser


class FTStreamClient:
    """Read parsed FT samples until the supplied stop event is set."""

    def __init__(
        self,
        host,
        port,
        sample_callback,
        connection_callback=None,
        connect_timeout_sec=3.0,
        read_timeout_sec=1.0,
        stale_timeout_sec=5.0,
        reconnect_delay_sec=2.0,
        max_buffer_bytes=65536,
    ):
        self._host = host
        self._port = port
        self._sample_callback = sample_callback
        self._connection_callback = connection_callback
        self._connect_timeout_sec = connect_timeout_sec
        self._read_timeout_sec = read_timeout_sec
        self._stale_timeout_sec = stale_timeout_sec
        self._reconnect_delay_sec = reconnect_delay_sec
        self._parser = SampleParser(max_buffer_bytes)

    def run(self, stop_event):
        """Connect, read, and reconnect until *stop_event* is set."""
        while not stop_event.is_set():
            connection = None
            try:
                connection = socket.create_connection(
                    (self._host, self._port),
                    timeout=self._connect_timeout_sec,
                )
                connection.settimeout(self._read_timeout_sec)
                self._parser.reset()
                self._report_connection(True, '')
                self._read_connection(connection, stop_event)
            except (ConnectionError, OSError, TimeoutError) as error:
                if not stop_event.is_set():
                    self._report_connection(False, str(error))
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except OSError:
                        pass

            if not stop_event.is_set():
                stop_event.wait(self._reconnect_delay_sec)

    def _read_connection(self, connection, stop_event):
        last_data_time = time.monotonic()
        while not stop_event.is_set():
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                if time.monotonic() - last_data_time >= self._stale_timeout_sec:
                    raise TimeoutError('FT stream became stale')
                continue

            if not chunk:
                raise ConnectionError('FT stream closed by the controller')
            last_data_time = time.monotonic()
            for sample in self._parser.feed(chunk):
                self._sample_callback(sample)

    def _report_connection(self, connected, message):
        if self._connection_callback is not None:
            self._connection_callback(connected, message)
