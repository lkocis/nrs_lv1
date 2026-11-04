"""Loopback test for TCP fragmentation and automatic reconnect."""

import socket
import threading

from robotiq_ft_streamer.client import FTStreamClient


def test_client_parses_fragmented_samples_and_reconnects():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', 0))
    listener.listen(2)
    port = listener.getsockname()[1]
    server_errors = []

    def serve():
        try:
            first, _ = listener.accept()
            with first:
                first.sendall(b'(1, 2, 3')
                first.sendall(b', 4, 5, 6)(bad)')
            second, _ = listener.accept()
            with second:
                second.sendall(b'(7,8,9,10,11,12)')
        except Exception as error:  # pragma: no cover - surfaced below
            server_errors.append(error)
        finally:
            listener.close()

    stop_event = threading.Event()
    samples = []
    connections = []

    def receive(sample):
        samples.append(sample)
        if len(samples) == 2:
            stop_event.set()

    server = threading.Thread(target=serve, daemon=True)
    server.start()
    client = FTStreamClient(
        host='127.0.0.1',
        port=port,
        sample_callback=receive,
        connection_callback=lambda state, message: connections.append(state),
        connect_timeout_sec=0.2,
        read_timeout_sec=0.1,
        stale_timeout_sec=0.5,
        reconnect_delay_sec=0.01,
    )
    worker = threading.Thread(target=client.run, args=(stop_event,))
    worker.start()
    worker.join(timeout=2.0)
    stop_event.set()
    server.join(timeout=2.0)

    assert not worker.is_alive()
    assert not server.is_alive()
    assert not server_errors
    assert samples == [
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0, 10.0, 11.0, 12.0),
    ]
    assert connections.count(True) == 2
    assert False in connections
