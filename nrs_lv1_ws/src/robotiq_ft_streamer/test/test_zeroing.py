"""Tests for software-zero collection and correction."""

from robotiq_ft_streamer.zeroing import ZeroingManager


def test_zeroing_averages_samples_and_suppresses_collection_output():
    zeroing = ZeroingManager()
    generation = zeroing.request(2)

    corrected, result = zeroing.process((1, 2, 3, 4, 5, 6))
    assert corrected is None
    assert result is None

    corrected, result = zeroing.process((3, 4, 5, 6, 7, 8))
    assert corrected is None
    assert result.generation == generation
    assert result.offset == (2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert zeroing.wait(generation, 0.01) == result

    corrected, result = zeroing.process((4, 5, 6, 7, 8, 9))
    assert corrected == (2.0, 2.0, 2.0, 2.0, 2.0, 2.0)
    assert result is None


def test_reconnect_resets_only_incomplete_collection_progress():
    zeroing = ZeroingManager()
    zeroing.request(2)
    zeroing.process((100, 100, 100, 100, 100, 100))

    zeroing.reset_progress()
    zeroing.process((2, 2, 2, 2, 2, 2))
    _, result = zeroing.process((4, 4, 4, 4, 4, 4))

    assert result.offset == (3.0, 3.0, 3.0, 3.0, 3.0, 3.0)


def test_timed_out_generation_can_be_cancelled_without_changing_offset():
    zeroing = ZeroingManager()
    generation = zeroing.request(2)

    assert zeroing.wait(generation, 0.001) is None
    zeroing.cancel(generation)
    corrected, result = zeroing.process((1, 2, 3, 4, 5, 6))

    assert corrected == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert result is None
