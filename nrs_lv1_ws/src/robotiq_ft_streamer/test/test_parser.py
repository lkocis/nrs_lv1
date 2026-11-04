"""Tests for fragmented and malformed Robotiq FT TCP frames."""

from robotiq_ft_streamer.parser import SampleParser


def test_parser_handles_fragmented_and_adjacent_samples():
    parser = SampleParser()

    assert parser.feed(b'ignored(1 , 2, 3') == []
    samples = parser.feed(b', 4, 5, 6)(-1,2e0,3.5,4,5,6)tail')

    assert samples == [
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        (-1.0, 2.0, 3.5, 4.0, 5.0, 6.0),
    ]


def test_parser_rejects_wrong_field_counts_nonfinite_and_bad_numbers():
    parser = SampleParser()

    samples = parser.feed(
        '(1,2,3)(1,2,3,4,5,nan)(1,2,3,4,5,nope)'
        '(7,8,9,10,11,12)'
    )

    assert samples == [(7.0, 8.0, 9.0, 10.0, 11.0, 12.0)]


def test_parser_recovers_after_oversized_incomplete_input():
    parser = SampleParser(max_buffer_bytes=64)

    assert parser.feed('(' + ('1' * 128)) == []
    assert parser.feed('(1,2,3,4,5,6)') == [
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    ]
