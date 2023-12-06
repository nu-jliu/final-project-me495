from polyglotbot import polyglotbot


def test_testname():
    my_state = polyglotbot.State.WAITING

    assert my_state == polyglotbot.State.WAITING, 'Found error with State'
