from betl_voice.tts_engine import clean_english as c


def test_spelled_letters_become_commas():
    assert c("Spell robot. R. O. B. O. T. That is correct!") == "Spell robot. R, O, B, O, T. That is correct!"
    assert c("R-O-B-O-T") == "R, O, B, O, T."
    assert c("is it a b c?") == "is it A, B, C?"


def test_normal_text_untouched():
    for s in ["I am a robot.", "Plan A is fine.", "Hello, I am Beetle.", "Room 4 B."]:
        assert c(s) == s
