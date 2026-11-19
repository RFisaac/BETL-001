"""Run with:  python3 -m pytest test/   (no ROS or models needed)"""

from betl_voice.router import parse
from betl_voice.translator import _detok, is_chinese, tidy_english


def k(text):
    i = parse(text)
    return i.kind, i.text


def test_translate_english():
    assert k("BEETLE PLEASE TRANSLATE WHERE IS THE TRAIN STATION") == ("translate", "where is the train station")
    assert k("HEY BEETLE TRANSLATE HOW MUCH IS THIS") == ("translate", "how much is this")
    assert k("beetle can you translate into chinese good morning") == ("translate", "good morning")
    assert k("BEETLE TRANSLATING I AM HUNGRY") == ("translate", "i am hungry")
    assert k("beetle how do you say thank you in chinese") == ("translate", "thank you")


def test_translate_chinese_and_mixed():
    assert k("BEETLE 请翻译火车站在哪里") == ("translate", "火车站在哪里")
    assert k("翻译一下 你好") == ("translate", "你好")
    assert k("请范逸 洗手间在哪里") == ("translate", "洗手间在哪里")
    assert k("BEETLE PLEASE TRANSLATE 这个多少钱") == ("translate", "这个多少钱")


def test_leading_junk_tolerated():
    assert k("哦 PLEASE TRANSLATE WHERE IS THE TRAIN STATION") == ("translate", "where is the train station")
    assert k("IT OLD TRANSLATE HOW MUCH DOES THIS COST") == ("translate", "how much does this cost")
    assert k("UM SAVE A NOTE CALL MOM") == ("note", "call mom")
    # but a real sentence that merely contains the word later is chat, not a command
    assert k("BEETLE I THINK WE SHOULD NOT TRANSLATE THIS")[0] == "chat"


def test_translate_without_phrase_asks():
    assert k("BEETLE PLEASE TRANSLATE") == ("translate", "")


def test_stop_note_chat_empty():
    assert k("BEETLE STOP")[0] == "stop"
    assert k("停下")[0] == "stop"
    assert k("BEETLE SAVE A NOTE BUY MORE SOLDER") == ("note", "buy more solder")
    assert k("BEETLE REMEMBER THAT THE HOTEL IS ROOM FOUR TWELVE") == ("note", "the hotel is room four twelve")
    assert k("BEETLE WHAT TIME IS IT")[0] == "chat"
    assert k("BEETLE")[0] == "empty"
    assert k("")[0] == "empty"


def test_text_helpers():
    assert tidy_english("WHERE IS THE TRAIN STATION") == "Where is the train station?"
    assert tidy_english("I WANT NOODLES") == "I want noodles."
    assert is_chinese("hello 你好") and not is_chinese("hello")
    assert _detok(["▁", "火车", "站", "在哪", "?"], chinese=True) == "火车站在哪？"
    assert _detok(["▁Where's", " the▁train▁station", "?"], chinese=False) == "Where's the train station?"


def test_voice_switching():
    for s in ["BEETLE SWITCH VOICES", "BEETLE CHANGE YOUR VOICE", "HEY BEETLE USE THE OTHER VOICE",
              "BEETLE SWAP VOICES PLEASE", "BEETLE 换个声音", "切换声音"]:
        assert k(s) == ("voice", "toggle"), s
    assert k("BEETLE USE THE ROBOT VOICE") == ("voice", "alt")
    assert k("BEETLE SWITCH TO THE ROBOT VOICE") == ("voice", "alt")
    assert k("BEETLE 用机器人的声音") == ("voice", "alt")
    assert k("BEETLE GO BACK TO THE CLEAR VOICE") == ("voice", "main")
    assert k("BEETLE USE THE NORMAL VOICE") == ("voice", "main")
    # a translation that happens to mention voices is still a translation
    assert k("BEETLE TRANSLATE CHANGE YOUR VOICE") == ("translate", "change your voice")


def test_stop_and_resume():
    assert k("STOP")[0] == "stop"
    for s in ["BEETLE RESUME", "BEETLE CONTINUE", "BEETLE ALL CLEAR", "BEETLE YOU CAN MOVE AGAIN", "BEETLE 继续"]:
        assert k(s)[0] == "resume", s


def test_notes():
    assert k("BEETLE SAVE A NOTE BUY BATTERIES") == ("note", "buy batteries")
    assert k("BEETLE TAKE A NOTE") == ("note", "")
    assert k("BEETLE PLEASE SAVE A NOTE TO BUY BATTERIES") == ("note", "buy batteries")
    assert k("BEETLE REMEMBER TO CALL MOM") == ("note", "call mom")
    assert k("BEETLE 记一下 明天八点出发") == ("note", "明天八点出发")
    for s in ["BEETLE READ MY NOTES", "BEETLE READ ME MY LAST NOTE", "BEETLE WHAT ARE MY NOTES",
              "BEETLE WHAT DID I JUST NOTE", "BEETLE 读一下我的笔记"]:
        assert k(s)[0] == "notes_read", s
    assert k("BEETLE DELETE THAT NOTE")[0] == "notes_delete"


def test_recall():
    assert k("BEETLE SEARCH MY NOTES FOR THE HOTEL") == ("recall", "the hotel")
    assert k("BEETLE WHAT DID I NOTE ABOUT BATTERIES") == ("recall", "batteries")
    assert k("BEETLE DO I HAVE ANY NOTES ABOUT THE TRAIN") == ("recall", "the train")
    assert k("BEETLE LOOK UP OUR FLIGHT NUMBER") == ("recall", "our flight number")
    assert k("BEETLE WHAT DO MY NOTES SAY ABOUT SHANGHAI") == ("recall", "shanghai")
    assert k("BEETLE 查一下酒店地址") == ("recall", "酒店地址")
    # ordinary questions stay chat (brain still checks notes for them)
    assert k("BEETLE WHAT TIME IS OUR FLIGHT")[0] == "chat"
