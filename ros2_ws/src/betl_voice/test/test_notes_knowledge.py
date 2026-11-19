import os

from betl_voice.chat import canned, first_sentences
from betl_voice.knowledge import Knowledge, speakable, split_passages
from betl_voice.notes import Notebook

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples", "knowledge")


def test_notebook_roundtrip(tmp_path):
    nb = Notebook(str(tmp_path / "notes.md"))
    assert nb.recent() == []
    nb.add("buy batteries")
    nb.add("  hotel room is   four twelve ")
    nb.add("我们明天八点出发")
    text = (tmp_path / "notes.md").read_text(encoding="utf-8")
    assert text.startswith("# BETL notes")
    assert [b for _, b in nb.recent(2)] == ["hotel room is four twelve", "我们明天八点出发"]
    assert nb.forget_last() == "我们明天八点出发"
    assert len(nb.all()) == 2


def test_search_finds_notes_and_files(tmp_path):
    nb = Notebook(str(tmp_path / "notes.md"))
    nb.add("hotel room is four twelve")
    nb.add("我们明天八点出发")
    kn = Knowledge(str(tmp_path / "notes.md"), EXAMPLES)
    top = lambda q: kn.search(q, k=1)[0]
    assert "four twelve" in top("what did I note about my hotel room")[2]
    assert top("what did I note about my hotel room")[3] == "notes"
    assert "Lotus Garden" in top("the beijing hotel")[2]
    assert "7:30" in top("when does the car pick us up for the great wall")[2]
    assert "15.2" in top("battery storage voltage")[2]
    assert "八点" in top("明天几点出发")[2]
    assert kn.search("capital of france") == []


def test_knowledge_picks_up_new_files(tmp_path):
    kdir = tmp_path / "k"
    kdir.mkdir()
    kn = Knowledge(str(tmp_path / "notes.md"), str(kdir))
    assert kn.search("wifi password") == []
    (kdir / "house.md").write_text("# House\n- The wifi password is purple-giraffe\n")
    assert "purple-giraffe" in kn.search("wifi password")[0][2]


def test_passages_and_speech():
    ps = split_passages("# Hotels\n- Shanghai: Riverside\n\nSome paragraph\ncontinues here.\n", "x")
    assert ps[0] == ("Hotels", "- Shanghai: Riverside", "x")
    assert ps[1][1] == "Some paragraph continues here."
    assert speakable("Hotels", "- **Shanghai**: [Riverside](http://x)") == "Hotels: Shanghai: Riverside"


def test_chat_helpers():
    assert first_sentences("One. Two! Three?") == "One. Two!"
    assert first_sentences("你好。我是机器人。很高兴。") == "你好。我是机器人。"
    assert canned("hello there")[0].startswith("Hello")
