from glob import glob

from setuptools import setup

package_name = "betl_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Isaac",
    maintainer_email="isaac@rustedfriend.com",
    description="BETL robot voice system",
    license="MIT",
    entry_points={
        "console_scripts": [
            "respeaker_node = betl_voice.respeaker_node:main",
            "listener_node = betl_voice.listener_node:main",
            "brain_node = betl_voice.brain_node:main",
            "tts_node = betl_voice.tts_node:main",
            "led_node = betl_voice.led_node:main",
            "lpc_voice = betl_voice.lpc_voice:main",
        ],
    },
)
