from setuptools import setup

setup(
    name="seatrac",
    version="0.1.0",
    packages=["seatrac"],
    description="SeaTrac Protocol",
    author="Ryan Govostes",
    author_email="rgovostes@whoi.edu",
    python_requires=">=3.7",
    entry_points={
        "console_scripts": [
            "seatrac=seatrac.__main__:main",
        ],
    },
)
