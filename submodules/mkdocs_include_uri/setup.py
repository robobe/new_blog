from setuptools import setup

setup(
    name="mkdocs_include_uri",
    version="0.1",
    packages=["mkdocs_include_uri"],
    entry_points={
        "mkdocs.plugins": [
            "include_uri = mkdocs_include_uri.my_plugin:MyPlugin",
        ]
    },
)