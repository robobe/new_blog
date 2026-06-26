from pathlib import Path

from plugin_loader import load_plugin_from_file


PLUGIN_PATH = Path(__file__).resolve().parent / "my_plugin.py"

plugin = load_plugin_from_file(
    str(PLUGIN_PATH),
    {"factor": 10}
)


result = plugin.process(5)
print(result)
