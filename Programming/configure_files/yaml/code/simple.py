from typing import Any
import yaml
import pathlib
import pprint

def load_yaml(path: str) -> Any:
    """Load a single-document YAML file using safe_load."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    file = pathlib.Path(__file__).parent.joinpath("simple.yaml").as_posix()
    data = load_yaml(file)
    pprint.pprint(data)

