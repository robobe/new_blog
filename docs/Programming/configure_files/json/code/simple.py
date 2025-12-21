import json
import pprint

try:
    with open("simple.json") as f:
        data = json.load(f)
        pprint.pprint(data)
except json.JSONDecodeError as e:
    print("Invalid JSON:", e)
