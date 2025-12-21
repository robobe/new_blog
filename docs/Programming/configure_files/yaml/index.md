---
title: YAML
tags:
    - python
    - yaml
---

YAML stands for “YAML Ain’t Markup Language.”
It’s a human-readable data format used to represent configuration and structured data.

YAML struct define by **indentation**
Use spaces only **not** TAB'S
Common style : 2 space per level


!!! Tip linter

    ```
    pip install --break-system-packages yamllint
    ```

!!! Tip python package

    ```
    pip install pyyaml
    ```

---

## simple YAML

### Dictionary (map / key-value)

```yaml title="simple.py"
---
key: value
```

### load and print
Simple python load yaml and print it using `pprint`

```python title="simple.py"
from typing import Any
import yaml
import pathlib

def load_yaml(path: str) -> Any:
    """Load a single-document YAML file using safe_load."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    file = pathlib.Path(__file__).parent.joinpath("simple.yaml").as_posix()
    data = load_yaml(file)
    print(data)
```

```bash title="usage"
python simple.py
#
<class 'dict'>
{'key': 'value'}
```

---

### Comments

Anything on a line after `#` is ignored by YAML parsers.

---

### List
Each `-` begins a new element in the sequence.

```yaml
---
list:
  - apple
  - banana
  - orange

alternative: [grape, mango, pineapple]

```

---

### Mapping
Unordered key-value pairs

```yaml
---
worker:
  id: 22222
  name: me
  age: 32

alternative: {id:2222, age:30, name: dddd}
```

---

### string
You don’t need quotes unless contain special character or start with `{,[,#]`

#### multiline 
```yaml title="just folded"
msg: >
  Hello
  world
```

```yaml title="multi line string"
msg2: |
  Hello
  world
```

```
{
  'msg': 'Hello world\n', 
  'msg2': 'Hello\nworld'
}
```
---

### null

```yaml
value1: null
value2: ~
value3:

```

```python
# simple.py result
{'value1': None, 'value2': None, 'value3': None}
```

---

### types
- string
- boolean
- integer
- float
- list
- dict
- null

```yaml
int_val: 10
float_val: 3.14
bool_true: true
bool_false: false
string1: hello
string2: "hello"
list: [1, 2, 3, 4, 5]
dict:
  key1: value1
  key2: value2
null_val: null

```

---

## linter
[github source](https://github.com/adrienverge/yamllint)
```
pip install --break-system-packages yamllint
```

### Configuration

```yaml
extends: default

rules:
  # 80 chars should be enough, but don't fail if a line is longer
  line-length:
    max: 80
    level: warning
```

```bash
yamllint -c /path/myconfig file-to-lint.yaml
```

---

## Schemas
YAML Language support uses JSON Schemas to understand the shape of a YAML file, including its value sets, defaults and descriptions. The schema support is shipped with JSON Schema Draft 7.

The association of a YAML file to a schema can be done either in the YAML file itself using a modeline or in the User or Workspace settings under the property `yaml.schemas.` in vscode settings

### Associating a schema in the YAML file
```yaml
# yaml-language-server: $schema=<urlToTheSchema>
```

---

## VSCode

### Extension
[![alt text](images/vscode_yaml.png)](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

### Settings

#### Schema

```json
yaml.schemas: {
    "/home/user/custom_schema.json": "someFilePattern.yaml",
}

```