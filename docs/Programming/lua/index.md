---
title: LUA
tags:
    - lua
---

Lua is a small, fast, embeddable scripting language designed to be:

- Lightweight: very small footprint
- Embeddable: easy to integrated into C/C++ apps
- Flexible: minimal syntax



## Install

```bash title="ubuntu 24.04"
sudo apt install lua5.4
```

### VSCode

[marketplace](https://marketplace.visualstudio.com/items?itemName=sumneko.lua)

---

## Core

### Tables are EVERYTHING

Lua doesn't have: 

- arrays
- struct
- classes

instead os has **tables** which as as:

- array
- dictionaries
- objects
- module


#### Table are dictionary

```c
person = {
    name = "Amir",
    age = 30,
    city = "Taipei"
}

print(person.name)       -- dot syntax (for string keys)
print(person["age"])     -- bracket syntax
```

##### Nested

```c
user = {
    name = "Amir",
    address = {
        city = "Taipei",
        zip = 100
    }
}

print(user.address.city)
```

##### Iterate

```c
for key, value in pairs(person) do
    print(key, value)
end
```

!!! tip "Missing key return nil"
    

#### Table as array

```c
arr = {10, 20, 30}
print(arr[1])  -- Lua starts at 1!
```

!!! warning "Lua starts at 1"
    


### Function first class

Function are values:

- store in variables
- pass as argument
- return from functions


```c
function add(a, b)
    return a + b
end

print(add(2, 3))
```


```c
f = add
print(f(5, 6))
```


### Data types

- nil (null)
- number
- string
- boolean
- table
- function


### Variables
In Lua, variables are:

- Global by default ❗
- Local only if you explicitly say **local**

```c
function foo()
    local x = 10
end

function bar()
    print(x)
end

foo()
bar()   -- nil
```

#### Local variable scope
- block scope
- function scope
- file/module scope


### Strings

#### Multiline strings

```c
s = [[
line 1
line 2
line 3
]]
```

#### Concatenation
Lua uses `..` 

```c
name = "Amir"
msg = "Hello " .. name

print(msg)
```

#### string methods

| Function                        | Description                     | Example                             | Result      |
| ------------------------------- | ------------------------------- | ----------------------------------- | ----------- |
| `string.len(s)`                 | Get length of string            | `string.len("abc")`                 | `3`         |
| `#s`                            | Length operator (shortcut)      | `#"abc"`                            | `3`         |
| `string.sub(s, i, j)`           | Substring from index `i` to `j` | `string.sub("hello",1,4)`           | `"hell"`    |
| `string.upper(s)`               | Convert to uppercase            | `string.upper("abc")`               | `"ABC"`     |
| `string.lower(s)`               | Convert to lowercase            | `string.lower("ABC")`               | `"abc"`     |
| `string.reverse(s)`             | Reverse string                  | `string.reverse("abc")`             | `"cba"`     |
| `string.find(s, pattern)`       | Find pattern (returns indices)  | `string.find("hello","lo")`         | `4,5`       |
| `string.match(s, pattern)`      | Extract first match             | `string.match("a1b2","%d+")`        | `"1"`       |
| `string.gmatch(s, pattern)`     | Iterate over matches            | loop over `"a1b2"`                  | `"a1","b2"` |
| `string.gsub(s, pattern, repl)` | Replace occurrences             | `string.gsub("hi all","all","Lua")` | `"hi Lua"`  |
| `string.format(fmt, ...)`       | Format string (like printf)     | `string.format("%d",10)`            | `"10"`      |
| `string.byte(s, i)`             | Get ASCII/byte value            | `string.byte("A")`                  | `65`        |
| `string.char(...)`              | Convert numbers to chars        | `string.char(65)`                   | `"A"`       |
| `string.rep(s, n)`              | Repeat string `n` times         | `string.rep("ab",3)`                | `"ababab"`  |


---

<div class="grid-container">
    <div class="grid-item">
        <a href="module">
            <p>module</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="metatable">
            <p>metatable</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="oop">
            <p>oop</p>
        </a>
    </div>
    
</div>