person = {
    name = "Amir",
    age = 30,
    city = "Taipei"
}

print(person.name)       -- dot syntax (for string keys)
print(person["age"])     -- bracket syntax


for key, value in pairs(person) do
    print(key, value)
end