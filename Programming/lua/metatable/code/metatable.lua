local defaults = {
    color = "red"
}

local obj = {
    size = 10
}

setmetatable(obj, {
    __index = defaults
})

print(obj.size)   -- 10 (from obj)
print(obj.color)  -- red (fallback to defaults)