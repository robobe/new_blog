local Counter = {}
Counter.__index = Counter

function Counter:new()
    local obj = {value = 0}
    return setmetatable(obj, Counter)
end

function Counter:inc()
    self.value = self.value + 1
end

function Counter:get()
    return self.value
end

local c = Counter:new()
c:inc()
c:inc()

print(c:get())   -- 2