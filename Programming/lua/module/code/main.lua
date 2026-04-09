package.path = package.path .. ";./mylibs/?.lua"

local mymodule = require("mymodule")
mymodule.hello()