set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

find_program(MUSL_CC aarch64-linux-musl-gcc REQUIRED)
find_program(MUSL_CXX aarch64-linux-musl-g++ REQUIRED)
set(CMAKE_C_COMPILER ${MUSL_CC})
set(CMAKE_CXX_COMPILER ${MUSL_CXX}
)