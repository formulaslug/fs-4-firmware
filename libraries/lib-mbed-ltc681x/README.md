# lib-mbed-ltc681x

`lib-mbed-ltc681x` is a library for interfacing with LTC681x chips using Arm Mbed.

## Installation

`lib-mbed-ltc681x` can be added to any existing Mbed 6 project by creating a `lib-mbed-ltc681x.lib` in the source tree with the following inside:

```
https://github.com/formulaslug/lib-mbed-ltc681x#main
```

Then, tell Mbed to update all dependencies

```bash
mbed-tools deploy
```

Once it is installed, you can add `lib-mbed-ltc681x` as a library to be built in your `CMakeLists.txt` file

```cmake
# Tell CMake to build lib-mbed-ltc681x
add_subdirectory(lib-mbed-ltc681x)
```

and add `lib-mbed-ltc681x` to the link libraries for your executable

```cmake
target_link_libraries(${APP_TARGET}
    mbed-os
    lib-mbed-ltc681x
    ...
)
```

## Usage


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)

