
# CLARA akà Amelie
### Cone-based Localization for Autonomous Racing Applications

Our custom SLAM for the FS competitions.

### Build
#### Production build

```bash
> cd clara
> mkdir build && cd build
> cmake .. 
> make
> ./tests/clara_tests
```

#### Test build

```bash
> cd clara
> mkdir build-debug && cd build-debug
> cmake .. -DDEBUG_MODE=1
> make
> ./tests/clara_tests
```

### Documentation

Created with doxygen (with Markdown support)
* [PDF](documentation/latex/refman.pdf)
* [HTML](documentation/html/index.html)

Update documentation:

```bash
> cd clara
> doxygen doxygen.config
> cd documentation/latex
> make
```

### Dependencies

* [fast-cpp-csv-parser](https://github.com/ben-strasser/fast-cpp-csv-parser)
    - header only (used for tests)
* [catch](https://github.com/catchorg/Catch2)
* [blaze](https://bitbucket.org/blaze-lib/blaze/overview)  
  * **Linux**: ToDo
  * **MacOS**: ToDo
