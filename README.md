
# CLARA akà Amelie
### Cone-based Localization for Autonomous Racing Applications

Our custom SLAM for the FS competitions.

### SIL

* Carmaker to `clara` and back
    - `object_t` has `yaw`, `car_x` and `car_y` additionly. Switch to the `sil_clara_only` branch in **this** project **AND** in the `external/object_methods` project. Do the same in the `carmaker_sil_hil` project for the updated "server" side

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

### Visualization

Done with python and anaconda. [Install anaconda](https://docs.anaconda.com/anaconda/install/) and follow the further steps.

```bash
> conda create --name clara python=3.6
> source activate clara
> conda install numpy matplotlib ipython
> conda install -c anaconda ipywidgets
> conda install --channel=conda-forge nb_conda_kernels
> jupyter nbextension enable --py widgetsnbextension
> pip install mpld3 
```

After creating an appropriate `.csv` or `.py` file, run:

```bash
> ipython notebook --NotebookApp.iopub_data_rate_limit=10000000000
```

in the `clara/build-debug/visualization` directory and run the `plot_loggings.ipynb` file.

### Dependencies

* [fast-cpp-csv-parser](https://github.com/ben-strasser/fast-cpp-csv-parser)
    - header only (used for tests)
* [catch](https://github.com/catchorg/Catch2)
* [blaze](https://bitbucket.org/blaze-lib/blaze/overview)  
  * **Linux**: ToDo
  * **MacOS**: ToDo
