# CLARA
### Cone-based Localization for Autonomous Racing Applications

The version 1.0 of this SLAM algorithm was designed by [Alexander Isenko](https://github.com/cirquit) for the Formula Student competition 2018 for the UAS Munich team - [municHMotorsport](https://munichmotorsport.de).

The corresponding master thesis of the LMU Munich can be found [here](https://github.com/cirquit/master-thesis).

If you see this on github, it's only a mirror of our internal municHMotorsport gitlab repository.

---

#### General Information

We designed this algorithm with performance, future proofness and minimal requirements in mind. Everything is header only besides our own [Extended Kalman Filter](https://github.com/cirquit/ekf) implementation which has to be installed globally.

It's a c++14 code base with doxygen and inline comments for almost every class, function or member.

Also it's **`virtual`**, **`new`**, **`delete`** and exception **free**.
We use **only** static allocation for every component to get the `O(1)` runtime complexity.

We purposefully implemented everything besides the matrix multiplication in the EKF by ourselves to get a deep dive in the algorithms which allows a full performance analysis and possible future improvements without waiting for maintainer outside of the team.

---

#### Interface

TODO

---

#### Build

[Install our Kalman Filter library (v1.0)](https://github.com/cirquit/ekf).

```bash
> git clone https://github.com/cirquit/clara
> mkdir build && cd build
> cmake .. -DENABLE_OPTIMIZATIONS_CLARA=ON
> make -j4
```

To enable tests:

```bash
> cmake .. -DENABLE_TESTS_CLARA=ON
```

You can enable critical only debugging (`=1`), all (`=2`) or none (`=0`) with:

```bash
> cmake .. -DENABLE_DEBUGGING_LVL_CLARA=2
```
---

#### Installation

You can easily reference the headers by hand and link to the `clara` library in your `CMakeLists.txt`. If you want a system-level installation just type `make install` in your `build` directory.

If you add the following to your own `CMakeLists.txt`:

```cmake
find_package(clara version 1.0 REQUIRED)
target_link_libraries(${your-awesome-executable} ${your-awesome-library} clara )
```

you can reference the library by

```c++
#include <clara-1.0/clara.h>
```

---

#### Source Code documentation

To create documentation install doxygen an run in the source directory:

```bash
> doxygen doxygen.config
> cd documentation/latex
> make
```

Now you can open the PDF at `documentation/latex/refman.pdf` or the static HTML at `documentation/html/index.html` in your favourite browser.
<!-- 
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

in the `clara/build-debug/visualization` directory and run the `plot_loggings.ipynb` file. -->

---

#### Dependencies

* [ekf](https://github.com/cirquit/ekf)
    - our `O(1)` extended kalman filter, install the same way as this library (see its [README.md](https://github.com/cirquit/ekf/README.md))
* [fast-cpp-csv-parser](https://github.com/ben-strasser/fast-cpp-csv-parser)
    - header only
    - used for tests
    - already included in the source, no need to download anything
* [catch](https://github.com/catchorg/Catch2)
    - header only
    - used for tests as this is a testing framework
    - already included in the source, no need to download anything
* [blaze](https://bitbucket.org/blaze-lib/blaze/overview)
```bash
> wget https://bitbucket.org/blaze-lib/blaze/downloads/blaze-3.3.tar.gz
> tar -xvf blaze-3.3.tar.gz
> sudo apt-get install libopenblas-dev
> sudo apt-get install libboost-all-dev
> cmake -DCMAKE_INSTALL_PREFIX=/usr/local/
> sudo make install
```
