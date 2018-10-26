# CLARA Notebooks

A summary of multiple notebooks which are used for debugging and plotting.

##### Get the data from our 1718 season

It can be downloaded [here](https://mega.nz/#!bkZVVQrR!pM1QrY_P5lMU_FLcLIULprbAtfl52hn_Rje958Q3_QM). The default directory until now is `playgrounds/logging-data-1718`.

##### Directory

* `scripts`
    - all python scripts which can be used for multiple plotting and debugging purposes
    - `plot-cluster-information-csv.py`
        +  create a valid `cppdebuginfo.py` file with multiple numpy arrays of our clustered data
        + read the ground truth from a csv file and plot them vs each other
    -  `plot-t3-tp-log.py`
        +  plot any of our t3-tp loggings
    -  `plot-cones-path-t3-tp-log.py`
        +  plot the blue/yellow cones, the planned path and the driven trajectory based on 4 csv files
    -  `plot-xy-positions.py`
        +  used to plot the different positions based on the changed yawrate
    -  `plot-skidpad-path.py`
        +  plots the middle line and the real driven line based on the skipa repository debug `skipa_debug.log` 
    -  `plot-yaw-comparisson.py`
        +  compares the different yaw rates by calculating the mean and std, and plotting them against each other
    -  `plot-velocity-comparisson.py`
        +  compares the different velocities (Correvit and the calculated from visual odometry)
    -  `plot-any-logging.py`
        +  a slightly more generic plotting approach to plot any csv values next to each other
*  `notebooks`
    -  ipython notebooks which are used for iterative development of algorithms and intermediate results
    -  `gmm-code-examples.ipynb`
        + we show how a gaussian mixture model works with examples
    -  `fuse_darknet_etas_logging.ipynb`
        +  example for our old file format. This is probably outdated but can be easily adapted for your formats. The key value is the filename of the image
    -  `plot-yaw-pos-comparisson.ipynb`
        + a whole notebook for yaw position comparissons. Obviously dependent on the debugging information created with `tests/clara_csv_replay`

##### How to run notebooks

Install [anaconda](https://conda.io/miniconda.html).

```bash
> conda create --name clara python=3.6
> source activate clara
> conda install numpy matplotlib ipython
> conda install -c anaconda ipywidgets
> conda install --channel=conda-forge nb_conda_kernels
> jupyter nbextension enable --py widgetsnbextension
> pip install mpld3 
```

Start the notebooks like this, to allow big logs if you need them:

```bash
> source activate clara
> ipython notebook --NotebookApp.iopub_data_rate_limit=10000000000
```


