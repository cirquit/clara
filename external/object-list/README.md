# Object List

Because we wanted to have a basic representation of our landmarks which we get from the object detection, we introduced this shared "library".

Usually this will be a repository with different branches, but specialied for the **clara** repository we will use this as a simple directory.

If you don't need this functionality (most probably you won't) you can simply delete `library/refined_cones.h`, this external directory, the include path in `library/CMakeLists.txt` and the function `get_clustered_observations` in `library/clara.h`.

This should be a compile time flag.