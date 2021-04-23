# Open3D C++ and Python interaction

In this humble and simple example I want to show how I use Open3D for developing
stuff.

Open3D, besides being an awesome 3D library, has great support for working both
with Python and C++. The power of this "dual" language scenario is that you can
prototype fast some algorithms in Python, without even compiling a source file
and once you have the idea ready, you can transparently use the C++ API from
Open3D to compile your code and deploy it somewhere(robots, etc).

## Requisites

You **need** to install Open3D from source, since you want to use the C++ api,
the best way you can do this is by following [these
instructions](http://www.open3d.org/docs/latest/compilation.html#compilation).
Make sure you also install the Python package.

In summary:

```sh
git clone --recurse-submodules https://github.com/intel-isl/Open3D
cd Open3D/

# Install dependencies
util/install_deps_ubuntu.sh

# Build library
mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Install the Open3D C++ Library and API on your system
sudo make install

# Install the Python library
make install-pip-package
```

## Workflow

Now that you have ready your tools let's get hands on. This repository is meant
to be an example and a starting point as well, but the roadmap looks like:

1.  Prototype your idea in Python, using Jupyter Notebooks or just plain scripts
2.  "Translate" your algorithm to C++, using the same parameters you find when
    using python.
3.  Build this example and inspect that the results are the same.
4.  Move your C++ code snippet to the project where you want to deploy the
    algorithm.

## Example: PlaneSegmentation

This is a trivial example, but given the fact that now Open3D supports this out
of the box it's a good starting point.

Now you are ready to open the [jupyter notebook](./plane_segmentation.ipynb)
and start playing, have fun!
