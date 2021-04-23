# Open3D C++ and Python interaction

- [Open3D C++ and Python interaction](#open3d-c-and-python-interaction)
  - [Requisites](#requisites)
  - [Workflow](#workflow)
  - [Example: PlaneSegmentation](#example-planesegmentation)
  - [Code Comparision](#code-comparision)
    - [Custom Visualization](#custom-visualization)
      - [Python](#python)
      - [C++](#c)
    - [Plane Segmentation](#plane-segmentation)
      - [Python](#python-1)
      - [C++](#c-1)

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

1. Prototype your idea in Python, using Jupyter Notebooks or just plain scripts
2. "Translate" your algorithm to C++, using the same parameters you find when
   using python.
3. Build this example and inspect that the results are the same.
4. Move your C++ code snippet to the project where you want to deploy the
   algorithm.

## Example: PlaneSegmentation

This is a trivial example, but given the fact that now Open3D supports this out
of the box it's a good starting point.

Now you are ready to open the [jupyter notebook](./plane_segmentation.ipynb)
and start playing, have fun!

## Code Comparision

This section is just to show you how similar the two implementation looks.

### Custom Visualization

#### Python

```python
# Create a new Window
vis = o3d.visualization.Visualizer()
vis.create_window(
    window_name=window_name,
    width=width,
    height=height,
    left=left,
    top=top,
    visible=True,
)

# Add the provided geometries to the canvas
for geom in geoms:
    vis.add_geometry(geom)

# Change the render options
render_options = vis.get_render_option()
render_options.point_size = point_size

# Change the viewpoint of the camera
view_control = vis.get_view_control()
view_control.set_lookat(lookat) if lookat else None
view_control.set_up(up) if up else None
view_control.set_front(front) if front else None
view_control.set_zoom(zoom) if zoom else None
vis.run()
```

#### C++

```cpp

// Create a new Window
o3d::visualization::Visualizer vis;
vis.CreateVisualizerWindow(window_name, width, height, left, top, true);

// Add the provided geometries to the canvas
for (const auto &geom : geometry_ptrs) {
    vis.AddGeometry(geom);
}

// Change the render options
o3d::visualization::RenderOption &render_options = vis.GetRenderOption();
render_options.SetPointSize(point_size);

// Change the viewpoint of the camera
o3d::visualization::ViewControl &view_control = vis.GetViewControl();
if (lookat != nullptr) {
    view_control.SetLookat(*lookat);
}
if (up != nullptr) {
    view_control.SetUp(*up);
}
if (front != nullptr) {
    view_control.SetFront(*front);
}
if (zoom != nullptr) {
    view_control.SetZoom(*zoom);
}

vis.Run();
```

### Plane Segmentation

#### Python

```python
def segment_and_visualize_point_cloud(pcd,
                                      # Default params taken from Open3D implementation
                                      distance_threshold=0.01,
                                      ransac_n=3,
                                      num_iterations=100):
    _, plane_points = pcd.segment_plane(distance_threshold=distance_threshold,
                                        ransac_n=ransac_n,
                                        num_iterations=num_iterations)

    plane_cloud = pcd.select_by_index(plane_points)
    plane_cloud.paint_uniform_color([1, 0, 0])

    rest_cloud = pcd.select_by_index(plane_points, invert=True)
    rest_cloud.paint_uniform_color([0, 0, 1])

    o3d.visualization.draw_geometries([plane_cloud, rest_cloud],
                                      lookat=lookat,
                                      up=up,
                                      front=front,
                                      zoom=zoom)
    return plane_cloud, rest_cloud
```

#### C++

```cpp
auto SegmentAndVisualizePointCloud(
    std::shared_ptr<o3d::geometry::PointCloud> pcd,
    // Default params taken from Open3D implementation
    const double distance_threshold = 0.01,
    const int ransac_n = 3,
    const int num_iterations = 100) {
    auto [plane_model, plane_points] =
        pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations);

    auto plane_cloud = pcd->SelectByIndex(plane_points);
    plane_cloud->PaintUniformColor({1, 0, 0});

    auto rest_cloud = pcd->SelectByIndex(plane_points, true);
    rest_cloud->PaintUniformColor({0, 0, 1});

    CustomDrawGeometries(
        {plane_cloud, rest_cloud}, &lookat, &up, &front, &zoom);

    return std::make_tuple(plane_cloud, rest_cloud);
}
```
