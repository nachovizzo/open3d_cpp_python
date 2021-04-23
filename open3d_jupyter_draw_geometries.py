# Based on
# https://github.com/intel-isl/Open3D/blob/master/examples/python/open3d_tutorial.py

import IPython.display
import PIL.Image
import numpy as np
import open3d as o3d


def jupyter_draw_geometries(
    geoms,
    # Window Options
    window_name="Open3D",
    width=1920,
    height=1080,
    left=50,
    top=50,
    # Render Options
    point_size=1,
    point_show_color_normal=False,
    mesh_show_wireframe=False,
    mesh_show_back_face=False,
    background_color=[1.0, 1.0, 1.0],
    show_coordinate_frame=False,
    # Camera Viewpoint Parameters
    front=None,
    lookat=None,
    up=None,
    zoom=None,
    # Filename
    filename=None,
):
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

    # Change the render options
    render_options = vis.get_render_option()
    render_options.point_size = point_size
    if point_show_color_normal:
        render_options.point_color_option = o3d.visualization.PointColorOption.Normal
    render_options.mesh_show_wireframe = mesh_show_wireframe
    render_options.mesh_show_back_face = mesh_show_back_face
    render_options.background_color = np.asarray(background_color)
    render_options.show_coordinate_frame = show_coordinate_frame

    # Add the provided geometries to the canvas
    for geom in geoms:
        vis.add_geometry(geom)

    # Change the viewpoint of the camera
    view_control = vis.get_view_control()
    view_control.set_lookat(lookat) if lookat else None
    view_control.set_up(up) if up else None
    view_control.set_front(front) if front else None
    view_control.set_zoom(zoom) if zoom else None
    vis.run()

    # Capture the image and display it in the jupyter notebook
    im = vis.capture_screen_float_buffer()
    vis.destroy_window()
    im = PIL.Image.fromarray((255 * np.asarray(im)).astype(np.uint8), "RGB")
    IPython.display.display(im)
    if filename:
        im.save(filename)
        print("Render saved to", filename)


o3d.visualization.draw_geometries = jupyter_draw_geometries
