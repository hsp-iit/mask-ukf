#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

import argparse
import cv2
import numpy
import os
import sys
import time
import trimesh
from pyquaternion import Quaternion

os.environ["PYOPENGL_PLATFORM"] = "egl"
import pyrender


class Renderer():
    def __init__(self, options):
        """Constructor."""

        # RGB frames path
        self.rgb_path = os.path.join('datasets', 'ycb-video', options.video_id, 'rgb')

        # Estimates path
        self.estimates_path = os.path.join('results', options.algorithm, 'nrt', options.mask_set, 'validation', options.object, options.video_id, 'object-tracking_estimate.txt')

        # Mesh path
        object_mesh_path = os.path.join('models', 'YCB_models', 'models', options.object, 'textured')
        if options.mesh_type == 'low-quality':
            object_mesh_path += '_simple'
        object_mesh_path += '.obj'

        # Mesh
        trimesh_mesh = trimesh.load(object_mesh_path)
        mesh = pyrender.Mesh.from_trimesh(trimesh_mesh)

        # Scene
        self.scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0])

        # Camera
        fx = 1066.8
        fy = 1067.5
        cx = 312.99
        cy = 241.31
        width = 640
        height = 480
        camera_transform = Quaternion(axis = [1.0, 0.0, 0.0], angle = numpy.pi).transformation_matrix

        self.camera = pyrender.IntrinsicsCamera(fx = fx, fy = fy, cx = cx, cy = cy)
        self.scene.add(self.camera, pose = camera_transform)

        # Light
        self.light = pyrender.PointLight(intensity = 20.0)
        self.scene.add(self.light)

        # Object node
        self.mesh_node = pyrender.Node(mesh = mesh, matrix = numpy.eye(4))
        self.scene.add_node(self.mesh_node)

        # Renderer
        self.renderer = pyrender.OffscreenRenderer(width, height)


    def load_estimates(self, file_path):
        """Load estimates from file."""

        data = []

        with open(file_path, newline='') as csv_data:
            for row in csv_data:
                data.append([float(num_string.rstrip()) for num_string in row.split(sep = " ") if num_string != ''])

        return numpy.array(data)


    def render(self, frame, object_pose):
        """Render object pose on top of input frame 'frame'."""

        # Update object pose
        self.scene.set_pose(self.mesh_node, object_pose)

        # Render the scene
        render_rgb, render_depth = self.renderer.render(self.scene)

        return render_rgb


    def show(self):
        """Show the entire rendered sequence."""

        # Load estimates
        estimates = self.load_estimates(self.estimates_path)

        for i in range(estimates.shape[0]):

            # Get current RGB frame
            rgb_input = cv2.imread(os.path.join(self.rgb_path, str(i + 1).zfill(6) + '.png'))

            # Set current pose
            pose = Quaternion(axis = estimates[i, 3:6], angle = estimates[i, 6]).transformation_matrix
            pose[0:3, 3] = estimates[i, 0:3]

            rgb = self.render([], pose)
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

            # Superimpose on rgb input
            where = rgb > 0
            rgb_input[where] = rgb[where]

            cv2.imshow('Viewer', rgb_input)

            cv2.waitKey(33)


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--algorithm', type = str, help = 'name of the algorithm, i.e. "mask-ukf", "icp" or "dense_fusion"', required = True)
    parser.add_argument('--mask_set', type = str, help = 'name of the mask set, i.e. "mrcnn" or "posecnn"', required = True)
    parser.add_argument('--object', type = str, help = 'name of the object', required = True)
    parser.add_argument('--video_id', type = str, help = 'id of the video from YCB-Video, e.g. 0048', required = True)
    parser.add_argument('--mesh_type', type = str, help = 'type of mesh to be used, i.e. "low-quality" or "high-quality"', default = 'low-quality')

    options = parser.parse_args()

    renderer = Renderer(options)
    renderer.show()


if __name__ == '__main__':
    main()
