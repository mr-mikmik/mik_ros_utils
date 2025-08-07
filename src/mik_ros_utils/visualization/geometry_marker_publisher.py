import numpy as np
import os

from mik_ros_utils.ros_utils import MarkerPublisher
from mik_ros_utils.aux.load_confs import get_load_object_params_fn, load_object_params, get_get_available_object_ids_fn, get_available_object_ids
from mik_ros_utils.aux.package_utils import get_mesh_path


class GeometryMarkerPublisher(MarkerPublisher):

    def __init__(self, object_id, *args, load_object_params_fn=None, get_mesh_path_fn=None, package_name=None, obj_params_file_name='object_params.yaml', **kwargs):
        self.object_id = object_id
        self.package_name = package_name if package_name else 'mik_ros_utils'
        self.load_object_params_fn = load_object_params_fn if load_object_params_fn else get_load_object_params_fn(self.package_name, obj_params_file_name)
        self.get_mesh_path_fn = get_mesh_path_fn if get_mesh_path_fn else get_get_available_object_ids_fn(self.package_name, obj_params_file_name)
        self.object_params = self.load_object_params_fn()[object_id]
        super().__init__(*args, **kwargs)
        self._set_marker_type()

    def _set_marker_type(self):
        mesh_path = self._get_mesh_path()
        if 'box' in self.object_id:
            self.marker_type = self.Marker.CUBE
            self.scale = self.object_params['size']
        elif 'cylinder' in self.object_id:
            self.marker_type = self.Marker.CYLINDER
            # for the cylinder, x and y are the diameters in x and y and z is the height
            self.scale = [2*self.object_params['size'][0], 2*self.object_params['size'][0], self.object_params['size'][1]]
        elif os.path.exists(mesh_path):
            self.marker_type = self.Marker.MESH_RESOURCE
            self.path = mesh_path
            self.scale = 0.001*np.ones(3) # the mesh is on mm, need to convert to meters

    def _get_mesh_path(self):
        mesh_path = self.get_mesh_path_fn(f'{self.object_id}.stl')
        return mesh_path