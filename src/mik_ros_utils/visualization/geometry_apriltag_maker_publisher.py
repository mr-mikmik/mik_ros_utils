import numpy as np
from .geometry_marker_publisher import GeometryMarkerPublisher


class GeometryApriltagMarkerPublisher(GeometryMarkerPublisher):

    def __init__(self, *args, camera_id=1, filtered=False, **kwargs):
        self.camera_id = camera_id
        self.filtered = filtered
        super().__init__(*args, **kwargs)

    def set_tag_reference_mode(self, camera_id=None):
        tag_pose_of = np.asarray(self.object_params['tag_pose_mf']) # mesh_frame
        self.frame_id = self.get_tag_frame_id(camera_id=camera_id)
        # set the position w.r.t to the tag
        self.data = tag_pose_of

    def get_tag_frame_id(self, camera_id=None):
        if camera_id is None:
            camera_id = self.camera_id
        tag_id = self.object_params['tag_id']
        tag_frame_id = f'tag_{tag_id}_{camera_id}'
        if self.filtered:
            tag_frame_id += '_filtered'
        return tag_frame_id