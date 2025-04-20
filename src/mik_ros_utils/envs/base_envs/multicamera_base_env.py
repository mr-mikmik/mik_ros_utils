from .ros_base_env import ROSBaseEnv


class MultiCameraBaseEnv(ROSBaseEnv):
    """
    This environment handles an arbitrary number of cameras.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
