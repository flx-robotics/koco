import rospkg

PKG_PATH_PREFIX = 'package://'


def resolve_ros_path(path):
    # type: (str) -> str
    """
    Takes a path that starts with 'package://' syntax and returns the resolved full absolute path.
    If the path is already absolute, returns it as is. If the path is relative,
    it is assumed that the first entry is a package name.

    Examples below return the same result:
    resolve_ros_path('package://my_pkg/config/file.yaml')
    resolve_ros_path('my_pkg/config/file.yaml')
    resolve_ros_path('/ws/src/my_pkg/config/file.yaml')
    """
    if path.startswith('/'):
        return path  # Path is already absolute

    if path.startswith(PKG_PATH_PREFIX):
        path = path[len(PKG_PATH_PREFIX):]

    splitted_path = path.split('/', 1)
    package_name = splitted_path[0]
    rel_path = splitted_path[1:]

    abs_path = rospkg.RosPack().get_path(package_name)
    return '/'.join([abs_path] + rel_path)


def get_rospkg_name_from_path(path):
    # type: (str) -> str
    """It parses a ROS package name out of the ros styled path.

    Examples below return the same package name 'my_pkg'.
    get_rospkg_name_from_path('package://my_pkg')
    get_rospkg_name_from_path('package://my_pkg/config')
    get_rospkg_name_from_path('my_pkg/config')
    """
    if path.startswith('/'):
        raise Exception("Package name cannot be resolved from the absolute path.")

    if path.startswith(PKG_PATH_PREFIX):
        path = path[len(PKG_PATH_PREFIX):]

    package_name = path.split('/', 1)[0]

    return package_name
