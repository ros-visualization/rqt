
def get_package_path(name):
    try:
        import rospkg
        r = rospkg.RosPack()
        return r.get_path(name)
    except ImportError:
        import roslib
        return roslib.packages.get_pkg_dir(name)
