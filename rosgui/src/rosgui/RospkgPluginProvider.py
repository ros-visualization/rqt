import rospkg

from RosPluginProvider import RosPluginProvider

class RospkgPluginProvider(RosPluginProvider):

    def __init__(self, export_tag, base_class_type):
        RosPluginProvider.__init__(self, export_tag, base_class_type)
        self.setObjectName('RospkgPluginProvider')

    def _find_rosgui_plugins(self):
        plugins = []
        r = rospkg.RosPack()
        for package_name in r.list():
            manifest = r.get_manifest(package_name)
            exports = manifest.get_export(self.export_tag_, 'plugin')
            for export in exports:
                plugins.append([package_name, str(export)])
        return plugins
