import imp, os, traceback

from xml.etree import ElementTree

from PluginDescriptor import PluginDescriptor
from PluginProvider import PluginProvider

class RosPluginProvider(PluginProvider):

    def __init__(self, export_tag, base_class_type):
        super(RosPluginProvider, self).__init__()
        self.setObjectName('RosPluginProvider')

        self.export_tag_ = export_tag
        self.base_class_type_ = base_class_type
        self.plugin_descriptors_ = {}

    def discover(self):
        # search for plugins
        plugin_descriptors = []
        plugin_file_list = self._find_rosgui_plugins()
        for plugin_name, xml_file_name in plugin_file_list:
            plugin_descriptors += self.__parse_plugin_xml(plugin_name, xml_file_name)
        # add list of discovered plugins to dictionary of known descriptors index by the plugin id
        for plugin_descriptor in plugin_descriptors:
            self.plugin_descriptors_[plugin_descriptor.plugin_id()] = plugin_descriptor
        return plugin_descriptors

    def load(self, plugin_id, plugin_context):
        # get class reference from plugin descriptor
        attributes = self.plugin_descriptors_[plugin_id].attributes()
        module_path = os.path.join(attributes['plugin_path'], attributes['library_path'], attributes['library_name'] + '.py')
        try:
            module = imp.load_source(attributes['library_name'], module_path)
        except NotImplementedError, e:
            print 'RosPluginProvider.load(%s):\n%s' % (plugin_id, e)
            return None
        except:
            print 'RosPluginProvider.load(%s) exception raised in imp.load_source(%s, %s):\n%s' % (plugin_id, attributes['library_name'], module_path, traceback.format_exc())
            raise
        class_ref = getattr(module, attributes['class_type'])

        # create plugin provider instance without context
        if class_ref.__init__.func_code.co_argcount == 1 and plugin_context is None:
            return class_ref()
        # create plugin instance
        return class_ref(self, plugin_context)

    def unload(self, plugin_instance):
        pass

    def _find_rosgui_plugins(self):
        raise NotImplementedError('override method in subclass')

    def __parse_plugin_xml(self, plugin_name, xml_file_name):
        plugin_descriptors = []
        plugin_path = os.path.dirname(os.path.abspath(xml_file_name))

        root = ElementTree.parse(xml_file_name)
        for library_el in root.getiterator('library'):
            library_path, library_name = os.path.split(library_el.attrib['path'])

            for class_el in library_el.getiterator('class'):
                # collect common attributes
                attributes = {
                    'plugin_name': plugin_name,
                    'plugin_path': plugin_path,
                    'library_path': library_path,
                    'library_name': library_name,
                }

                # add class attributes
                for key, value in class_el.items():
                    attributes['class_' + key] = value

                # skip classes with non-matching base_class_type
                class_base_class_type = None
                if attributes.has_key('class_base_class_type'):
                    class_base_class_type = attributes['class_base_class_type']
                if class_base_class_type != self.base_class_type_:
                    continue

                # generate unique identifier
                plugin_id = os.path.join(plugin_path, plugin_name)
                if attributes.has_key('class_name'):
                    plugin_id = os.path.join(plugin_id, attributes['class_name'])
                attributes['plugin_id'] = plugin_id

                # check if plugin is available
                plugin_file = os.path.join(attributes['plugin_path'], attributes['library_path'], attributes['class_name']) + '.py'
                attributes['not_available'] = plugin_name if not os.path.exists(plugin_file) else ''

                plugin_descriptor = PluginDescriptor(plugin_id, attributes)

                # set action attributes (plugin providers might have none)
                action_attributes, groups = self.__parse_rosguiplugin(class_el)
                if len(action_attributes) > 0:
                    plugin_descriptor.set_action_attributes(
                        action_attributes['label'],
                        action_attributes.get('statustip', None),
                        action_attributes.get('icon', None),
                        action_attributes.get('icontype', None),
                    )
                # add group attributes
                for group in groups:
                    plugin_descriptor.add_group_attributes(
                        group['label'],
                        group.get('statustip', None),
                        group.get('icon', None),
                        group.get('icontype', None),
                    )

                # add plugin_descriptor to list 
                plugin_descriptors.append(plugin_descriptor)

        return plugin_descriptors

    def __parse_rosguiplugin(self, class_el):
        # create default plugin descriptor and group
        plugin_attributes = {}
        groups = []

        # update descriptor and group from rosguiplugin tag
        guiplugin_el = class_el.find('rosguiplugin')
        if guiplugin_el is not None:
            plugin_attributes.update(self.__parse_action_group(guiplugin_el))
            for group_el in guiplugin_el.getiterator('group'):
                groups.append(self.__parse_action_group(group_el))

        return plugin_attributes, groups

    def __parse_action_group(self, group_el):
        attributes = {}
        for tag in ['label', 'icon', 'statustip']:
            text = group_el.findtext(tag)
            if text:
                attributes[tag] = str(text)

        icon_el = group_el.find('icon')
        if icon_el is not None:
            icon_type_attrib = icon_el.get('type')
            if icon_type_attrib is not None:
                attributes['icontype'] = str(icon_type_attrib)

        return attributes
