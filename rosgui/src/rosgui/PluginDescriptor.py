class PluginDescriptor(object):

    def __init__(self, plugin_id, attributes=None):
        self._dict = {}
        self._dict['plugin_id'] = plugin_id
        self._dict['attributes'] = attributes or {}

    def plugin_id(self):
        return self._dict['plugin_id']

    def attributes(self):
        return self._dict['attributes']

    def action_attributes(self):
        return self._dict.get('action', {})

    def set_action_attributes(self, label, statustip=None, icon=None, icontype=None):
        self._dict['action'] = {
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        }

    def groups(self):
        return self._dict.get('groups', [])

    def add_group_attributes(self, label, statustip=None, icon=None, icontype=None):
        if 'groups' not in self._dict:
            self._dict['groups'] = []
        self._dict['groups'].append({
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        })
