class PluginDescriptor(object):

    def __init__(self, plugin_id, attributes=None):
        self.dict_ = {}
        self.dict_['plugin_id'] = plugin_id
        self.dict_['attributes'] = attributes or {}

    def plugin_id(self):
        return self.dict_['plugin_id']

    def attributes(self):
        return self.dict_['attributes']

    def action_attributes(self):
        return self.dict_.get('action', {})

    def set_action_attributes(self, label, statustip=None, icon=None, icontype=None):
        self.dict_['action'] = {
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        }

    def groups(self):
        return self.dict_.get('groups', [])

    def add_group_attributes(self, label, statustip=None, icon=None, icontype=None):
        if 'groups' not in self.dict_:
            self.dict_['groups'] = []
        self.dict_['groups'].append({
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        })
