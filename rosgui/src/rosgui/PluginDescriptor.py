class PluginDescriptor(object):

    def __init__(self, plugin_id, attributes=None):
        self.dict_ = {}
        self.dict_['plugin_id'] = plugin_id
        self.dict_['attributes'] = attributes if attributes is not None else {}

    def plugin_id(self):
        return self.dict_['plugin_id']

    def attributes(self):
        return self.dict_['attributes']

    def action_attributes(self):
        if self.dict_.has_key('action'):
            return self.dict_['action']
        return {}

    def set_action_attributes(self, label, statustip=None, icon=None, icontype=None):
        self.dict_['action'] = {
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        }

    def groups(self):
        if self.dict_.has_key('groups'):
            return self.dict_['groups']
        return []

    def add_group_attributes(self, label, statustip=None, icon=None, icontype=None):
        if not self.dict_.has_key('groups'):
            self.dict_['groups'] = []
        self.dict_['groups'].append({
            'label': label,
            'statustip': statustip,
            'icon': icon,
            'icontype': icontype,
        })
