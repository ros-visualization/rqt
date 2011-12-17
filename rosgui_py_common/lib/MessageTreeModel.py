import TreeModel
reload(TreeModel) # force reload to update on changes during runtime

class MessageTreeModel(TreeModel.TreeModel):

    def __init__(self, parent=None, column_names=None):
        super(MessageTreeModel, self).__init__(parent, column_names)


    def add_message(self, message_instance, message_name='', message_type='', message_path=''):
        if message_instance is None:
            return
        self._recursive_create_items(self.rootItem, message_instance, message_name, message_type, message_path)


    def _get_data_for_path(self, slot_name, slot_type, slot_path):
        return (slot_name, slot_type, slot_path)


    def _recursive_create_items(self, parent, slot, slot_name, slot_type, slot_path):
        child = self.TreeItem(self._get_data_for_path(slot_name, slot_type, slot_path), parent)
        parent.appendChild(child)

        if hasattr(slot, '__slots__') and hasattr(slot, '_slot_types'):
            for child_slot_name, child_slot_type in zip(slot.__slots__, slot._slot_types):
                child_slot_path = slot_path + '/' + child_slot_name
                self._recursive_create_items(child, getattr(slot, child_slot_name), child_slot_name, child_slot_type, child_slot_path)

        elif type(slot) in (list, tuple) and (len(slot) > 0) and hasattr(slot[0], '__slots__'):
            child_slot_type = slot_type[:slot_type.find('[')]
            for index, slot in enumerate(slot):
                child_slot_name = slot_name + '[%d]' % index
                child_slot_path = slot_path + '/' + child_slot_name
                self._recursive_create_widget_items(child, slot, child_slot_name, child_slot_type, child_slot_path)
