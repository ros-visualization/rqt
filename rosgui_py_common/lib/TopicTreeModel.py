import roslib
roslib.load_manifest('rosgui_py_common')
import rospy

import TreeModel
reload(TreeModel) # force reload to update on changes during runtime

class TopicTreeModel(TreeModel.TreeModel):
    def __init__(self, parent=None):
        super(TopicTreeModel, self).__init__(parent)
        self._clear()
        topic_list = rospy.get_published_topics()
        for topic_name, topic_type in topic_list:
            topic_name = topic_name.strip('/')
            message = roslib.message.get_message_class(topic_type)()
            self._recursive_create_items(self.rootItem, topic_name, message)


    def _recursive_create_items(self, item, topic_name, field):
        child = self.TreeItem((topic_name, type(field)), item)
        item.appendChild(child)

        if hasattr(field, '__slots__'):
            for slot_name in field.__slots__:
                self._recursive_create_items(child, slot_name, getattr(field, slot_name))
