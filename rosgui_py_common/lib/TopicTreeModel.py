import roslib
roslib.load_manifest('rosgui_py_common')
import rospy

import MessageTreeModel
reload(MessageTreeModel) # force reload to update on changes during runtime

class TopicTreeModel(MessageTreeModel.MessageTreeModel):

    def __init__(self, parent=None):
        super(TopicTreeModel, self).__init__(parent, ['Slot', 'Type', 'Path'])
        topic_list = rospy.get_published_topics()
        for topic_path, topic_type in topic_list:
            topic_name = topic_path.strip('/')
            message_instance = roslib.message.get_message_class(topic_type)()
            self.add_message(message_instance, topic_name, topic_type, topic_path)


    def _get_data_for_path(self, slot_name, slot_type, slot_path):
        return (slot_name, slot_type, slot_path)
