import roslib
roslib.load_manifest('rosgui_py_common')
import rospy

class TopicDict(object):

    def __init__(self):
        self.update_topics()


    def get_topics(self):
        return self.topic_dict


    def update_topics(self):
        self.topic_dict = {}
        topic_list = rospy.get_published_topics()
        for topic_name, topic_type in topic_list:
            message = roslib.message.get_message_class(topic_type)()
            self.topic_dict.update(self._recursive_create_field_dict(topic_name, message))


    def _recursive_create_field_dict(self, topic_name, field):
        field_dict = {}
        field_dict[topic_name] = {
            'type': type(field),
            'children': {},
        }

        if hasattr(field, '__slots__'):
            for slot_name in field.__slots__:
                field_dict[topic_name]['children'].update(self._recursive_create_field_dict(slot_name, getattr(field, slot_name)))

        return field_dict


if __name__ == '__main__':
    import pprint
    pprint.pprint(TopicDict().get_topics())
