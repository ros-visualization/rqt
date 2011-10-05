#include <rosgui_cpp/plugin_descriptor.h>

namespace rosgui_cpp {

PluginDescriptor::PluginDescriptor(const QString& plugin_id, const QMap<QString, QString>& attributes)
{
  plugin_id_ = plugin_id;
  attributes_ = attributes;
}

const QString& PluginDescriptor::pluginId() const
{
  return plugin_id_;
}

const QMap<QString, QString>& PluginDescriptor::attributes() const
{
  return attributes_;
}

const QMap<QString, QString>& PluginDescriptor::actionAttributes() const
{
  return action_attributes_;
}

void PluginDescriptor::setActionAttributes(const QString& label, const QString& statustip, const QString& icon, const QString& icontype)
{
  action_attributes_["label"] = label;
  action_attributes_["statustip"] = statustip;
  action_attributes_["icon"] = icon;
  action_attributes_["icontype"] = icontype;
}

int PluginDescriptor::countGroups() const
{
  return groups_.size();
}

QMap<QString, QString> PluginDescriptor::group(int index) const
{
  return groups_[index];
}

void PluginDescriptor::addGroupAttributes(const QString& label, const QString& statustip, const QString& icon, const QString& icontype)
{
  QMap<QString, QString> attributes;
  attributes["label"] = label;
  attributes["statustip"] = statustip;
  attributes["icon"] = icon;
  attributes["icontype"] = icontype;
  groups_.append(attributes);
}

QMap<QString, QString> PluginDescriptor::toDictionary() const
{
  QMap<QString, QString> dict;
  QString plugin_prefix = plugin_id_ + ".";
  dict[plugin_prefix + "plugin_id"] = plugin_id_;
  for (QMap<QString, QString>::const_iterator it = attributes_.constBegin(); it != attributes_.constEnd(); it++)
  {
    dict[plugin_prefix + QString("attributes.") + it.key()] = it.value();
  }
  for (QMap<QString, QString>::const_iterator it = action_attributes_.constBegin(); it != action_attributes_.constEnd(); it++)
  {
    dict[plugin_prefix + QString("action.") + it.key()] = it.value();
  }
  int group_index = 1;
  for (QVector<QMap<QString, QString> >::const_iterator it = groups_.constBegin(); it != groups_.constEnd(); it++)
  {
    QString prefix = QString("groups.") + QString::number(group_index) + QString(".");
    for (QMap<QString, QString>::const_iterator jt = it->constBegin(); jt != it->constEnd(); jt++)
    {
      dict[plugin_prefix + prefix + jt.key()] = jt.value();
    }
    group_index++;
  }
  return dict;
}

} // namespace
