#ifndef rosgui_cpp__PluginDescriptor_H
#define rosgui_cpp__PluginDescriptor_H

#include <QMap>
#include <QString>
#include <QVector>

namespace rosgui_cpp
{

class PluginDescriptor
{

public:

  PluginDescriptor(const QString& plugin_id, const QMap<QString, QString>& attributes = (QMap<QString, QString>()));

  const QString& pluginId() const;

  const QMap<QString, QString>& attributes() const;

  const QMap<QString, QString>& actionAttributes() const;

  void setActionAttributes(const QString& label, const QString& statustip = QString(), const QString& icon = QString(), const QString& icontype = QString());

  int countGroups() const;

  QMap<QString, QString> group(int index) const;

  void addGroupAttributes(const QString& label, const QString& statustip = QString(), const QString& icon = QString(), const QString& icontype = QString());

  QMap<QString, QString> toDictionary() const;

protected:

  QString plugin_id_;

  QMap<QString, QString> attributes_;

  QMap<QString, QString> action_attributes_;

  QVector<QMap<QString, QString> > groups_;

};

} // namespace

#endif // rosgui_cpp__PluginDescriptor_H
