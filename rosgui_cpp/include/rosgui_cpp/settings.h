#ifndef rosgui_cpp__Settings_H
#define rosgui_cpp__Settings_H

#include "generic_proxy.h"

#include <QString>
#include <QStringList>
#include <QVariant>

namespace rosgui_cpp
{

class Settings
  : private GenericProxy
{

public:

  Settings(QObject* obj);

  Settings getSettings(const QString& prefix);

  QStringList allKeys() const;

//  int beginReadArray(const QString& prefix);

//  void beginWriteArray(const QString& prefix, int size = -1);

  QStringList childGroups() const;

  QStringList childKeys() const;

  bool contains(const QString& key) const;

//  void endArray();

  void remove(const QString& key);

//  void setArrayIndex(int i);

  void setValue(const QString& key, const QVariant& value);

  QVariant value(const QString& key, const QVariant& defaultValue = QVariant()) const;

};

} // namespace

#endif // rosgui_cpp__Settings_H
