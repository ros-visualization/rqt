#ifndef rosgui_cpp__GenericProxy_H
#define rosgui_cpp__GenericProxy_H

#include <QObject>

namespace rosgui_cpp
{

class GenericProxy
{

public:

  GenericProxy(QObject* obj = 0);

  QObject* proxiedObject();

  void setProxiedObject(QObject* obj);

  bool invokeMethod(const char* member, QGenericArgument val0 = QGenericArgument(), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument());

  bool invokeMethodWithReturn(const char* member, QGenericReturnArgument ret, QGenericArgument val0 = QGenericArgument(), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument());

private:

  QObject* object_;

};

} // namespace

#endif // rosgui_cpp__GenericProxy_H
