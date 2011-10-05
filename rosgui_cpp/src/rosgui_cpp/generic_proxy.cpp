#include <rosgui_cpp/generic_proxy.h>

#include <QMetaObject>

namespace rosgui_cpp {

GenericProxy::GenericProxy(QObject* obj)
  : object_(obj)
{}

QObject* GenericProxy::proxiedObject()
{
  return object_;
}

void GenericProxy::setProxiedObject(QObject* obj)
{
  object_ = obj;
}

bool GenericProxy::invokeMethod(const char* member, QGenericArgument val0, QGenericArgument val1, QGenericArgument val2, QGenericArgument val3, QGenericArgument val4, QGenericArgument val5, QGenericArgument val6, QGenericArgument val7, QGenericArgument val8, QGenericArgument val9)
{
  if (!object_) return false;
  //qDebug("GenericProxy::invokeMethod(%s, %s)", object_->objectName().toStdString().c_str(), member);
  return QMetaObject::invokeMethod(object_, member, Qt::DirectConnection, val0, val1, val2, val3, val4, val5, val6, val7, val8, val9);
}

bool GenericProxy::invokeMethodWithReturn(const char* member, QGenericReturnArgument ret, QGenericArgument val0, QGenericArgument val1, QGenericArgument val2, QGenericArgument val3, QGenericArgument val4, QGenericArgument val5, QGenericArgument val6, QGenericArgument val7, QGenericArgument val8, QGenericArgument val9)
{
  if (!object_) return false;
  //qDebug("GenericProxy::invokeMethodWithReturn(%s, %s)", object_->objectName().toStdString().c_str(), member);
  return QMetaObject::invokeMethod(object_, member, Qt::DirectConnection, ret, val0, val1, val2, val3, val4, val5, val6, val7, val8, val9);
}

} // namespace
