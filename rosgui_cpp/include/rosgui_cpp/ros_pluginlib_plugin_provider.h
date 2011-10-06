#ifndef rosgui_cpp__RosPluginlibPluginProvider_H
#define rosgui_cpp__RosPluginlibPluginProvider_H

#include "plugin.h"
#include "plugin_context.h"
#include "plugin_descriptor.h"
#include "plugin_provider.h"

#include <pluginlib/class_loader.h>

#include <QList>
#include <QMap>
#include <QString>

#include <string>
#include <vector>

namespace rosgui_cpp
{

template<typename T>
class RosPluginlibPluginProvider
  : public PluginProvider
{

public:

  static RosPluginlibPluginProvider<T>* create_instance(const QString& export_tag, const QString& base_class_type)
  {
    return new RosPluginlibPluginProvider<T>(export_tag, base_class_type);
  }

  RosPluginlibPluginProvider(const QString& export_tag, const QString& base_class_type)
    : PluginProvider()
    , export_tag_(export_tag)
    , base_class_type_(base_class_type)
    , class_loader_(0)
  {}

  virtual ~RosPluginlibPluginProvider()
  {
    if (class_loader_)
    {
      delete class_loader_;
    }
  }

  virtual QMap<QString, QString> discover()
  {
    return PluginProvider::discover();
  }

  virtual QList<PluginDescriptor*> discover_descriptors()
  {
    if (class_loader_)
    {
      delete class_loader_;
    }
    class_loader_ = new pluginlib::ClassLoader<T>(export_tag_.toStdString(), base_class_type_.toStdString());

    QList<PluginDescriptor*> descriptors;

    std::vector<std::string> classes = class_loader_->getDeclaredClasses();
    for (std::vector<std::string>::iterator it = classes.begin(); it != classes.end(); it++)
    {
      std::string lookup_name = *it;

      std::string name = class_loader_->getName(lookup_name);
      std::string type = class_loader_->getClassType(lookup_name);
      std::string base_class_type = class_loader_->getBaseClassType();

      QMap<QString, QString> attributes;
      attributes["class_name"] = name.c_str();
      attributes["class_type"] = type.c_str();
      attributes["class_base_class_type"] = base_class_type.c_str();

      // check if plugin is available
      // TODO: check if library exists
      attributes["not_available"] = ""; // lookup_name.c_str()

      PluginDescriptor* plugin_descriptor = new PluginDescriptor(lookup_name.c_str(), attributes);

      std::string description = class_loader_->getClassDescription(lookup_name);
      // TODO: extract data from plugingui-tag from manifest (description, icons, grouping)
      plugin_descriptor->setActionAttributes(name.c_str(), description.c_str());

      // add plugin descriptor
      descriptors.append(plugin_descriptor);
    }
    return descriptors;
  }

  virtual void* load(const QString& plugin_id, PluginContext* plugin_context)
  {
    return load_explicit_type(plugin_id, plugin_context);
  }

  virtual Plugin* load_plugin(const QString& plugin_id, PluginContext* plugin_context)
  {
    T* instance = load_explicit_type(plugin_id, plugin_context);
    if (instance == 0)
    {
      return 0;
    }
    Plugin* plugin = dynamic_cast<Plugin*>(instance);
    if (plugin == 0)
    {
      // TODO: garbage instance
      qWarning("RosPluginlibPluginProvider::load_plugin() called on non-plugin plugin provider");
      return 0;
    }
    return plugin;
  }

  virtual T* load_explicit_type(const QString& plugin_id, PluginContext* plugin_context)
  {
    std::string lookup_name = plugin_id.toStdString();

    if (!class_loader_->isClassAvailable(lookup_name))
    {
      qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) class not available", lookup_name.c_str());
      return 0;
    }

    try
    {
      class_loader_->loadLibraryForClass(lookup_name);
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) could not load library (%s)", lookup_name.c_str(), e.what());
      return 0;
    }

    T* instance = 0;
    try
    {
      instance = class_loader_->createClassInstance(lookup_name);
    }
    catch (pluginlib::PluginlibException& e)
    {
      qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) failed creating instance (%s)", lookup_name.c_str(), e.what());
      return 0;
    }

    if (!instance)
    {
      qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) failed creating instance", lookup_name.c_str());
      return 0;
    }

    // pass context to plugin
    Plugin* plugin = dynamic_cast<Plugin*>(instance);
    if (plugin)
    {
      try
      {
        init_plugin(plugin_id, plugin_context, plugin);
      }
      catch (std::exception& e)
      {
        // TODO: garbage instance
        qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) failed initializing plugin (%s)", lookup_name.c_str(), e.what());
        return 0;
      }
    }

    //qDebug("RosPluginlibPluginProvider::load_explicit_type(%s) succeeded", lookup_name.c_str());

    return instance;
  }

  virtual void unload(void* /*instance*/)
  {
    // TODO: implement unload
  }

protected:

  virtual void init_plugin(const QString& /*plugin_id*/, PluginContext* plugin_context, Plugin* plugin)
  {
    plugin->initPlugin(*plugin_context);
  }

private:

  QString export_tag_;

  QString base_class_type_;

  pluginlib::ClassLoader<T>* class_loader_;

};

} // namespace

#endif // rosgui_cpp__RosPluginlibPluginProvider_H
