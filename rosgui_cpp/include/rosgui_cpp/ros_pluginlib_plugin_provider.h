#ifndef rosgui_cpp__RosPluginlibPluginProvider_H
#define rosgui_cpp__RosPluginlibPluginProvider_H

#include "plugin.h"
#include "plugin_context.h"
#include "plugin_descriptor.h"
#include "plugin_provider.h"

#include <pluginlib/class_loader.h>
#include <pluginlib/boost_fs_wrapper.h>

#include <QCoreApplication>
#include <QEvent>
#include <QList>
#include <QMap>
#include <QObject>
#include <QString>

#include <fstream>
#include <string>
#include <vector>

//#define USE_PATCHED_PLUGINLIB

namespace rosgui_cpp
{

template<typename T>
class RosPluginlibPluginProvider
  : public QObject
  , public PluginProvider
{

public:

  static RosPluginlibPluginProvider<T>* create_instance(const QString& export_tag, const QString& base_class_type)
  {
    return new RosPluginlibPluginProvider<T>(export_tag, base_class_type);
  }

  RosPluginlibPluginProvider(const QString& export_tag, const QString& base_class_type)
    : QObject()
    , PluginProvider()
    , export_tag_(export_tag)
    , base_class_type_(base_class_type)
    , class_loader_(0)
  {
    unload_libraries_event_ = QEvent::registerEventType();
  }

  RosPluginlibPluginProvider(const RosPluginlibPluginProvider& other)
    : QObject()
    , PluginProvider()
    , export_tag_(other.export_tag_)
    , base_class_type_(other.base_class_type_)
    , class_loader_(0)
  {
    unload_libraries_event_ = QEvent::registerEventType();
  }

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
#ifdef USE_PATCHED_PLUGINLIB
      std::string package_path = class_loader_->getPackagePath(lookup_name);
      std::string library_path = class_loader_->getClassRelativeLibraryPath(lookup_name);
      library_path = pluginlib::joinPaths(package_path, library_path);
      library_path.append(Poco::SharedLibrary::suffix());
      attributes["not_available"] = !std::ifstream(library_path.c_str()) ? lookup_name.c_str() : "";
#else
      attributes["not_available"] = "";
#endif

      PluginDescriptor* plugin_descriptor = new PluginDescriptor(lookup_name.c_str(), attributes);
      QString label = name.c_str();
      QString statustip = class_loader_->getClassDescription(lookup_name).c_str();
      QString icon;
      QString icontype;
#ifdef USE_PATCHED_PLUGINLIB
      parseManifest(lookup_name, package_path, label, statustip, icon, icontype, plugin_descriptor);
#endif
      plugin_descriptor->setActionAttributes(label, statustip, icon, icontype);

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
    instances_[instance] = plugin_id;

    return instance;
  }

  virtual void unload(void* instance)
  {
    if (!instances_.contains(instance))
    {
      qCritical("RosPluginlibPluginProvider::unload() instance not found");
      return;
    }

    QString lookup_name = instances_.take(instance);
    //qDebug("RosPluginlibPluginProvider::unload() instance '%s'", lookup_name.toStdString().c_str());
    libraries_to_unload_.append(lookup_name);

    QCoreApplication::postEvent(this, new QEvent(static_cast<QEvent::Type>(unload_libraries_event_)));
  }

protected:

  virtual void init_plugin(const QString& /*plugin_id*/, PluginContext* plugin_context, Plugin* plugin)
  {
    plugin->initPlugin(*plugin_context);
  }

private slots:

  bool event(QEvent* e)
  {
    if (e->type() == unload_libraries_event_)
    {
      unload_pending_libraries();
      return true;
    }
    return QObject::event(e);
  }

private:

#ifdef USE_PATCHED_PLUGINLIB
  bool parseManifest(const std::string& lookup_name, const std::string& package_path, QString& label, QString& statustip, QString& icon, QString& icontype, PluginDescriptor* plugin_descriptor)
  {
    std::string manifest_path = class_loader_->getPluginManifestPath(lookup_name);
    //qDebug("RosPluginlibPluginProvider::parseManifest() manifest_path \"%s\"", manifest_path.c_str());
    TiXmlDocument doc;
    bool loaded = doc.LoadFile(manifest_path);
    if (!loaded)
    {
      if (doc.ErrorRow() > 0)
      {
        qWarning("RosPluginlibPluginProvider::parseManifest() could not load manifest \"%s\" (%s [line %d, column %d])", manifest_path.c_str(), doc.ErrorDesc(), doc.ErrorRow(), doc.ErrorCol());
      }
      else
      {
        qWarning("RosPluginlibPluginProvider::parseManifest() could not load manifest \"%s\" (%s)", manifest_path.c_str(), doc.ErrorDesc());
      }
      return false;
    }

    std::string library_path = class_loader_->getClassRelativeLibraryPath(lookup_name);
    //qDebug("RosPluginlibPluginProvider::parseManifest() library_path \"%s\"", library_path.c_str());
    std::string class_type = class_loader_->getClassType(lookup_name);

    // search library-tag with specific path-attribute
    TiXmlElement* library_element = doc.FirstChildElement("library");
    while (library_element)
    {
//      if (library_path.compare(library_element->Attribute("path")) == 0)
//      {
        // search class-tag with specific type- and base_class_type-attribute
        TiXmlElement* class_element = library_element->FirstChildElement("class");
        while (class_element)
        {
          if (class_type.compare(class_element->Attribute("type")) == 0 && base_class_type_.compare(class_element->Attribute("base_class_type")) == 0)
          {
            TiXmlElement* gui_plugin_element = class_element->FirstChildElement("rosguiplugin");
            if (gui_plugin_element)
            {
              // extract meta information
              parseActionAttributes(gui_plugin_element, package_path, label, statustip, icon, icontype);

              // extract grouping information
              TiXmlElement* group_element = gui_plugin_element->FirstChildElement("group");
              while (group_element)
              {
                QString group_label;
                QString group_statustip;
                QString group_icon;
                QString group_icontype;
                parseActionAttributes(group_element, package_path, group_label, group_statustip, group_icon, group_icontype);
                plugin_descriptor->addGroupAttributes(group_label, group_statustip, group_icon, group_icontype);

                group_element = group_element->NextSiblingElement("group");
              }
            }
            return true;
          }
          class_element = class_element->NextSiblingElement("class");
        }
        break;
//      }

      library_element = library_element->NextSiblingElement("library");
    }

    qWarning("RosPluginlibPluginProvider::parseManifest() could not handle manifest \"%s\"", manifest_path.c_str());
    return false;
  }

  void parseActionAttributes(TiXmlElement* element, const std::string& package_path, QString& label, QString& statustip, QString& icon, QString& icontype)
  {
    TiXmlElement* child_element;
    if ((child_element = element->FirstChildElement("label")) != 0)
    {
      label = child_element->GetText();
    }
    if ((child_element = element->FirstChildElement("icon")) != 0)
    {
      icontype = child_element->Attribute("type");
      if (icontype == "file")
      {
        // prepend base path
        icon = pluginlib::joinPaths(package_path, child_element->GetText()).c_str();
      }
      else
      {
        icon = child_element->GetText();
      }
    }
    if ((child_element = element->FirstChildElement("statustip")) != 0)
    {
      statustip = child_element->GetText();
    }
  }
#endif

  void unload_pending_libraries()
  {
    while (!libraries_to_unload_.empty())
    {
      QString lookup_name = libraries_to_unload_.takeFirst();
      //qDebug("RosPluginlibPluginProvider::unload_pending_libraries() unloadLibraryForClass(%s)", lookup_name.toStdString().c_str());
#ifdef USE_PATCHED_PLUGINLIB
      class_loader_->unloadLibraryForClass(lookup_name.toStdString());
#else
      qWarning("RosPluginlibPluginProvider::unload_pending_libraries() not supported in the used ROS version");
#endif
    }
  }

  QString export_tag_;

  QString base_class_type_;

  int unload_libraries_event_;

  pluginlib::ClassLoader<T>* class_loader_;

  QMap<void*, QString> instances_;

  QList<QString> libraries_to_unload_;

};

} // namespace

#endif // rosgui_cpp__RosPluginlibPluginProvider_H
