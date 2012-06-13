/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef qt_gui_cpp__RosPluginlibPluginProvider_H
#define qt_gui_cpp__RosPluginlibPluginProvider_H

#include "plugin.h"
#include "plugin_context.h"
#include "plugin_descriptor.h"
#include "plugin_provider.h"

#include <boost/shared_ptr.hpp>

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

namespace qt_gui_cpp
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
      std::string library_path = class_loader_->getClassLibraryPath(lookup_name);
      library_path.append(Poco::SharedLibrary::suffix());
      attributes["not_available"] = !std::ifstream(library_path.c_str()) ? QString("library ").append(lookup_name.c_str()).append(" not found (may be it must be built?)") : "";

      PluginDescriptor* plugin_descriptor = new PluginDescriptor(lookup_name.c_str(), attributes);
      QString label = name.c_str();
      QString statustip = class_loader_->getClassDescription(lookup_name).c_str();
      QString icon;
      QString icontype;
      std::string package_path = ros::package::getPath(class_loader_->getClassPackage(lookup_name));
      size_t package_path_length = package_path.length();
      assert(library_path.compare(0, package_path_length, package_path) == 0);
      std::string relative_library_path = library_path.substr(package_path_length + 1);
      parseManifest(lookup_name, package_path, relative_library_path, label, statustip, icon, icontype, plugin_descriptor);
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

    boost::shared_ptr<T> instance;
    try
    {
      instance = create_plugin(lookup_name, plugin_context);
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      qWarning("RosPluginlibPluginProvider::load_explicit_type(%s) could not load library (%s)", lookup_name.c_str(), e.what());
      return 0;
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
    Plugin* plugin = dynamic_cast<Plugin*>(&*instance);
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
    instances_[&*instance] = instance;

    return &*instance;
  }

  virtual void unload(void* instance)
  {
    if (!instances_.contains(instance))
    {
      qCritical("RosPluginlibPluginProvider::unload() instance not found");
      return;
    }

    boost::shared_ptr<T> pointer = instances_.take(instance);
    libraries_to_unload_.append(pointer);

    QCoreApplication::postEvent(this, new QEvent(static_cast<QEvent::Type>(unload_libraries_event_)));
  }

protected:

  virtual boost::shared_ptr<T> create_plugin(const std::string& lookup_name, PluginContext* /*plugin_context*/ = 0)
  {
    return class_loader_->createInstance(lookup_name);
  }

  virtual void init_plugin(const QString& /*plugin_id*/, PluginContext* plugin_context, Plugin* plugin)
  {
    plugin->initPlugin(*plugin_context);
  }

private slots:

  bool event(QEvent* e)
  {
    if (e->type() == unload_libraries_event_)
    {
      libraries_to_unload_.clear();
      return true;
    }
    return QObject::event(e);
  }

private:

  bool parseManifest(const std::string& lookup_name, const std::string& package_path, const std::string& /*relative_library_path*/, QString& label, QString& statustip, QString& icon, QString& icontype, PluginDescriptor* plugin_descriptor)
  {
    //qDebug("RosPluginlibPluginProvider::parseManifest() relative_library_path \"%s\"", relative_library_path.c_str());

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

    // search library-tag with specific path-attribute
    std::string class_type = class_loader_->getClassType(lookup_name);
    TiXmlElement* library_element = doc.FirstChildElement("library");
    while (library_element)
    {
//      if (relative_library_path.compare(library_element->Attribute("path")) == 0)
//      {
        // search class-tag with specific type- and base_class_type-attribute
        TiXmlElement* class_element = library_element->FirstChildElement("class");
        while (class_element)
        {
          if (class_type.compare(class_element->Attribute("type")) == 0 && base_class_type_.compare(class_element->Attribute("base_class_type")) == 0)
          {
            TiXmlElement* qtgui_element = class_element->FirstChildElement("qtgui");
            if (qtgui_element)
            {
              // extract meta information
              parseActionAttributes(qtgui_element, package_path, label, statustip, icon, icontype);

              // extract grouping information
              TiXmlElement* group_element = qtgui_element->FirstChildElement("group");
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

  void unload_pending_libraries()
  {
  }

  QString export_tag_;

  QString base_class_type_;

  int unload_libraries_event_;

  pluginlib::ClassLoader<T>* class_loader_;

  QMap<void*, boost::shared_ptr<T> > instances_;

  QList<boost::shared_ptr<T> > libraries_to_unload_;

};

} // namespace

#endif // qt_gui_cpp__RosPluginlibPluginProvider_H
