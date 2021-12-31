/****************************************************************************
* MeshLab                                                           o o     *
* An extendible mesh processor                                    o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005, 2009                                          \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#include <stdlib.h>
#include <assert.h>
#include <QtCore/QVariant>
#include "qgetopt.h"

#include <iostream>
using namespace std;

GetOpt::GetOpt(int argc, char *argv[] ) {
  appname = argv[0];
  for(int i = 1; i < argc; i++)
    args.push_back(argv[i]);
}

GetOpt::GetOpt(const QStringList &a): args(a) {
  appname = a[0];
  args = a;
  args.pop_front();
}

 //add an option without a value
void GetOpt::addSwitch(char s, const QString &name, const QString &description, bool *b ) {
  Option option;
  assert(!findOption(s, option));
  assert(!findArg(name, option));
  option.type = Option::SWITCH;
  option.o = s;
  option.name = name;
  option.description = description;
  option.boolean_value = b;
  options.push_back(option);
}

  //add a valued option (v will be left untouched if the option is not given)
void GetOpt::addOption(char s, const QString &name, const QString &description, QVariant *v ) {
  Option option(Option::OPTION, s, name, description);
  option.value = v;

  assert(!findOption(s, option)); //TODO make this check systematic
  assert(!findArg(name, option));

  options.push_back(option);


}
  //add an argument
void GetOpt::addArgument(const QString &name, const QString &description, QVariant *v) {
  Option option;
  option.value = v;
  addArgument(name, description, option);
}

void GetOpt::addArgument(const QString &name, const QString &description, QString *v) {
  Option option;
  option.string_value = v;
  addArgument(name, description, option);
}

void GetOpt::addArgument(const QString &name, const QString &description, double *v) {
  Option option;
  option.double_value = v;
  addArgument(name, description, option);
}

void GetOpt::addArgument(const QString &name, const QString &description, int *v) {
  Option option;
  option.int_value = v;
  addArgument(name, description, option);
}

void GetOpt::addArgument(const QString &name, const QString &description, bool *v) {
  Option option;
  option.boolean_value = v;
  addArgument(name, description, option);
}

void GetOpt::addArgument(const QString &name, const QString &description, Option option) {
    assert(!findArg(name, option));
    option.type = Option::ARGUMENT;
    option.name = name;
    option.description = description;
    options.push_back(option);
}

void GetOpt::addOption(char s, const QString &longname, const QString &description, QString *v) {
    Option option(Option::OPTION, s, longname, description);
    option.string_value = v;
    options.push_back(option);
}
void GetOpt::addOption(char s, const QString &longname, const QString &description, double *v) {
    Option option(Option::OPTION, s, longname, description);
    option.double_value = v;
    options.push_back(option);
}
void GetOpt::addOption(char s, const QString &longname, const QString &description, int *v) {
    Option option(Option::OPTION, s, longname, description);
    option.int_value = v;
    options.push_back(option);
}
void GetOpt::addOption(char s, const QString &longname, const QString &description, bool *v) {
    Option option(Option::OPTION, s, longname, description);
    option.boolean_value = v;
    options.push_back(option);
}

  //add an optional agrument
void GetOpt::addOptionalArgument(const QString &name, const QString &description, QVariant *v) {
  Option option;
  assert(!findArg(name, option));
  option.type = Option::OPTIONAL;
  option.name = name;
  option.description = description;
  option.value = v;
  options.push_back(option);
}
  //return application name
QString &GetOpt::applicationName() {
  return appname;
}

  //return usage string
QString GetOpt::usage() {
  QString u = "Usage: " + appname;
  //arguments
  bool has_optionals = false;
  bool has_options = false;
  for(int i = 0; i < options.size(); i++) {
    if(options[i].type == Option::OPTION) has_options = true;
    if(options[i].type == Option::OPTIONAL) has_optionals = true;
    if(options[i].type != Option::ARGUMENT) continue;
    u += " <" + options[i].name + ">";
  }
  //optional arguments
  if(has_optionals) {
    u += " [";
    for(int i = 0; i < options.size(); i++) {
      if(options[i].type != Option::OPTIONAL) continue;
      u += "<" + options[i].name + ">";
    }
    u += "]";
  }
  if(has_options) {
    u += " [-";
    for(int i = 0; i < options.size(); i++) {
      if(options[i].type != Option::OPTION) continue;
      u += options[i].o;
    }
    u += "]";
  }
  u += "\n\n";
  //compute maxlen:
  int maxlen = 0;
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    int len = o.name.size() + 2;
    switch(o.type) {
      case Option::ARGUMENT:
      case Option::OPTIONAL: break;
      case Option::SWITCH:   len += 5; break;
      case Option::OPTION:   len += 16; break;
      default: break;
    }
    if(len > maxlen) maxlen = len;
  }
  //print options and arguments in the given order
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    QString line = "";
    switch(o.type) {
      case Option::ARGUMENT:
      case Option::OPTIONAL: line += o.name; break;
      case Option::SWITCH:   line += "-" + QString(o.o) + " --" + o.name; break;
      case Option::OPTION:   line += "-" + QString(o.o) + " <val> --" + o.name + " <val>"; break;
      default: break;
    }
    QString blank = "";
    blank.resize(maxlen - line.size());
    blank.fill(' ');
    line += blank + formatDesc(o.description, maxlen) + "\n";
    u += line;
  }
  return u;
}

void GetOpt::parse() {
  QString error;
  if(!parse(error)) {
    cerr << qPrintable(error) << endl << endl << qPrintable(usage()) << endl << endl;
    exit(0);
  }
}

bool GetOpt::parse(QString &error) {
  for(int i = 0; i < args.size(); i++) {
    QString arg = args[i];
    if(args[i] == "-h" || args[i] == "--help") {
      cout << qPrintable(usage()) << endl << qPrintable(help) << endl;
      exit(0);
    }
    //long option
    if(arg.startsWith( QString::fromLatin1( "--" ) ) ) {
      arg = arg.mid( 2 );
      if(arg.isEmpty()) {
        error = "'--' feature not supported, yet";
        return false;
      }
      Option o;
      if(!findArg(arg, o)) {
         error = "Unknown option: '" + arg + "'";
        return false;
      }
      if(o.type == Option::SWITCH) {
        *(o.boolean_value) = true;
      } else { //OPTION
        i++;
        if(args.size() <= i) {
          error =  "Missing value for option: " + arg;
          return false;
        }
        arg = args[i];
        if(i == args.size() || arg[0] == '-') {
          error = "Missing argument after option '" + arg + "'";
          return false;
        }
        if(!parseOption(o, arg))
            return false;
      }

    //option
    } else if( arg[0] == '-' ) {
      if(arg.size() != 2) {
        error = "Invalid option: " + arg;
        return false;
      }
      Option o;
      if(!findOption(arg[1].toLatin1(), o)) {
         error = "Unknown option: '" + arg + "'";
        return false;
      }
      if(o.type == Option::SWITCH) {
        *(o.boolean_value) = true;
      } else { //OPTION
        i++;
        if(args.size() <= i) {
          error =  "Missing value for option: " + arg;
          return false;
        }
        arg = args[i];
        if(i == args.size()) {
          error = "Missing argument after option '" + arg + "'";
          return false;
        }
        QVariant v(arg);
        if(!v.canConvert(o.value->type()) || !v.convert(o.value->type())) {
          error = "Error while parsing option " + o.name + ": cannot convert " +
                  arg + " to: " + o.value->typeName();
          return false;
        }
        *(o.value) = v;
      }
    //argument
    } else {
      arguments.push_back(arg);
    }
  }
  //test arguments
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    if(o.type != Option::ARGUMENT) continue;
    if(arguments.isEmpty()) {
      error = "Too few arguments, could not parse argument '" + o.name + "'";
      return false;
    }
    if(!parseOption(o, arguments.front()))
        return false;
    arguments.pop_front();
  }
   //test arguments
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    if(o.type != Option::OPTIONAL) continue;
    if(arguments.isEmpty()) break;
    if(!parseOption(o, arguments.front()))
        return false;
    arguments.pop_front();
  }
  //test arguments
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    if(o.type != Option::ARGUMENT) continue;
    if(arguments.isEmpty()) break;
    if(!parseOption(o, arguments.front()))
        return false;
    arguments.pop_front();
  }
  if(!arguments.isEmpty() && !unlimitedArgs) {
    error = "Too many arguments";
    return false;
  }
  return true;
}

bool GetOpt::findOption(char c, Option &option) {
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    if(o.type != Option::OPTION && o.type != Option::SWITCH) continue;
    if(o.o == c) {
      option = o;
      return true;
    }
  }
  return false;
}

bool GetOpt::findArg(const QString &name, Option &option) {
  for(int i = 0; i < options.size(); i++) {
    Option &o = options[i];
    if(o.name == name) {
      option = o;
      return true;
    }
  }
  return false;
}

QString GetOpt::formatDesc(QString desc, int len) {
  QString output;
  //search for last space before 79 - len characters
  while(!desc.isEmpty()) {
    int pos = desc.lastIndexOf(" ", 79 - len);
    if(pos == -1) {
      output += desc;
      break;
    }
    output += desc.left(pos) + "\n";
    QString blank;
    blank.resize(len);
    blank.fill(' ');
    output += blank;
    desc = desc.mid(pos+1);
  }
  return output;
}

bool GetOpt::parseOption(GetOpt::Option &o, const QString &arg) {
    QVariant::Type type;
    if(o.value) type = o.value->type();
    if(o.string_value) type = QVariant::String;
    if(o.double_value) type = QVariant::Double;
    if(o.int_value) type = QVariant::Int;
    if(o.boolean_value) type = QVariant::Bool;
    QVariant v(arg);

    if(!v.canConvert(type) || !v.convert(type)) {
      cerr << "Error while parsing option " << qPrintable(o.name) << ": cannot convert " <<
              qPrintable(arg) << " to: " << qPrintable(v.typeName()) << endl;
      return false;
    }
    if(o.value)
      *(o.value) = v;
    if(o.string_value) *(o.string_value) = v.toString();
    if(o.double_value) *(o.double_value) = v.toDouble();
    if(o.int_value) *(o.int_value) = v.toInt();
    if(o.boolean_value) *(o.boolean_value) = v.toBool();
    return true;
}
