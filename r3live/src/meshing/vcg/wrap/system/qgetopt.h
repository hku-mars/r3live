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

#ifndef GETOPT_H
#define GETOPT_H

#include <QString>
#include <QStringList>
#include <QMap>
#include <QVariant>

/* Example usage:

  QString file1, file2;
  QString o_gamma = "10";
  QString o_scale = "0.7";

  GetOpt opt(argc, argv);
  opt.addArgument("img1", "first image", &file1);
  opt.addArgument("img2", "second image", &file2);
  opt.addOption('g', "gamma", "weigth to derivatives of images (default: 10)", &o_gamma);
  opt.addOption('s', "scale", "scale [0.5-0.99] for multiscale approach (default: 0.7)", &o_scale);

  opt.parse();          */

class GetOpt {
 protected:
  struct Option {
    enum Type { SWITCH, OPTION, ARGUMENT, OPTIONAL };
    Type type;
    char o;
    QString name;
    QString description;
    QVariant *value;
    QString *string_value;
    double *double_value;
    int *int_value;
    bool *boolean_value;

    Option(): value(NULL), string_value(NULL), double_value(NULL), int_value(NULL), boolean_value(NULL) {}
    Option(Type _type, char _o, QString _name, QString _descr):
        type(_type), o(_o), name(_name), description(_descr),
        value(NULL), string_value(NULL), double_value(NULL), int_value(NULL), boolean_value(NULL) {}
  };

  bool unlimitedArgs;
  QList<Option> options;

 public:
  QString appname;          //application name
  QString help;             //help text
  QStringList args;         //original argument vector
  QStringList arguments;    //arbitrary long list of arguments if unlimitedArgs is true

  GetOpt(): unlimitedArgs(false) {}
  GetOpt(int argc, char *argv[] );
  GetOpt(const QStringList &a);

  //add an option without a value
  void addSwitch(char s, const QString &longname, const QString &description, bool *b );

  //add a valued option (v will be left untouched if the option is not given)
  void addOption(char s, const QString &longname, const QString &description, QVariant *v);
  void addOption(char s, const QString &longname, const QString &description, QString *v);
  void addOption(char s, const QString &longname, const QString &description, double *v);
  void addOption(char s, const QString &longname, const QString &description, int *v);
  void addOption(char s, const QString &longname, const QString &description, bool *v);


  //add an argument
  void addArgument(const QString &name, const QString &description, QVariant *v);
  void addArgument(const QString &name, const QString &description, QString *v);
  void addArgument(const QString &name, const QString &description, double *v);
  void addArgument(const QString &name, const QString &description, int *v);
  void addArgument(const QString &name, const QString &description, bool *v);
  void addArgument(const QString &name, const QString &description, Option option);

  //add an optional agrument
  void addOptionalArgument(const QString &name, const QString &description, QVariant *v);

  //allow an unlimited number of optional arguments
  void allowUnlimitedArguments(bool allow) { unlimitedArgs = allow; }

  //set help if someone uses -h or --help option
  void setHelp(QString &_help) { help = _help; }

  //parses the command line and fill variables or print an error message and exits
  void parse();

  //return usage string
  QString usage();

  //return argv[0]
  QString &applicationName();

 protected:
  //parses and return true on success
  bool parse(QString &error);
  //return options or switch
  bool findOption(char c, Option &option);
  //return any named argument
  bool findArg(const QString &name, Option &option);
  //split desc into n pieces of the right length TODO: check for newlines also
  QString formatDesc(QString desc, int len);

  bool parseOption(Option &option, const QString &arg);
};

#endif

