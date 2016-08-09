#ifndef _registry_h_
#define _registry_h_

// Authors: Sascha Lange
// Copyright (c) 2004, Neuroinformatics Group, University of Osnabrueck
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, 
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice, 
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the University of Osnabrueck nor the names of its 
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <vector>
#include <string>

class Plant;
class Controller;
class Reward;
class Graphic;
class Statistics;
class Observer;
class Input;
class Output;

/** Central registry of factory classes that are able to construct instances
    of an unknown class. */
class AbstractRegistry{
public:
  /** this interface declares an abstract factory, that is able to construct
      instances of one class. Each class needs a different implementation of 
      the abstract factory. */
  class Factory {  
  public:
    virtual ~Factory() {}
    virtual void* create() const=0;
  };    

  virtual ~AbstractRegistry();
  
  /** add a factory for the class "name" */
  virtual void add(const char* name, const char* desc, Factory* factory);

  /** lookup a factory for the requested class "name" */
  virtual Factory* lookup(const char* name) const;

  virtual int listEntries(char* buf, int buf_size) const;

  virtual void listEntries(std::vector<std::string>* names , std::vector<std::string>* desc) const;

protected:

  AbstractRegistry();              // Abstract class, can not be created
  
  struct Entry {                   // mapping name -> factory
    std::string name;
    std::string desc;
    Factory* factory;
  };

  std::vector< Entry > entries;
};


/** Central registry of factory classes that are able to construct instances
    of plants. Implemented as singleton. */
class PlantFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static PlantFactory* getThePlantFactory();

  Plant* create(const char* name) const;

protected:
  static PlantFactory* the_plantFactory;
};


/** Central registry of factory classes that are able to construct instances
    of Controller. Implemented as singleton. */
class ControllerFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static ControllerFactory* getTheControllerFactory();

  Controller* create(const char* name) const;

protected:
  static ControllerFactory* the_controllerFactory;
};

/** Central registry of factory classes that are able to construct instances
 of Reward. Implemented as singleton. */
class RewardFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static RewardFactory* getTheRewardFactory();
  
  Reward* create(const char* name) const;
  
protected:
  static RewardFactory* the_rewardFactory;
};

/** Central registry of factory classes that are able to construct instances
    of Graphic. Implemented as singleton. */
class GraphicFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static GraphicFactory* getTheGraphicFactory();

  Graphic* create(const char* name) const;

protected:
  static GraphicFactory* the_graphicFactory;
};

/** Central registry of factory classes that are able to construct instances
    of Statistics. Implemented as singleton. */
class StatisticsFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static StatisticsFactory* getTheStatisticsFactory();

  Statistics* create(const char* name) const;

protected:
  static StatisticsFactory* the_statisticsFactory;
};

/** Central registry of factory classes that are able to construct instances
    of Observer. Implemented as singleton. */
class ObserverFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static ObserverFactory* getTheObserverFactory();

  Observer* create(const char* name) const;

protected:
  static ObserverFactory* the_observerFactory;
};

/** Central registry of factory classes that are able to construct instances
    of Input. Implemented as singleton. */
class InputFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static InputFactory* getTheInputFactory();

  Input* create(const char* name) const;

protected:
  static InputFactory* the_inputFactory;
};

/** Central registry of factory classes that are able to construct instances
    of Output. Implemented as singleton. */
class OutputFactory : public AbstractRegistry {
public:
  /** Implemented as singleton */
  static OutputFactory* getTheOutputFactory();

  Output* create(const char* name) const;

protected:
  static OutputFactory* the_outputFactory;
};

// Das folgende Makro generiert beim Aufruf eine Farbrikklasse, z.B.
// factory_MyPlant. Davon wird genau ein statisches Objekt angelegt, damit der
// Konstruktor der Farbikklasse aufgerufen wird. Dieser sorgt wiederum daf�r,
// dass die Fabrikklasse bei der zentralen Registry angemeldet wird.
// Von nun an steht die Fabrik in der Registry zur Verf�gung. Statische 
// Variablen werden beim Programmstart generell initialisiert, bevor main 
// aufgerufen wird.

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central PLANT factory. */
#define REGISTER_GRAPHIC(name, desc)                                    \
class _graphicfactory_##name : public GraphicFactory::Factory {         \
public:                                                                 \
  _graphicfactory_##name() {                                            \
    GraphicFactory::getTheGraphicFactory()->add(#name, desc, this);     \
  }                                                                     \
  void* create() const { return new name (); }                          \
};                                                                      \
static _graphicfactory_##name *_graphicfactory_instance_##name =        \
  new _graphicfactory_##name ();    

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central PLANT factory. */
#define REGISTER_PLANT(name, desc)                                      \
class _plantfactory_##name : public PlantFactory::Factory {             \
public:                                                                 \
  _plantfactory_##name() {                                              \
    PlantFactory::getThePlantFactory()->add(#name, desc, this);         \
  }                                                                     \
  void* create() const { return new name (); }                          \
};                                                                      \
static _plantfactory_##name *_plantfactory_instance_##name =            \
  new _plantfactory_##name ();    


/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central CONTROLLER factory. */
#define REGISTER_CONTROLLER(name, desc)                                 \
class _controllerfactory_##name : public ControllerFactory::Factory {   \
public:                                                                 \
  _controllerfactory_##name() {                                         \
    ControllerFactory::getTheControllerFactory()->add(#name, desc, this);\
  }                                                                     \
  void* create() const { return new name (); }                          \
};                                                                      \
static _controllerfactory_##name *_controllerfactory_instance_##name =  \
  new _controllerfactory_##name ();    

/** This macro creates an implementation of the abstract factory for the
 class "name" and adds it to the central REWARD factory. */
#define REGISTER_REWARD(name, desc)                                     \
class _rewardfactory_##name : public RewardFactory::Factory {           \
public:                                                                 \
_rewardfactory_##name() {                                               \
RewardFactory::getTheRewardFactory()->add(#name, desc, this);           \
}                                                                       \
void* create() const { return new name (); }                            \
};                                                                      \
static _rewardfactory_##name *_rewardfactory_instance_##name =          \
new _rewardfactory_##name ();   

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central STATISTICS factory. */
#define REGISTER_STATISTICS(name, desc)                                  \
class _statisticsfactory_##name : public StatisticsFactory::Factory {    \
public:                                                                  \
  _statisticsfactory_##name() {                                          \
    StatisticsFactory::getTheStatisticsFactory()->add(#name, desc, this);\
  }                                                                      \
  void* create() const { return new name (); }                           \
};                                                                       \
static _statisticsfactory_##name *_statisticsfactory_instance_##name =   \
  new _statisticsfactory_##name ();

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central OBSERVER factory. */
#define REGISTER_OBSERVER(name, desc)                                 \
class _observerfactory_##name : public ObserverFactory::Factory {     \
public:                                                               \
  _observerfactory_##name() {                                         \
    ObserverFactory::getTheObserverFactory()->add(#name, desc, this); \
  }                                                                   \
  void* create() const { return new name (); }                        \
};                                                                    \
static _observerfactory_##name *_observerfactory_instance_##name =    \
  new _observerfactory_##name ();

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central INPUT factory. */
#define REGISTER_INPUT(name, desc)                                \
class _inputfactory_##name : public InputFactory::Factory {       \
public:                                                           \
  _inputfactory_##name() {                                        \
    InputFactory::getTheInputFactory()->add(#name, desc, this);   \
  }                                                               \
  void* create() const { return new name (); }                    \
};                                                                \
static _inputfactory_##name *_inputfactory_instance_##name =      \
  new _inputfactory_##name ();

/** This macro creates an implementation of the abstract factory for the
    class "name" and adds it to the central OUTPUT factory. */
#define REGISTER_OUTPUT(name, desc)                                 \
class _outputfactory_##name : public OutputFactory::Factory {       \
public:                                                             \
  _outputfactory_##name() {                                         \
    OutputFactory::getTheOutputFactory()->add(#name, desc, this);   \
  }                                                                 \
  void* create() const { return new name (); }                      \
};                                                                  \
static _outputfactory_##name *_outputfactory_instance_##name =      \
  new _outputfactory_##name ();

#endif
