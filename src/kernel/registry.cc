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


#include "registry.h"
#include "string.h"
#include "stdlib.h"

#define ERROUT_REG( __x__ ) std::cerr << "#ERROR: " << __FILE__ << " in Line: " << __LINE__ << " in " << __PRETTY_FUNCTION__ << __x__ << "\n"

AbstractRegistry::AbstractRegistry()
{entries.clear();}

AbstractRegistry::~AbstractRegistry()
{
  for (unsigned int i=0; i < entries.size(); i++) {
    delete entries[i].factory;
  }
  entries.clear();
}

void AbstractRegistry::add(const char* name, const char* desc, Factory *factory) {
  Entry ne;
  ne.name = name;
  ne.desc = desc;
  ne.factory = factory;
  entries.push_back(ne);
}

AbstractRegistry::Factory* AbstractRegistry::lookup(const char* name) const
{
  for (unsigned int i=0; i < entries.size(); i++) {
    if (std::string(name) == entries[i].name) {
      return entries[i].factory;
    }
  }
  return 0;  // nicht gefunden
}

int AbstractRegistry::listEntries(char* buf, int buf_size) const 
{
  buf[0] = '\0';
  int available = buf_size - 1;
  for (unsigned int i=0; i < entries.size(); i++) {
    strncat(buf, entries[i].name.c_str(), available);
    available -= strlen(entries[i].name.c_str());
    strncat(buf, "\n\t", available);
    available -= 2;
    strncat(buf, entries[i].desc.c_str(), available);
    available -= strlen(entries[i].desc.c_str());
    strncat(buf, "\n", available);
    available -= 1;
    if (available < 0) {
      return 1;
    }
  }
  buf[buf_size-1] = '\0'; // safety first
  return 0;
}
      
void AbstractRegistry::listEntries(std::vector<std::string>* names , std::vector<std::string>* desc) const
{
    names->clear();
    if (desc!=0) desc->clear();
    for (unsigned int i=0; i < entries.size(); i++) {
        names->push_back( entries[i].name );
        if (desc!=0) desc->push_back( entries[i].desc );
    }
}

Plant* PlantFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Plant*) factory->create();
  }
}

PlantFactory* PlantFactory::getThePlantFactory() 
{
  if (! the_plantFactory) {
    the_plantFactory = new PlantFactory();
  }
  return the_plantFactory;
}

Controller* ControllerFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Controller*) factory->create();
  }
}

ControllerFactory* ControllerFactory::getTheControllerFactory() 
{
  if (! the_controllerFactory) {
    the_controllerFactory = new ControllerFactory();
  }
  return the_controllerFactory;
}

Reward* RewardFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Reward*) factory->create();
  }
}

RewardFactory* RewardFactory::getTheRewardFactory() 
{
  if (! the_rewardFactory) {
    the_rewardFactory = new RewardFactory();
  }
  return the_rewardFactory;
}

Graphic* GraphicFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Graphic*) factory->create();
  }
}

GraphicFactory* GraphicFactory::getTheGraphicFactory() 
{
  if (! the_graphicFactory) {
    the_graphicFactory = new GraphicFactory();
  }
  return the_graphicFactory;
}

Statistics* StatisticsFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Statistics*) factory->create();
  }
}

StatisticsFactory* StatisticsFactory::getTheStatisticsFactory()
{
  if (! the_statisticsFactory) {
    the_statisticsFactory = new StatisticsFactory();
  }
  return the_statisticsFactory;
}

Observer* ObserverFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Observer*) factory->create();
  }
}

ObserverFactory* ObserverFactory::getTheObserverFactory()
{
  if (! the_observerFactory) {
    the_observerFactory = new ObserverFactory();
  }
  return the_observerFactory;
}

Input* InputFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Input*) factory->create();
  }
}

InputFactory* InputFactory::getTheInputFactory()
{
  if (! the_inputFactory) {
    the_inputFactory = new InputFactory();
  }
  return the_inputFactory;
}

Output* OutputFactory::create(const char* name) const
{
  Factory* factory = lookup(name);
  if (! factory) {
    return 0;
  }
  else {
    return (Output*) factory->create();
  }
}

OutputFactory* OutputFactory::getTheOutputFactory()
{
  if (! the_outputFactory) {
    the_outputFactory = new OutputFactory();
  }
  return the_outputFactory;
}

// die Registry wird NICHT statisch erzeugt. Bei statischer Erzeugung wäre
// nicht sicher, dass sie vor den ebenfall statisch erzeugten Factoryklassen
// angelegt worden wäre (the_plantFactory könnte beim Aufruf des Konstruktors 
// der Factory also noch ins Nirvana zeigen). Daher wird die Registry erst bei
// Bedarf von getThePlantFactory erzeugt.

PlantFactory*       PlantFactory::the_plantFactory              = 0;
ControllerFactory*  ControllerFactory::the_controllerFactory    = 0;
RewardFactory*      RewardFactory::the_rewardFactory            = 0;
GraphicFactory*     GraphicFactory::the_graphicFactory          = 0;
StatisticsFactory*  StatisticsFactory::the_statisticsFactory    = 0;
ObserverFactory*    ObserverFactory::the_observerFactory        = 0;
InputFactory*       InputFactory::the_inputFactory              = 0;
OutputFactory*      OutputFactory::the_outputFactory            = 0;
