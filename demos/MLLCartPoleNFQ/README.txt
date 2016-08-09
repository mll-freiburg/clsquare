Info
====

Dieses Verzeichnis enthält Beispiele zur Verwendung des MLLCartPole-Moduls.
Hierbei handelt es sich um eine simulierte Nachbildung eines im Machine
Learning Lab Freiburg vorhandenen realen Cart-Pole-Systems. In dieser
Simulation erprobte Steuerungen sollten generell auch auf dem realen
System anwendbar sein.


Training
========

./train
  oder
CLSquare scripts/train.cls

-> ruft nach jeder Episode test.bash auf
-> mit test1.cls werden 20 zufällige Startstellungen getestet
-> benötigt Modul NFQControl


Wiedergabe eines Beispielversuchs
=================================

./replay

-> Netz 1: zufällig initialisiertes Netz
-> Netz 7: erster Kontakt mit Zielzustand, aber nicht stabil
-> Netz 157: erste von jedem Startzustand erfolgreiche Policy, leicht unruhig
-> Netz 221: Ende des Trainings, völlig stabile Policy


Ausführliches Testen
====================

-> Idee: nach Training wird testen auf alle Netze im 
Verzeichnis ./nets angewandt.
-> Erzeugt Datei test.exhaustive.stat mit 1000 zufälligen Startstellungen

Aufruf mit:
./exhaustive_test.bash               für Standardtest
  oder
./exhaustive_test.bash <cls-Datei>   für eigenen Test


Automatisches Auswerten
=======================

in CLSquare/tools gibt es StatisticEvaluation mit Programm stateval

Aufruf mit:

CLSquare/bin/stateval <.stat-Datei>, z.B.
CLSquare/bin/stateval test.exhaustive.stat

