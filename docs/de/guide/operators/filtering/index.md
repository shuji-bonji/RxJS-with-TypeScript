---
description: "RxJS-Filterungs-Operatoren extrahieren nur die notwendigen Daten aus Streams basierend auf Bedingungen und Zeit. filter, take, skip, debounceTime, throttleTime, distinct, first, last und mehr - mit Auswahlrichtlinien nach Verwendungszweck und praktischen Anwendungsbeispielen."
---

# Filterungs-Operatoren

RxJS-Filterungs-Operatoren sind wichtige Werkzeuge, um nur die benötigten Daten aus einem Stream auszuwählen und unnötige Daten nicht durchzulassen.
Dies verbessert die Effizienz und Leistung Ihrer Anwendung erheblich.

Filterungs-Operatoren sind eine Gruppe von RxJS-Operatoren, die Werte innerhalb eines Streams filtern und nur diejenigen durchlassen, die bestimmte Bedingungen erfüllen.
Durch die Steuerung des Datenflusses und die Verarbeitung nur der notwendigen Werte können Sie effiziente Datenverarbeitungs-Pipelines aufbauen.


## Operatorenliste
### ◾ Grundlegende Filterungs-Operatoren

| Operator | Beschreibung |
|:---|:---|
| [filter](./filter) | Lässt nur Werte durch, die eine Bedingung erfüllen |
| [take](./take) | Nimmt nur die ersten angegebenen Werte |
| [takeLast](./takeLast) | Nimmt die letzten angegebenen Werte |
| [takeWhile](./takeWhile) | Nimmt Werte, solange eine Bedingung erfüllt ist |
| [skip](./skip) | Überspringt die ersten angegebenen Werte |
| [skipLast](./skipLast) | Überspringt die letzten angegebenen Werte |
| [skipWhile](./skipWhile) | Überspringt Werte, solange eine Bedingung erfüllt ist |
| [skipUntil](./skipUntil) | Überspringt Werte bis ein anderes Observable feuert |
| [first](./first) | Nimmt den ersten Wert oder den ersten Wert, der eine Bedingung erfüllt |
| [last](./last) | Nimmt den letzten Wert oder den letzten Wert, der eine Bedingung erfüllt |
| [elementAt](./elementAt) | Nimmt den Wert am angegebenen Index |
| [find](./find) | Findet den ersten Wert, der eine Bedingung erfüllt |
| [findIndex](./findIndex) | Nimmt den Index des ersten Wertes, der eine Bedingung erfüllt |
| [ignoreElements](./ignoreElements) | Ignoriert alle Werte und lässt nur Completion/Error durch |


### ◾ Zeitbasierte Filterungs-Operatoren

| Operator | Beschreibung |
|:---|:---|
| [debounceTime](./debounceTime) | Gibt den letzten Wert aus, wenn für die angegebene Zeit keine Eingabe erfolgt |
| [throttleTime](./throttleTime) | Lässt den ersten Wert durch und ignoriert neue Werte für die angegebene Zeit |
| [auditTime](./auditTime) | Gibt den letzten Wert nach der angegebenen Zeit aus |
| [audit](./audit) | Steuert den Zeitraum mit einem benutzerdefinierten Observable und gibt den letzten Wert aus |
| [sampleTime](./sampleTime) | Samplet den neuesten Wert in bestimmten Zeitintervallen |


### ◾ Bedingungsbasierte Filterungs-Operatoren

| Operator | Beschreibung |
|:---|:---|
| [distinct](./distinct) | Entfernt alle doppelten Werte (gibt nur eindeutige Werte aus) |
| [distinctUntilChanged](./distinctUntilChanged) | Entfernt aufeinanderfolgende doppelte Werte |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Erkennt nur Änderungen bestimmter Properties |


## Praktische Anwendungsfälle

- [Praktische Anwendungsfälle](./practical-use-cases.md) stellt praktische Beispiele vor, die mehrere Filterungs-Operatoren kombinieren (Echtzeit-Suche, unendliches Scrollen usw.).
