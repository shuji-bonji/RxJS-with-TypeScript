---
description: Erklärung, wie Daten innerhalb eines Streams mithilfe von RxJS-Transformationsoperatoren verarbeitet und transformiert werden. Von einfachen Transformationen mit map, scan, mergeMap, switchMap, concatMap bis hin zu asynchronen Transformationen, Buffering und Windowing werden praktische Muster mit TypeScript-Typsicherheit anhand zahlreicher Codebeispiele vorgestellt.
---

# Transformationsoperatoren

Transformationsoperatoren werden verwendet, um Daten innerhalb einer RxJS-Pipeline zu transformieren und zu verarbeiten.
Durch die Transformation von Werten in neue Formen kann der reaktive Datenfluss flexibler und leistungsfähiger gesteuert werden.


## Operatorenliste
### ◾ Einfache Wertetransformation

|Operator|Beschreibung|
|---|---|
|[map](./map)|Wendet eine Transformationsfunktion auf jeden Wert an|

### ◾ Akkumulationsverarbeitung

|Operator|Beschreibung|
|---|---|
|[scan](./scan)|Erzeugt Werte akkumulativ|
|[reduce](./reduce)|Gibt nur das endgültige akkumulierte Ergebnis aus|

### ◾ Paar- und Gruppierungsverarbeitung

|Operator|Beschreibung|
|---|---|
|[pairwise](./pairwise)|Verarbeitet zwei aufeinanderfolgende Werte als Paar|
|[groupBy](./groupBy)|Gruppiert Werte basierend auf einem Schlüssel|

### ◾ Asynchrone Transformation

|Operator|Beschreibung|
|---|---|
|[mergeMap](./mergeMap) |Transformiert jeden Wert in ein Observable und kombiniert parallel|
|[switchMap](./switchMap) |Wechselt zum neuesten Observable|
|[concatMap](./concatMap) |Führt jedes Observable nacheinander aus|
|[exhaustMap](./exhaustMap) |Ignoriert neue Eingaben während der Ausführung|
|[expand](./expand) |Erweitert Ergebnisse rekursiv|

### ◾ Batch-Verarbeitung

|Operator|Beschreibung|
|---|---|
|[buffer](./buffer) |Sammelt Werte zum Timing eines anderen Observables|
|[bufferTime](./bufferTime) |Sammelt Werte in regelmäßigen Zeitintervallen|
|[bufferCount](./bufferCount) |Sammelt in bestimmter Anzahl|
|[bufferWhen](./bufferWhen) |Buffering mit dynamischer Kontrolle der Endbedingung|
|[bufferToggle](./bufferToggle) |Buffering mit unabhängiger Steuerung von Start und Ende|
|[windowTime](./windowTime) |Teilt in Sub-Observables in regelmäßigen Zeitintervallen|


## Praktische Transformationsmuster

In realen Anwendungen ermöglichen Transformationsoperatoren in Kombination
folgende Verarbeitungen:

- Eingabevalidierung und Feedback
- Optimale Steuerung asynchroner API-Anfragen
- Formatierung, Aggregation und Normalisierung von Daten
- Batch-Verarbeitung und Gruppierung von Ereignisströmen

👉 Weitere Details: Siehe [Praktische Transformationsmuster](./practical-use-cases).

## 🚨 Hinweise

Um häufige Fehler bei der Verwendung von Transformationsoperatoren zu vermeiden, siehe auch:

- **[Seiteneffekte in map](/de/guide/anti-patterns/common-mistakes#5-seiteneffekte-in-map)** - `map` als reine Funktion verwenden
- **[Ungeeignete Operatorauswahl](/de/guide/anti-patterns/common-mistakes#12-ungeeignete-operatorauswahl)** - Angemessene Unterscheidung von Higher-Order-Operatoren
