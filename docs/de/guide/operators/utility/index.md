---
description: "Utility-Operatoren unterstützen die Steuerung von Seiteneffekten, Verzögerungsverarbeitung und Subscription-Verwaltung in RxJS. tap, delay, finalize, takeUntil, startWith, retry, repeat und mehr - mit Erklärungen zur Verwendung häufig verwendeter Operatoren und praktischen Mustern in der Praxis."
---

# Utility-Operatoren

RxJS-Utility-Operatoren sind eine Gruppe von Operatoren, die **nicht die Hauptzwecke der Datenverarbeitung wie Transformation oder Filterung erfüllen, sondern Hilfsprozesse des Streams (Seiteneffekte, Zustandssteuerung, UI-Anpassung usw.) übernehmen**.

Diese Seite kategorisiert Operatoren nach Zweck und bietet eine Liste, in der Sie grundlegende Verwendungszwecke überprüfen können.
Für detaillierte Verwendung und praktische Beispiele siehe die jeweiligen Seiten oder den Abschnitt [Praktische Anwendungsfälle](./practical-use-cases.md).


## Operatorenliste (nach Zweck)

### ◾ Seiteneffekte und Zustandssteuerung

| Operator | Beschreibung | Häufig kombinierte Operatoren |
|--------------|------|------------------|
| [tap](./tap.md) | Führt Seiteneffekte ohne Wertänderung aus (Logging, UI-Updates usw.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Führt Bereinigung beim Stream-Ende aus | `tap`, `catchError` |


### ◾ Timing und Verzögerungssteuerung

| Operator | Beschreibung | Häufig kombinierte Operatoren |
|--------------|------|------------------|
| [delay](./delay.md) | Verzögert die Ausgabe jedes Wertes um die angegebene Zeit | `tap`, `concatMap` |
| [timeout](./timeout.md) | Gibt einen Fehler aus, wenn die Ausgabe eine bestimmte Zeit überschreitet | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | Beendet Subscription, wenn das angegebene Observable benachrichtigt | `interval`, `fromEvent` |


### ◾ Anfangswert, Wiederholung, Array-Konvertierung usw.

| Operator | Beschreibung | Häufig kombinierte Operatoren |
|--------------|------|------------------|
| [startWith](./startWith.md) | Gibt einen Anfangswert am Beginn des Streams aus | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Re-subscribt den gesamten Stream nach Abschluss | `tap`, `delay` |
| [retry](./retry.md) | Wiederholt bei Fehlern | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Gibt alle Stream-Werte als ein Array aus (bei Abschluss) | `concatMap`, `take` |


## Hinweise

- Unterschied zwischen `retry` und `repeat`:
  - `retry`: **Wiederholt bei Fehler**
  - `repeat`: **Wiederholt bei normalem Abschluss**
- `toArray` gibt keine Werte aus, solange es nicht abgeschlossen ist, daher ist die Verwendung mit `take()` usw. üblich.
