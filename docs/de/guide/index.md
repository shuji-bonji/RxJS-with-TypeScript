---
description: Leitfaden zum systematischen Erlernen von RxJS in TypeScript. Von Observable-Grundlagen bis zu Subjects, Operatoren, Fehlerbehandlung und Tests.
---

# Leitfaden

Ein Leitfaden zum systematischen Erlernen von RxJS in TypeScript-Umgebungen.
Durch das schrittweise Durcharbeiten der folgenden Abschnitte können Sie RxJS von den Grundlagen bis zu fortgeschrittenen Anwendungen systematisch verstehen.

## Inhaltsverzeichnis

### 1. Einführung in RxJS
- [Einführung](/de/guide/introduction)
- [Einrichtung der Lernumgebung](/de/guide/starter-kid.md)
- [Was ist RxJS?](/de/guide/basics/what-is-rxjs)
- [Was ist ein Stream?](/de/guide/basics/what-is-a-stream)
- [Unterschiede zwischen Promise und RxJS](/de/guide/basics/promise-vs-rxjs)

### 2. Observable-Grundlagen
- [Was ist ein Observable?](/de/guide/observables/what-is-observable)
- [Erstellen von Observables](/de/guide/observables/creation)
- [Ereignisse in Streams umwandeln](/de/guide/observables/events)
- [Ereignisse, die nicht mit fromEvent verwendet werden können](/de/guide/observables/events#cannot-used-fromEvent)
- [Ereignisliste](/de/guide/observables/events-list)
- [Lebenszyklus von Observables](/de/guide/observables/observable-lifecycle)
- [Cold Observables und Hot Observables](/de/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [Was sind Creation Functions?](/de/guide/creation-functions/)
- [Grundlegende Erstellung](/de/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Schleifengenerierung](/de/guide/creation-functions/loop/) - range, generate
- [HTTP-Kommunikation](/de/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Kombination](/de/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Auswahl/Partitionierung](/de/guide/creation-functions/selection/) - race, partition
- [Bedingte Verzweigung](/de/guide/creation-functions/conditional/) - iif, defer
- [Steuerung](/de/guide/creation-functions/control/) - scheduled, using

### 4. Operatoren verstehen
- [Übersicht über Operatoren](/de/guide/operators/)
- [Pipeline-Konzept](/de/guide/operators/pipeline)
- [Transformationsoperatoren](/de/guide/operators/transformation/) - map, scan, mergeMap, switchMap, buffer-Serie, window-Serie usw.
- [Filteroperatoren](/de/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinct usw.
- [Kombinationsoperatoren](/de/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, *All-Operatoren usw.
- [Utility-Operatoren](/de/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil usw.
- [Bedingte Operatoren](/de/guide/operators/conditional/) - defaultIfEmpty, every, isEmpty usw.
- [Multicasting](/de/guide/operators/multicasting/) - share, shareReplay usw.

### 5. Subject und Multicasting
- [Was ist ein Subject?](/de/guide/subjects/what-is-subject)
- [Arten von Subjects](/de/guide/subjects/types-of-subject)
- [Funktionsweise von Multicasting](/de/guide/subjects/multicasting)
- [Anwendungsfälle für Subjects](/de/guide/subjects/use-cases)

### 6. Fehlerbehandlung
- [Fehlerbehandlungsstrategien](/de/guide/error-handling/strategies)
- [Zwei Orte der Fehlerbehandlung](/de/guide/error-handling/error-handling-locations)
- [Integration von try-catch und RxJS](/de/guide/error-handling/try-catch-integration)
- [retry und catchError](/de/guide/error-handling/retry-catch)
- [finalize und complete](/de/guide/error-handling/finalize)

### 7. Nutzung von Schedulern
- [Steuerung asynchroner Verarbeitung](/de/guide/schedulers/async-control)
- [Arten und Verwendung von Schedulern](/de/guide/schedulers/types)
- [Zusatz: Grundlagen zu Tasks und Schedulern](/de/guide/schedulers/task-and-scheduler-basics)

### 8. Debugging-Techniken für RxJS
- [Übersicht über Debugging-Techniken](/de/guide/debugging/)
- [Häufige Debugging-Szenarien](/de/guide/debugging/common-scenarios)
- [Benutzerdefinierte Debugging-Tools](/de/guide/debugging/custom-tools)
- [Performance-Debugging](/de/guide/debugging/performance)

### 9. Testmethoden
- [Unit-Tests für RxJS](/de/guide/testing/unit-tests)
- [Verwendung von TestScheduler](/de/guide/testing/test-scheduler)
- [Marble-Testing](/de/guide/testing/marble-testing)

### 10. RxJS Anti-Pattern-Sammlung
- [Was sind Anti-Patterns?](/de/guide/anti-patterns/)
- [Häufige Fehler und Gegenmaßnahmen](/de/guide/anti-patterns/common-mistakes)
- [If-Statement-Verschachtelungshölle innerhalb von subscribe](/de/guide/anti-patterns/subscribe-if-hell)
- [Vermischung von Promise und Observable](/de/guide/anti-patterns/promise-observable-mixing)
- [One-Liner-Hölle und schrittweise Trennung](/de/guide/anti-patterns/one-liner-hell)
- [Checkliste zur Vermeidung von Anti-Patterns](/de/guide/anti-patterns/checklist)

### 11. Überwindung von RxJS-Schwierigkeiten
- [Warum ist RxJS schwierig?](/de/guide/overcoming-difficulties/)
- [Die Barriere des konzeptuellen Verständnisses](/de/guide/overcoming-difficulties/conceptual-understanding)
- [Die Barriere des Lebenszyklus-Managements](/de/guide/overcoming-difficulties/lifecycle-management)
- [Unsicherheit bei der Operatorauswahl](/de/guide/overcoming-difficulties/operator-selection)
- [Verständnis von Timing und Reihenfolge](/de/guide/overcoming-difficulties/timing-and-order)
- [Schwierigkeiten beim Zustandsmanagement](/de/guide/overcoming-difficulties/state-and-sharing)
- [Kombination mehrerer Streams](/de/guide/overcoming-difficulties/stream-combination)
- [Die Debugging-Barriere](/de/guide/overcoming-difficulties/debugging-guide)

### 13. Sammlung praktischer Muster
- [Übersicht über praktische Muster](/de/guide/practical-patterns/)
- [UI-Ereignisverarbeitung](/de/guide/practical-patterns/ui-events) - Klicks, Scroll, Drag & Drop usw.
- [API-Aufrufe](/de/guide/practical-patterns/api-calls) - HTTP-Kommunikation, parallele/serielle Verarbeitung, Fehlerbehandlung
- [Formularverarbeitung](/de/guide/practical-patterns/form-handling) - Echtzeit-Validierung, Auto-Save, Verknüpfung mehrerer Felder
- [Erweiterte Formularmuster](/de/guide/practical-patterns/advanced-form-patterns) - JSON Patch, Auto-Save großer Formulare, Undo/Redo, gemeinsame Bearbeitung
- [Echtzeit-Datenverarbeitung](/de/guide/practical-patterns/real-time-data) - WebSocket, SSE, Polling, Verbindungsverwaltung
- [Cache-Strategien](/de/guide/practical-patterns/caching-strategies) - Daten-Caching, TTL, Invalidierung, Offline-Unterstützung
- [Praktische Fehlerbehandlung](/de/guide/practical-patterns/error-handling-patterns) - API-Aufruffehler, Retry-Strategien, globale Fehlerbehandlung
- [Bedingte Verzweigung innerhalb von subscribe](/de/guide/practical-patterns/subscribe-branching) - Verzweigungen innerhalb von subscribe vermeiden, Verzweigungen innerhalb der Pipeline

### Anhang
- [Übersicht Anhang](/de/guide/appendix/)
- [Eingebettete Entwicklung und reaktive Programmierung](/de/guide/appendix/embedded-reactive-programming)
- [Reaktive Methoden außerhalb von ReactiveX](/de/guide/appendix/reactive-patterns-beyond-rxjs)
- [Gesamtkarte der reaktiven Architektur](/de/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> Dieser Leitfaden ist so strukturiert, dass Sie Ihr Verständnis von RxJS schrittweise und systematisch vertiefen können.
> Bitte konsultieren Sie die einzelnen Abschnitte nach Bedarf frei.
