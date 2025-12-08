---
description: "Ein Lernhandbuch zum systematischen Erlernen von RxJS in einer TypeScript-Umgebung. Es bietet schrittweise, praktische Erklärungen, die alles von Observable-Grundlagen bis hin zu Subjects, verschiedenen Operatoren, Fehlerbehandlung, Schedulern und Testtechniken abdecken."
---

# Anleitung

Dieser Leitfaden hilft Ihnen beim systematischen Erlernen von RxJS in einer TypeScript-Umgebung.
Indem Sie die folgenden Abschnitte der Reihe nach durchgehen, können Sie ein strukturiertes Verständnis von RxJS erlangen, von den Grundlagen bis zu fortgeschrittenen Konzepten.

## Inhaltsverzeichnis

### 1. RxJS Einführung
- [Erste Schritte](/de/guide/introduction)
- [Einrichten Ihrer Lernumgebung](/de/guide/starter-kid)
- [Was ist RxJS?](/de/guide/basics/what-is-rxjs)
- [Was ist ein Stream?](/de/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/de/guide/basics/promise-vs-rxjs)

### 2. Grundlagen von Observable
- [Was ist eine Observable?](/de/guide/observables/what-is-observable)
- [Wie man eine Observable erstellt](/de/guide/observables/creation)
- [Streaming-Ereignisse](/de/guide/observables/events)
- [Ereignis-Liste](/de/guide/observables/events-list)
- [Observer vs Subscriber](/de/guide/observables/observer-vs-subscriber)
- [Lebenszyklus von Observablen](/de/guide/observables/observable-lifecycle)
- [Kalte Observablen und heiße Observablen](/de/guide/observables/cold-and-hot-observables)

### 3. Erstellungsfunktionen
- [Was sind Erstellungsfunktionen?](/de/guide/creation-functions/)
- [Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Funktionen zur Schleifenbildung](/de/guide/creation-functions/loop/) - range, generate
- [HTTP-Kommunikationsfunktionen](/de/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Kombinationsfunktionen](/de/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Auswahl- und Partitionsfunktionen](/de/guide/creation-functions/selection/) - race, partition
- [Bedingte Funktionen](/de/guide/creation-functions/conditional/) - iif, defer
- [Kontroll-Funktionen](/de/guide/creation-functions/control/) - scheduled, using

### 4. Verstehen von Operatoren
- [Überblick über Operatoren](/de/guide/operators/)
- [Pipeline-Konzepte](/de/guide/operators/pipeline)
- [Transformations-Operatoren](/de/guide/operators/transformation/) - map, scan, mergeMap, switchMap, etc.
- [Filterungs-Operatoren](/de/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, etc.
- [Kombinations-Operatoren](/de/guide/operators/combination/) - withLatestFrom, mergeWith, etc.
- [Dienstprogramm-Operatoren](/de/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Multicasting](/de/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subjekte und Multicasting
- [Was ist ein Subject?](/de/guide/subjects/what-is-subject)
- [Arten von Subjects](/de/guide/subjects/types-of-subject)
- [Wie funktioniert Multicasting?](/de/guide/subjects/multicasting)
- [Anwendungsfälle für Subjects](/de/guide/subjects/use-cases)

### 6. Fehlerbehandlung
- [Strategien der Fehlerbehandlung](/de/guide/error-handling/strategies)
- [Zwei Orte für die Fehlerbehandlung](/de/guide/error-handling/error-handling-locations)
- [Integration von try-catch in RxJS](/de/guide/error-handling/try-catch-integration)
- [retry und catchError](/de/guide/error-handling/retry-catch)
- [finalize und complete](/de/guide/error-handling/finalize)

### 7. Verwendung von Schedulern
- [Steuerung der asynchronen Verarbeitung](/de/guide/schedulers/async-control)
- [Scheduler-Typen und Verwendung](/de/guide/schedulers/types)
- [Task- und Scheduler-Grundlagen](/de/guide/schedulers/task-and-scheduler-basics)

### 8. RxJS Debugging-Techniken
- [Überblick über Debugging-Techniken](/de/guide/debugging/)
- [Übliche Debugging-Szenarien](/de/guide/debugging/common-scenarios)
- [Benutzerdefinierte Debugging-Tools](/de/guide/debugging/custom-tools)
- [Performance-Debugging](/de/guide/debugging/performance)

### 9. Test-Techniken
- [RxJS-Einheitstests](/de/guide/testing/unit-tests)
- [TestScheduler verwenden](/de/guide/testing/test-scheduler)
- [Marble-Tests](/de/guide/testing/marble-testing)

### 10. RxJS Anti-Patterns Sammlung
- [Was sind Anti-Patterns?](/de/guide/anti-patterns/)
- [Häufige Fehler und Lösungen](/de/guide/anti-patterns/common-mistakes)
- [Flag-Wildwuchs](/de/guide/anti-patterns/flag-management)
- [Verschachtelte if-Anweisungen im Abonnement](/de/guide/anti-patterns/subscribe-if-hell)
- [Vermischung von Promises und Observables](/de/guide/anti-patterns/promise-observable-mixing)
- [Einzeiler-Hölle](/de/guide/anti-patterns/one-liner-hell)
- [Checkliste zur Vermeidung von Anti-Patterns](/de/guide/anti-patterns/checklist)

### 11. Überwindung von RxJS-Schwierigkeiten
- [Warum RxJS schwierig ist](/de/guide/overcoming-difficulties/)
- [Hürde des konzeptionellen Verständnisses](/de/guide/overcoming-difficulties/conceptual-understanding)
- [Die Hürde des Lebenszyklusmanagements](/de/guide/overcoming-difficulties/lifecycle-management)
- [Dilemmata bei der Auswahl von Operatoren](/de/guide/overcoming-difficulties/operator-selection)
- [Verstehen von Timing und Reihenfolge](/de/guide/overcoming-difficulties/timing-and-order)
- [Schwierigkeiten bei der Zustandsverwaltung](/de/guide/overcoming-difficulties/state-and-sharing)
- [Kombinieren mehrerer Streams](/de/guide/overcoming-difficulties/stream-combination)
- [Debugging-Herausforderungen](/de/guide/overcoming-difficulties/debugging-guide)

### 13. Praktische Patterns-Sammlung
- [Überblick über Practical Patterns](/de/guide/practical-patterns/)
- [UI-Ereignisbehandlung](/de/guide/practical-patterns/ui-events)
- [API-Aufrufe](/de/guide/practical-patterns/api-calls)
- [Formularverarbeitung](/de/guide/practical-patterns/form-handling)
- [Erweiterte Formularmuster](/de/guide/practical-patterns/advanced-form-patterns)
- [Datenverarbeitung in Echtzeit](/de/guide/practical-patterns/real-time-data)
- [Caching-Strategien](/de/guide/practical-patterns/caching-strategies)
- [Muster für die Fehlerbehandlung](/de/guide/practical-patterns/error-handling-patterns)
- [Bedingte Verzweigung in Abonnements](/de/guide/practical-patterns/subscribe-branching)

### Anhang
- [Anhangsübersicht](/de/guide/appendix/)
- [Eingebettete Entwicklung und reaktive Programmierung](/de/guide/appendix/embedded-reactive-programming)
- [Reaktive Muster jenseits von RxJS](/de/guide/appendix/reactive-patterns-beyond-rxjs)
- [Karte der reaktiven Architektur](/de/guide/appendix/reactive-architecture-map)
- [Reaktive Programmierung überdacht](/de/guide/appendix/reactive-programming-reconsidered)
- [RxJS und Reactive Streams-Ökosystem](/de/guide/appendix/rxjs-and-reactive-streams-ecosystem)

---

> [!NOTE]
> Dieser Leitfaden ist so aufgebaut, dass er Ihr Verständnis von RxJS Schritt für Schritt und systematisch vertieft. Sie können bei Bedarf auf jeden Abschnitt verweisen.
