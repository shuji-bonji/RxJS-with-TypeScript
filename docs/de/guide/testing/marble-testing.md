---
description: "Marble-Tests für asynchrone RxJS-Streams mit visueller Darstellung. Cold vs Hot Observables, Marble-Notation und TestScheduler mit TypeScript-Beispielen."
---

# Einführung in Marble-Tests

In RxJS gibt es eine Methode namens "Marble-Tests", mit der das Verhalten asynchroner Streams **visuell dargestellt** und getestet werden kann.

Hier lernen Sie von den Grundlagen der Marble-Tests bis hin zu einfachen Beispielen.

## Was ist Marble-Notation?

Marble-Notation ist eine Methode, **Zeitverlauf und Ereignisauftreten** als Zeichenkette darzustellen.

### Grundregeln

| Symbol | Bedeutung |
|:----|:----|
| `-` | Zeitverlauf (1 Frame vorwärts) |
| `a`, `b`, `c` | Emittierter Wert (beliebiges Zeichen) |
| `|` | Abschluss (complete) |
| `#` | Fehler (error) |

Zum Beispiel

```text
--a-b--c-|
```
Dies bedeutet:
- Nach 2 Frames wird `a` emittiert
- 1 Frame später `b`
- 2 Frames später `c`
- Weitere 1 Frame später Abschluss

## Unterschied zwischen Cold und Hot

### Cold Observable

Cold Observables werden "bei jeder Subscription von Anfang an abgespielt".

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestScheduler Grundlagen', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('einfacher Test', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Hot Observables sind "bereits laufende" Streams.
Bei einer Subscription ab der Mitte werden nur Werte ab diesem Zeitpunkt empfangen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestScheduler Grundlagen', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('einfacher Test', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## Einfaches Beispiel für Marble-Tests

Zum Beispiel beim Testen des `debounceTime`-Operators

```ts
import { debounceTime } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestScheduler Grundlagen', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('einfacher Test', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Hier wird verifiziert, dass nur das zuletzt emittierte `c` ausgegeben wird.

## Hinweise

- Ein Zeichen in der Marble-Notation repräsentiert standardmäßig **1 Frame (10ms)** (kann je nach Umgebung eingestellt werden)
- Zeitabhängige Operatoren wie `debounceTime`, `delay`, `interval` **funktionieren gut mit Marble-Tests**
- Mit `expectObservable` wird die Ausgabe des Streams verifiziert
- `expectSubscriptions` ist eine fortgeschrittene Funktion zur Verifizierung des Subscription-Timings, wird aber hier nicht behandelt

## Zusammenfassung

Marble-Tests sind eine sehr leistungsstarke Methode, um RxJS-Code-Tests **zu visualisieren und intuitiv** zu verstehen.

- **Beachten Sie den Unterschied zwischen Cold und Hot**
- **Stellen Sie Zeitverlauf und Ereignisse als Zeichenkette dar**
- **Auch komplexe asynchrone Streams ermöglichen klare Tests**

Beginnen Sie zunächst mit einfachen Marble-Tests zu üben!
