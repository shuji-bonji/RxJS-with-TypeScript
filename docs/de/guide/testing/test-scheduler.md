---
description: "TestScheduler für zeitbasierte RxJS-Tests mit virtueller Zeit. Marble-Notation, Cold/Hot Observables und TypeScript-Beispiele für debounceTime und delay."
---

# Testen mit TestScheduler

Der `TestScheduler` von RxJS ist ein leistungsstarkes Tool zum präzisen Testen zeitbasierter Operatoren. In diesem Kapitel wird die Verwendung von TestScheduler systematisch erklärt.

## Was ist TestScheduler?

Normalerweise funktionieren Observables zeitabhängig. Zum Beispiel warten Operatoren wie `delay()` oder `debounceTime()` eine bestimmte Zeit.
In Tests ist es ineffizient, tatsächlich zu warten, daher verwendet `TestScheduler` virtuelle Zeit, um Tests sofort durchzuführen.

> [!TIP]
> TestScheduler verwendet "virtuelle Zeit", sodass kein Warten in Echtzeit erforderlich ist.

## Grundlegende Struktur von TestScheduler

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
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Erstellt ein Cold Observable, bei dem der Stream bei jeder Subscription unabhängig beginnt
- `hot()`: Erstellt ein Hot Observable, bei dem der Stream bereits läuft
- `expectObservable()`: Verifiziert die Ausgabe des Observable mit Marble-Notation

## Cold Observable und Hot Observable

|Typ|Merkmal|Verwendungszweck|
|:---|:---|:---|
|Cold Observable|Streamt Daten bei jeder Subscription von Anfang an|HTTP-Requests usw.|
|Hot Observable|Der Datenfluss hat bereits begonnen und wird mit Subscribern geteilt|Benutzerereignisse, WebSocket usw.|

## Grundlagen der Marble-Notation

Marble-Notation ist eine Methode, den Zeitverlauf von Observables als Zeichenkette darzustellen.

|Symbol|Bedeutung|
|:---|:---|
|`-`|Zeitverlauf (1 Frame)|
|`a`, `b`, `c`|Emittierte Werte|
|`|`|Abschluss|
|`#`|Fehler|
|`() `|Mehrere Werte gleichzeitig emittieren (mehrere Ereignisse)|

#### Beispiel

```
--a--b--c|    // Nach 2 Frames a, dann b, c, Abschluss
```

## Testbeispiel mit virtueller Zeit

### Test von debounceTime

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Test mit virtueller Zeit', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test des debounceTime-Operators', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ← Dies hier!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```

## Testen der Fehlerbehandlung

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Test der Fehlerbehandlung', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Fehler mit catchError abfangen', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```

## Zusammenfassung

- Mit TestScheduler können Tests ohne Warten in Echtzeit durchgeführt werden
- Verstehen und unterscheiden Sie Cold/Hot Observables
- Nutzen Sie Marble-Notation, um den Zeitverlauf zu visualisieren
- Auch komplexe asynchrone Streams können präzise getestet werden

> [!NEXT]
> Als Nächstes lernen Sie über fortgeschrittenere Marble-Tests (Anpassung von Marble-Strings und Kombinationen mehrerer Streams).
