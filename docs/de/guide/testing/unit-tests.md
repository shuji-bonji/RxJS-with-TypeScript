---
description: "Unit-Tests für RxJS mit TestScheduler, Marble-Tests und Mocks. Praktischer Leitfaden für synchrone, asynchrone und zeitbasierte Test-Strategien."
---

# Unit-Tests für RxJS

Code, der RxJS verwendet, enthält viele asynchrone Operationen und erfordert einen anderen Ansatz als herkömmliche Testmethoden. Dieser Leitfaden erklärt von grundlegenden Methoden bis hin zu fortgeschrittenen Techniken, wie man Code mit RxJS effektiv testet.

## Testen synchroner Observables

Beginnen wir mit dem einfachsten Fall: dem Testen von Observables, die synchron abgeschlossen werden.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Zu testende Funktion
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Grundlegende Observable-Tests', () => {
  it('verdoppelt Werte', () => {
    // Test-Observable
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Erwartetes Ergebnis
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Ausführung und Verifizierung
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Testen asynchroner Observables

Bei asynchronen Observables nutzen wir die asynchrone Unterstützung des Test-Frameworks.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Zu testende asynchrone Funktion
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Test asynchroner Observables', () => {
  it('empfängt asynchrone Werte in Reihenfolge', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];

    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Asynchrone Tests mit Promise-Konvertierung

Es gibt auch die Methode, Observables mit `firstValueFrom()` oder `lastValueFrom()` in Promises zu konvertieren und async/await von modernem JS/TS zu nutzen.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Zu testende Funktion
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Tests mit Promise-Konvertierung', () => {
  it('wartet auf verzögerte Verarbeitung vor Verifizierung', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Observable in Promise konvertieren
    const result = await lastValueFrom(result$);

    // Erwartetes Ergebnis
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Verwendung von TestScheduler

RxJS bietet einen speziellen Scheduler namens `TestScheduler`, mit dem zeitbasierte Operatoren effizient getestet werden können.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Verwendung von TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test von debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );

      const expected = '----------(d|)';

      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> Marble-Test-Notation
> Bei Verwendung von `TestScheduler` wird die Marble-Diagramm-Notation verwendet, um den Zeitverlauf darzustellen.

## Zeitmanipulation ermöglichen

Beim Testen von zeitabhängigem Code (delay, debounceTime usw.) wird `TestScheduler` verwendet, um die Zeit zu kontrollieren.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Zeitkontrolle', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test mit Zeitvorspulung', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Dauert eigentlich 3 Sekunden, wird aber in der Testumgebung sofort ausgeführt
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Testen der Fehlerbehandlung (TestScheduler-Version)

Es ist auch wichtig, das Verhalten von Observables bei Fehlerauftritt zu testen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Test der Fehlerbehandlung', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Observable meldet einen Fehler', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('Fehler mit catchError abfangen und durch Wert ersetzen', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## Marble-Tests

Für das Testen komplexer Streams verwenden wir Marble-Diagramme, um Testerwartungen intuitiv auszudrücken.

### Hot Observable vs Cold Observable

Mit TestScheduler können zwei Arten von Observables erstellt werden: hot und cold. Es ist wichtig, diesen Unterschied zu verstehen und entsprechend zu testen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Hot vs Cold Observable Test', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable erzeugt für jede Subscription einen unabhängigen Stream', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (unabhängig für jeden Subscriber)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Erste Subscription
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Zweite Subscription (beginnt von vorn)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable teilt den Stream zwischen Subscribers', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (geteilt zwischen Subscribers)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Verspätete Subscription (empfängt nur Werte ab Subscriptionsbeginn)
      // expectObservable(source, '-----^---').toBe('-----b--c|', { b: 2, c: 3 });
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Subscription von Anfang an (empfängt alle Werte)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Test eines tatsächlichen Subject als Hot Observable', () => {
    // Nicht-TestScheduler-Version
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Erster Subscriber
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Werte emittieren
    subject.next(1);
    subject.next(2);

    // Zweiter Subscriber (ab der Mitte)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Weitere Werte emittieren
    subject.next(3);
    subject.complete();

    // Verifizierung
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Nur Werte nach Subscriptionsbeginn

    // Aufräumen
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observables generieren Daten bei jeder Subscription unabhängig, während Hot Observables Daten teilen und verteilen.

## Verwendung von Mocks und Stubs

### Mocken von abhängigen Services

Beim Testen von Services, die RxJS verwenden, werden externe Abhängigkeiten häufig gemockt.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Zu testender Service
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Service-Test', () => {
  it('filtert nur aktive Benutzer', () => {
    // Mock-API-Service
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verifizierung
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Verwendung von Stubs

Stubs sind einfache Objekte, die externe Daten oder APIs simulieren, von denen der zu testende Code abhängt.
Sie eliminieren Abhängigkeiten von externen Ressourcen und ermöglichen unabhängige Tests.
Sie geben nur feste Werte zurück und haben keine interne Logik.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Zu testender Service
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('UserService Test', () => {
  it('gibt nur aktive Benutzer zurück', () => {
    // Stub erstellen
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Zu testender Service
    const userService = new UserService(stubApiService);

    // Ergebnis überprüfen
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Spionage auf Subscriptions

Man kann Spies verwenden, um zu verifizieren, dass Subscriptions korrekt durchgeführt werden.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Subscription-Test', () => {
  it('subscribed mit geeigneten Handlern', () => {
    const subject = new Subject();

    // Spies für Handler erstellen
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Subscription
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Werte emittieren
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verifizierung
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Best Practices

|Best Practice|Beschreibung|
|---|---|
|Single-Responsibility-Prinzip einhalten|Um testbaren Code zu schreiben, sollte jede Funktion oder Klasse eine einzige Verantwortung haben. Dies macht auch Tests einfacher.|
|Externe Abhängigkeiten mocken|Externe Abhängigkeiten wie HTTP-Requests oder Timer sollten gemockt werden, um in einer vorhersehbaren Umgebung zu testen.|
|Geeignete Techniken für asynchronen Code verwenden|Für asynchrone Tests sollte die geeignete Methode gewählt werden: TestScheduler, done()-Callback oder async/await.|
|Marble-Tests nutzen|Für das Testen komplexer Streams sollten Marble-Diagramme verwendet werden, um Testerwartungen intuitiv auszudrücken.|

## Zusammenfassung

Das Testen von RxJS-Code hat andere Aspekte als herkömmlicher JavaScript-Code, wie die synchrone/asynchrone Natur und zeitabhängiges Verhalten. Durch die Auswahl geeigneter Testmethoden können Sie mit Zuversicht hochwertigen reaktiven Code entwickeln. Beachten Sie insbesondere Folgendes:

- Einfache Subscription-Tests für synchrone Observables
- TestScheduler oder Promise-Konvertierung für asynchrone Operationen
- Marble-Tests für zeitabhängigen Code
- Mocken externer Abhängigkeiten für eine isolierte Testumgebung
- Befolgen Sie das Single-Responsibility-Prinzip und entwerfen Sie testbaren Code

## Verwandte Abschnitte

- **[Häufige Fehler und Gegenmaßnahmen](/de/guide/anti-patterns/common-mistakes#15-mangel-an-tests)** - Anti-Patterns im Zusammenhang mit Tests überprüfen
- **[Verwendung von TestScheduler](/de/guide/testing/test-scheduler)** - Detailliertere Verwendung von TestScheduler
- **[Marble-Tests](/de/guide/testing/marble-testing)** - Fortgeschrittene Marble-Test-Techniken
