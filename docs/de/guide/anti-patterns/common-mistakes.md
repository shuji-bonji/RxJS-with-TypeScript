---
description: "15 häufige Fehler bei der Verwendung von RxJS mit TypeScript und ihre Lösungen, detailliert erklärt mit echten Codebeispielen. Ein praktischer Leitfaden zur Vermeidung von Problemen, die in der Praxis häufig auftreten, wie öffentliche Subjects, verschachtelte subscribes, vergessene Abmeldungen und unsachgemäße Fehlerbehandlung."
---

# Häufige Fehler und Gegenmaßnahmen

Auf dieser Seite werden 15 häufige Anti-Patterns bei der Verwendung von RxJS mit TypeScript und ihre jeweiligen Lösungen ausführlich erklärt.

## Inhaltsverzeichnis

1. [Öffentliche Veröffentlichung von Subjects](#1-öffentliche-veröffentlichung-von-subjects)
2. [Verschachtelte subscribe (Callback-Hölle)](#2-verschachtelte-subscribe-callback-hölle)
3. [Vergessenes unsubscribe (Speicherleck)](#3-vergessenes-unsubscribe-speicherleck)
4. [Missbrauch von shareReplay](#4-missbrauch-von-sharereplay)
5. [Nebenwirkungen in map](#5-nebenwirkungen-in-map)
6. [Ignorieren des Unterschieds zwischen Cold/Hot Observable](#6-ignorieren-des-unterschieds-zwischen-cold-hot-observable)
7. [Unangemessene Vermischung von Promise und Observable](#7-unangemessene-vermischung-von-promise-und-observable)
8. [Ignorieren von Backpressure](#8-ignorieren-von-backpressure)
9. [Unterdrückung von Fehlern](#9-unterdrückung-von-fehlern)
10. [Lecks bei DOM-Event-Subscriptions](#10-lecks-bei-dom-event-subscriptions)
11. [Mangel an Typsicherheit (übermäßige Verwendung von any)](#11-mangel-an-typsicherheit-übermäßige-verwendung-von-any)
12. [Ungeeignete Operator-Auswahl](#12-ungeeignete-operator-auswahl)
13. [Übermäßige Komplexität](#13-übermäßige-komplexität)
14. [Zustandsänderung innerhalb von subscribe](#14-zustandsänderung-innerhalb-von-subscribe)
15. [Fehlen von Tests](#15-fehlen-von-tests)


## 1. Öffentliche Veröffentlichung von Subjects

### Problem

Wenn `Subject` direkt veröffentlicht wird, kann `next()` von außen aufgerufen werden, was die Zustandsverwaltung unvorhersehbar macht.

### ❌ Schlechtes Beispiel

```ts
import { Subject } from 'rxjs';

// Subject direkt exportieren
export const cartChanged$ = new Subject<void>();

// Jeder kann next() von einer anderen Datei aus aufrufen
cartChanged$.next(); // Kann zu unerwarteten Zeitpunkten aufgerufen werden
```

### ✅ Gutes Beispiel

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Als schreibgeschütztes Observable veröffentlichen
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // Zustandsänderungen nur über dedizierte Methoden steuern
  add(item: string): void {
    this._items$.next([...this._items$.value, item]);
  }

  remove(item: string): void {
    this._items$.next(
      this._items$.value.filter(i => i !== item)
    );
  }
}

export const cartStore = new CartStore();
```

### Erklärung

- Mit `asObservable()` in ein schreibgeschütztes `Observable` konvertieren
- Zustandsänderungen nur über dedizierte Methoden möglich machen
- Verbessert die Nachverfolgbarkeit von Änderungen und erleichtert das Debugging


## 2. Verschachtelte subscribe (Callback-Hölle)

### Problem

Wenn `subscribe` innerhalb von `subscribe` aufgerufen wird, führt dies zur Callback-Hölle und macht Fehlerbehandlung und Abbruchverarbeitung komplex.

### ❌ Schlechtes Beispiel

```ts
import { of } from 'rxjs';

// API-Aufruf-Simulation
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Verschachtelte subscribe
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ Gutes Beispiel

```ts
import { of } from 'rxjs';
import { switchMap } from 'rxjs';

function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
};


// Mit höheren Operatoren flach machen
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### Erklärung

- Verwendung von höheren Operatoren wie `switchMap`, `mergeMap`, `concatMap`, `exhaustMap`
- Fehlerbehandlung an einem Ort möglich
- Abmeldung nur einmal erforderlich
- Verbesserte Code-Lesbarkeit


## 3. Vergessenes unsubscribe (Speicherleck)

### Problem

Wenn unendliche Streams (wie Event-Listener) nicht abgemeldet werden, tritt ein Speicherleck auf.

### ❌ Schlechtes Beispiel

```ts
import { fromEvent } from 'rxjs';

// Bei der Initialisierung einer Komponente
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // Subscription wird nicht abgemeldet!
}

// Event-Listener bleibt auch nach Zerstörung der Komponente bestehen
```

### ✅ Gutes Beispiel

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('cleanup'))
    ).subscribe(() => {
      console.log('resized');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ Weiteres gutes Beispiel (mit Subscription)

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('resized');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### Erklärung

- Das `takeUntil`-Pattern wird empfohlen (deklarativ und klar)
- Manuelle Verwaltung mit `Subscription` ist ebenfalls wirksam
- Immer Abmeldung bei Zerstörung der Komponente durchführen


## 4. Missbrauch von shareReplay

### Problem

Wenn `shareReplay` ohne Verständnis seiner Funktionsweise verwendet wird, können alte Daten wiedergegeben werden oder Speicherlecks auftreten.

### ❌ Schlechtes Beispiel

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Unbegrenzte Puffergröße setzen
const shared$ = interval(1000).pipe(
  shareReplay() // Standard ist unbegrenzter Puffer
);

// Werte bleiben im Speicher, auch wenn keine Abonnenten mehr vorhanden sind
```

### ✅ Gutes Beispiel

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Puffergröße und Referenzzählung explizit angeben
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Ressourcen freigeben, wenn keine Abonnenten mehr vorhanden sind
  })
);
```

### Erklärung

- `bufferSize` explizit angeben (normalerweise 1)
- `refCount: true` für automatische Freigabe bei fehlenden Abonnenten
- Für HTTP-Requests und andere abgeschlossene Streams ist `shareReplay({ bufferSize: 1, refCount: true })` sicher


## 5. Nebenwirkungen in map

### Problem

Wenn Zustand innerhalb des `map`-Operators geändert wird, führt dies zu unvorhersehbarem Verhalten.

### ❌ Schlechtes Beispiel

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // Nebenwirkung!
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter erhöht sich unerwartet
```

### ✅ Gutes Beispiel

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Nur reine Transformation
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Nebenwirkungen mit tap trennen
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// Zustandsakkumulation mit scan
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Erklärung

- `map` als reine Funktion verwenden
- Nebenwirkungen (Logging, API-Aufrufe usw.) in `tap` trennen
- Zustandsakkumulation mit `scan` oder `reduce`


## 6. Ignorieren des Unterschieds zwischen Cold/Hot Observable

### Problem

Wenn ohne Verständnis verwendet wird, ob ein Observable Cold oder Hot ist, führt dies zu doppelter Ausführung oder unerwartetem Verhalten.

### ❌ Schlechtes Beispiel

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - HTTP-Request wird bei jeder Subscription ausgeführt
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Request 2 (unnötige Duplizierung)
```

### ✅ Gutes Beispiel

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// In Hot Observable konvertieren und teilen
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Verwendet gecachtes Ergebnis
```

### Erklärung

- Cold Observable: Wird bei jeder Subscription ausgeführt (`of`, `from`, `fromEvent`, `ajax` usw.)
- Hot Observable: Wird unabhängig von Subscriptions ausgeführt (`Subject`, multicast-Observables usw.)
- Mit `share` / `shareReplay` kann Cold zu Hot konvertiert werden


## 7. Unangemessene Vermischung von Promise und Observable

### Problem

Wenn Promise und Observable nicht ordnungsgemäß konvertiert und vermischt werden, wird die Fehlerbehandlung und Abbruchverarbeitung unvollständig.

### ❌ Schlechtes Beispiel

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise direkt verwendet
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Verschachtelte Promise
    console.log(data, moreData);
  });
});
```

### ✅ Gutes Beispiel

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise zu Observable konvertieren und vereinheitlichen
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Erklärung

- Promise mit `from` zu Observable konvertieren
- Einheitliche Verarbeitung innerhalb der Observable-Pipeline
- Fehlerbehandlung und Abbruch werden einfach


## 8. Ignorieren von Backpressure

### Problem

Wenn hochfrequente Events ohne Kontrolle verarbeitet werden, sinkt die Leistung.

### ❌ Schlechtes Beispiel

```ts
import { fromEvent } from 'rxjs';

// Eingabeereignisse direkt verarbeiten
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // API-Aufruf bei jeder Eingabe (Überlast)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ Gutes Beispiel

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Debouncing und Abbruch anwenden
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms warten
  distinctUntilChanged(), // Nur wenn sich Wert ändert
  switchMap(query => searchAPI(query)) // Alte Requests abbrechen
).subscribe(results => {
  console.log('Results:', results);
});
```

### Erklärung

- Mit `debounceTime` eine bestimmte Zeit warten
- Mit `throttleTime` maximale Frequenz begrenzen
- Mit `distinctUntilChanged` Duplikate ausschließen
- Mit `switchMap` alte Requests abbrechen


## 9. Unterdrückung von Fehlern

### Problem

Wenn Fehler nicht ordnungsgemäß behandelt werden, wird das Debugging schwierig und die Benutzererfahrung leidet.

### ❌ Schlechtes Beispiel

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Fehler ignorieren
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // Fehlerinformationen gehen verloren
).subscribe(data => {
  console.log(data); // null kommt an, Ursache unklar
});
```

### ✅ Gutes Beispiel

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

interface ApiResponse {
  data: unknown;
  error?: string;
}

ajax.getJSON<ApiResponse>('https://api.example.com/data').pipe(
  catchError(error => {
    console.error('API Error:', error);
    // Benutzer benachrichtigen
    showErrorToast('Fehler beim Abrufen der Daten');
    // Alternativen Wert mit Fehlerinformationen zurückgeben
    return of({ data: null, error: error.message } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Fallback mode due to:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### Erklärung

- Fehler protokollieren
- Feedback an Benutzer geben
- Alternativen Wert mit Fehlerinformationen zurückgeben
- Retry-Strategie in Betracht ziehen (`retry`, `retryWhen`)


## 10. Lecks bei DOM-Event-Subscriptions

### Problem

Wenn DOM-Event-Listener nicht ordnungsgemäß freigegeben werden, tritt ein Speicherleck auf.

### ❌ Schlechtes Beispiel

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Event-Listener registrieren
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // Keine Abmeldung
  }

  destroy(): void {
    this.button.remove();
    // Listener bleibt bestehen
  }
}
```

### ✅ Gutes Beispiel

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;
  private readonly destroy$ = new Subject<void>();

  constructor() {
    this.button = document.createElement('button');

    fromEvent(this.button, 'click').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => {
      console.log('clicked');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### Erklärung

- Zuverlässige Abmeldung mit dem `takeUntil`-Pattern
- `destroy$` bei Zerstörung der Komponente auslösen
- Listener vor Entfernung des DOM-Elements freigeben


## 11. Mangel an Typsicherheit (übermäßige Verwendung von any)

### Problem

Übermäßige Verwendung von `any` deaktiviert die Typprüfung von TypeScript und erhöht die Wahrscheinlichkeit von Laufzeitfehlern.

### ❌ Schlechtes Beispiel

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Typprüfung funktioniert nicht
fetchUser().pipe(
  map(user => user.naem) // Tippfehler! Wird erst zur Laufzeit bemerkt
).subscribe(console.log);
```

### ✅ Gutes Beispiel

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

interface User {
  name: string;
  age: number;
}

function fetchUser(): Observable<User> {
  return new Observable<User>(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Typprüfung funktioniert
fetchUser().pipe(
  map(user => user.name) // Fehler wird zur Kompilierzeit erkannt
).subscribe(console.log);
```

### Erklärung

- Interfaces oder Typ-Aliase definieren
- Typparameter von `Observable<T>` explizit angeben
- TypeScript-Typinferenz maximal nutzen


## 12. Ungeeignete Operator-Auswahl

### Problem

Die Verwendung ungeeigneter Operatoren führt zu Ineffizienz oder unerwartetem Verhalten.

### ❌ Schlechtes Beispiel

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Suche bei jedem Button-Klick (alte Requests werden nicht abgebrochen)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Gutes Beispiel

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Nur neuesten Request verarbeiten (alte Requests werden automatisch abgebrochen)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Verwendung der wichtigsten höheren Operatoren

| Operator | Verwendung |
|---|---|
| `switchMap` | Nur neuesten Stream verarbeiten (Suche, Autocomplete) |
| `mergeMap` | Parallele Verarbeitung (Reihenfolge unwichtig) |
| `concatMap` | Sequentielle Verarbeitung (Reihenfolge wichtig) |
| `exhaustMap` | Neue Eingaben während der Ausführung ignorieren (Button-Klick-Prävention) |

### Erklärung

- Verhalten jedes Operators verstehen
- Geeignete Auswahl je nach Anwendungsfall
- Details siehe [Transformationsoperatoren](/de/guide/operators/transformation/)


## 13. Übermäßige Komplexität

### Problem

Einfache Prozesse werden mit RxJS übermäßig kompliziert gemacht.

### ❌ Schlechtes Beispiel

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Einfache Array-Transformation mit RxJS kompliziert machen
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Gutes Beispiel

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Array-Verarbeitung ist mit normalem JavaScript ausreichend
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// RxJS für asynchrone, ereignisgesteuerte Verarbeitung verwenden
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Erklärung

- RxJS für asynchrone Verarbeitung und Event-Streams verwenden
- Synchrone Array-Verarbeitung ist mit normalem JavaScript ausreichend
- Balance zwischen Komplexität und Nutzen berücksichtigen


## 14. Zustandsänderung innerhalb von subscribe

### Problem

Wenn Zustand direkt innerhalb von `subscribe` geändert wird, wird das Testen schwierig und es wird zur Ursache von Bugs.

### ❌ Schlechtes Beispiel

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // Zustandsänderung innerhalb von subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ Gutes Beispiel

```ts
import { interval, BehaviorSubject } from 'rxjs';
import { scan, tap } from 'rxjs';

class Counter {
  private readonly count$ = new BehaviorSubject<number>(0);

  start(): void {
    interval(1000).pipe(
      scan(acc => acc + 1, 0),
      tap(count => this.count$.next(count))
    ).subscribe();

    // UI abonniert count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### Erklärung

- Zustand mit `BehaviorSubject` oder `scan` verwalten
- `subscribe` nur als Trigger verwenden
- Testbares und reaktives Design


## 15. Fehlen von Tests

### Problem

Wenn RxJS-Code ohne Tests in die Produktionsumgebung gebracht wird, treten leicht Regressionen auf.

### ❌ Schlechtes Beispiel

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Ohne Tests bereitstellen
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Gutes Beispiel

```ts
import { TestScheduler } from 'rxjs/testing';
import { getEvenNumbers } from './numbers';

describe('getEvenNumbers', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should emit only even numbers doubled', () => {
    scheduler.run(({ expectObservable }) => {
      const expected = '1s 0 1s 4 1s 8';
      expectObservable(getEvenNumbers()).toBe(expected);
    });
  });
});
```

### Erklärung

- Marble-Tests mit `TestScheduler` durchführen
- Asynchrone Verarbeitung kann synchron getestet werden
- Details siehe [Testmethoden](/de/guide/testing/unit-tests)


## Zusammenfassung

Durch das Verständnis und Vermeiden dieser 15 Anti-Patterns können Sie robusteren und wartbareren RxJS-Code schreiben.

## Referenzen

Diese Anti-Pattern-Sammlung wurde unter Bezugnahme auf folgende vertrauenswürdige Quellen erstellt.

### Offizielle Dokumentation und Repositories
- **[RxJS Offizielle Dokumentation](https://rxjs.dev/)** - Offizielle Referenz für Operatoren und API
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Diskussion über shareReplay-Speicherleck-Problem

### Anti-Patterns und Best Practices
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (Mai 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### Zusätzliche Ressourcen
- **[Learn RxJS](https://www.learnrxjs.io/)** - Praktischer Leitfaden für Operatoren und Patterns
- **[RxJS Marbles](https://rxmarbles.com/)** - Visuelles Verständnis von Operatoren

## Verwendung bei Code-Reviews

Überprüfen Sie, ob Ihr Code Anti-Patterns enthält.

👉 **[Anti-Pattern-Vermeidungs-Checkliste](./checklist)** - Überprüfen Sie Ihren Code mit 15 Prüfpunkten

Von jedem Prüfpunkt aus können Sie direkt zu den Details des entsprechenden Anti-Patterns auf dieser Seite springen.

## Nächste Schritte

- **[Fehlerbehandlung](/de/guide/error-handling/strategies)** - Lernen Sie detailliertere Fehlerbehandlungsstrategien
- **[Testmethoden](/de/guide/testing/unit-tests)** - Erwerben Sie effektive Testmethoden für RxJS-Code
- **[Verständnis von Operatoren](/de/guide/operators/)** - Lernen Sie die detaillierte Verwendung jedes Operators

Integrieren Sie diese Best Practices in Ihr tägliches Coding und schreiben Sie hochwertigen RxJS-Code!
