---
description: "Gedetailleerde uitleg van 15 veelvoorkomende fouten bij het gebruik van RxJS met TypeScript, met daadwerkelijke codevoorbeelden. Biedt praktische gids om problemen te voorkomen die vaak voorkomen in de praktijk, zoals extern blootgestelde Subject, geneste subscribe, vergeten unsubscribe, en ontoereikende foutafhandeling."
---

# Veelvoorkomende Fouten en Oplossingen

Deze pagina legt in detail 15 anti-patronen uit die vaak worden gezien bij het gebruik van RxJS met TypeScript, samen met hun oplossingen.

## Inhoudsopgave

1. [Subject Extern Blootgesteld](#1-subject-extern-blootgesteld)
2. [Geneste subscribe (Callback Hell)](#2-geneste-subscribe-callback-hell)
3. [unsubscribe Vergeten (Geheugenlek)](#3-unsubscribe-vergeten-geheugenlek)
4. [shareReplay Misbruik](#4-sharereplay-misbruik)
5. [Bijwerkingen in map](#5-bijwerkingen-in-map)
6. [Cold/Hot Observable Verschil Negeren](#6-cold-hot-observable-verschil-negeren)
7. [Promise en Observable Ongepast Vermengen](#7-promise-en-observable-ongepast-vermengen)
8. [Backpressure Negeren](#8-backpressure-negeren)
9. [Fouten Onderdrukken](#9-fouten-onderdrukken)
10. [DOM Event Subscription Lek](#10-dom-event-subscription-lek)
11. [Gebrek aan Type-veiligheid (Overmatig Gebruik van any)](#11-gebrek-aan-type-veiligheid-overmatig-gebruik-van-any)
12. [Onjuiste Operator Selectie](#12-onjuiste-operator-selectie)
13. [Overmatige Complexiteit](#13-overmatige-complexiteit)
14. [Status Wijziging in subscribe](#14-status-wijziging-in-subscribe)
15. [Gebrek aan Tests](#15-gebrek-aan-tests)


## 1. Subject Extern Blootgesteld

### Probleem

Als je een `Subject` direct blootstelt, kan externe code `next()` aanroepen, waardoor statusbeheer onvoorspelbaar wordt.

### ❌ Slecht Voorbeeld

```ts
import { Subject } from 'rxjs';

// Subject direct exporteren
export const cartChanged$ = new Subject<void>();

// Iedereen kan next() aanroepen vanuit een ander bestand
cartChanged$.next(); // Kan op onverwachte momenten worden aangeroepen
```

### ✅ Goed Voorbeeld

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Publiceren als alleen-lezen Observable
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // Statuswijzigingen beheren via toegewijde methoden
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

### Uitleg

- Converteer naar alleen-lezen `Observable` met `asObservable()`
- Maak statuswijzigingen alleen mogelijk via toegewijde methoden
- Verbetert traceerbaarheid van wijzigingen en maakt debuggen eenvoudiger


## 2. Geneste subscribe (Callback Hell)

### Probleem

Als je nog een `subscribe` aanroept binnen een `subscribe`, val je in callback hell en worden foutafhandeling en annuleringsverwerking complex.

### ❌ Slecht Voorbeeld

```ts
import { of } from 'rxjs';

// Simulatie van API aanroepen
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Geneste subscribe
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('klaar', result);
    });
  });
});
```

### ✅ Goed Voorbeeld

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


// Afvlakken met higher-order operators
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('klaar', result);
});
```

### Uitleg

- Gebruik higher-order operators zoals `switchMap`, `mergeMap`, `concatMap`
- Foutafhandeling mogelijk op één plaats
- Opzeggen van abonnement gebeurt ook in één keer
- Verbeterde leesbaarheid van code


## 3. unsubscribe Vergeten (Geheugenlek)

### Probleem

Als je abonnementen op oneindige streams (zoals event listeners) niet opzegt, ontstaan geheugenlekken.

### ❌ Slecht Voorbeeld

```ts
import { fromEvent } from 'rxjs';

// Bij initialisatie van component
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('vergroot/verkleind');
  });
  // Abonnement niet opgezegd!
}

// Event listener blijft bestaan zelfs als component wordt vernietigd
```

### ✅ Goed Voorbeeld

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('opruimen'))
    ).subscribe(() => {
      console.log('vergroot/verkleind');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ Ander Goed Voorbeeld (Gebruik van Subscription)

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('vergroot/verkleind');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### Uitleg

- `takeUntil` patroon wordt aanbevolen (declaratief en duidelijk)
- Handmatig beheer met `Subscription` is ook effectief
- Voer altijd opzeggen van abonnement uit bij vernietiging van component


## 4. shareReplay Misbruik

### Probleem

Bij gebruik van `shareReplay` zonder het gedrag te begrijpen, kunnen oude data worden gereproduceerd of geheugenlekken ontstaan.

### ❌ Slecht Voorbeeld

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Onbeperkte buffergrootte instellen
const shared$ = interval(1000).pipe(
  shareReplay() // Standaard is onbeperkte buffer
);

// Waarden blijven in geheugen zelfs als er geen abonnees meer zijn
```

### ✅ Goed Voorbeeld

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Buffergrootte en reference counting expliciet specificeren
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Geef resources vrij als er geen abonnees meer zijn
  })
);
```

### Uitleg

- Specificeer `bufferSize` expliciet (meestal 1)
- Met `refCount: true` automatisch vrijgeven als er geen abonnees meer zijn
- Voor streams die voltooien zoals HTTP requests is `shareReplay({ bufferSize: 1, refCount: true })` veilig


## 5. Bijwerkingen in map

### Probleem

Status wijzigen binnen de `map` operator veroorzaakt onvoorspelbaar gedrag.

### ❌ Slecht Voorbeeld

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // Bijwerking!
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter neemt onverwacht toe
```

### ✅ Goed Voorbeeld

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Alleen pure transformatie
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Schei bijwerkingen af met tap
const withLogging$ = source$.pipe(
  tap(value => console.log('Verwerken:', value))
);

// Gebruik scan voor accumulatie van status
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Uitleg

- Gebruik `map` als pure functie
- Schei bijwerkingen (logs, API aanroepen, etc.) af in `tap`
- Gebruik `scan` of `reduce` voor statusaccumulatie


## 6. Cold/Hot Observable Verschil Negeren

### Probleem

Gebruiken zonder te begrijpen of een Observable Cold of Hot is, veroorzaakt dubbele uitvoering of onverwacht gedrag.

### ❌ Slecht Voorbeeld

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - HTTP request wordt uitgevoerd bij elk abonnement
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Request 2 (onnodige duplicatie)
```

### ✅ Goed Voorbeeld

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Converteer naar Hot Observable en deel
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Gebruikt gecachete resultaat
```

### Uitleg

- Cold Observable: Wordt uitgevoerd bij elk abonnement (`of`, `from`, `fromEvent`, `ajax`, etc.)
- Hot Observable: Wordt uitgevoerd onafhankelijk van abonnementen (`Subject`, gemulticast Observable, etc.)
- Converteer Cold naar Hot met `share` / `shareReplay`


## 7. Promise en Observable Ongepast Vermengen

### Probleem

Promise en Observable zonder correcte conversie vermengen resulteert in onvolledige foutafhandeling of annuleringsverwerking.

### ❌ Slecht Voorbeeld

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise direct gebruiken
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Geneste Promise
    console.log(data, moreData);
  });
});
```

### ✅ Goed Voorbeeld

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise converteren naar Observable voor uniforme verwerking
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Uitleg

- Converteer Promise naar Observable met `from`
- Verwerk uniform binnen Observable pipeline
- Foutafhandeling en annulering worden eenvoudiger


## 8. Backpressure Negeren

### Probleem

Events met hoge frequentie verwerken zonder controle vermindert de prestaties.

### ❌ Slecht Voorbeeld

```ts
import { fromEvent } from 'rxjs';

// Invoer events direct verwerken
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // API aanroep bij elke invoer (overbelasting)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Zoeken naar:', query);
}
```

### ✅ Goed Voorbeeld

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Debounce en annulering toepassen
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms wachten
  distinctUntilChanged(), // Alleen wanneer waarde verandert
  switchMap(query => searchAPI(query)) // Annuleer oude requests
).subscribe(results => {
  console.log('Resultaten:', results);
});
```

### Uitleg

- Wacht een bepaalde tijd met `debounceTime`
- Beperk maximale frequentie met `throttleTime`
- Elimineer duplicaten met `distinctUntilChanged`
- Annuleer oude requests met `switchMap`


## 9. Fouten Onderdrukken

### Probleem

Fouten niet correct afhandelen maakt debuggen moeilijk en vermindert de gebruikerservaring.

### ❌ Slecht Voorbeeld

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Fouten negeren
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // Foutinformatie gaat verloren
).subscribe(data => {
  console.log(data); // null komt aan maar oorzaak onbekend
});
```

### ✅ Goed Voorbeeld

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
    console.error('API Fout:', error);
    // Informeer gebruiker
    showErrorToast('Ophalen van data is mislukt');
    // Retourneer alternatieve waarde met foutinformatie
    return of({ data: null, error: error.message } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Fallback modus vanwege:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### Uitleg

- Log fouten
- Geef feedback aan gebruikers
- Retourneer alternatieve waarde met foutinformatie
- Overweeg retry strategie (`retry`, `retryWhen`)


## 10. DOM Event Subscription Lek

### Probleem

DOM event listeners niet correct vrijgeven veroorzaakt geheugenlekken.

### ❌ Slecht Voorbeeld

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Event listener registreren
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('geklikt');
    });

    // Abonnement niet opgezegd
  }

  destroy(): void {
    this.button.remove();
    // Listener blijft bestaan
  }
}
```

### ✅ Goed Voorbeeld

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
      console.log('geklikt');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### Uitleg

- Zeg abonnement betrouwbaar op met `takeUntil` patroon
- Activeer `destroy$` bij vernietiging van component
- Geef listener vrij voor verwijderen van DOM element


## 11. Gebrek aan Type-veiligheid (Overmatig Gebruik van any)

### Probleem

Overmatig gebruik van `any` schakelt TypeScript type checking uit en maakt runtime fouten waarschijnlijker.

### ❌ Slecht Voorbeeld

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Type checking werkt niet
fetchUser().pipe(
  map(user => user.naem) // Typfout! Niet opgemerkt tot runtime
).subscribe(console.log);
```

### ✅ Goed Voorbeeld

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

// Type checking werkt
fetchUser().pipe(
  map(user => user.name) // Fout detectie tijdens compilatie
).subscribe(console.log);
```

### Uitleg

- Definieer interfaces of type aliases
- Specificeer `Observable<T>` type parameters
- Maak maximaal gebruik van TypeScript type inferentie


## 12. Onjuiste Operator Selectie

### Probleem

Operators gebruiken die niet passen bij het doel is inefficiënt of veroorzaakt onverwacht gedrag.

### ❌ Slecht Voorbeeld

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Zoeken bij elke button klik (oude requests worden niet geannuleerd)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Goed Voorbeeld

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Alleen laatste request verwerken (oude requests automatisch annuleren)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Gebruik van Belangrijke Higher-order Operators

| Operator | Toepassing |
|---|---|
| `switchMap` | Alleen laatste stream verwerken (zoeken, autocomplete) |
| `mergeMap` | Parallelle verwerking (volgorde onbelangrijk) |
| `concatMap` | Sequentiële verwerking (volgorde belangrijk) |
| `exhaustMap` | Negeer nieuwe input tijdens uitvoering (voorkom dubbelklik) |

### Uitleg

- Begrijp het gedrag van elke operator
- Juiste keuze volgens use case
- Zie [Transformatie Operators](/nl/guide/operators/transformation/) voor details


## 13. Overmatige Complexiteit

### Probleem

Gevallen waarin eenvoudig te schrijven verwerking onnodig complex wordt gemaakt met RxJS.

### ❌ Slecht Voorbeeld

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Eenvoudige array transformatie complex maken met RxJS
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Goed Voorbeeld

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Array verwerking is voldoende met gewone JavaScript
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// Gebruik RxJS voor asynchrone, event-gedreven verwerking
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Uitleg

- Gebruik RxJS voor asynchrone verwerking of event streams
- Synchrone array verwerking is voldoende met gewone JavaScript
- Overweeg de balans tussen complexiteit en voordelen


## 14. Status Wijziging in subscribe

### Probleem

Status direct wijzigen binnen `subscribe` maakt testen moeilijk en veroorzaakt bugs.

### ❌ Slecht Voorbeeld

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // Status wijzigen in subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Aantal:', this.count);
  }
}
```

### ✅ Goed Voorbeeld

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

    // UI abonneert zich op count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Aantal:', count);
  }
}
```

### Uitleg

- Beheer status met `BehaviorSubject` of `scan`
- Gebruik `subscribe` als trigger
- Testbaar en reactief ontwerp


## 15. Gebrek aan Tests

### Probleem

RxJS code zonder tests naar productieomgeving deployen maakt regressie waarschijnlijker.

### ❌ Slecht Voorbeeld

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Deployen zonder tests
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Goed Voorbeeld

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

### Uitleg

- Voer marble tests uit met `TestScheduler`
- Maak asynchrone verwerking synchroon testbaar
- Zie [Testmethoden](/nl/guide/testing/unit-tests) voor details


## Samenvatting

Door deze 15 anti-patronen te begrijpen en te vermijden, kun je robuustere en meer onderhoudbare RxJS code schrijven.

## Referenties

Deze anti-patronen collectie is gemaakt met referentie aan de volgende betrouwbare bronnen.

### Officiële Documentatie en Repositories
- **[RxJS Officiële Documentatie](https://rxjs.dev/)** - Officiële referentie voor operators en API
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Discussie over shareReplay geheugenlek probleem

### Anti-patronen en Best Practices
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (mei 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01/ngrx-anti-patterns/)** - Brian Love

### Aanvullende Bronnen
- **[Learn RxJS](https://www.learnrxjs.io/)** - Praktische gids voor operators en patronen
- **[RxJS Marbles](https://rxmarbles.com/)** - Visueel begrip van operators

## Gebruik voor Code Review

Controleer of je code anti-patronen bevat.

👉 **[Anti-patroon Preventie Checklist](./checklist)** - Evalueer code met 15 checkpunten

Je kunt direct naar details van het corresponderende anti-patroon op deze pagina springen vanuit elk checkpunt.

## Volgende Stappen

- **[Foutafhandeling](/nl/guide/error-handling/strategies)** - Leer meer gedetailleerde foutafhandelingsstrategieën
- **[Testmethoden](/nl/guide/testing/unit-tests)** - Verwerf effectieve testmethoden voor RxJS code
- **[Begrip van Operators](/nl/guide/operators/)** - Leer gedetailleerd gebruik van elke operator

Integreer deze best practices in je dagelijkse codering en schrijf hoogwaardige RxJS code!
