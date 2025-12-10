---
description: "Una spiegazione dettagliata di 15 errori comuni nell'uso di RxJS con TypeScript e di come affrontarli, con esempi di codice reali per evitare problemi frequenti nella pratica, come l'esposizione esterna del Subject, i subscribe annidati, la dimenticanza di unsubscribe e la gestione impropria degli errori."
---

# Errori comuni e come affrontarli

Questa pagina illustra 15 anti-pattern comuni nell'uso di RxJS in TypeScript e le rispettive soluzioni.

## Indice

1. [Esposizione esterna del Subject](#1-esposizione-esterna-del-subject)
2. [Subscribe annidati (callback hell)](#2-subscribe-annidati-callback-hell)
3. [Dimenticare unsubscribe (memory leak)](#3-dimenticare-unsubscribe-memory-leak)
4. [Uso improprio di shareReplay](#4-uso-improprio-di-sharereplay)
5. [Effetti collaterali in map](#5-effetti-collaterali-in-map)
6. [Ignorare differenze Cold/Hot Observable](#6-ignorare-differenze-cold-hot-observable)
7. [Mischiare Promise e Observable in modo improprio](#7-mischiare-promise-e-observable-in-modo-improprio)
8. [Ignorare la backpressure](#8-ignorare-la-backpressure)
9. [Soppressione degli errori](#9-soppressione-degli-errori)
10. [Leak sottoscrizioni eventi DOM](#10-leak-sottoscrizioni-eventi-dom)
11. [Mancanza di type safety (uso eccessivo di any)](#11-mancanza-di-type-safety-uso-eccessivo-di-any)
12. [Selezione impropria dell'operatore](#12-selezione-impropria-delloperatore)
13. [Eccessiva complessità](#13-eccessiva-complessita)
14. [Cambiamenti di stato in subscribe](#14-cambiamenti-di-stato-in-subscribe)
15. [Mancanza di test](#15-mancanza-di-test)


## 1. Esposizione esterna del Subject

### Problema

Esporre `Subject` così com'è comporta una chiamata esterna a `next()`, rendendo imprevedibile la gestione dello stato.

### ❌ Cattivo esempio

```ts
import { Subject } from 'rxjs';

// Esportiamo il Subject così com'è
export const cartChanged$ = new Subject<void>();

// Chiunque può chiamare next() da un altro file
cartChanged$.next(); // Potrebbe essere chiamato in un momento inaspettato
```

### ✅ Buon esempio

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Esporre come Observable di sola lettura
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // I cambiamenti di stato sono controllati da metodi dedicati
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

### Spiegazione

- Convertito in `Observable` di sola lettura con `asObservable()`
- Le modifiche allo stato possono essere effettuate solo tramite metodi dedicati
- Migliora la tracciabilità delle modifiche e facilita il debug


## 2. Subscribe annidati (callback hell)

### Problema

Chiamare più `subscribe` all'interno di `subscribe` porta al callback hell e complica la gestione degli errori e delle cancellazioni.

### ❌ Cattivo esempio

```ts
import { of } from 'rxjs';

// Simulazione di chiamate API
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Subscribe annidati
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ Buon esempio

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
}


// Appiattire utilizzando operatori di ordine superiore
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### Spiegazione

- Uso di operatori di ordine superiore come `switchMap`, `mergeMap` e `concatMap`
- La gestione degli errori può essere fatta in un unico posto
- Annullamento dell'iscrizione una sola volta
- Migliore leggibilità del codice


## 3. Dimenticare unsubscribe (memory leak)

### Problema

La mancata cancellazione di stream infiniti (ad esempio, listener di eventi) causa memory leak.

### ❌ Cattivo esempio

```ts
import { fromEvent } from 'rxjs';

// Durante l'inizializzazione del componente
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // Unsubscribe dimenticato!
}

// I listener di eventi rimangono anche se il componente viene distrutto
```

### ✅ Buon esempio

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

### ✅ Un altro buon esempio (come usare Subscription)

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

### Spiegazione

- Il pattern `takeUntil` è consigliato (dichiarativo e non ambiguo)
- È utile anche la gestione manuale tramite `Subscription`
- Annullare sempre la sottoscrizione alla distruzione del componente


## 4. Uso improprio di shareReplay

### Problema

L'uso di `shareReplay` senza comprenderne il funzionamento può causare la riproduzione di dati vecchi e memory leak.

### ❌ Cattivo esempio

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Rendere illimitata la dimensione del buffer
const shared$ = interval(1000).pipe(
  shareReplay() // Default a buffer illimitato
);

// I valori rimarranno in memoria anche se non ci sono più sottoscrittori
```

### ✅ Buon esempio

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Specificare esplicitamente la dimensione del buffer e il numero di riferimenti
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Rilascia le risorse quando non ci sono più sottoscrittori
  })
);
```

### Spiegazione

- Specificare esplicitamente `bufferSize` (di solito 1)
- `refCount: true` per il rilascio automatico quando non ci sono più sottoscrittori
- `shareReplay({ bufferSize: 1, refCount: true })` è sicuro per i flussi che si completano, come le richieste HTTP


## 5. Effetti collaterali in map

### Problema

La modifica dello stato all'interno dell'operatore `map` causa un comportamento imprevedibile.

### ❌ Cattivo esempio

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // Effetto collaterale!
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // Il counter aumenta inaspettatamente
```

### ✅ Buon esempio

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Solo conversione pura
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Gli effetti collaterali sono separati con tap
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// Usare scan per accumulare lo stato
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Spiegazione

- Utilizzare `map` come funzione pura
- Gli effetti collaterali (log, chiamate API, ecc.) sono separati in `tap`
- Usare `scan` o `reduce` per accumulare lo stato


## 6. Ignorare differenze Cold/Hot Observable

### Problema

L'uso di un Observable senza capire se è Cold o Hot può portare a esecuzioni duplicate e a comportamenti inaspettati.

### ❌ Cattivo esempio

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - La richiesta HTTP viene eseguita per ogni sottoscrizione
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Richiesta 1
data$.subscribe(console.log); // Richiesta 2 (inutile duplicazione)
```

### ✅ Buon esempio

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Convertire in Hot Observable e condividere
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Richiesta 1
data$.subscribe(console.log); // Usa i risultati in cache
```

### Spiegazione

- Cold Observable: eseguito per ogni sottoscrizione (`of`, `from`, `fromEvent`, `ajax`, ecc.)
- Hot Observable: eseguito indipendentemente dalla sottoscrizione (`Subject`, Observable multicast, ecc.)
- I Cold possono essere convertiti in Hot con `share` / `shareReplay`


## 7. Mischiare Promise e Observable in modo improprio

### Problema

Mischiare Promise e Observable senza un'adeguata conversione porta a una gestione incompleta degli errori e delle cancellazioni.

### ❌ Cattivo esempio

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// La Promise viene usata così com'è
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Promise annidata
    console.log(data, moreData);
  });
});
```

### ✅ Buon esempio

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Convertire la Promise in Observable e unificare
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Spiegazione

- Convertire Promise in Observable con `from`
- Elaborazione uniforme nella pipeline di Observable
- Gestione degli errori e cancellazione più semplice


## 8. Ignorare la backpressure

### Problema

La gestione incontrollata degli eventi ad alta frequenza comporta prestazioni scadenti.

### ❌ Cattivo esempio

```ts
import { fromEvent } from 'rxjs';

// Elabora gli eventi di input così come sono
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // Chiamata API su ogni input (sovraccarico)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ Buon esempio

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Applica il debounce e l'annullamento
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // Aspetta 300ms
  distinctUntilChanged(), // Solo quando il valore cambia
  switchMap(query => searchAPI(query)) // Annulla le vecchie richieste
).subscribe(results => {
  console.log('Results:', results);
});
```

### Spiegazione

- `debounceTime` attende per un certo periodo di tempo
- `throttleTime` limita la frequenza massima
- `distinctUntilChanged` per escludere i duplicati
- Annullare le vecchie richieste con `switchMap`


## 9. Soppressione degli errori

### Problema

La mancata gestione degli errori rende difficile il debugging e degrada l'esperienza dell'utente.

### ❌ Cattivo esempio

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Ignoriamo l'errore
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // L'informazione sull'errore viene persa
).subscribe(data => {
  console.log(data); // Arriva null, causa sconosciuta
});
```

### ✅ Buon esempio

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
    // Notifica all'utente
    showErrorToast('Impossibile recuperare i dati');
    // Restituire un valore alternativo con le informazioni sull'errore
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

### Spiegazione

- Registrare gli errori nel log
- Fornire feedback all'utente
- Restituire valori alternativi con informazioni sugli errori
- Considerare strategie di retry (`retry`, `retryWhen`)


## 10. Leak sottoscrizioni eventi DOM

### Problema

Se i listener di eventi DOM non vengono rilasciati correttamente, si verificano memory leak.

### ❌ Cattivo esempio

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Registra un listener di eventi
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // Non è stato effettuato l'unsubscribe
  }

  destroy(): void {
    this.button.remove();
    // Il listener rimane
  }
}
```

### ✅ Buon esempio

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

### Spiegazione

- Annullamento affidabile della sottoscrizione con il pattern `takeUntil`
- Lanciare `destroy$` alla distruzione del componente
- Rilasciare i listener prima di cancellare gli elementi del DOM


## 11. Mancanza di type safety (uso eccessivo di any)

### Problema

L'uso massiccio di `any` disabilita il controllo dei tipi in TypeScript ed è soggetto a errori a runtime.

### ❌ Cattivo esempio

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Il controllo del tipo non funziona
fetchUser().pipe(
  map(user => user.naem) // Typo! Non notato fino al runtime
).subscribe(console.log);
```

### ✅ Buon esempio

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

// Il controllo del tipo funziona
fetchUser().pipe(
  map(user => user.name) // Rilevamento degli errori in fase di compilazione
).subscribe(console.log);
```

### Spiegazione

- Definire interfacce e alias di tipo
- Parametri di tipo espliciti per `Observable<T>`
- Sfruttare al meglio l'inferenza dei tipi di TypeScript


## 12. Selezione impropria dell'operatore

### Problema

L'uso di operatori non adatti allo scopo può portare a comportamenti inefficienti o inaspettati.

### ❌ Cattivo esempio

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Ricerca su ogni clic del pulsante (le vecchie richieste non vengono cancellate)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Buon esempio

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Elaborare solo le richieste più recenti (quelle vecchie vengono automaticamente cancellate)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Uso degli operatori chiave di ordine superiore

| Operatore | Uso |
|---|---|
| `switchMap` | Elabora solo lo stream più recente (ricerca, autocompletamento) |
| `mergeMap` | Elaborazione parallela (in qualsiasi ordine) |
| `concatMap` | Elaborazione sequenziale (l'ordine è importante) |
| `exhaustMap` | Ignora i nuovi input durante l'esecuzione (evita clic ripetuti) |

### Spiegazione

- Comprendere il comportamento di ciascun operatore
- Selezionare quello appropriato per il proprio caso d'uso
- Per ulteriori informazioni, vedere [Operatori di trasformazione](/it/guide/operators/transformation/)


## 13. Eccessiva complessità

### Problema

Casi in cui RxJS complica eccessivamente processi che potrebbero essere scritti in modo semplice.

### ❌ Cattivo esempio

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Complicare le trasformazioni di semplici array con RxJS
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Buon esempio

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// La gestione degli array è sufficiente con il normale JavaScript
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// RxJS è utilizzato per l'elaborazione asincrona e guidata dagli eventi
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Spiegazione

- RxJS è usato per l'elaborazione asincrona e i flussi di eventi
- Il normale JavaScript è sufficiente per l'elaborazione sincrona degli array
- Considerare l'equilibrio tra complessità e benefici


## 14. Cambiamenti di stato in subscribe

### Problema

La modifica dello stato direttamente all'interno di `subscribe` rende difficile il test e causa bug.

### ❌ Cattivo esempio

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // Cambia stato in subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ Buon esempio

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

    // L'interfaccia utente si iscrive a count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### Spiegazione

- Lo stato è gestito da `BehaviorSubject` o `scan`
- `subscribe` è usato come trigger
- Design testabile e reattivo


## 15. Mancanza di test

### Problema

Distribuire il codice RxJS in produzione senza testarlo è soggetto a regressioni.

### ❌ Cattivo esempio

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Distribuire senza testare
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Buon esempio

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

### Spiegazione

- Test di marble con `TestScheduler`
- I processi asincroni possono essere testati in modo sincrono
- Per maggiori informazioni, vedere [Metodologie di test](/it/guide/testing/unit-tests)


## Riepilogo

Comprendere ed evitare questi 15 anti-pattern vi permetterà di scrivere codice RxJS più robusto e manutenibile.

## Riferimenti

Questa raccolta di anti-pattern è stata creata facendo riferimento alle seguenti fonti affidabili.

### Repository della documentazione ufficiale
- **[RxJS Official Documentation](https://rxjs.dev/)** - Riferimento ufficiale per operatori e API
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Discussione sul problema dei memory leak di shareReplay

### Anti-pattern e best practice
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (maggio 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### Risorse aggiuntive
- **[Learn RxJS](https://www.learnrxjs.io/)** - Una guida pratica agli operatori e ai pattern
- **[RxJS Marbles](https://rxmarbles.com/)** - Comprensione visiva degli operatori

## Utilizzo per la revisione del codice

Verifica la presenza di anti-pattern nel tuo codice.

👉 **[Checklist per evitare gli anti-pattern](./checklist)** - Rivedi il codice con 15 voci della checklist

Da ogni voce della checklist puoi passare direttamente ai dettagli dell'anti-pattern corrispondente in questa pagina.

## Prossimi passi

- **[Gestione degli errori](/it/guide/error-handling/strategies)** - Impara strategie di gestione degli errori più dettagliate
- **[Metodologie di test](/it/guide/testing/unit-tests)** - Impara a testare efficacemente il codice RxJS
- **[Comprendere gli operatori](/it/guide/operators/)** - Impara a usare ogni operatore in dettaglio

Incorpora queste best practice nella tua codifica quotidiana e scrivi codice RxJS di qualità!
