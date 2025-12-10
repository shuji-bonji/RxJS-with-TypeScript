---
description: "from() - Funzione di Creazione che converte array, Promise, iterabili e oggetti simil-Observable in stream Observable con integrazione asincrona senza interruzioni"
---

# from() - Converti da Array, Promise, ecc.

`from()` è una Funzione di Creazione che crea un Observable da array, Promise, iterabili e oggetti simil-Observable.

## Panoramica

`from()` converte strutture dati esistenti (array, Promise, iterabili, ecc.) in stream Observable. In particolare, è frequentemente usata per integrare l'elaborazione asincrona (Promise) nel mondo RxJS.

**Firma**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: from()](https://rxjs.dev/api/index/function/from)

## Uso Base

`from()` accetta una varietà di tipi di input.

```typescript
import { from } from 'rxjs';

// Crea da array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valore array:', value),
  complete: () => console.log('Array completo')
});

// Crea da Promise
const promise$ = from(Promise.resolve('Risultato Promise'));
promise$.subscribe({
  next: value => console.log('Risultato Promise:', value),
  complete: () => console.log('Promise completa')
});

// Crea da iterabile
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valore iterabile:', value),
  complete: () => console.log('Iterabile completo')
});

// Output:
// Valore array: 1
// Valore array: 2
// Valore array: 3
// Array completo
// Valore iterabile: 1
// Valore iterabile: 2
// Valore iterabile: 3
// Iterabile completo
// Risultato Promise: Risultato Promise
// Promise completa
```

## Caratteristiche Importanti

### 1. Emette Ogni Elemento dell'Array Individualmente

Quando `from()` riceve un array, emette ogni elemento dell'array individualmente in sequenza.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Output:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Differenza da `of()`**:
> - `of([1, 2, 3])` → Emette l'array stesso come singolo valore
> - `from([1, 2, 3])` → Emette ogni elemento `1`, `2`, `3` separatamente

### 2. Elabora Promise Automaticamente

Passando una Promise emetterà il valore risolto e completerà immediatamente.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Recupero dati completato'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Completo')
});

// Output dopo 1 secondo:
// Recupero dati completato
// Completo
```

> [!WARNING]
> Se la Promise viene rigettata, Observable emette un errore.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Errore')).subscribe({
>   error: err => console.error('Si è verificato un errore:', err)
> });
> ```

### 3. Supporto per Iterabili

Oltre agli array, supporta oggetti iterabili come `Set`, `Map` e `Generator`.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Output: A, B, C

// Map (coppie chiave-valore)
from(new Map([['key1', 'value1'], ['key2', 'value2']])).subscribe(console.log);
// Output: ['key1', 'value1'], ['key2', 'value2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// Output: 1, 2, 3
```

### 4. Cold Observable

`from()` è un **Cold Observable**. Ogni subscription avvia un'esecuzione indipendente.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Subscriber A:', val));
numbers$.subscribe(val => console.log('Subscriber B:', val));

// Ogni subscriber elabora l'array indipendentemente
// Output:
// Subscriber A: 1
// Subscriber A: 2
// Subscriber A: 3
// Subscriber B: 1
// Subscriber B: 2
// Subscriber B: 3
```

> [!NOTE]
> **Caratteristiche Cold Observable**:
> - Un'esecuzione indipendente viene avviata per ogni subscription
> - Ogni subscriber riceve il proprio stream di dati
> - Anche le Promise vengono valutate per subscription
>
> Vedi [Cold Observable e Hot Observable](/it/guide/observables/cold-and-hot-observables) per maggiori informazioni.

## Differenza tra from() e of()

La differenza più importante tra i due è il modo in cui vengono gestiti gli array.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - emette l'array come singolo valore
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - emette ogni elemento dell'array individualmente
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Funzione di Creazione | Gestione Array | Scopo |
|-------------------|-----------|------|
| `of([1, 2, 3])` | Emette l'array stesso | Vuoi trattare l'array come dato |
| `from([1, 2, 3])` | Emette ogni elemento individualmente | Vuoi elaborare elementi array uno per uno |

## Casi d'Uso Pratici

### 1. Stream Chiamate API

Stream client HTTP basati su Promise come Fetch API e axios.

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('Errore API:', error);
      return of({ id: 0, name: 'Sconosciuto', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('Utente:', user));
```

### 2. Elaborazione Sequenziale di Elementi Array

Esegui elaborazione asincrona sequenzialmente per ogni elemento dell'array.

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // Rate limiting
    )
  )
).subscribe(data => console.log('Recuperato:', data));
```

### 3. Elaborazione Iterator Asincrono

Sono supportati anche gli iterator asincroni (generatori async).

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// Output: 1, 2, 3
```

### 4. Integrazione Event Emitter

Stream Node.js EventEmitter e sistemi di eventi personalizzati.

```typescript
import { from } from 'rxjs';

// Oggetto personalizzato iterabile
class DataSource {
  *[Symbol.iterator]() {
    yield 'Dati A';
    yield 'Dati B';
    yield 'Dati C';
  }
}

from(new DataSource()).subscribe(console.log);
// Output: Dati A, Dati B, Dati C
```

## Uso in Pipeline

`from()` è utile quando si usano dati esistenti come punto di partenza per l'elaborazione pipeline.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Prodotto A', price: 1000 },
  { id: 2, name: 'Prodotto B', price: 2000 },
  { id: 3, name: 'Prodotto C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Totale:', total));
// Output: Totale: 3000
```

## Errori Comuni

### 1. Fraintendere il Timing di Esecuzione Promise

```typescript
// ❌ Sbagliato - Promise inizia l'esecuzione al momento della creazione
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Già iniziata
from(promise).subscribe(console.log); // Non al momento della subscription

// ✅ Corretto - usa defer() se vuoi eseguire al momento della subscription
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // Esegue al momento della subscription
```

> [!WARNING]
> **Promise Non è Valutata Lazily**
>
> Promise inizia l'esecuzione quando viene creata. `from(promise)` avvolge solo una Promise già in esecuzione. Se vuoi eseguire al momento della subscription, usa `defer(() => from(promise))`.

### 2. Confondere Array con of()

```typescript
import { from, map, of } from "rxjs";

// ❌ Diverso dall'intenzione - l'intero array viene emesso
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: [1, 2, 3] (l'array stesso)

// ✅ Corretto - elabora ogni elemento individualmente
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6
```

## Considerazioni sulle Performance

Le performance di `from()` dipendono dal tipo di input.

> [!TIP]
> **Suggerimenti di Ottimizzazione**:
> - Quando si elaborano grandi quantità di dati (migliaia o decine di migliaia di elementi), limita il numero di operazioni concorrenti quando combini con `concatMap` e `mergeMap`.
> - Quando elabori array di Promise, considera l'uso di `forkJoin` o `combineLatest`.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Limita esecuzione concorrente a 5
  )
).subscribe(data => console.log(data));
```

## Funzioni di Creazione Correlate

| Funzione | Differenza | Uso |
|----------|------|----------|
| **[of()](/it/guide/creation-functions/basic/of)** | Emette argomenti in sequenza | Vuoi emettere valori così come sono |
| **[fromEvent()](/it/guide/creation-functions/basic/fromEvent)** | Stream eventi | Gestisci eventi DOM o EventEmitter |
| **[defer()](/it/guide/creation-functions/conditional/defer)** | Ritarda la generazione fino alla subscription | Serve esecuzione lazy della Promise |
| **ajax()** | Dedicato alla comunicazione HTTP | Vuoi completare richieste HTTP dentro RxJS |

## Riepilogo

- `from()` crea Observable da array, Promise e iterabili
- Emette ogni elemento di un array separatamente (diverso da `of()`)
- Elabora automaticamente Promise ed emette il risultato
- Ideale per integrare elaborazione asincrona nel mondo RxJS
- Nota che Promise viene eseguita al momento della creazione (usa `defer()` per esecuzione lazy)

## Prossimi Passi

- [fromEvent() - Converti Eventi in Observable](/it/guide/creation-functions/basic/fromEvent)
- [defer() - Ritarda la Generazione Fino alla Subscription](/it/guide/creation-functions/conditional/defer)
- [Torna alle Funzioni di Creazione Base](/it/guide/creation-functions/basic/)
