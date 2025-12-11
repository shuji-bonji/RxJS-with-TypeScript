---
description: "from() - Creation Function die Observables genereert van arrays, Promises, iterables, strings, etc. Bestaande datastructuren eenvoudig streamen. TypeScript type-veilige conversiepatronen, combinatie met async/await, en gedetailleerd verschil met of() uitgelegd."
---

# from() - Converteer Arrays/Promises

`from()` is een Creation Function die Observables genereert van arrays, Promises, iterables en Observable-achtige objecten.

## Overzicht

`from()` converteert bestaande datastructuren (arrays, Promises, iterables, etc.) naar Observable streams. Vooral handig voor het integreren van async operaties (Promises) in de RxJS-wereld.

**Signature**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Officiële Documentatie**: [📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

## Basisgebruik

`from()` accepteert verschillende inputtypes.

```typescript
import { from } from 'rxjs';

// Van array maken
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array waarde:', value),
  complete: () => console.log('Array compleet')
});

// Van Promise maken
const promise$ = from(Promise.resolve('Promise resultaat'));
promise$.subscribe({
  next: value => console.log('Promise resultaat:', value),
  complete: () => console.log('Promise compleet')
});

// Van iterable maken
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable waarde:', value),
  complete: () => console.log('Iterable compleet')
});

// Output:
// Array waarde: 1
// Array waarde: 2
// Array waarde: 3
// Array compleet
// Iterable waarde: 1
// Iterable waarde: 2
// Iterable waarde: 3
// Iterable compleet
// Promise resultaat: Promise resultaat
// Promise compleet
```

## Belangrijke Kenmerken

### 1. Emit Array-elementen Afzonderlijk

`from()` emit bij arrays elk element afzonderlijk in volgorde.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Output:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Verschil met `of()`**:
> - `of([1, 2, 3])` → Emit array zelf als één waarde
> - `from([1, 2, 3])` → Emit elementen `1`, `2`, `3` afzonderlijk

### 2. Automatische Promise Verwerking

Bij Promises wordt de resolved waarde geëmit en direct compleet.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Data ophalen compleet'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Compleet')
});

// Na 1 seconde output:
// Data ophalen compleet
// Compleet
```

> [!WARNING]
> Bij Promise rejection emit de Observable een error.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Fout')).subscribe({
>   error: err => console.error('Fout opgetreden:', err)
> });
> ```

### 3. Iterable Support

Ondersteunt iterables zoals `Set`, `Map`, `Generator`, etc.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Output: A, B, C

// Map (key-value pairs)
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

`from()` is een **Cold Observable**. Bij elke subscription start een onafhankelijke uitvoering.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Subscriber A:', val));
numbers$.subscribe(val => console.log('Subscriber B:', val));

// Elke subscriber verwerkt de array onafhankelijk
// Output:
// Subscriber A: 1
// Subscriber A: 2
// Subscriber A: 3
// Subscriber B: 1
// Subscriber B: 2
// Subscriber B: 3
```

> [!NOTE]
> **Cold Observable Kenmerken**
> - Bij elke subscription start een onafhankelijke uitvoering
> - Elke subscriber ontvangt zijn eigen datastream
> - Ook bij Promises wordt bij elke subscription opnieuw geëvalueerd
>
> Zie [Cold en Hot Observables](/nl/guide/observables/cold-and-hot-observables) voor meer details.

## Verschil tussen from() en of()

Het belangrijkste verschil is hoe arrays worden behandeld.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - Emit array als één waarde
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - Emit elk array-element afzonderlijk
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Creation Function | Array behandeling | Gebruik |
|-------------------|-------------------|---------|
| `of([1, 2, 3])` | Emit array zelf | Array als data gebruiken |
| `from([1, 2, 3])` | Emit elk element afzonderlijk | Array-elementen één voor één verwerken |

## Praktische Gebruikssituaties

### 1. API Aanroepen Streamen

Stream Promise-gebaseerde HTTP clients zoals Fetch API of axios.

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
      console.error('API Fout:', error);
      return of({ id: 0, name: 'Unknown', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('User:', user));
```

### 2. Array-elementen Sequentieel Verwerken

Voer async operaties sequentieel uit op elk array-element.

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
).subscribe(data => console.log('Opgehaald:', data));
```

### 3. Async Iterator Verwerking

Ondersteunt ook async iterators (async generators).

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

### 4. Event Emitter Integratie

Stream Node.js EventEmitter of custom event systemen.

```typescript
import { from } from 'rxjs';

// Iterable custom object
class DataSource {
  *[Symbol.iterator]() {
    yield 'Data A';
    yield 'Data B';
    yield 'Data C';
  }
}

from(new DataSource()).subscribe(console.log);
// Output: Data A, Data B, Data C
```

## Gebruik in Pipelines

`from()` is handig als startpunt voor pipeline verwerking van bestaande data.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Product A', price: 1000 },
  { id: 2, name: 'Product B', price: 2000 },
  { id: 3, name: 'Product C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Totaal bedrag:', total));
// Output: Totaal bedrag: 3000
```

## Veelgemaakte Fouten

### 1. Verkeerde Promise Executie Timing

```typescript
// ❌ Fout - Promise wordt uitgevoerd bij creatie
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Al gestart
from(promise).subscribe(console.log); // Niet bij subscription

// ✅ Correct - Gebruik defer() voor uitvoering bij subscription
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // Uitgevoerd bij subscription
```

> [!WARNING]
> **Promises zijn niet lazy**
>
> Promises starten uitvoering bij creatie. `from(promise)` wrapt alleen een al lopende Promise. Gebruik `defer(() => from(promise))` voor uitvoering bij subscription.

### 2. Array en of() Verwarren

```typescript
import { from, map, of } from "rxjs";

// ❌ Niet de bedoeling - Hele array wordt geëmit
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: [1, 2, 3] (array zelf)

// ✅ Correct - Elk element afzonderlijk verwerken
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6
```

## Prestatieoverwegingen

`from()` prestaties variëren per inputtype.

> [!TIP]
> **Optimalisatie Tips**:
> - Bij grote datasets (duizenden elementen), beperk parallellisme bij `concatMap` of `mergeMap`.
> - Overweeg `forkJoin` of `combineLatest` bij Promise arrays.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Beperk parallelle uitvoering tot 5
  )
).subscribe(data => console.log(data));
```

## Gerelateerde Creation Functions

| Function | Verschil | Gebruik |
|----------|----------|---------|
| **[of()](/nl/guide/creation-functions/basic/of)** | Emit argumenten in volgorde | Waarden direct emittende |
| **[fromEvent()](/nl/guide/creation-functions/basic/fromEvent)** | Stream events | DOM events of EventEmitter |
| **[defer()](/nl/guide/creation-functions/conditional/defer)** | Lazy creatie bij subscription | Lazy Promise executie |
| **ajax()** | HTTP-specifiek | RxJS HTTP requests |

## Samenvatting

- `from()` genereert Observables van arrays, Promises, iterables
- Emit array-elementen afzonderlijk (verschilt van `of()`)
- Verwerkt Promises automatisch en emit resultaat
- Ideaal voor integratie van async operaties in RxJS
- Let op: Promises starten bij creatie (gebruik `defer()` voor lazy executie)

## Volgende Stappen

- [fromEvent() - Converteer events naar Observable](/nl/guide/creation-functions/basic/fromEvent)
- [defer() - Lazy creatie bij subscription](/nl/guide/creation-functions/conditional/defer)
- [Terug naar basis overzicht](/nl/guide/creation-functions/basic/)
