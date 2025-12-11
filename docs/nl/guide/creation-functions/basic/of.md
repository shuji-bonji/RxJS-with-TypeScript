---
description: "of() - RxJS Creation Function die opgegeven waarden sequentieel emit. De eenvoudigste manier om Observables te maken, ideaal voor testdata en mocks. TypeScript type-inferentie implementatiepatronen, omgaan met arrays en objecten, en verschil met from() uitgelegd met praktische codevoorbeelden."
---

# of() - Sequentiële Waarde Emissie

`of()` is de eenvoudigste Creation Function die opgegeven waarden één voor één in volgorde emit.

## Overzicht

`of()` emit de doorgegeven argumenten in volgorde direct bij subscription, en completet onmiddellijk na alle waarden. Wordt vaak gebruikt in testcode en het maken van mockdata.

**Signature**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Officiële Documentatie**: [📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

## Basisgebruik

`of()` accepteert meerdere waarden gescheiden door komma's.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Compleet')
});

// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Waarde: 4
// Waarde: 5
// Compleet
```

## Belangrijke Kenmerken

### 1. Synchrone Emissie

`of()` emit alle waarden **synchroon** direct bij subscription.

```typescript
import { of } from 'rxjs';

console.log('Voor subscription');

of('A', 'B', 'C').subscribe(value => console.log('Waarde:', value));

console.log('Na subscription');

// Output:
// Voor subscription
// Waarde: A
// Waarde: B
// Waarde: C
// Na subscription
```

### 2. Directe Completion

Na het emittende van alle waarden, wordt onmiddellijk `complete` genotificeerd.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Compleet!')
});

// Output: 1, 2, 3, Compleet!
```

### 3. Emit Elk Type Waarde

Kan primitieve types tot objecten en arrays emittenden.

```typescript
import { of } from 'rxjs';

// Primitieve types
of(42, 'hello', true).subscribe(console.log);

// Objecten
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Arrays (emit array zelf als één waarde)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Output: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` is een **Cold Observable**. Bij elke subscription start een onafhankelijke uitvoering.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Eerste subscription
values$.subscribe(val => console.log('Subscriber A:', val));

// Tweede subscription (onafhankelijk uitgevoerd)
values$.subscribe(val => console.log('Subscriber B:', val));

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
> - Voor data-sharing gebruik `share()` om Hot te maken
>
> Zie [Cold en Hot Observables](/nl/guide/observables/cold-and-hot-observables) voor meer details.

## Verschil tussen of() en from()

`of()` en `from()` gedragen zich anders bij arrays. Dit is een veelvoorkomend verwarringspunt.

```typescript
import { of, from } from 'rxjs';

// of() - Emit array als één waarde
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// from() - Emit elk array-element afzonderlijk
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

> [!IMPORTANT]
> **Selectiecriteria**:
> - Array zelf emittende → `of([1, 2, 3])`
> - Array-elementen afzonderlijk emittende → `from([1, 2, 3])`

## Praktische Gebruikssituaties

### 1. Testdata & Mock Creatie

`of()` wordt het meest gebruikt voor het maken van mockdata in testcode.

```typescript
import { of } from 'rxjs';

// User data mock
function getMockUser$() {
  return of({
    id: 1,
    name: 'Test User',
    email: 'test@example.com'
  });
}

// Gebruik in test
getMockUser$().subscribe(user => {
  console.log('User:', user.name); // User: Test User
});
```

### 2. Standaardwaarden Leveren

Gebruik voor fallback-waarden bij fouten of standaardwaarden.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('Invalid ID'));
  }
  return of({ id, data: 'some data' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Fout:', err.message);
    return of({ id: 0, data: 'default data' }); // Standaardwaarde
  })
).subscribe(result => console.log(result));
// Output: Fout: Invalid ID
//       { id: 0, data: 'default data' }
```

### 3. Meerdere Waarden Geleidelijk Emittende

Gebruik voor het uitvoeren van meerdere stappen in volgorde.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Loading...', 'Processing...', 'Done!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Output (elke seconde):
// Loading...
// Processing...
// Done!
```

### 4. Waarden Retourneren met Voorwaardelijke Vertakking

Combineer met `iif()` of `switchMap()` om waarden te retourneren op basis van voorwaarden.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Welcome back!'),
  of('Please log in')
).subscribe(console.log);
// Output: Welcome back!
```

## Gebruik in Pipelines

`of()` wordt gebruikt als startpunt van pipelines of om data in te injecteren.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Alleen even getallen
  map(n => n * 10)           // Vermenigvuldig met 10
).subscribe(console.log);
// Output: 20, 40
```

## Veelgemaakte Fouten

### 1. Array Rechtstreeks Doorgeven

```typescript
// ❌ Fout - Hele array wordt als één waarde geëmit
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// ✅ Correct - Gebruik from() om elementen afzonderlijk te emittende
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3

// ✅ Of gebruik spread syntax
of(...[1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

### 2. Verwarring met Async Operaties

`of()` emit synchroon. Het wordt niet asynchroon.

```typescript
// ❌ Dit wordt niet asynchroon
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() wordt onmiddellijk uitgevoerd, Promise object wordt geëmit

// ✅ Gebruik from() om Promises te streamen
from(fetchDataFromAPI()).subscribe(console.log);
```

## Prestatieoverwegingen

`of()` is zeer lichtgewicht met bijna geen overhead. Bij grote aantallen waarden (duizenden+) let op:

> [!TIP]
> Voor grote aantallen waarden (duizenden+) overweeg `from()` of `range()` te gebruiken.

## Gerelateerde Creation Functions

| Function | Verschil | Gebruik |
|----------|----------|---------|
| **[from()](/nl/guide/creation-functions/basic/from)** | Converteer arrays/Promises | Stream iterables of Promises |
| **range()** | Genereer nummerbereik | Emit opeenvolgende nummers |
| **EMPTY** | Emit niets, direct compleet | Lege stream nodig |

## Samenvatting

- `of()` is de eenvoudigste Creation Function die opgegeven waarden in volgorde emit
- Emit synchroon bij subscription en completet direct
- Ideaal voor testdata en mock creatie
- Emit array zelf (verschilt van `from()`)
- Gebruik `from()` voor async operaties

## Volgende Stappen

- [from() - Converteer arrays/Promises](/nl/guide/creation-functions/basic/from)
- [Combinatie Creation Functions](/nl/guide/creation-functions/combination/)
- [Terug naar basis overzicht](/nl/guide/creation-functions/basic/)
