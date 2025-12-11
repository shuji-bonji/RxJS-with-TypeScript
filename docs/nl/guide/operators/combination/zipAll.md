---
description: zipAll is een operator die een Higher-order Observable (Observable van Observables) neemt en de corresponderende geordende waarden van elke interne Observable paart en uitgeeft als een array.
titleTemplate: ':title'
---

# zipAll - Paar corresponderende waarden van elke interne Observable

De `zipAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**paart de corresponderende geordende waarden van elke interne Observable** en geeft ze uit als een array.

## 🔰 Basissyntax en gebruik

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable met drie interne Observables
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Paar corresponderende geordende waarden van elke interne Observable
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Output:
// [0, 0, 0] ← Alle 1e waarden
// [1, 1, 1] ← Alle 2e waarden
// (Voltooit hier: 3e Observable emitteert slechts 2 waarden)
```

- Verzamelt interne Observables wanneer Higher-order Observable **voltooit**
- **Paart de waarden met dezelfde index** van elke interne Observable
- **Wanneer de kortste interne Observable voltooit**, voltooit het geheel

[🌐 RxJS Officiële Documentatie - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 Typische gebruikspatronen

- **Match meerdere API-responses in volgorde**
- **Vergelijk waarden met dezelfde timing van meerdere streams**
- **Combineer parallelle verwerkingsresultaten in volgorde**

## 🧠 Praktisch codevoorbeeld

Voorbeeld van het paren van de corresponderende waarden van meerdere tellers

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Maak drie tellers met verschillende snelheden
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Langzaam: ${n}`)),
  interval(500).pipe(take(5), map(n => `Normaal: ${n}`)),
  interval(300).pipe(take(6), map(n => `Snel: ${n}`))
);

// Paar corresponderende geordende waarden van elke teller
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Output:
// [Langzaam: 0, Normaal: 0, Snel: 0]
// [Langzaam: 1, Normaal: 1, Snel: 1]
// [Langzaam: 2, Normaal: 2, Snel: 2]
// [Langzaam: 3, Normaal: 3, Snel: 3]
// (Voltooit hier: "Langzaam" teller emitteert slechts 4 waarden)
```

## 🔄 Gerelateerde Creation Function

Terwijl `zipAll` voornamelijk wordt gebruikt voor het afvlakken van Higher-order Observables,
gebruik de **Creation Function** `zip` voor normaal paren van meerdere Observables.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Creation Function versie (meest voorkomend gebruik)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

Zie [Hoofdstuk 3: Creation Functions - zip](/nl/guide/creation-functions/combination/zip).

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| [combineLatestAll](/nl/guide/operators/combination/combineLatestAll) | Combineer laatste waarden van alle interne Observables |
| [mergeAll](/nl/guide/operators/combination/mergeAll) | Abonneer op alle interne Observables parallel |
| [concatAll](/nl/guide/operators/combination/concatAll) | Abonneer op interne Observables in volgorde |
| [switchAll](/nl/guide/operators/combination/switchAll) | Schakel naar nieuwe interne Observable |

## 🔄 zipAll vs combineLatestAll

| Operator | Combinatiemethode | Voltooiingstiming |
|---|---|---|
| `zipAll` | Paart waarden met de **dezelfde index** | Wanneer de **kortste** interne Observable voltooit |
| `combineLatestAll` | Combineert **laatste waarden** | Wanneer **alle** interne Observables voltooien |

```ts
// zipAll: [0e, 0e, 0e], [1e, 1e, 1e], ...
// combineLatestAll: [laatste, laatste, laatste], [laatste, laatste, laatste], ...
```

## ⚠️ Belangrijke opmerkingen

### Higher-order Observable moet voltooien

`zipAll` wacht om interne Observables te verzamelen totdat de Higher-order Observable (buitenste Observable) **voltooit**.

#### ❌ Niets output omdat Higher-order Observable niet voltooit
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // Niets output
```

#### ✅ Voltooi met take
```ts
interval(1000).pipe(
  take(3), // Voltooi na 3
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Voltooit met kortste interne Observable

Wanneer de **kortste interne Observable voltooit**, voltooit het geheel.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 waarden
  of(1, 2)           // 2 waarden ← Kortste
).pipe(
  zipAll()
).subscribe(console.log);

// Output: [1, 1], [2, 2]
// (Voltooit bij 2. 3, 4, 5 worden niet gebruikt)
```

### Backpressure (Geheugengebruik)

Wanneer interne Observables met verschillende snelheden emitteren, **stapelen waarden van snellere interne Observables op in het geheugen**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// Snelle teller (100ms) waarden stapelen op in geheugen terwijl gewacht wordt op langzame teller (10000ms)
of(
  interval(10000).pipe(take(3)), // Langzaam
  interval(100).pipe(take(100))  // Snel
).pipe(
  zipAll()
).subscribe(console.log);
```

Als het snelheidsverschil groot is, let op geheugengebruik.
