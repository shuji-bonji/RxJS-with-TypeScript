---
description: combineLatestAll neemt een Higher-order Observable (Observable van Observables) en combineert de laatste waarde van elk wanneer alle interne Observables ten minste één keer hebben geëmitteerd.
titleTemplate: ':title'
---

# combineLatestAll - Combineer laatste waarden van alle interne Observables

De `combineLatestAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**zodra alle interne Observables ten minste één keer hebben geëmitteerd**, combineert hun **laatste waarden** en geeft ze uit als een array.

## 🔰 Basissyntax en gebruik

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// Higher-order Observable met drie interne Observables
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Combineer laatste waarden zodra alle interne Observables ten minste één keer hebben geëmitteerd
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Output:
// [1, 3, 0] ← Wanneer alle ten minste één keer hebben geëmitteerd (na 2 seconden)
// [2, 3, 0] ← 1e Observable emitteert 2 (na 3 seconden)
// [2, 3, 1] ← 3e Observable emitteert 1 (na 4 seconden)
```

- Verzamelt interne Observables wanneer Higher-order Observable **voltooit**
- **Zodra alle interne Observables ten minste één keer hebben geëmitteerd**, begint het combineren
- Wanneer een interne Observable een waarde emitteert, **combineert alle laatste waarden** en geeft uit

[🌐 RxJS Officiële Documentatie - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Typische gebruikspatronen

- **Combineer laatste resultaten van meerdere API-aanroepen**
- **Synchroniseer laatste waarden van meerdere formulierinvoervelden**
- **Integreer meerdere realtime databronnen**

## 🔄 Gerelateerde Creation Function

Terwijl `combineLatestAll` voornamelijk wordt gebruikt voor het afvlakken van Higher-order Observables,
gebruik de **Creation Function** `combineLatest` voor normale multi-Observable combinaties.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation Function versie (meest voorkomend gebruik)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Zie [Hoofdstuk 3: Creation Functions - combineLatest](/nl/guide/creation-functions/combination/combineLatest).

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| [mergeAll](/nl/guide/operators/combination/mergeAll) | Abonneer op alle interne Observables parallel |
| [concatAll](/nl/guide/operators/combination/concatAll) | Abonneer op interne Observables in volgorde |
| [switchAll](/nl/guide/operators/combination/switchAll) | Schakel naar nieuwe interne Observable |
| [zipAll](/nl/guide/operators/combination/zipAll) | Paar waarden in corresponderende volgorde van elke interne Observable |

## ⚠️ Belangrijke opmerkingen

### Higher-order Observable moet voltooien

`combineLatestAll` wacht om interne Observables te verzamelen totdat de Higher-order Observable (buitenste Observable) **voltooit**.

#### ❌ Niets output omdat Higher-order Observable niet voltooit
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Niets output
```

#### ✅ Voltooi met take
```ts
interval(1000).pipe(
  take(3), // Voltooi na 3
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Alle interne Observables moeten ten minste één keer emitteren

Er worden geen waarden uitgegeven totdat alle interne Observables **ten minste één keer hebben geëmitteerd**.

```ts
import { of, NEVER } from 'rxjs';
import { combineLatestAll } from 'rxjs';

// Niets output als zelfs één interne Observable nooit emitteert
of(
  of(1, 2, 3),
  NEVER // Emitteert nooit
).pipe(
  combineLatestAll()
).subscribe(console.log); // Niets output
```

### Geheugengebruik

Let op geheugengebruik als er veel interne Observables zijn, aangezien **laatste waarden van alle interne Observables in het geheugen worden bewaard**.
