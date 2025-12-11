---
description: mergeAll is een operator die een Higher-order Observable (Observable van Observables) neemt en parallel abonneert op alle interne Observables om de waarden af te vlakken.
titleTemplate: ':title | RxJS'
---

# mergeAll - Vlak alle interne Observables parallel af

De `mergeAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**abonneert parallel op alle interne Observables**, en vlakt hun waarden af.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start een nieuwe teller voor elke klik (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Abonneer parallel op alle tellers
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Output (met 3 klikken):
// 0 (1e teller)
// 1 (1e teller)
// 0 (2e teller) ← parallelle uitvoering
// 2 (1e teller)
// 1 (2e teller)
// 0 (3e teller) ← parallelle uitvoering
// ...
```

- **Abonneer parallel** op elke interne Observable die wordt geëmitteerd door Higher-order Observable
- **Combineer waarden** van alle interne Observables tot een **enkele stream**
- Kan aantal gelijktijdige abonnementen beperken (`mergeAll(2)` = maximaal 2 gelijktijdig)

[🌐 RxJS Officiële Documentatie - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 Typische gebruikspatronen

- **Voer meerdere API-aanroepen parallel uit**
- **Start onafhankelijke streams voor elke gebruikersactie**
- **Integreer meerdere realtime verbindingen zoals WebSocket en EventSource**

## 🧠 Praktisch codevoorbeeld

Voorbeeld van het uitvoeren van gelijktijdige API-aanroepen (gesimuleerd) bij elke invoerwijziging

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Voer zoekwoorden in';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Debounce invoergebeurtenissen
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Gesimuleerde API-aanroep voor elke invoerwaarde
const results$ = search$.pipe(
  map(query =>
    // Gesimuleerde API-aanroep (500ms vertraging)
    of(`Resultaat: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Voer alle API-aanroepen parallel uit
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- **Alle API-aanroepen worden parallel uitgevoerd**, zelfs als gebruiker snel invoer wijzigt
- Oude zoekresultaten kunnen na nieuwe resultaten verschijnen (geen volgordegarantie)

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| `mergeMap` | Afkorting voor `map` + `mergeAll` (vaak gebruikt) |
| [concatAll](/nl/guide/operators/combination/concatAll) | Abonneer op interne Observables in volgorde (wacht op vorige voltooiing) |
| [switchAll](/nl/guide/operators/combination/switchAll) | Schakel naar nieuwe interne Observable (annuleer oude) |
| [exhaustAll](/nl/guide/operators/combination/exhaustAll) | Negeer nieuwe interne Observables tijdens uitvoering |

## ⚠️ Belangrijke opmerkingen

### Gelijktijdige abonnementen beperken

Het niet beperken van gelijktijdige abonnementen kan prestatieproblemen veroorzaken.

```ts
// Beperk gelijktijdige abonnementen tot 2
higherOrder$.pipe(
  mergeAll(2) // Maximaal 2 gelijktijdige uitvoeringen
).subscribe();
```

### Geen volgordegarantie

Omdat `mergeAll` gelijktijdig uitvoert, is **de volgorde van waarden niet gegarandeerd**.
Als volgorde cruciaal is, gebruik [concatAll](/nl/guide/operators/combination/concatAll).
