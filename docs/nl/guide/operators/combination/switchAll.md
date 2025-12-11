---
description: switchAll is een operator die een Higher-order Observable (Observable van Observables) neemt, schakelt naar een nieuwe interne Observable en annuleert de oude.
---

# switchAll - Schakel naar nieuwe interne Observable

De `switchAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**schakelt telkens wanneer een nieuwe interne Observable wordt geëmitteerd**, en annuleert de oude interne Observable.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start een nieuwe teller voor elke klik (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Schakel naar nieuwe teller (annuleer oude teller)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Output (met 3 klikken):
// 0 (1e teller)
// 1 (1e teller)
// ← Klik hier (1e geannuleerd)
// 0 (2e teller) ← Schakel naar nieuwe teller
// ← Klik hier (2e geannuleerd)
// 0 (3e teller) ← Schakel naar nieuwe teller
// 1 (3e teller)
// 2 (3e teller)
```

- Wanneer een nieuwe interne Observable wordt geëmitteerd door Higher-order Observable, **schakelt onmiddellijk**
- Vorige interne Observable wordt **automatisch geannuleerd**
- Alleen de laatste interne Observable is altijd actief

[🌐 RxJS Officiële Documentatie - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 Typische gebruikspatronen

- **Zoekfunctionaliteit (annuleer oude zoekopdrachten bij elke invoer)**
- **Autocomplete**
- **Realtime data-updates (schakel naar laatste databron)**

## 🧠 Praktisch codevoorbeeld

Voorbeeld van het annuleren van oude zoekopdrachten en alleen de laatste zoekopdracht uitvoeren bij elke invoer

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Voer zoekwoorden in';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Debounce invoergebeurtenissen
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Gesimuleerde zoek-API-aanroep voor elke invoerwaarde
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Gesimuleerde zoek-API-aanroep (1 seconde vertraging)
    return of(`Zoekresultaten: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Zoekopdracht #${id}] ${msg} (${elapsed} seconden)`;
      })
    );
  }),
  switchAll() // Annuleer oude zoekopdracht wanneer nieuwe start
);

results$.subscribe(result => {
  output.innerHTML = ''; // Wis vorige resultaten
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- **Oude zoekopdrachten worden automatisch geannuleerd** wanneer gebruiker invoer wijzigt
- Alleen de laatste zoekresultaten worden altijd weergegeven

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| `switchMap` | Afkorting voor `map` + `switchAll` (meest gebruikt) |
| [mergeAll](/nl/guide/operators/combination/mergeAll) | Abonneer op alle interne Observables parallel |
| [concatAll](/nl/guide/operators/combination/concatAll) | Abonneer op interne Observables in volgorde (wacht op vorige voltooiing) |
| [exhaustAll](/nl/guide/operators/combination/exhaustAll) | Negeer nieuwe interne Observables tijdens uitvoering |

## ⚠️ Belangrijke opmerkingen

### Geheugenlek preventie

`switchAll` helpt geheugenlekken te voorkomen door **automatisch oude interne Observables te annuleren**.
Het is ideaal voor frequente nieuwe verzoeken zoals zoekopdrachten of autocomplete.

### Niet-voltooiende interne Observables

Zelfs als interne Observable niet voltooit, zal het automatisch schakelen wanneer een nieuwe interne Observable wordt geëmitteerd.

```ts
// interval voltooit nooit, maar wordt automatisch geannuleerd bij volgende klik
clicks$.pipe(
  map(() => interval(1000)), // Voltooit nooit
  switchAll()
).subscribe();
```

### Optimaal wanneer alleen laatste waarde belangrijk is

Gebruik `switchAll` wanneer u geen resultaten van oude verwerking nodig heeft en **alleen het laatste resultaat belangrijk is**.
Als alle resultaten nodig zijn, gebruik [mergeAll](/nl/guide/operators/combination/mergeAll).
