---
description: concatAll is een operator die een Higher-order Observable (Observable van Observables) neemt en waarden afvlakt door in volgorde te abonneren op interne Observables. Het start de volgende pas nadat de vorige Observable is voltooid.
titleTemplate: ':title | RxJS'
---

# concatAll - Vlak interne Observables sequentieel af

De `concatAll` operator neemt een **Higher-order Observable** (Observable van Observables),
**abonneert op interne Observables in volgorde**, en vlakt hun waarden af. Het start niet met de volgende totdat de vorige Observable is voltooid.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start een nieuwe teller voor elke klik (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Abonneer op tellers in volgorde (start volgende na voltooiing van vorige)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Output (met 3 klikken):
// 0 (1e teller)
// 1 (1e teller)
// 2 (1e teller) ← Voltooid
// 0 (2e teller) ← Start na voltooiing 1e
// 1 (2e teller)
// 2 (2e teller) ← Voltooid
// 0 (3e teller) ← Start na voltooiing 2e
// ...
```

- **Abonneer in volgorde** op elke interne Observable die wordt geëmitteerd door Higher-order Observable
- **Start niet met volgende** totdat vorige interne Observable is voltooid
- Volgorde van waarden is gegarandeerd

[🌐 RxJS Officiële Documentatie - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 Typische gebruikspatronen

- **Voer API-aanroepen in volgorde uit (voer volgende uit na voltooiing van vorig verzoek)**
- **Speel animaties in volgorde af**
- **Verwerk bestandsuploads sequentieel**

## 🧠 Praktisch codevoorbeeld

Voorbeeld van het in volgorde uitvoeren van API-aanroepen (gesimuleerd) voor elke knopklik

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'API Aanroep';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Knopklikgebeurtenis
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: Gesimuleerde API-aanroep voor elke klik
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Gesimuleerde API-aanroep (2 seconden vertraging)
    return of(`API aanroep #${id} voltooid`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} seconden)`;
      })
    );
  }),
  concatAll() // Voer alle API-aanroepen in volgorde uit
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Zelfs met opeenvolgende knopklikken worden **API-aanroepen in volgorde uitgevoerd**
- Volgende API-aanroep start na voltooiing van vorige

## 🔄 Gerelateerde operators

| Operator | Beschrijving |
|---|---|
| `concatMap` | Afkorting voor `map` + `concatAll` (vaak gebruikt) |
| [mergeAll](/nl/guide/operators/combination/mergeAll) | Abonneer op alle interne Observables parallel |
| [switchAll](/nl/guide/operators/combination/switchAll) | Schakel naar nieuwe interne Observable (annuleer oude) |
| [exhaustAll](/nl/guide/operators/combination/exhaustAll) | Negeer nieuwe interne Observables tijdens uitvoering |

## ⚠️ Belangrijke opmerkingen

### Backpressure (Wachtrij-ophoping)

Als de emissieratio van interne Observable sneller is dan de voltooiingsratio, zullen **onverwerkte Observables zich ophopen in de wachtrij**.

```ts
// Klik elke seconde → API-aanroep duurt 2 seconden
// → Wachtrij kan continu groeien
```

In dit geval, overweeg deze tegenmaatregelen:
- Gebruik `switchAll` (verwerk alleen laatste)
- Gebruik `exhaustAll` (negeer tijdens uitvoering)
- Voeg debounce of throttling toe

### Pas op voor oneindige Observables

Als vorige Observable **nooit voltooit, zal de volgende nooit starten**.

#### ❌ interval voltooit nooit, dus 2e teller start nooit
```ts
clicks$.pipe(
  map(() => interval(1000)), // Voltooit nooit
  concatAll()
).subscribe();
```
#### ✅ Voltooi met take
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Voltooit na 3
  concatAll()
).subscribe();
```
