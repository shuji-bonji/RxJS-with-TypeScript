---
description: reduce is een RxJS conversieoperator die alle waarden in een stream accumuleert en alleen het eindresultaat uitvoert bij voltooiing. Het is ideaal voor situaties waar alleen het uiteindelijke aggregatieresultaat nodig is, zoals het berekenen van sommen, gemiddelden, maxima, minima, het aggregeren van objecten en het bouwen van arrays. In tegenstelling tot scan, voert het geen tussenresultaten uit en kan het niet worden gebruikt met oneindige streams omdat streamvoltooiing vereist is.
titleTemplate: ':title | RxJS'
---

# reduce - Voert alleen het eindresultaat van accumulatie uit

De `reduce` operator past een cumulatieve functie toe op elke waarde in de stream en voert **alleen het uiteindelijke cumulatieve resultaat** uit bij streamvoltooiing.
Het werkt hetzelfde als `Array.prototype.reduce` voor arrays, zonder uitvoer van tussenresultaten.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 15 (alleen eindresultaat)
```

- `acc` is de cumulatieve waarde, `curr` is de huidige waarde.
- De waarden worden sequentieel geaccumuleerd, beginnend vanaf de initiële waarde (`0` in dit geval).
- Er wordt geen waarde uitgevoerd tot de stream voltooit, en **alleen het eindresultaat** wordt bij voltooiing uitgevoerd.

[🌐 RxJS Officiële Documentatie - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Typische gebruikspatronen

- Berekenen van sommen, gemiddelden, maxima en minima van getallen
- Aggregeren en transformeren van objecten
- Bouwen of combineren van arrays
- Wanneer alleen het uiteindelijke aggregatieresultaat nodig is

## 🔍 Verschil met scan

| Operator | Uitvoertiming | Uitvoerinhoud | Gebruik |
|:---|:---|:---|:---|
| `reduce` | **Slechts eenmaal bij voltooiing** | Uiteindelijk cumulatief resultaat | Aggregatie waar alleen eindresultaat nodig is |
| `scan` | **Elke keer voor elke waarde** | Alles inclusief tussenresultaten | Realtime aggregatie/statusbeheer |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 1, 3, 6, 10, 15
```

## 🧠 Praktisch codevoorbeeld (met UI)

Dit voorbeeld telt de waarden van meerdere invoervelden op en toont het eindresultaat bij een klik op de knop.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Maak invoervelden
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Waarde ${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Bereken knop
const button = document.createElement('button');
button.textContent = 'Bereken som';
document.body.appendChild(button);

// Resultaat weergavegebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Bereken som bij knopklik
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Haal alle invoerwaarden op
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Totaal: ${total}`;
  console.log('Totaal:', total);
});
```

- Bij knopklik worden alle invoerwaarden opgeteld en alleen het eindtotaal wordt weergegeven.
- Tussenresultaten worden niet uitgevoerd.

## 🎯 Object aggregatie voorbeeld

Dit is een praktisch voorbeeld van het aggregeren van meerdere waarden in een object.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Voedsel', price: 500 },
  { category: 'Drank', price: 200 },
  { category: 'Voedsel', price: 800 },
  { category: 'Drank', price: 150 },
  { category: 'Voedsel', price: 300 },
];

// Aggregeer totaalprijs per categorie
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Totaal per categorie:', result);
});

// Output:
// Totaal per categorie: { Voedsel: 1600, Drank: 350 }
```

## 🎯 Array constructie voorbeeld

Hier is een voorbeeld van het combineren van streamwaarden in een array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Verzamelde array:', array);
});

// Output:
// Verzamelde array: [0, 1, 2, 3, 4]
```

::: tip
Bij het bouwen van een array, overweeg de beknoptere [`toArray`](/nl/guide/operators/utility/toArray) operator te gebruiken.
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Output: [0, 1, 2, 3, 4]
```
:::

## 💡 Type-veilig reduce gebruiken

Hier is een voorbeeld van het benutten van TypeScript's type-inferentie.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Aggregeer telling per actietype
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Actie-aggregatie:', result);
});

// Output:
// Actie-aggregatie: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Opmerkingen

### ❌ Oneindige streams voltooien niet (Belangrijk)

> [!WARNING]
> **`reduce` zal geen enkele waarde uitvoeren tot `complete()` wordt aangeroepen.** Oneindige streams (`interval`, `fromEvent`, etc.) veroorzaken ongelukken in de praktijk, omdat er permanent geen waarde beschikbaar is.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Slecht voorbeeld: Oneindige stream dus geen waarde wordt uitgevoerd
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Geen output (stream voltooit niet)
```

**Tegenmaatregel 1: Gebruik `scan` wanneer rollende aggregatie vereist is**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Goed voorbeeld: Krijg tussenresultaten in realtime
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 0, 1, 3, 6, 10 (voert cumulatieve waarde elke keer uit)
```

**Tegenmaatregel 2: Als alleen de eindwaarde nodig is, gebruik `scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Goed voorbeeld: Accumuleer met scan, krijg alleen eindwaarde
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Output: 10 (alleen eindresultaat)
```

**Tegenmaatregel 3: Gebruik `take` om de eindvoorwaarde te specificeren**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Goed voorbeeld: Stel eindvoorwaarde in met take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 10
```

> [!TIP]
> **Selectiecriteria**:
> - Tussenresultaten zijn vereist → `scan`
> - Alleen eindresultaat nodig & streamvoltooiing is gegarandeerd → `reduce`
> - Alleen eindresultaat nodig & oneindige stream → `scan` + `takeLast(1)` of `take` + `reduce`

### Geheugengebruik

Wanneer de cumulatieve waarde een groot object of array is, moet rekening worden gehouden met geheugengebruik.

```ts
// Voorbeeld dat geheugen-aandacht vereist
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Accumuleer grote hoeveelheden data
    return acc;
  }, [])
).subscribe();
```

## 📚 Gerelateerde operators

- [`scan`](/nl/guide/operators/transformation/scan) - Voert een tussenresultaat uit voor elke waarde
- [`toArray`](/nl/guide/operators/utility/toArray) - Combineer alle waarden in een array
- [`count`](https://rxjs.dev/api/operators/count) - Telt het aantal waarden
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Krijg minimum- en maximumwaarden

## Samenvatting

De `reduce` operator accumuleert alle waarden in een stream en voert **alleen het eindresultaat bij voltooiing** uit. Dit is geschikt wanneer tussenresultaten niet nodig zijn en alleen het uiteindelijke aggregatieresultaat nodig is. Echter, aangezien geen resultaat wordt verkregen als de stream niet voltooit, moet u `scan` gebruiken voor oneindige streams, of een exitvoorwaarde instellen met `take` of vergelijkbaar.
